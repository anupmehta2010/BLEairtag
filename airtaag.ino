/*
  ╔════════════════════════════════════════════════════════════════════════════╗
  ║  ESP32-C3 Super Mini Advanced Bluetooth AirTag Tracker                    ║
  ║  with MPU6050 Motion Detection & Complete State Machine                   ║
  ║  v4.0 - FINAL: Complete State Flow with Reset Button Logic               ║
  ║                                                                            ║
  ║  BEHAVIOR:                                                                 ║
  ║  • Reset → 1x Default Beep → Pairing Mode (Fast LED Blink)               ║
  ║  • Connection → Default Connected Beep → LED Constant (In Range)         ║
  ║  • Out of Range → Sync LED Blink + Sync Beep (continuous)               ║
  ║  • Back in Range → LED Constant, Beep silent, Loop continues            ║
  ║  • Disconnected → 1x Final Beep → Pairing Mode (Fast LED Blink)         ║
  ╚════════════════════════════════════════════════════════════════════════════╝
*/

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <BLEScan.h>
#include <Wire.h>
#include <Preferences.h>
#include <esp_sleep.h>
#include <driver/rtc_io.h>
#include <esp_attr.h>


// ╔═══════════════════════════════════════════════════════════════════════════╗
// ║ PIN CONFIGURATION                                                         ║
// ╚═══════════════════════════════════════════════════════════════════════════╝

#define BLUE_LED_PIN       8      // Built-in LED (active LOW)
#define BUZZER_PIN         2      // Active buzzer via transistor
#define BOOT_BUTTON_PIN    9      // Built-in BOOT button
#define MPU_INT_PIN        3      // MPU6050 interrupt
#define MPU_SDA_PIN        5      // MPU6050 I2C data
#define MPU_SCL_PIN        4      // MPU6050 I2C clock
#define MPU_ADDR           0x68   // MPU6050 I2C address


// ╔═══════════════════════════════════════════════════════════════════════════╗
// ║ MPU6050 REGISTER DEFINITIONS                                              ║
// ╚═══════════════════════════════════════════════════════════════════════════╝

#define MPU_PWR_MGMT_1      0x6B
#define MPU_INT_ENABLE      0x38
#define MPU_INT_STATUS      0x3A
#define MPU_ACCEL_XOUT_H    0x3B
#define MPU_MOT_THR         0x1F
#define MPU_MOT_DUR         0x20
#define MPU_MOT_DETECT_CTRL 0x69
#define MPU_INT_PIN_CFG     0x37


// ╔═══════════════════════════════════════════════════════════════════════════╗
// ║ DEVICE STATE ENUMERATION                                                  ║
// ╚═══════════════════════════════════════════════════════════════════════════╝

enum DeviceState {
  STATE_PAIRING_MODE,       // Waiting for BLE connection
  STATE_CONNECTED_IN_RANGE, // Connected & within RSSI threshold (NEAR)
  STATE_CONNECTED_OUT_RANGE // Connected & out of RSSI threshold (FAR)
};


// ╔═══════════════════════════════════════════════════════════════════════════╗
// ║ CONFIGURATION CONSTANTS                                                   ║
// ╚═══════════════════════════════════════════════════════════════════════════╝

const uint8_t MOTION_THRESHOLD = 15;
const uint8_t MOTION_DURATION = 1;
const int RSSI_AVG_WINDOW = 3;
const int RSSI_NEAR_THRESHOLD = -75;
const int RSSI_HYSTERESIS = 3;
const int SCAN_SECONDS = 1;

const unsigned long IDLE_TIMEOUT_MS = 180000;
const unsigned long LED_BLINK_FAST = 150;           // Pairing mode: fast blink
const unsigned long LED_BLINK_SYNC = 500;           // Out of range: synchronized blink
const unsigned long BLE_RECONNECT_DELAY = 2000;

// ─── BEEP PATTERNS ───
const unsigned long BEEP_SYNC_INTERVAL = 500;       // Out of range: sync with LED
const unsigned long BEEP_PAIRING_INTERVAL = 1000;   // Pairing mode: default interval
const unsigned long BEEP_DURATION = 100;            // Beep length
const unsigned long BUTTON_DEBOUNCE_MS = 50;        // Button debounce time


// ╔═══════════════════════════════════════════════════════════════════════════╗
// ║ GLOBAL STATE VARIABLES                                                    ║
// ╚═══════════════════════════════════════════════════════════════════════════╝

Preferences prefs;
BLEScan* pScan = nullptr;
BLEServer* pServer = nullptr;
BLEAdvertising* pAdv = nullptr;

int rssiBuf[RSSI_AVG_WINDOW];
int rssiIdx = 0;
bool haveAvg = false;

bool deviceConnected = false;
bool wasConnected = false;
bool isNear = false;
bool motionDetected = false;
bool bootComplete = false;
bool bleInitialized = false;

// ─── STATE MANAGEMENT ───
DeviceState currentState = STATE_PAIRING_MODE;
DeviceState previousState = STATE_PAIRING_MODE;

// ─── TIMING VARIABLES ───
unsigned long lastActivity = 0;
unsigned long lastBlinkMs = 0;
unsigned long lastStatusMs = 0;
unsigned long lastScanMs = 0;
unsigned long lastReconnectAttempt = 0;
unsigned long lastBeepMs = 0;
unsigned long lastButtonPressMs = 0;

// ─── STATE CHANGE FLAGS ───
bool stateJustChanged = false;

esp_sleep_wakeup_cause_t wakeup_cause;


// ╔═══════════════════════════════════════════════════════════════════════════╗
// ║ MPU6050 I2C HELPER FUNCTIONS                                              ║
// ╚═══════════════════════════════════════════════════════════════════════════╝

void writeMPURegister(uint8_t reg, uint8_t data) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(data);
  Wire.endTransmission();
}

uint8_t readMPURegister(uint8_t reg) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, (uint8_t)1);
  return Wire.read();
}


// ╔═══════════════════════════════════════════════════════════════════════════╗
// ║ MPU6050 INITIALIZATION & CONFIGURATION                                    ║
// ╚═══════════════════════════════════════════════════════════════════════════╝

void setupMPU6050() {
  Serial.println("[MPU] Initializing MPU6050...");
  
  writeMPURegister(MPU_PWR_MGMT_1, 0x00);
  delay(100);
  
  writeMPURegister(MPU_INT_PIN_CFG, 0x30);
  writeMPURegister(MPU_MOT_THR, MOTION_THRESHOLD);
  writeMPURegister(MPU_MOT_DUR, MOTION_DURATION);
  writeMPURegister(MPU_MOT_DETECT_CTRL, 0x15);
  writeMPURegister(MPU_INT_ENABLE, 0x40);
  
  Serial.println("[MPU] Motion interrupt configured");
}

void readMPUAccel(float& ax, float& ay, float& az) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(MPU_ACCEL_XOUT_H);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, (uint8_t)6);
  
  int16_t rawAx = (Wire.read() << 8) | Wire.read();
  int16_t rawAy = (Wire.read() << 8) | Wire.read();
  int16_t rawAz = (Wire.read() << 8) | Wire.read();
  
  ax = rawAx / 16384.0;
  ay = rawAy / 16384.0;
  az = rawAz / 16384.0;
}


// ╔═══════════════════════════════════════════════════════════════════════════╗
// ║ AUDIO FEEDBACK (BUZZER) - COMPLETE PATTERNS                              ║
// ╚═══════════════════════════════════════════════════════════════════════════╝

void beepOnce(int duration = 100) {
  digitalWrite(BUZZER_PIN, HIGH);
  delay(duration);
  digitalWrite(BUZZER_PIN, LOW);
}

// ─── RESET BEEP: 1x Default tone (startup only) ───
void beepReset() {
  Serial.println("[BEEP] Reset button - 1x default beep");
  beepOnce(200);
}

// ─── CONNECTION BEEP: Default connected tone ───
void beepConnected() {
  Serial.println("[BEEP] Connected - default connection beep");
  beepOnce(150);
  delay(50);
  beepOnce(150);
}

// ─── DISCONNECTION BEEP: Final beep (1x) ───
void beepDisconnected() {
  Serial.println("[BEEP] Disconnected - 1x final beep");
  beepOnce(200);
}

// ─── STARTUP BEEP SEQUENCE ───
void beepStartup() {
  beepOnce(200);
  delay(100);
  beepOnce(200);
  delay(100);
  beepOnce(300);
}

// ─── SYNCHRONIZED BEEP FOR OUT OF RANGE (Matches LED blink 500ms) ───
void beepSyncOutOfRange() {
  unsigned long now = millis();
  if (now - lastBeepMs >= BEEP_SYNC_INTERVAL) {
    lastBeepMs = now;
    beepOnce(BEEP_DURATION);
  }
}


// ╔═══════════════════════════════════════════════════════════════════════════╗
// ║ VISUAL FEEDBACK (LED) - COMPLETE PATTERNS                                ║
// ╚═══════════════════════════════════════════════════════════════════════════╝

void ledOn() {
  digitalWrite(BLUE_LED_PIN, LOW);
}

void ledOff() {
  digitalWrite(BLUE_LED_PIN, HIGH);
}

void ledStartupPattern() {
  for (int i = 0; i < 4; i++) {
    ledOn();
    delay(100);
    ledOff();
    delay(100);
  }
}

// ─── CONSTANT LED: In Range (no blinking) ───
void ledConstantOn() {
  ledOn();
}

// ─── FAST BLINK: Pairing mode (150ms) ───
void ledBlinkFastPairing() {
  unsigned long now = millis();
  if (now - lastBlinkMs >= LED_BLINK_FAST) {
    lastBlinkMs = now;
    digitalWrite(BLUE_LED_PIN, !digitalRead(BLUE_LED_PIN));
  }
}

// ─── SYNCHRONIZED BLINK: Out of Range (500ms, matches beep) ───
void ledBlinkSyncOutOfRange() {
  unsigned long now = millis();
  if (now - lastBlinkMs >= LED_BLINK_SYNC) {
    lastBlinkMs = now;
    digitalWrite(BLUE_LED_PIN, !digitalRead(BLUE_LED_PIN));
  }
}


// ╔═══════════════════════════════════════════════════════════════════════════╗
// ║ RSSI PROCESSING & PROXIMITY DETECTION                                     ║
// ╚═══════════════════════════════════════════════════════════════════════════╝

int getAverageRSSI() {
  if (!haveAvg) return -127;
  long sum = 0;
  for (int i = 0; i < RSSI_AVG_WINDOW; i++) sum += rssiBuf[i];
  return sum / RSSI_AVG_WINDOW;
}

void pushRSSI(int rssi) {
  rssiBuf[rssiIdx++] = rssi;
  if (rssiIdx >= RSSI_AVG_WINDOW) {
    rssiIdx = 0;
    haveAvg = true;
  }
}

void updateProximity(int rssiAvg) {
  if (!isNear && rssiAvg >= RSSI_NEAR_THRESHOLD + RSSI_HYSTERESIS) {
    isNear = true;
    Serial.printf("[PROX] FAR → NEAR (RSSI: %d dBm) ✓ In Range\n", rssiAvg);
    lastBeepMs = 0;
    stateJustChanged = true;
  } else if (isNear && rssiAvg <= RSSI_NEAR_THRESHOLD - RSSI_HYSTERESIS) {
    isNear = false;
    Serial.printf("[PROX] NEAR → FAR (RSSI: %d dBm) ✗ Out of Range\n", rssiAvg);
    lastBeepMs = 0;
    stateJustChanged = true;
  }
}


// ╔═══════════════════════════════════════════════════════════════════════════╗
// ║ STATE MACHINE LOGIC                                                       ║
// ╚═══════════════════════════════════════════════════════════════════════════╝

void updateDeviceState() {
  // Determine next state based on connection and proximity
  DeviceState nextState = currentState;
  
  if (!deviceConnected) {
    nextState = STATE_PAIRING_MODE;
  } else if (deviceConnected && isNear) {
    nextState = STATE_CONNECTED_IN_RANGE;
  } else if (deviceConnected && !isNear) {
    nextState = STATE_CONNECTED_OUT_RANGE;
  }
  
  // Detect state change
  if (nextState != currentState) {
    previousState = currentState;
    currentState = nextState;
    stateJustChanged = true;
    lastBeepMs = 0;
    lastBlinkMs = 0;
    
    Serial.print("[STATE] Transition: ");
    printStateTransition();
  }
}

void printStateTransition() {
  String prev, curr;
  
  if (previousState == STATE_PAIRING_MODE) prev = "PAIRING";
  else if (previousState == STATE_CONNECTED_IN_RANGE) prev = "IN_RANGE";
  else if (previousState == STATE_CONNECTED_OUT_RANGE) prev = "OUT_RANGE";
  
  if (currentState == STATE_PAIRING_MODE) curr = "PAIRING";
  else if (currentState == STATE_CONNECTED_IN_RANGE) curr = "IN_RANGE";
  else if (currentState == STATE_CONNECTED_OUT_RANGE) curr = "OUT_RANGE";
  
  Serial.printf("%s → %s\n", prev.c_str(), curr.c_str());
}

void executeStateLogic() {
  // ─── STATE 1: PAIRING MODE ───
  if (currentState == STATE_PAIRING_MODE) {
    ledBlinkFastPairing();  // Fast LED blink (150ms)
    // No automatic beeping in pairing mode, only manual feedback
    
    if (stateJustChanged) {
      Serial.println("[FB] Pairing Mode: Fast LED blink, waiting for connection");
      stateJustChanged = false;
    }
  }
  
  // ─── STATE 2: CONNECTED & IN RANGE ───
  else if (currentState == STATE_CONNECTED_IN_RANGE) {
    ledConstantOn();  // LED constant ON
    // No beeping - silent mode (happy state)
    
    if (stateJustChanged) {
      Serial.println("[FB] Connected & In Range: LED ON, Silent");
      stateJustChanged = false;
    }
  }
  
  // ─── STATE 3: CONNECTED & OUT OF RANGE ───
  else if (currentState == STATE_CONNECTED_OUT_RANGE) {
    ledBlinkSyncOutOfRange();     // Synchronized LED blink (500ms)
    beepSyncOutOfRange();          // Synchronized beep (500ms)
    
    if (stateJustChanged) {
      Serial.println("[FB] Connected & Out of Range: LED Blink + Beep Sync (500ms)");
      stateJustChanged = false;
    }
  }
}


// ╔═══════════════════════════════════════════════════════════════════════════╗
// ║ RESET BUTTON HANDLING                                                     ║
// ╚═══════════════════════════════════════════════════════════════════════════╝

void checkResetButton() {
  unsigned long now = millis();
  static bool lastButtonState = HIGH;
  bool currentButtonState = digitalRead(BOOT_BUTTON_PIN);
  
  // Debounce the button
  if (currentButtonState == LOW && lastButtonState == HIGH) {
    if (now - lastButtonPressMs > BUTTON_DEBOUNCE_MS) {
      lastButtonPressMs = now;
      
      Serial.println("\n╔════════════════════════════════════════════════════════╗");
      Serial.println("║  RESET BUTTON PRESSED                                  ║");
      Serial.println("╚════════════════════════════════════════════════════════╝\n");
      
      // Play reset beep
      beepReset();
      
      // Reset to pairing mode
      if (deviceConnected) {
        Serial.println("[RESET] Disconnecting from BLE...");
        BLEDevice::deinit(false);
        delay(500);
        bleInitialized = false;
      }
      
      // Reinitialize
      lastBeepMs = 0;
      lastBlinkMs = 0;
      setupBLE();
      
      Serial.println("[RESET] Returned to Pairing Mode\n");
    }
  }
  
  lastButtonState = currentButtonState;
}


// ╔═══════════════════════════════════════════════════════════════════════════╗
// ║ BLE SERVER CALLBACKS                                                      ║
// ╚═══════════════════════════════════════════════════════════════════════════╝

class ServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) override {
    Serial.println("\n[BLE] ✓ Client connected");
    deviceConnected = true;
    wasConnected = true;
    lastActivity = millis();
    lastReconnectAttempt = millis();
    lastBeepMs = 0;
    
    // Stop advertising immediately on connect
    pAdv->stop();
    
    // Play connection beep
    beepConnected();
    
    // Reset proximity state
    isNear = false;
    haveAvg = false;
    
    stateJustChanged = true;
  }
  
  void onDisconnect(BLEServer* pServer) override {
    Serial.println("\n[BLE] ✗ Client disconnected");
    deviceConnected = false;
    
    // Play disconnection beep
    beepDisconnected();
    
    lastReconnectAttempt = millis();
    lastBeepMs = 0;
    lastBlinkMs = 0;
    
    stateJustChanged = true;
  }
};


// ╔═══════════════════════════════════════════════════════════════════════════╗
// ║ BLE INITIALIZATION                                                        ║
// ╚═══════════════════════════════════════════════════════════════════════════╝

void setupBLE() {
  Serial.println("[BLE] Initializing BLE...");
  
  if (bleInitialized) {
    BLEDevice::deinit(false);
    delay(500);
  }
  
  BLEDevice::init("ESP32-AirTag");
  
  BLEDevice::setPower(ESP_PWR_LVL_P9, ESP_BLE_PWR_TYPE_ADV);
  BLEDevice::setPower(ESP_PWR_LVL_P9, ESP_BLE_PWR_TYPE_SCAN);
  
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new ServerCallbacks());
  
  BLEService* pService = pServer->createService(BLEUUID((uint16_t)0x180A));
  
  BLECharacteristic* pChar = pService->createCharacteristic(
    BLEUUID((uint16_t)0x2A00),
    BLECharacteristic::PROPERTY_READ
  );
  pChar->setValue("ESP32-AirTag");
  pService->start();
  
  pAdv = BLEDevice::getAdvertising();
  pAdv->addServiceUUID(BLEUUID((uint16_t)0x180A));
  pAdv->setScanResponse(true);
  pAdv->setMinPreferred(0x06);
  pAdv->setMaxPreferred(0x12);
  pAdv->start();
  
  pScan = BLEDevice::getScan();
  pScan->setActiveScan(false);
  pScan->setInterval(30);
  pScan->setWindow(30);
  
  bleInitialized = true;
  Serial.println("[BLE] ✓ BLE initialized and advertising\n");
}


// ╔═══════════════════════════════════════════════════════════════════════════╗
// ║ SETUP                                                                     ║
// ╚═══════════════════════════════════════════════════════════════════════════╝

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("\n╔════════════════════════════════════════════════════════╗");
  Serial.println("║  ESP32-C3 Advanced AirTag Tracker                     ║");
  Serial.println("║  v4.0 - Complete State Machine                        ║");
  Serial.println("║  Reset→Beep→Pairing | Connected→Beep→InRange        ║");
  Serial.println("║  OutOfRange→SyncLED+Beep | Disconnected→Beep→Pairing║");
  Serial.println("╚════════════════════════════════════════════════════════╝\n");
  
  // ─── GPIO INITIALIZATION ───
  pinMode(BLUE_LED_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(BOOT_BUTTON_PIN, INPUT_PULLUP);
  pinMode(MPU_INT_PIN, INPUT_PULLUP);
  
  ledOff();
  digitalWrite(BUZZER_PIN, LOW);
  
  // ─── I2C INITIALIZATION ───
  Wire.begin(MPU_SDA_PIN, MPU_SCL_PIN);
  delay(100);
  
  // ─── MPU6050 INITIALIZATION ───
  Wire.beginTransmission(MPU_ADDR);
  if (Wire.endTransmission() != 0) {
    Serial.println("[ERROR] MPU6050 not detected! Check wiring.");
    while (1) {
      beepOnce(100);
      delay(100);
      beepOnce(100);
      delay(1000);
    }
  }
  
  setupMPU6050();
  
  // ─── STARTUP FEEDBACK ───
  Serial.println("[BOOT] Device starting...");
  beepStartup();
  ledStartupPattern();
  
  // ─── BLE INITIALIZATION ───
  setupBLE();
  
  // ─── STATE INITIALIZATION ───
  for (int i = 0; i < RSSI_AVG_WINDOW; i++) {
    rssiBuf[i] = -127;
  }
  
  currentState = STATE_PAIRING_MODE;
  lastActivity = millis();
  lastBeepMs = millis();
  bootComplete = true;
  
  Serial.println("[BOOT] ✓ Initialization complete. Waiting for connection...\n");
}


// ╔═══════════════════════════════════════════════════════════════════════════╗
// ║ MAIN LOOP - COMPLETE STATE MACHINE                                        ║
// ╚═══════════════════════════════════════════════════════════════════════════╝

void loop() {
  unsigned long now = millis();
  
  // ─── CHECK RESET BUTTON ───
  checkResetButton();
  
  // ─── BLE CONNECTION MANAGEMENT ───
  if (deviceConnected && !wasConnected) {
    wasConnected = true;
    isNear = false;
    Serial.println("[STATE] Connected - Proximity monitoring active");
    for (int i = 0; i < RSSI_AVG_WINDOW; i++) {
      rssiBuf[i] = -70;
    }
    rssiIdx = 0;
    haveAvg = false;
  }
  
  if (!deviceConnected && wasConnected) {
    wasConnected = false;
    isNear = false;
    Serial.println("[STATE] Lost connection");
    
    // Restart advertising
    if (now - lastReconnectAttempt > BLE_RECONNECT_DELAY) {
      lastReconnectAttempt = now;
      pAdv->start();
      Serial.println("[BLE] Advertising restarted");
    }
  }
  
  // ─── MOTION DETECTION ───
  uint8_t intStatus = readMPURegister(MPU_INT_STATUS);
  if (intStatus & 0x40) {
    lastActivity = now;
    motionDetected = true;
    Serial.println("[MOT] Motion detected!");
  }
  
  // ─── BLE SCANNING & PROXIMITY DETECTION ───
  if (deviceConnected && (now - lastScanMs >= (SCAN_SECONDS * 1000))) {
    lastScanMs = now;
    
    BLEScanResults* scanResults = pScan->start(SCAN_SECONDS, false);
    int bestRSSI = -127;
    
    for (int i = 0; i < scanResults->getCount(); i++) {
      int rssi = scanResults->getDevice(i).getRSSI();
      if (rssi > bestRSSI) bestRSSI = rssi;
    }
    
    pScan->clearResults();
    pushRSSI(bestRSSI);
    int avgRSSI = getAverageRSSI();
    
    if (haveAvg) {
      updateProximity(avgRSSI);
    }
  }
  
  // ─── UPDATE STATE MACHINE ───
  updateDeviceState();
  
  // ─── EXECUTE STATE LOGIC ───
  executeStateLogic();
  
  // ─── STATUS LOGGING ───
  if (now - lastStatusMs >= 5000) {
    lastStatusMs = now;
    int avgRSSI = getAverageRSSI();
    
    String stateStr;
    if (currentState == STATE_PAIRING_MODE) stateStr = "PAIRING";
    else if (currentState == STATE_CONNECTED_IN_RANGE) stateStr = "IN_RANGE";
    else if (currentState == STATE_CONNECTED_OUT_RANGE) stateStr = "OUT_RANGE";
    
    Serial.printf("[STATUS] State: %s | RSSI: %d dBm | Motion: %s\n",
                  stateStr.c_str(),
                  avgRSSI,
                  motionDetected ? "YES" : "NO");
    motionDetected = false;
  }
  
  delay(10);
}