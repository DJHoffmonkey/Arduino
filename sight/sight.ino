#include <Arduino.h>
#include <U8g2lib.h>
#include <Wire.h>

/* * MASTER AVIONICS (ESP32-C3 #1)
 * Role: MSP Data Parsing, HUD Rendering (72x40), Serial Bridge to Slave
 * Wiring: 
 * - HUD Screen: Pins 5 (SDA), 6 (SCL)
 * - FC Input: Pin 20 (RX)
 * - Slave Link: Pin 21 (TX) -> Connect to RX of Slave ESP
 */

// HUD Constructor
U8G2_SSD1306_72X40_ER_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE, /* clock=*/ 6, /* data=*/ 5);

// --- MISSION CONSTANTS ---
const int ARM_RC_CHANNEL = 13; 
const float LEAD_SENSITIVITY = 6.0;
const float ACCEL_1G = 512.0;
const float VOLT_THRESHOLD = 14.0;
const unsigned long BATT_DEBOUNCE = 2000;

// --- SYSTEM VARIABLES ---
float vBat = 16.0, currentG = 1.0, roll = 0, pitch = 0, lastPitch = 0, lastRoll = 0;
float filteredX = 0, filteredY = 0;
uint16_t armSwitchValue = 1000; // Default Disarmed
bool sessionHasMSP = false, currentlyReceiving = false, insaneModeActive = false;
unsigned long lastRequest = 0, lastDataTime = 0, lowVoltTimer = 0, lastBroadcast = 0;

// --- THE STARK HACK: Hardware Overdrive ---
void applyHardwareBoost(bool enable) {
  if (enable) {
    u8g2.sendF("c", 0xD9); u8g2.sendF("c", 0xF1); // Max Pre-charge
    u8g2.sendF("c", 0xDB); u8g2.sendF("c", 0x40); // Max VCOMH
    u8g2.sendF("c", 0x8D); u8g2.sendF("c", 0x14); // Enable Pump
    insaneModeActive = true;
  } else {
    u8g2.sendF("c", 0xD9); u8g2.sendF("c", 0x22); // Reset
    u8g2.sendF("c", 0xDB); u8g2.sendF("c", 0x20); 
    u8g2.sendF("c", 0x8D); u8g2.sendF("c", 0x14); 
    insaneModeActive = false;
  }
}

void sendMSPRequest(uint8_t cmd) { 
  uint8_t req[] = {'$', 'M', '<', 0, cmd, cmd}; 
  Serial1.write(req, 6); 
}

void readMSPResponse() {
  while (Serial1.available() >= 6) {
    if (Serial1.peek() != '$') { Serial1.read(); continue; }
    if (Serial1.available() >= 6) {
      if (Serial1.read() == '$' && Serial1.read() == 'M' && Serial1.read() == '>') {
        uint8_t size = Serial1.read(); 
        uint8_t cmd = Serial1.read();
        sessionHasMSP = true; lastDataTime = millis();
        
        if (cmd == 108) { // ATTITUDE
          int16_t angX = Serial1.read() | (Serial1.read() << 8);
          int16_t angY = Serial1.read() | (Serial1.read() << 8); 
          roll = angX / 10.0; pitch = angY / 10.0;
          for (int i = 0; i < size - 4; i++) Serial1.read();
        } 
        else if (cmd == 102) { // RAW IMU
          for (int i = 0; i < 4; i++) Serial1.read();
          int16_t az = Serial1.read() | (Serial1.read() << 8); 
          currentG = (((float)az / ACCEL_1G) * 0.1) + (currentG * 0.9);
          for (int i = 0; i < size - 6; i++) Serial1.read();
        } 
        else if (cmd == 110) { // ANALOG
          vBat = ( (Serial1.read() / 10.0) * 0.2) + (vBat * 0.8);
          for (int i = 0; i < size - 1; i++) Serial1.read();
        } 
        else if (cmd == 105) { // RC
          for (int i = 0; i < 8; i++) Serial1.read();
          for (int i = 4; i < ARM_RC_CHANNEL; i++) { Serial1.read(); Serial1.read(); }
          armSwitchValue = Serial1.read() | (Serial1.read() << 8); 
          int bytesConsumed = 8 + ((ARM_RC_CHANNEL - 4) * 2) + 2; 
          for (int i = 0; i < (size - bytesConsumed); i++) Serial1.read(); 
        } 
        else { for (int i = 0; i < size; i++) Serial1.read(); }
      }
    }
  }
}

void setup() {
  // UART1: RX Pin 20 (FC), TX Pin 21 (Slave Bridge)
  Serial1.begin(115200, SERIAL_8N1, 20, 21);
  
  u8g2.begin();
  u8g2.setContrast(20); // Startup Dim
  applyHardwareBoost(false);
}

void loop() {
  // 1. Data Acquisition from Flight Controller
  if (millis() - lastRequest > 50) { 
    sendMSPRequest(108); sendMSPRequest(105); 
    sendMSPRequest(102); sendMSPRequest(110); 
    lastRequest = millis(); 
  }
  readMSPResponse();

  // 2. Brightness Control (Arm-linked)
  bool isArmed = (armSwitchValue > 1800);
  if (isArmed && !insaneModeActive) {
    u8g2.setContrast(255);
    applyHardwareBoost(true);
  } else if (!isArmed && insaneModeActive) {
    u8g2.setContrast(20);
    applyHardwareBoost(false);
  }

  // 3. Physics & Ghost Simulation Logic
  currentlyReceiving = (millis() - lastDataTime < 1000);
  
  if (!sessionHasMSP || !currentlyReceiving) {
    // --- GHOST MODE (Simulation for Bench Testing) ---
    float t = millis() / 1000.0;
    filteredX = sin(t * 0.7) * 12.0; 
    filteredY = cos(t * 0.5) * 10.0;
    currentG = 1.0 + (sin(t * 1.2) * 0.1);
    vBat = 16.2 + (sin(t * 0.3) * 0.2);
    pitch = sin(t * 0.5) * 20.0;
    roll = cos(t * 0.8) * 45.0;
  } else {
    // --- REAL MODE (FC Physics) ---
    float pDelta = (pitch - lastPitch); 
    float rDelta = (roll - lastRoll);
    filteredX = ( (rDelta * -LEAD_SENSITIVITY) * 0.12) + (filteredX * 0.88);
    filteredY = ( (pDelta * LEAD_SENSITIVITY) * 0.12) + (filteredY * 0.88);
    lastPitch = pitch; 
    lastRoll = roll;
  }

  // 4. Serial Broadcast to Slave (20Hz)
  if (millis() - lastBroadcast > 50) {
    // Protocol: V, G, P, R
    Serial1.print(vBat); Serial1.print(",");
    Serial1.print(currentG); Serial1.print(",");
    Serial1.print(pitch); Serial1.print(",");
    Serial1.println(roll);
    lastBroadcast = millis();
  }

  // 5. HUD Rendering
  u8g2.firstPage();
  do {
    // Main reticle ring
    u8g2.drawCircle(36, 20, 17);
    // Dynamic Pipper
    u8g2.drawBox(36 + (int)filteredX - 1, 20 + (int)filteredY - 1, 3, 3);
    
    // Static G-Meter Bar
    u8g2.drawLine(0, 10, 0, 30);
    int gPos = 20 - (int)((currentG - 1.0) * 8.0);
    u8g2.drawHLine(0, constrain(gPos, 10, 30), 3);
  } while (u8g2.nextPage());
}