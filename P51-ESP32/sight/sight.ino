#include <Arduino.h>
#include <U8g2lib.h>
#include <Wire.h>
#include <esp_log.h>
#include <driver/uart.h>

/* --- MASTER AVIONICS (ESP32-C3 #1) ---
 * Role: MSP Parsing, K-14 HUD, 3-Char Tagged Serial Broadcast
 */

// Name the serial ports
// Corrected Aliases for ESP32-C3
//auto& console = Serial;      // USB CDC (to PC) - Only needs .begin(baud)
//auto& toSlave = Serial0;     // Hardware UART0 (to Slave) - Can take 4 arguments
//auto& toAndFromFC = Serial1; // Hardware UART1 (to FC) - Can take 4 arguments

// HARDWARE SERIAL DEFINITIONS GO HERE
#define console Serial 
HardwareSerial toAndFromFC(1); 
HardwareSerial toSlave(0);

// --- MISSION CONSTANTS ---
const int BOOT_BUTTON = 9;   
const int LED_BLUE = 8;
const int BUILT_IN_LED_SCL = 6;
const int BUILT_IN_LED_SDA = 5;     
const int SLAVE_TX = 2;
const int SLAVE_RX = 3;
const int RX_FROM_FC = 20;
const int TX_TO_FC = 21;
const int ARM_RC_CHANNEL = 14; 
const float LEAD_SENSITIVITY = 6.0;
const float ACCEL_1G = 512.0;
const float VOLT_THRESHOLD = 14.0;
const unsigned long BATT_DEBOUNCE = 2000;

// HUD Constructor (72x40 0.42" OLED)
U8G2_SSD1306_72X40_ER_F_HW_I2C u8g2_builtin(U8G2_R0, /* reset=*/ U8X8_PIN_NONE, BUILT_IN_LED_SCL, BUILT_IN_LED_SDA);

// --- SYSTEM VARIABLES ---
int manualContrast = 20; 
float currentContrast = 20.0; 
bool manualMode = false, insaneModeActive = false, txMax = false, lastBootState = HIGH, isLongPress = false;
unsigned long buttonDownTime = 0, lastRequest = 0, lastDataTime = 0, lowVoltTimer = 0, lastBroadcast = 0;

// Flight Data
float vBat = 16.0, currentG = 1.0, roll = 0, pitch = 0, lastPitch = 0, lastRoll = 0, heading = 0, altitude = 0, baro = 0, airSpeed = 0;
float filteredX = 0, filteredY = 0;
uint16_t armSwitchValue = 1000;
bool sessionHasMSP = false, currentlyReceiving = false, showLowBatText = false, isBenchMode = true;

// --- HARDWARE BOOST ---
void applyHardwareBoost(bool enable) {
  if (enable) {
    u8g2_builtin.sendF("c", 0xD9); 
    u8g2_builtin.sendF("c", 0xF1);
    u8g2_builtin.sendF("c", 0xDB); 
    u8g2_builtin.sendF("c", 0x40);
    u8g2_builtin.sendF("c", 0x8D); 
    u8g2_builtin.sendF("c", 0x14);
    insaneModeActive = true;
  } else {
    u8g2_builtin.sendF("c", 0xD9); 
    u8g2_builtin.sendF("c", 0x22);
    u8g2_builtin.sendF("c", 0xDB); 
    u8g2_builtin.sendF("c", 0x20);
    u8g2_builtin.sendF("c", 0x8D); 
    u8g2_builtin.sendF("c", 0x14);
    insaneModeActive = false;
  }
}

void sendMSPRequest(uint8_t cmd) { 
  uint8_t req[] = {'$', 'M', '<', 0, cmd, cmd}; 
  toAndFromFC.write(req, 6); 
}

void readMSPResponse() {
  while (toAndFromFC.available() >= 6) { 
    if (toAndFromFC.peek() != '$') { 
      toAndFromFC.read(); 
      continue; 
    }

    if (toAndFromFC.read() == '$' && toAndFromFC.read() == 'M' && toAndFromFC.read() == '>') {
      uint8_t size = toAndFromFC.read(); 
      uint8_t cmd = toAndFromFC.read();
      sessionHasMSP = true; 
      lastDataTime = millis();

      // --- THE CORE FIX: CAPTURE ENTIRE PAYLOAD FIRST ---
      uint8_t p [size]; 
      for (int i = 0; i < size; i++) {
        unsigned long startWait = millis();
        while (toAndFromFC.available() == 0 && millis() - startWait < 15); 
        p [i] = toAndFromFC.read();
      }
      
      // Consume the 1-byte checksum that always follows the payload
      while (toAndFromFC.available() == 0); 
      toAndFromFC.read(); 

      // --- NOW EXTRACT DATA FROM THE CAPTURED BYTES ---
      if (cmd == 108 && size >= 6) { // ATTITUDE
        int16_t angX = p [0] | (p [1] << 8); 
        int16_t angY = p [2] | (p [3] << 8); 
        int16_t head = p [4] | (p [5] << 8); 
        roll = angX / 10.0; 
        pitch = angY / 10.0; 
        heading = (float)head;
      } 
      else if (cmd == 109 && size >= 4) { // ALTITUDE
        // for(int i=0; i<4; i++) {
        //   console.print(p [i]); console.print(" ");
        // }
        int32_t altCm = (int32_t)p [0] | ((int32_t)p [1] << 8) | ((int32_t)p [2] << 16) | ((int32_t)p [3] << 24);
        altitude = (float)altCm * 0.0328084; 
        baro = 29.92;
        // Speed is in cm/s, converting to MPH
        int16_t speedCmS = p [4] | (p [5] << 8);
        airSpeed = (float)speedCmS * 0.0223694;
      }
      else if (cmd == 102 && size >= 6) { // RAW IMU
        int16_t az = p [4] | (p [5] << 8); 
        currentG = (((float)az / ACCEL_1G) * 0.1) + (currentG * 0.9);
      } 
      else if (cmd == 110 && size >= 1) { // VOLTS
        vBat = ((p [0] / 10.0) * 0.2) + (vBat * 0.8);
      } 
      else if (cmd == 105) { // RC CHANNELS
        int armIdx = (ARM_RC_CHANNEL - 1) * 2; // Channel 13 = Index 24
        // Only process if the packet is actually long enough to contain our channel
        if (size >= (armIdx + 2)) {
          armSwitchValue = p [armIdx] | (p [armIdx + 1] << 8);
        }
      } 
    }
  }
}
void setup() {
  // Disable internal system logging to prevent UART chatter
  esp_log_level_set("*", ESP_LOG_NONE);

  // LINK TO COMPUTER (USB Debugging / Console)
  // Ensure "USB CDC On Boot" is ENABLED in the Arduino IDE Tools menu
  console.begin(115200); 

  // LINK TO FLIGHT CONTROLLER (Pins 20/21)
  // We start this early so it has the best chance of a clean sync
  toAndFromFC.begin(115200, SERIAL_8N1, RX_FROM_FC, TX_TO_FC); 

  // HARDWARE INITIALIZATION
  // Setting pinModes immediately prevents "Load Access Fault" crashes 
  pinMode(BOOT_BUTTON, INPUT_PULLUP);
  pinMode(LED_BLUE, OUTPUT);
  digitalWrite(LED_BLUE, HIGH); 
  
  // PERIPHERALS
  u8g2_builtin.begin();
  u8g2_builtin.setContrast(20);
  u8g2_builtin.setFont(u8g2_font_6x10_tf);
  
  // Hardware boost is false by default to ensure stability during boot
  applyHardwareBoost(false);

  console.println("MASTER ONLINE - CHECKING FC...");
  
  unsigned long bootCheckStart = millis();
  unsigned long maxWaitTime = 20000; 
  
  // THE BOOT SCAN LOOP
  // yield() and delay() keep the Watchdog Timer (WDT) from triggering a crash
  while (millis() - bootCheckStart < maxWaitTime) {
    yield(); 

    // Update Display Status
    u8g2_builtin.clearBuffer();
    u8g2_builtin.drawStr(0, 10, "WAITING FOR FC...");
    u8g2_builtin.setCursor(0, 25);
    u8g2_builtin.print("TIMEOUT: ");
    u8g2_builtin.print((maxWaitTime - (millis() - bootCheckStart)) / 1000);
    u8g2_builtin.print("s");
    u8g2_builtin.sendBuffer();

    // Clear buffer of any noise or partial packets
    while(toAndFromFC.available()) toAndFromFC.read(); 

    // Send MSP Request
    sendMSPRequest(108);
    
    // Give the FC 200ms to process and reply
    delay(200); 
    
    readMSPResponse();

    // If we get a valid MSP response, we have an FC!
    if (sessionHasMSP) {
      break;
    }

    // Manual Override: Pressing the boot button forces Bench Mode
    if (digitalRead(BOOT_BUTTON) == LOW) {
      break;
    }
  }

  // --- MODE ASSIGNMENT AND FINAL PORT INIT ---
  if (sessionHasMSP) {
    isBenchMode = false;
    console.println("MODE: FLIGHT (MSP DETECTED)");
  } else {
    isBenchMode = true;
    console.println("MODE: BENCH (FC TIMEOUT)");
  }

  // 1. START SLAVE PORT (Always active for both modes)
  // We do this here to ensure Bench Mode actually sends data to the cockpit
  toSlave.begin(115200, SERIAL_8N1, SLAVE_RX, SLAVE_TX);

  // 2. FORCE UART0 (toSlave) to lock to Slave Pins
  // This prevents UART0 from accidentally floating or conflicting with FC pins
  uart_set_pin(UART_NUM_0, SLAVE_TX, SLAVE_RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

  // 3. IF IN FLIGHT MODE: Apply the Hardware Pin Lock for the FC
  if (!isBenchMode) {
    // Force UART1 to claim the FC pins (20/21)
    uart_set_pin(UART_NUM_1, TX_TO_FC, RX_FROM_FC, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    console.println("FLIGHT HARDWARE PIN LOCK APPLIED");
  } else {
    console.println("BENCH MODE: SLAVE ACTIVE, FC UART BYPASSED");
  }

  // Visual confirmation of boot completion
  digitalWrite(LED_BLUE, LOW); 
}

void loop() {
  // INPUT HANDLING
  bool currentBootState = digitalRead(BOOT_BUTTON);
  
  if (currentBootState == LOW && lastBootState == HIGH) { 
    buttonDownTime = millis(); 
    isLongPress = false; 
  }
  else if (currentBootState == HIGH && lastBootState == LOW && !isLongPress) {
    if (manualMode) { 
      manualContrast = (manualContrast >= 255) ? 20 : manualContrast + 40; 
    } else { 
      txMax = !txMax; 
    }
  } 
  else if (currentBootState == LOW && (millis() - buttonDownTime > 1000) && !isLongPress) { 
    manualMode = !manualMode; 
    isLongPress = true; 
  }
  lastBootState = currentBootState;

  // DATA ACQUISITION (LOCKED MODE)
  if (isBenchMode) {
    float t = millis() / 1000.0;
    
    // Physics Sweeps
    pitch = sin(t * 0.5) * 20.0; 
    roll = cos(t * 0.8) * 45.0; 
    
    // Altimeter: Swings from 500ft to 7500ft
    altitude = 4000 + (sin(t * 0.2) * 3500); 

    // Airspeed: Swings from 50mph to 700mph
    // We use a slower sine wave so you can study the non-linear compression
    airSpeed = 375 + (sin(t * 0.15) * 325); 

    heading += 0.5; 
    if (heading >= 360) heading = 0;
    
    vBat = 16.2;
    currentG = 1.0;
    currentlyReceiving = false; 
  }
  else {
    // STAGGERED REQUESTS (Prevents Buffer Collisions)
    static int mspStep = 0;
    if (millis() - lastRequest > 25) { // Request ONE item every 25ms
      mspStep++;
      if (mspStep == 1) sendMSPRequest(108); // ATTITUDE
      if (mspStep == 2) sendMSPRequest(109); // ALTITUDE
      if (mspStep == 3) sendMSPRequest(105); // RC CHANNELS (The "WAR" bit)
      if (mspStep == 4) sendMSPRequest(102); // RAW IMU
      if (mspStep == 5) {
        sendMSPRequest(110); // VOLTS
        mspStep = 0;         // Reset cycle
      }
      lastRequest = millis(); 
    }
    readMSPResponse();
    
    // Tighten the timeout: If no data for 250ms, mark as disconnected
    currentlyReceiving = (millis() - lastDataTime < 250);
  }

  // SHARED PHYSICS
  bool isArmed = (armSwitchValue > 1800);
  bool warActive = isArmed || txMax;
  
  float pDelta = (pitch - lastPitch); 
  float rDelta = (roll - lastRoll);
  
  filteredX = ((rDelta * -LEAD_SENSITIVITY) * 0.12) + (filteredX * 0.88);
  filteredY = ((pDelta * LEAD_SENSITIVITY) * 0.12) + (filteredY * 0.88);
  
  lastPitch = pitch; 
  lastRoll = roll;

  // POWER & CONTRAST
  if (isArmed && vBat < VOLT_THRESHOLD) {
    if (lowVoltTimer == 0) {
      lowVoltTimer = millis();
    }
    if (millis() - lowVoltTimer > BATT_DEBOUNCE) {
      showLowBatText = true;
    }
  } else { 
    lowVoltTimer = 0; 
    showLowBatText = false; 
  }

  int targetContrast = manualMode ? manualContrast : (warActive ? 255 : 20);
  
  if (abs(currentContrast - targetContrast) > 1) {
    float step = (targetContrast > currentContrast) ? 0.10 : 0.25;
    currentContrast += (targetContrast - currentContrast) * step; 
    u8g2_builtin.setContrast((int)currentContrast);
    
    if (currentContrast > 210 && !insaneModeActive) {
      applyHardwareBoost(true);
    }
    if (currentContrast < 90 && insaneModeActive) {
      applyHardwareBoost(false);
    }
  }

  // THE 3-CHAR TAGGED BROADCAST
  if (millis() - lastBroadcast > 50) {
    toSlave.print("ROL:"); 
    toSlave.print(roll); 
    toSlave.print(",");
    
    toSlave.print("PIT:"); 
    toSlave.print(pitch); 
    toSlave.print(",");
    
    toSlave.print("HED:"); 
    toSlave.print(heading); 
    toSlave.print(",");
    
    toSlave.print("ALT:"); 
    toSlave.print(altitude);
    toSlave.print(",");

    toSlave.print("SPD:"); 
    toSlave.print(airSpeed);
    toSlave.print(",");
    
    toSlave.print("BAT:"); 
    toSlave.print(vBat); 
    toSlave.print(",");
    
    toSlave.print("GFO:"); 
    toSlave.print(currentG);
    toSlave.print(",");

    toSlave.print("BAR:"); 
    toSlave.print(baro, 1);
    toSlave.print(",");
    
    toSlave.print("WAR:"); 
    toSlave.println(warActive ? 1 : 0);
    
    lastBroadcast = millis();
  }

  // HUD RENDERING
  float distSq = (filteredX * filteredX) + (filteredY * filteredY);
  bool blink = (millis() % 200 < 100);
  bool ledOn = (distSq < 1.5) || (showLowBatText && blink);
  digitalWrite(LED_BLUE, ledOn ? LOW : HIGH);

  u8g2_builtin.firstPage();
  do {
    u8g2_builtin.setFont(u8g2_font_04b_03_tr);
    
    const char* sym = "-";
    if (!isBenchMode) {
      if (currentlyReceiving) {
        sym = warActive ? "*" : "+";
      } else {
        sym = "X";
      }
    }
    
    u8g2_builtin.drawStr(2, 5, sym); 
    u8g2_builtin.drawStr(66, 5, sym);
    u8g2_builtin.drawStr(2, 39, sym); 
    u8g2_builtin.drawStr(66, 39, sym);

    u8g2_builtin.drawCircle(36, 20, 17); 
    u8g2_builtin.drawCircle(36, 20, 18);
    
    int curX = 36 + (int)filteredX; 
    int curY = 20 + (int)filteredY;
    
    u8g2_builtin.drawBox(curX - 1, curY - 1, 3, 3);
    
    for (int i = 0; i < 6; i++) {
      float a = i * 1.047;
      int bx = curX + (cos(a) * 12) - 1;
      int by = curY + (sin(a) * 12) - 1;
      u8g2_builtin.drawBox(bx, by, 2, 2);
    }
    
    u8g2_builtin.drawLine(0, 10, 0, 30);
    int gY = constrain(20 - (int)((currentG - 1.0) * 8.0), 10, 30);
    u8g2_builtin.drawHLine(0, gY, 3);

    if (showLowBatText) {
      char vBuf[8]; 
      dtostrf(vBat, 4, 1, vBuf), u8g2_builtin.drawStr(25, 39, "LOW "); 
      u8g2_builtin.drawStr(45, 39, vBuf);
    }
    if (manualMode) {
      u8g2_builtin.drawStr(28, 5, "MAN");
    }
  } while (u8g2_builtin.nextPage());
}