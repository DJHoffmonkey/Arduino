#include <TFT_eSPI.h>
#include <U8g2lib.h>
#include <Wire.h>

// --- P-51 VINTAGE PALETTE ---
#define P51_CHARCOAL 0x18C3
#define P51_RADIUM   0xEF55
#define P51_DIRTY_W  0xDEDB
#define P51_SKY      0x641D 
#define P51_EARTH    0x4221 

// --- MSP COMMAND IDS ---
#define MSP_RC 105
#define MSP_RAW_GPS 106
#define MSP_ATTITUDE 108
#define MSP_ALTITUDE 109
#define MSP_ANALOG   110
#define MSP_AIRSPEED 118
#define MSP_NAV_STATUS 121

// --- HARDWARE CONFIG ---
#define console Serial0      
#define toAndFromFC Serial1  
#define RX_FROM_FC 18
#define TX_TO_FC 17
#define CH_FLAPS 7
#define CH_GEAR 5
#define CH_ARM 14

#define isDebug true

TaskHandle_t OLEDTask;
float sharedHeading = 0; // We use this to pass data between cores

TFT_eSPI tft = TFT_eSPI();
TFT_eSprite canvas = TFT_eSprite(&tft);
TFT_eSprite ucSprite = TFT_eSprite(&tft);
// Initialize OLED (SSD1306 128x64)
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);

// --- SHARED DATA ---
bool isBenchMode = true, gearDown = true, isArmed = false;
float roll = 0, pitch = 0, alt = 0, airSpeed = 0, vsi = 0, heading = 0, vBat = 0, lastAltLog = -999.0, lastVBatLog = -999.0;
unsigned long lastRequestTime = 0, lastDataTime = 0, lastLogTime = 0;
uint8_t flapPos = 0; // 0=Clean, 1=Takeoff, 2=Landing
uint16_t rcChannels[14]; // To safely cover your 14 channels
uint16_t distToHome = 0;
int16_t  dirToHome = 0; // Absolute bearing from FC to Home
bool lastGearDown = false, gearInTransit = false;
uint32_t gearTimer = 0;
const uint32_t GEAR_CYCLE_TIME = 5000;

void setup() {
  console.begin(115200);
  while (!console);
  delay(500);
  console.println("--- P-51 COCKPIT: BARE METAL START ---");

  tft.init();
  tft.setRotation(1);
  tft.fillScreen(TFT_BLACK);
  canvas.createSprite(80, 80); 
  canvas.setTextDatum(MC_DATUM);
  ucSprite.createSprite(70, 22); 
  ucSprite.setTextDatum(MC_DATUM);  
  toAndFromFC.begin(115200, SERIAL_8N1, RX_FROM_FC, TX_TO_FC);


  // Initialize I2C and OLED
  Wire.begin(8, 9); // SDA on 8, SCL on 9 for ESP32-S3
  Wire.setClock(800000); // Set I2C to 400kHz
  u8g2.begin();
  
  console.println("--- OLED MAG COMPASS ONLINE ---");

  // Handshake
  unsigned long startScan = millis();
  while (millis() - startScan < 5000) {
    sendMSPRequest(MSP_ATTITUDE);
    delay(100); 
    if (parseMSP()) {
      isBenchMode = false;
      console.println("!!! FC CONNECTED !!!");
      break;
    }
  }

  // Start the OLED task on Core 0 (Main loop runs on Core 1)
  xTaskCreatePinnedToCore(
      oledTaskCode,   /* Function to implement the task */
      "OLEDTask",     /* Name of the task */
      10000,          /* Stack size in words */
      NULL,           /* Task input parameter */
      1,              /* Priority of the task */
      &OLEDTask,      /* Task handle */
      0);             /* Core where the task should run */
}


void setupOLED() {
  Wire.begin(8, 9); // SDA, SCL
  u8g2.begin();
}

void drawOLED(float heading) {
  static uint32_t lastOLED = 0;
  // 500ms (2Hz) is the "Sweet Spot" to minimize TFT flickering
  if (millis() - lastOLED < 500) return; 
  lastOLED = millis();

  u8g2.clearBuffer();
  
  // --- 1. MAGNETIC COMPASS ---
  u8g2.setFont(u8g2_font_7x14_tf); 
  u8g2.drawStr(0, 12, "MAG. COMPASS");
  
  // Cast float to int for the display
  int iHeading = (int)heading % 360;
  if (iHeading < 0) iHeading += 360; // Standardize 0-359
  
  char hBuf[4];
  sprintf(hBuf, "%03d", iHeading);
  
  u8g2.setFont(u8g2_font_logisoso22_tn); // Nice big military digits
  u8g2.drawStr(0, 42, hBuf);
  u8g2.drawTriangle(18, 48, 23, 58, 13, 58); // Pointer arrow

  // --- 2. MISSION CLOCK (Mins:Secs) ---
  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.drawStr(80, 12, "MISSION");

  uint32_t totalSecs = millis() / 1000;
  uint32_t m = totalSecs / 60;
  uint32_t s = totalSecs % 60;
  
  char tBuf[8];
  sprintf(tBuf, "%02d:%02d", m, s); // Minutes:Seconds
  
  u8g2.setFont(u8g2_font_9x15_tf);
  u8g2.drawStr(80, 32, tBuf);

  u8g2.sendBuffer(); // This is the heavy lifting command
}

// --- CORE MSP LOGIC ---

void sendMSPRequest(uint8_t cmd) {
  uint8_t request[] = {0x24, 0x4D, 0x3C, 0x00, cmd, cmd};
  toAndFromFC.write(request, 6);
}

bool parseMSP() {
  if (toAndFromFC.available() < 6) return false;
  if (toAndFromFC.peek() != '$') { toAndFromFC.read(); return false; }

  uint8_t head[3];
  toAndFromFC.readBytes(head, 3);
  if (head[1] != 'M' || head[2] != '>') return false;

  uint8_t size = toAndFromFC.read();
  uint8_t cmd  = toAndFromFC.read();
  
  // CRITICAL: Buffer must be large enough for 14+ channels (28+ bytes)
  uint8_t payload[64]; 
  toAndFromFC.readBytes(payload, size);
  uint8_t crc = toAndFromFC.read(); 

  lastDataTime = millis();
  isBenchMode = false;

  // --- RC DATA (GEAR, FLAPS, ARMING) ---
  if (cmd == 105) { // MSP_RC
    // 1. ARMING (Channel 14)
    int armIdx = (CH_ARM - 1) * 2; // Byte Index 26
    if (size >= (armIdx + 2)) {
      uint16_t aRaw = payload[armIdx] | (payload[armIdx + 1] << 8);
      bool newArmedState = (aRaw > 1500); 
      
      // Debug output only on state change
      if (isDebug && (newArmedState != isArmed)) {
        console.printf("DEBUG -> SYSTEM: %s (Raw PWM: %d)\n", 
                      newArmedState ? "!!! ARMED !!!" : "DISARMED", aRaw);
      }
      isArmed = newArmedState;
    }
    // 2. GEAR (Channel 6)
    int gearIdx = (CH_GEAR - 1) * 2;
    if (size >= (gearIdx + 2)) {
      uint16_t newGearVal = payload[gearIdx] | (payload[gearIdx + 1] << 8);
      bool newGearDown = (newGearVal > 1500); // Usually <1500 is "Down" for gear
      if (isDebug && (newGearDown != gearDown)) {
        console.printf("DEBUG -> GEAR: %s (%d)\n", newGearDown ? "DOWN" : "UP", newGearVal);
      }
      gearDown = newGearDown;
    }

    // 3. FLAPS (Channel 7 - 3 Position)
    int flapIdx = (CH_FLAPS - 1) * 2;
    if (size >= (flapIdx + 2)) {
      uint16_t fRaw = payload[flapIdx] | (payload[flapIdx + 1] << 8);
      uint8_t newFlapPos;
      
      if (fRaw < 1300)      newFlapPos = 0; // CLEAN
      else if (fRaw < 1700) newFlapPos = 1; // TAKEOFF
      else                  newFlapPos = 2; // LANDING

      if (isDebug && (newFlapPos != flapPos)) {
        const char* states[] = {"CLEAN", "TAKEOFF", "LANDING"};
        console.printf("DEBUG -> FLAPS: %s (Raw PWM: %d)\n", states[newFlapPos], fRaw);
      }
      flapPos = newFlapPos;    
    }
    
  }
  // --- ATTITUDE ---
  else if (cmd == MSP_ATTITUDE && size >= 6) {
    roll  = (int16_t)(payload[0] | (payload[1] << 8)) / 10.0;
    pitch = (int16_t)(payload[2] | (payload[3] << 8)) / 10.0;
    heading = (int16_t)(payload[4] | (payload[5] << 8));
  } 

  // --- ALTITUDE (V2 10-BYTE) ---
  else if (cmd == MSP_ALTITUDE && size >= 10) {
    int32_t estAlt  = (int32_t)(payload[0] | (payload[1] << 8) | (payload[2] << 16) | (payload[3] << 24));
    vsi             = (int16_t)(payload[4] | (payload[5] << 8));
    int32_t baroAlt = (int32_t)(payload[6] | (payload[7] << 8) | (payload[8] << 16) | (payload[9] << 24));
    alt = (estAlt == 0) ? (baroAlt / 30.48) : (estAlt / 30.48);

    if (isDebug && abs(alt - lastAltLog) > 0.1) { // 0.1ft threshold
       console.printf("DEBUG -> ALT: %.2f ft | VSI: %d\n", alt, (int)vsi);
       lastAltLog = alt;
    }
  }

  // --- BATTERY ---
  else if (cmd == MSP_ANALOG && size >= 1) {
    vBat = payload[0] / 10.0;
    if (isDebug && abs(vBat - lastVBatLog) > 0.05) {
       console.printf("DEBUG -> BATTERY: %.2fV\n", vBat);
       lastVBatLog = vBat;
    }
  }
  else if (cmd == 106) { // MSP_RAW_GPS
      if (size >= 16) {
          uint8_t fixType = payload[0];
          uint8_t numSats = payload[1];
          
          // 3. Calculate Speed (Byte 12-13 as confirmed)
          uint16_t groundSpeedCMS = (uint16_t)(payload[12] | (payload[13] << 8));
          airSpeed = groundSpeedCMS * 0.0223694;
          if (airSpeed < 1.5) airSpeed = 0;

      }
  }
  else if (cmd == MSP_NAV_STATUS) { 
      if (size >= 6) {
        // iNAV V2 Map: [0]GPS_mode, [1]Nav_state, [2-3]Dist, [4-5]Direction
        // We trust the FC to handle when and where 'Home' is defined.
        distToHome = (uint16_t)(payload[3] | (payload[4] << 8));
        dirToHome  = (int16_t)(payload[5] | (payload[6] << 8));

        if (isDebug) {
          static uint16_t lastDebugDist = 0;
          static int16_t  lastDebugDir  = 0;

          if (abs((int)distToHome - (int)lastDebugDist) > 1 || abs((int)dirToHome - (int)lastDebugDir) > 1) {
            console.printf("DEBUG -> FC HOME: %dm | BRG: %d deg\n", distToHome, dirToHome);
            lastDebugDist = distToHome;
            lastDebugDir = dirToHome;
          }

        }
      }
  }

  // --- AIRSPEED ---
  // else if (cmd == MSP_AIRSPEED && size >= 2) {
  //   float prevSpeed = airSpeed;
  //   int16_t rawSpeed = (int16_t)(payload[0] | (payload[1] << 8)); 
  //   airSpeed = rawSpeed * 0.02237; 
  //   if (isDebug && abs(airSpeed - prevSpeed) > 0.5) {
  //      console.printf("DEBUG -> AIRSPEED: %.1f MPH\n", airSpeed);
  //   }
  // }

  return true;
}

void updateMSP() {
  static int cycle = 0;
  
  // 1. SEND REQUESTS (Every 40ms to keep it snappy)
  if (millis() - lastRequestTime > 10) {
    cycle = (cycle + 1) % 10; // Updated to 5 to cover all cases
    
    switch(cycle) {
      case 0: case 2: case 4: case 6: case 8: sendMSPRequest(MSP_ATTITUDE); break;
      case 1: sendMSPRequest(MSP_ALTITUDE); break;
      case 3: sendMSPRequest(MSP_ANALOG); break;
      case 5: sendMSPRequest(MSP_RC); break; 
      case 7: sendMSPRequest(MSP_RAW_GPS); break;
      case 9: sendMSPRequest(MSP_NAV_STATUS); break;
    }
    lastRequestTime = millis();
  }

  // 2. PROCESS INCOMING DATA (The "Missing" Engine)
  // This loop drains the Serial buffer as long as there's a header (6 bytes min)
  while (toAndFromFC.available() >= 6) {
    parseMSP();
  }
}

float getAirspeedAngle(float mph) {
  float deg = -90; // Start at 12 o'clock
  
  if (mph <= 10.0) {
    // 0 to 10 mph = 75 degrees of sweep
    deg += (mph / 10.0) * 75.0;
  } else if (mph <= 30.0) {
    // 10 to 30 mph = 105 degrees of sweep (3 o'clock is 90, 6 o'clock is 180)
    // This lands '30' exactly at 6 o'clock (180 deg from start)
    deg += 75.0 + ((mph - 10.0) / 20.0) * 105.0;
  } else {
    // 30 to 100 mph = Remaining 180 degrees (6 o'clock back to 12)
    deg += 180.0 + ((mph - 30.0) / 70.0) * 180.0;
  }
  return deg;
}

void drawAirspeed(int x, int y, float speed) {
  canvas.fillSprite(P51_CHARCOAL);
  int cx = 40, cy = 40;
  
  canvas.fillSmoothCircle(cx, cy, 33, P51_EARTH); 
  canvas.fillSmoothCircle(cx, cy, 24, 0x18C3); 

  canvas.setTextColor(P51_RADIUM);
  
  // Custom numbers as requested
  int labels[] = {10, 20, 30, 50, 70, 90};
  int labelCount = 6;

  for (int i = 0; i <= 100; i += 2) {
    float angleDeg = getAirspeedAngle((float)i);
    float rad = angleDeg * (PI / 180.0);
    
    bool isLabel = false;
    for(int k=0; k<labelCount; k++) { if(i == labels[k]) isLabel = true; }

    if (isLabel) {
      int tx = cx + 24 * cos(rad); // Nudged in 1px to avoid bezel
      int ty = cy + 24 * sin(rad);
      canvas.drawCentreString(String(i), tx, ty - 4, 2);
      canvas.drawLine(cx + 28 * cos(rad), cy + 28 * sin(rad), cx + 33 * cos(rad), cy + 33 * sin(rad), P51_RADIUM);
    } else if (i % 10 == 0 || i == 0) {
      canvas.drawLine(cx + 29 * cos(rad), cy + 29 * sin(rad), cx + 33 * cos(rad), cy + 33 * sin(rad), P51_RADIUM);
    } else {
      canvas.drawLine(cx + 31 * cos(rad), cy + 31 * sin(rad), cx + 33 * cos(rad), cy + 33 * sin(rad), P51_RADIUM);
    }
  }

  canvas.setTextColor(P51_RADIUM);
  canvas.drawCentreString("MPH", cx, cy - 12, 1); 

  float needleAngle = getAirspeedAngle(constrain(speed, 0, 100));
  float sRad = needleAngle * (PI / 180.0);
  
  // Shadow
  canvas.drawLine(cx+1, cy+1, cx+1 + 32*cos(sRad), cy+1 + 32*sin(sRad), 0x0841);
  // Needle
  canvas.drawWideLine(cx, cy, cx + 32*cos(sRad), cy + 32*sin(sRad), 2, TFT_WHITE, 0x18C3);

  canvas.fillCircle(cx, cy, 3, TFT_BLACK);
  canvas.pushSprite(x - 40, y - 40);
}

void drawHorizon(int x, int y, float rll, float ptch) {
  canvas.fillSprite(P51_SKY);
  int cx = 40, cy = 40, cr = 36;
  
  // 1. Moving World (Sky/Earth)
  float radRoll = rll * (PI / 180.0);
  float tanRoll = tan(radRoll);
  float pOffset = constrain(ptch * 1.2, -30, 30);

  for (int i = -40; i <= 40; i++) {
    float horizonY = cy - pOffset + (i * tanRoll);
    if (horizonY < 80) {
      canvas.drawLine(cx + i, constrain(horizonY, 0, 80), cx + i, 80, P51_EARTH);
    }
  }

  // 2. JUST THE TDC TRIANGLE (Fixed at top)
  // x-coords: cx-4, cx+4, cx | y-coords: 5, 5, 13
  canvas.fillTriangle(cx - 3, 3, cx + 3, 3, cx, 3+3+1, TFT_YELLOW);

  // 3. Yellow Bank Ticks
  for (int a = -60; a <= 60; a += 30) {
    if (a == 0) continue; 
    float tr = (a - 90) * (PI / 180.0);
    canvas.drawLine(cx + (cr-4)*cos(tr), cy + (cr-4)*sin(tr), 
                    cx + cr*cos(tr), cy + cr*sin(tr), TFT_YELLOW);
  }

  // 4. THE TRIDENT (Floating Pin & Wings)
  canvas.fillCircle(cx, cy - 1, 2, TFT_YELLOW); 
  canvas.fillRect(cx - 10 - 18, cy - 1.5, 18, 3, TFT_YELLOW); // Left
  canvas.fillRect(cx + 10, cy - 1.5, 18, 3, TFT_YELLOW);  // Right

  // 5. Text
  canvas.setTextColor(0x2104); 
  canvas.drawCentreString("AN 5736-1A", cx, 72, 1);

  canvas.pushSprite(x - 40, y - 40);
}

void drawAltimeter(int x, int y, float alt) {
  canvas.fillSprite(P51_CHARCOAL);
  int cx = 40, cy = 40;
  
  // 1. Background Layers
  canvas.fillSmoothCircle(cx, cy, 33, P51_EARTH); // Aged ring
  canvas.fillSmoothCircle(cx, cy, 22, 0x18C3);    // Sunken center

  // 2. Dial Markings
  canvas.setTextColor(P51_RADIUM);
  for (int i = 0; i < 10; i++) {
    float angle = (i * 36 - 90) * (PI / 180.0);
    
    // Numbers
    int tx = cx + 25 * cos(angle);
    int ty = cy + 25 * sin(angle);
    canvas.drawCentreString(String(i), tx, ty - 4, 2);
    
    // Major Ticks (100ft)
    canvas.drawLine(cx + 29 * cos(angle), cy + 29 * sin(angle), 
                    cx + 33 * cos(angle), cy + 33 * sin(angle), P51_RADIUM);
    
    // Minor Ticks (20ft) - FIXED MATH HERE
    for (int j = 1; j < 5; j++) {
      float subAngle = (i * 36 + j * 7.2 - 90) * (PI / 180.0);
      canvas.drawLine(cx + 31 * cos(subAngle), cy + 31 * sin(subAngle), 
                      cx + 33 * cos(subAngle), cy + 33 * sin(subAngle), P51_RADIUM);
    }
  }

  // 3. Kollsman Window (Tiny 3-digit)
  int kx = cx + 16, ky = cy - 6;
  canvas.fillRect(kx, ky, 14, 10, TFT_BLACK); 
  canvas.drawRect(kx, ky, 14, 10, 0x4228); 
  canvas.setTextColor(TFT_WHITE);
  canvas.drawCentreString("299", kx + 7, ky + 1, 1); 

  // 4. Labels
  canvas.setTextColor(P51_RADIUM);
  canvas.drawCentreString("ALT", cx, cy + 6, 1);
  canvas.setTextColor(0x2104); 
  canvas.drawCentreString("1000 FEET", cx, cy - 3, 1);

  // 5. Hand Logic (Using fmod to keep rotations clean)
  float deg100   = (fmod(alt, 1000.0) / 1000.0) * 360.0;
  float deg1000  = (fmod(alt, 10000.0) / 10000.0) * 360.0;
  float deg10000 = (fmod(alt, 100000.0) / 100000.0) * 360.0;

  // 6. Draw Hands (Ordering is vital)
  drawAltHand(cx, cy, deg10000, 31, 1, true);  // 10k Crow's Foot
  drawAltHand(cx, cy, deg1000, 20, 4, false);  // 1k Fat Hand
  drawAltHand(cx, cy, deg100, 34, 2, false);   // 100ft Long Hand

  canvas.fillCircle(cx, cy, 3, TFT_BLACK); 
  canvas.pushSprite(x - 40, y - 40);
}

// Ensure this helper function is exactly as written below:
void drawAltHand(int cx, int cy, float deg, int len, int width, bool is10k) {
  float rad = (deg - 90.0) * (PI / 180.0);
  int px = cx + len * cos(rad);
  int py = cy + len * sin(rad);
  
  if (is10k) {
    canvas.drawLine(cx, cy, px, py, TFT_WHITE);
    // Crow's foot triangle tip
    float x1 = cx + (len - 5) * cos(rad + 0.2);
    float y1 = cy + (len - 5) * sin(rad + 0.2);
    float x2 = cx + (len - 5) * cos(rad - 0.2);
    float y2 = cy + (len - 5) * sin(rad - 0.2);
    canvas.fillTriangle(px, py, (int)x1, (int)y1, (int)x2, (int)y2, TFT_WHITE);
  } else {
    canvas.drawWideLine(cx, cy, px, py, width, TFT_WHITE, 0x18C3);
  }
}

void drawTurn(int x, int y, float heading, float homeBearing) {
  canvas.fillSprite(P51_CHARCOAL); 
  int cx = 40, cy = 40;
  int winY = cy - 18; 
  int winH = 22;
  int winLeft = cx - 28;

  // 1. Internal Shadow
  canvas.fillRect(winLeft, winY, 56, winH, 0x0841); 

  // --- NEW: GREEN HOME BAR LOGIC ---
  // Calculate shortest distance to home bearing
  float homeDelta = homeBearing - heading;
  if (homeDelta > 180) homeDelta -= 360;
  if (homeDelta < -180) homeDelta += 360;

  // Map homeDelta to the drum curve
  float homeRad = homeDelta * (PI / 180.0);
  float homeX = cx + (sin(homeRad) * 45); 

  // Draw the green bar if it's within the window view
  if (homeX > winLeft && homeX < winLeft + 56) {
    // A bright "Radium" Green for the home marker
    canvas.fillRect((int)homeX - 2, winY + 1, 4, winH - 2, 0x07E0); 
  }
  // --- END HOME BAR ---

  // 2. The Drum Logic
  canvas.setTextColor(P51_RADIUM);
  float startAngle = floor(heading / 5.0) * 5.0;

  for (float a = startAngle - 40; a <= startAngle + 40; a += 5) {
    float delta = a - heading;
    float rad = delta * (PI / 180.0);
    float scrollX = cx + (sin(rad) * 45); 

    int displayAngle = ((int)(a + 3600) % 360);

    if (scrollX > winLeft - 5 && scrollX < winLeft + 61) {
      int tickLen = (displayAngle % 10 == 0) ? 7 : 4;
      canvas.drawFastVLine((int)scrollX, winY + winH - tickLen - 2, tickLen, P51_RADIUM);

      if (displayAngle % 30 == 0) {
        String label = String(displayAngle / 10);
        canvas.drawCentreString(label, (int)scrollX, winY + 2, 2);
      }
    }
  }

  // 3. Masking (Paint over the bleed)
  canvas.fillRect(0, 0, 80, winY, P51_CHARCOAL);             
  canvas.fillRect(0, winY + winH, 80, 80 - (winY+winH), P51_CHARCOAL); 
  canvas.fillRect(0, winY, winLeft, winH, P51_CHARCOAL);     
  canvas.fillRect(winLeft + 56, winY, 80 - (winLeft+56), winH, P51_CHARCOAL); 

  // 4. Stationary Elements
  canvas.drawRect(winLeft, winY, 56, winH, 0x4228); 
  // Center Lubber Line (White)
  canvas.fillRect(cx - 1, winY - 3, 2, winH + 6, TFT_WHITE); 

  canvas.setTextColor(0x4228);
  canvas.drawCentreString("DIREC.GYRO", cx, cy + 10, 1);
  canvas.drawCentreString("AN 5735-1A", cx, cy + 20, 1);
  canvas.pushSprite(x - 40, y - 40);
}

void drawBank(int x, int y, float rll) {
  canvas.fillSprite(P51_CHARCOAL);
  int cx = 40, cy = 40;
  
  // 1. The Background Housing (slightly aged/browned)
  canvas.fillSmoothCircle(cx, cy, 32, 0x4228); 

  // 2. The "Bomb" Top Reference (Nudged 1px Left)
  // Wider at the top, tapering down to the tube
  canvas.fillSmoothRoundRect(cx - 3, cy - 28, 5, 12, 2, P51_DIRTY_W); // Top body
  // 3. The Yellow "Wings" (The Static Blocks)
  // Note: These are slightly curved in the real plane
  canvas.fillRoundRect(cx - 28, cy + 2, 12, 10, 2, 0xCE60); 
  canvas.fillRoundRect(cx + 16, cy + 2, 12, 10, 2, 0xCE60);

  // 4. The Slip Ball
  float ballX = cx + constrain(rll * 0.5, -15, 15);
  
  // Ball Shadow & Body
  canvas.fillCircle(ballX, cy + 7, 6, TFT_BLACK);
  canvas.fillCircle(ballX, cy + 7, 5, 0x7BEF); 
  // Specular highlight to make it look like a steel ball in liquid
  canvas.fillCircle(ballX - 2, cy + 5, 2, TFT_WHITE);

  canvas.pushSprite(x - 40, y - 40);
}

void drawVSI(int x, int y, float vspd) {
  canvas.fillSprite(P51_CHARCOAL);
  int cx = 40, cy = 40;
  
  // 1. Dark Oil-Stained Background & Hub
  canvas.fillRect(0, 0, 80, 80, 0x2104); 
  canvas.fillSmoothCircle(cx, cy, 14, 0x0841); 

  canvas.setTextColor(P51_RADIUM);
  
  // 2. Expanded Scale Mapping
  // 0 is 180 deg (9 o'clock)
  // 1 is ~145 deg (roughly 10:30 / 7:30)
  // 2 is ~110 deg (roughly 11:30 / 6:30)
  // 4 is ~40 deg  (This puts it right at 2pm and 4pm)
  float scale[] = {1.0, 2.0, 4.0};
  float offsets[] = {35, 70, 140}; 
  
  for (int i = 0; i < 3; i++) {
    float rUp = (180 - offsets[i]) * (PI / 180.0);
    float rDn = (180 + offsets[i]) * (PI / 180.0);
    
    // Numbers: Tucked at 20px radius to stay clear of the bezel
    canvas.drawCentreString(String((int)scale[i]), cx + 20 * cos(rUp), cy + 20 * sin(rUp) - 4, 2);
    canvas.drawCentreString(String((int)scale[i]), cx + 20 * cos(rDn), cy + 20 * sin(rDn) - 4, 2);
    
    // Ticks: Reaching out to the edge
    canvas.drawLine(cx+28*cos(rUp), cy+28*sin(rUp), cx+40*cos(rUp), cy+40*sin(rUp), P51_RADIUM);
    canvas.drawLine(cx+28*cos(rDn), cy+28*sin(rDn), cx+40*cos(rDn), cy+40*sin(rDn), P51_RADIUM);
  }

  // 3. The Left-Side "Niche" Details
  // .5 markers and Brackets
  canvas.drawCentreString(".5", 8, cy - 20, 1);
  canvas.drawLine(2, cy - 8, 8, cy - 8, P51_RADIUM); 
  canvas.drawLine(8, cy - 12, 8, cy - 4, P51_RADIUM); 

  canvas.drawCentreString(".5", 8, cy + 12, 1);
  canvas.drawLine(2, cy + 8, 8, cy + 8, P51_RADIUM); 
  canvas.drawLine(8, cy + 4, 8, cy + 12, P51_RADIUM);

  // 4. The "Single 6" at 3 o'clock (0 degrees)
  canvas.drawCentreString("6", cx + 26, cy - 4, 2);
  canvas.drawLine(cx + 34, cy, cx + 40, cy, P51_RADIUM);

  // 5. Zero Marker (9 o'clock)
  canvas.drawLine(0, cy, 10, cy, P51_RADIUM);

  // 6. Calibrated Needle Logic
  float absV = abs(vspd);
  float move = 0;
  // Interpolation to match the new wide-sweep offsets
  if (absV <= 1.0)      move = absV * 35.0;
  else if (absV <= 2.0) move = 35.0 + (absV - 1.0) * 35.0;
  else if (absV <= 4.0) move = 70.0 + (absV - 2.0) * 35.0; // Linear 70 to 140
  else                  move = 140.0 + (constrain(absV, 4, 6) - 4.0) * 20.0; // Meets at 180 total (3 o'clock)

  float finalDeg = (vspd >= 0) ? (180.0 - move) : (180.0 + move);
  float nRad = finalDeg * (PI / 180.0);
  
  // Needle
  canvas.drawLine(cx+1, cy+1, cx+1 + 36*cos(nRad), cy+1 + 36*sin(nRad), 0x0841);
  canvas.drawWideLine(cx, cy, cx + 36*cos(nRad), cy + 36*sin(nRad), 2, TFT_WHITE, 0x0841);

  canvas.fillCircle(cx, cy, 3, TFT_BLACK);
  canvas.pushSprite(x - 40, y - 40);
}

void drawGearStatus(int screenX, int screenY) {
  ucSprite.fillSprite(P51_CHARCOAL); // Clear background

  // Local center for the 70x22 sprite
  int localCX = 34; 
  int localCY = 11; 
  int spacing = 20; 

  // Logic for the 7-second transit
  if (gearDown != lastGearDown) {
    gearInTransit = true;
    gearTimer = millis();
    lastGearDown = gearDown;
  }
  if (gearInTransit && (millis() - gearTimer > 7000)) {
    gearInTransit = false;
  }

  // RED LIGHT (UNSAFE/TRANSIT)
  if (gearInTransit) {
    ucSprite.fillCircle(localCX + spacing, localCY, 6, TFT_RED);
    ucSprite.drawCircle(localCX + spacing, localCY, 7, 0x8000); // Glow
  }

  // GREEN LIGHT (LOCKED DOWN)
  if (gearDown && !gearInTransit) {
    ucSprite.fillCircle(localCX - spacing, localCY, 6, TFT_GREEN);
    ucSprite.drawCircle(localCX - spacing, localCY, 7, 0x03E0); // Glow
  }

  // Finally, push the small gear box to the physical screen coordinates
  ucSprite.pushSprite(screenX, screenY);
}

void loop() {
  static unsigned long lastFrame = 0;
  
  if (isBenchMode) {
    float t = millis() / 1000.0;
    airSpeed = 45 + sin(t * 0.5) * 35;
    alt = 225.0 + (sin(t * 0.2) * 225.0);
    roll = sin(t) * 35;
    pitch = cos(t * 0.7) * 10;
    vsi = cos(t * 0.2) * 400.0; // Match cm/s scale
    heading += 0.2;
    if (heading >= 360) heading = 0;
    vBat = 12.2;
  } else {
    updateMSP();
  }

  sharedHeading = heading; // Update the value for the other core to see

  drawAirspeed(33, 48, airSpeed);
  drawTurn(140, 48, (float)heading, (float)dirToHome);
  drawHorizon(253, 54, roll, pitch);
  drawAltimeter(33, 144, alt);
  drawBank(146, 170, roll);
  drawVSI(253, 170, vsi / 100.0); // Convert cm/s to m/s for display
  drawGearStatus(140,220);
  yield(); // Let S3 background tasks (WiFi/BT stack) breathe
}

void oledTaskCode(void * pvParameters) {
  for(;;) {
    u8g2.clearBuffer();
    const int hY = 32; 
    const int xL = 22;
    const int rL = 24; // Stretched for 9mm
    const int xR = 80;
    const int rR = 19; // Stretched for 7mm

    // =============================================================
    // --- 1. REMOTE COMPASS (LEFT) ---
    // =============================================================
    u8g2.setFont(u8g2_font_4x6_tr);
    u8g2.drawStr(xL-2, hY-rL+7, "N");
    u8g2.drawStr(xL-2, hY+rL-2, "S");
    u8g2.drawStr(xL+rL-7, hY+2, "E");
    u8g2.drawStr(xL-rL+2, hY+2, "W");

    // Ticks every 30 degrees
    for (int a = 0; a < 360; a += 30) {
      float tR = (a - 90) * (PI/180);
      u8g2.drawLine(xL+(rL-1)*cos(tR), hY+(rL-1)*sin(tR), xL+(rL-4)*cos(tR), hY+(rL-4)*sin(tR));
    }

    // --- HEADING NEEDLE (Double-Bar Frame) ---
    float radH = (sharedHeading - 90.0) * (PI / 180.0);
    float cosH = cos(radH); float sinH = sin(radH);
    float cosOH = cos(radH + PI/2); float sinOH = sin(radH + PI/2);
    float wH = 1.8; // Gap width

    // Parallel Rails
    u8g2.drawLine(xL + wH*cosOH, hY + wH*sinOH, xL + (rL-2)*cosH + wH*cosOH, hY + (rL-2)*sinH + wH*sinOH);
    u8g2.drawLine(xL - wH*cosOH, hY - wH*sinOH, xL + (rL-2)*cosH - wH*cosOH, hY + (rL-2)*sinH - wH*sinOH);
    // Tip Closure
    u8g2.drawLine(xL + (rL-2)*cosH + wH*cosOH, hY + (rL-2)*sinH + wH*sinOH, xL + (rL-2)*cosH - wH*cosOH, hY + (rL-2)*sinH - wH*sinOH);
    // Counter-Tail
    u8g2.drawLine(xL, hY, xL - (rL-10)*cosH, hY - (rL-10)*sinH);

    // --- COURSE INDICATOR (Perpendicular T-Bar) ---
    // This is the "other needle" in your photo. 
    float radC = (dirToHome - 90.0) * (PI / 180.0);
    float cosC = cos(radC); float sinC = sin(radC);
    float cosOC = cos(radC + PI/2); float sinOC = sin(radC + PI/2);
    // Perpendicular cross-bar at the edge
    u8g2.drawLine(xL + (rL-3)*cosC + 6*cosOC, hY + (rL-3)*sinC + 6*sinOC,
                  xL + (rL-3)*cosC - 6*cosOC, hY + (rL-3)*sinC - 6*sinOC);

    // =============================================================
    // --- 2. MISSION CLOCK (RIGHT) ---
    // =============================================================
    u8g2.drawStr(xR-3, hY-rR+7, "12");
    u8g2.drawStr(xR-2, hY+rR-2, "6");
    u8g2.drawStr(xR+rR-6, hY+2, "3");
    u8g2.drawStr(xR-rR+1, hY+2, "9");

    uint32_t ts = millis() / 1000;
    float mRad = (((ts / 60) % 60) * 6 - 90) * (PI/180);
    float sRad = ((ts % 60) * 6 - 90) * (PI/180);

    // Minute Hand (Sword shape)
    u8g2.drawLine(xR, hY, xR + (rR-6)*cos(mRad), hY + (rR-6)*sin(mRad));
    u8g2.drawLine(xR, hY, xR + (rR-8)*cos(mRad+0.12), hY + (rR-8)*sin(mRad+0.12));
    u8g2.drawLine(xR, hY, xR + (rR-8)*cos(mRad-0.12), hY + (rR-8)*sin(mRad-0.12));

    // Second Hand (Thin sweep)
    u8g2.drawLine(xR, hY, xR + (rR-2)*cos(sRad), hY + (rR-2)*sin(sRad));

    u8g2.sendBuffer();
    
    // Speeding up the task to 15Hz (66ms) for fluid motion
    vTaskDelay(pdMS_TO_TICKS(66)); 
  }
}