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
#define isCOMPASS_DISPLAY_ACTIVE true
#define isENGINE_DISPLAY_ACTIVE true


SemaphoreHandle_t i2cMutex;
TaskHandle_t OLED_CompassAndClockTask;
float sharedHeading = 0; // We use this to pass data between cores

TFT_eSPI tft = TFT_eSPI();
TFT_eSprite canvas = TFT_eSprite(&tft);
TFT_eSprite ucSprite = TFT_eSprite(&tft);

// Compass display is wide (128x64)
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2_CompassAndClock(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);
// Engine display is tall (64x128)
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2_Engine(U8G2_R3, /* reset=*/ U8X8_PIN_NONE);

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

// --- GLOBAL TELEMETRY DATA ---
volatile uint32_t missionStartTime = 0, finalFlightTime = 0;
volatile float sharedVoltage = 16.8f; // Battery voltage from telemetry
volatile float sharedThrottle = 0.0f; // Current throttle % (0.0 to 1.0)
volatile float sharedBattery = 100.0f;

void setup() {
  console.begin(115200);
  while (!console);
  delay(500);
  if (isDebug) console.println("--- P-51 COCKPIT: BARE METAL START ---");

  tft.init();
  tft.setRotation(1);
  tft.fillScreen(TFT_BLACK);
  canvas.createSprite(80, 80); 
  canvas.setTextDatum(MC_DATUM);
  ucSprite.createSprite(70, 22); 
  ucSprite.setTextDatum(MC_DATUM);  
  toAndFromFC.begin(115200, SERIAL_8N1, RX_FROM_FC, TX_TO_FC);

  i2cMutex = xSemaphoreCreateMutex();

  // Handshake
  unsigned long startScan = millis();
  while (millis() - startScan < 5000) {
    sendMSPRequest(MSP_ATTITUDE);
    delay(100); 
    if (parseMSP()) {
      isBenchMode = false;
      if (isDebug) console.println("!!! FC CONNECTED !!!");
      break;
    }
  }

  if (isCOMPASS_DISPLAY_ACTIVE || isENGINE_DISPLAY_ACTIVE ) {
    Wire.begin(8, 9); 
    Wire.setClock(400000); // 400kHz is safer for SSD1306 than 800kHz to prevent flickering
  }

  if (isCOMPASS_DISPLAY_ACTIVE) setupOLED_CompassAndClock();
  if (isENGINE_DISPLAY_ACTIVE) setupOLED_Engine();

  xTaskCreatePinnedToCore(oled_MasterTask, "DualOLED", 8192, NULL, 1, NULL, 0);

}

void setupOLED_CompassAndClock() {
  u8g2_CompassAndClock.setI2CAddress(0x3C * 2);
  u8g2_CompassAndClock.begin();
  u8g2_CompassAndClock.setFlipMode(1); // Ensure landscape
  if (isDebug) console.println("--- OLED MAG COMPASS ONLINE ---");
}
void setupOLED_Engine() {
  u8g2_Engine.setI2CAddress(0x3D * 2);
  u8g2_Engine.begin();
  u8g2_Engine.setFlipMode(1); // Ensure landscape
  if (isDebug) console.println("--- OLED ENGINE ONLINE ---");
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
    vBat = 16.8; // Simulating a 4S pack for the Tacho math
    // Simulating throttle cycling from 0 to 100% every 12 seconds
    sharedThrottle = (sin(t * 0.5) * 0.5) + 0.5; 
    // Simulating fuel draining slowly over time, then resetting
    sharedBattery = 100.0 - (fmod(t, 60.0) * 1.66);
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

// --- HELPER: PREVENT COORDINATE WRAPPING GLITCH ---
void drawSafeLine(U8G2 &canvas, float x1, float y1, float x2, float y2) {
    canvas.drawLine((int)constrain(x1, 0, 127), (int)constrain(y1, 0, 63), 
                    (int)constrain(x2, 0, 127), (int)constrain(y2, 0, 63));
}

void drawCompassFrame(U8G2 &canvas, float centerX, float centerY, float r) {
    canvas.setFont(u8g2_font_5x7_tr);
    
    // Get font metrics for perfect centering
    int fHeight = canvas.getMaxCharHeight();
    int fAscent = canvas.getAscent();

    // 1. DYNAMIC LABELS (N, S, E, W)
    // We calculate half-width for every label to ensure true center-alignment
    
    // NORTH: Top-center
    int nW = canvas.getStrWidth("N");
    canvas.drawStr(centerX - (nW / 2), centerY - (r * 0.85f), "N");

    // SOUTH: Bottom-center (Using fHeight to offset from the bottom)
    int sW = canvas.getStrWidth("S");
    canvas.drawStr(centerX - (sW / 2), centerY + (r * 0.85f) + fAscent, "S");

    // EAST: Right-middle (Using fAscent/2 for vertical centering)
    int eW = canvas.getStrWidth("E");
    canvas.drawStr(centerX + (r * 0.85f) - (eW / 2), centerY + (fAscent / 2), "E");

    // WEST: Left-middle
    int wW = canvas.getStrWidth("W");
    canvas.drawStr(centerX - (r * 0.85f) - (wW / 2), centerY + (fAscent / 2), "W");

    // 2. CENTER HUB (Scaled to 10% of radius, min 3px)
    int hubSize = max(3, (int)(r * 0.1f));
    canvas.drawBox((int)centerX - (hubSize / 2), (int)centerY - (hubSize / 2), hubSize, hubSize);

    // 3. TICKS (Fully Parametric)
    for (int angle = 30; angle < 360; angle += 30) {
        if (angle % 90 == 0) continue; 
        float rad = (angle - 90.0f) * (M_PI / 180.0f);
        
        // Ticks are 15% of the radius
        float tLen = r * 0.15f;
        
        // Outer point to Inner point
        drawSafeLine(canvas, 
            centerX + r * cosf(rad), centerY + r * sinf(rad),
            centerX + (r - tLen) * cosf(rad), centerY + (r - tLen) * sinf(rad)
        );
    }
}

void drawHeadingNeedle(U8G2 &canvas, float centerX, float centerY, float r, float heading) {
    float rad = (heading - 90.0f) * (M_PI / 180.0f);
    float cosH = cosf(rad), sinH = sinf(rad);
    float gX = -sinH * 2.5f, gY = cosH * 2.5f; 

    // Arrow Tip at 75% of radius (stays inside N/E/S/W)
    float tipR = r * 0.75f;
    // Body Rails at 60% of radius
    float bodyR = r * 0.60f;

    for (int side = -1; side <= 1; side += 2) {
        float rX = gX * side, rY = gY * side;
        drawSafeLine(canvas, centerX - bodyR * cosH + rX, centerY - bodyR * sinH + rY, centerX + bodyR * cosH + rX, centerY + bodyR * sinH + rY);
    }

    float tipX = centerX + tipR * cosH, tipY = centerY + tipR * sinH;
    drawSafeLine(canvas, centerX + bodyR * cosH + gX, centerY + bodyR * sinH + gY, tipX, tipY);
    drawSafeLine(canvas, centerX + bodyR * cosH - gX, centerY + bodyR * sinH - gY, tipX, tipY);
    drawSafeLine(canvas, centerX - bodyR * cosH + gX, centerY - bodyR * sinH + gY, centerX - bodyR * cosH - gX, centerY - bodyR * sinH - gY);
}

// --- 2. HOME NEEDLE (PARAMETRIC & 2-PIXEL THICK) ---
void drawHomeNeedle(U8G2 &canvas, float centerX, float centerY, float r, float heading) {
    float rad = (heading - 90.0f) * (M_PI / 180.0f);
    float cosH = cosf(rad);
    float sinH = sinf(rad);
    float pX = -sinH; // Perpendicular vector for thickness
    float pY = cosH;

    // Tip: Set at 65% of dial radius to stay nested inside Heading needle
    float tipR = r * 0.65f;
    float tipX = centerX + tipR * cosH;
    float tipY = centerY + tipR * sinH;

    // 1. DRAW 2-PIXEL THICK BODY & TAIL
    // Tail length is 30% of dial radius
    float tailR = r * 0.30f;
    
    for (float i = -0.5f; i <= 0.5f; i += 1.0f) {
        float shiftX = i * pX;
        float shiftY = i * pY;
        
        // Body line
        drawSafeLine(canvas, centerX + shiftX, centerY + shiftY, tipX + shiftX, tipY + shiftY);
        // Tail line
        drawSafeLine(canvas, centerX + shiftX, centerY + shiftY, centerX - tailR * cosH + shiftX, centerY - tailR * sinH + shiftY);
    }

    // 2. DRAW THE T-HEAD CAP
    // Cap width scales with dial size (approx 15% of radius)
    float capWidth = r * 0.15f; 
    drawSafeLine(canvas, tipX + (pX * capWidth), tipY + (pY * capWidth), 
                         tipX - (pX * capWidth), tipY - (pY * capWidth));
}

void drawMissionClock(U8G2 &canvas, float centerX, float centerY, float r, uint32_t msTotal) {
    canvas.setFont(u8g2_font_5x7_tr);
    
    // 1. LABELS (10-minute dial)
    canvas.drawStr(centerX - 5, centerY - 10, "10"); 
    canvas.drawStr(centerX - 3, centerY + 16, "5");
    canvas.drawStr(centerX + 11, centerY + 3,  "2"); 
    canvas.drawStr(centerX - 18, centerY + 3,  "7");

    // 2. CLEAN DIAL DOTS (Skip positions 0, 90, 180, 270 degrees)
    for (int i = 0; i < 360; i += 36) { 
        if (i == 0 || i == 90 || i == 180 || i == 270) continue;
        float rad = (i - 90.0f) * (M_PI / 180.0f);
        canvas.drawPixel((int)(centerX + r * cosf(rad)), (int)(centerY + r * sinf(rad)));
    }

    // --- 3. MISSION MINUTES HAND (Thick/Bold - 10 Min Lap) ---
    float minMS = (float)(msTotal % 600000);
    float minRad = (minMS * 0.0006f - 90.0f) * (M_PI / 180.0f);
    float minTipX = centerX + (r * 0.65f) * cosf(minRad);
    float minTipY = centerY + (r * 0.65f) * sinf(minRad);
    
    float mPX = -sinf(minRad), mPY = cosf(minRad);
    for (float i = -0.5f; i <= 0.5f; i += 1.0f) {
        drawSafeLine(canvas, centerX + (mPX * i), centerY + (mPY * i), minTipX + (mPX * i), minTipY + (mPY * i));
    }

    // --- 4. BALANCED SECONDS NEEDLE (Thin/Long - 1 Min Lap) ---
    uint32_t smoothSecMs = (msTotal / 100) * 100; // 10Hz "High-Beat"
    float secMS = (float)(smoothSecMs % 60000);
    float secRad = (secMS * 0.006f - 90.0f) * (M_PI / 180.0f);
    
    // Tip (Forward)
    float secTipX = centerX + (r - 1.0f) * cosf(secRad);
    float secTipY = centerY + (r - 1.0f) * sinf(secRad);
    
    // Tail (Backward - approx 3-4 pixels)
    float secTailX = centerX - (4.0f * cosf(secRad));
    float secTailY = centerY - (4.0f * sinf(secRad));
    
    // Draw the full needle from Tail to Tip
    drawSafeLine(canvas, secTailX, secTailY, secTipX, secTipY);

    // 5. CENTER HUB
    canvas.drawBox((int)centerX - 1, (int)centerY - 1, 3, 3);
}


void drawTacho(U8G2 &canvas, float centerX, float centerY, float rpm, float maxRpm) {
    canvas.setFont(u8g2_font_u8glib_4_tf); 

    for (int i = 0; i <= 10; i++) {
        float angle = 225.0f - (i * 27.0f); 
        float rad = (angle - 90.0f) * (M_PI / 180.0f);
        
        int rOut = 23, rIn = (i % 2 == 0) ? 18 : 20; 
        canvas.drawLine((int)(centerX + rOut * cosf(rad)), (int)(centerY + rOut * sinf(rad)), 
                        (int)(centerX + rIn * cosf(rad)), (int)(centerY + rIn * sinf(rad)));

        // Radial Number Placement (10, 30, 50, 70, 90)
        if (i == 1 || i == 3 || i == 5 || i == 7 || i == 9) {
            char buf[3]; sprintf(buf, "%d", i * 10);
            int tx = (int)(centerX + 11 * cosf(rad));
            int ty = (int)(centerY + 11 * sinf(rad));
            canvas.drawStr(tx - 3, ty + 2, buf);
        }
    }

    // TAPERED NEEDLE
    float needleRad = (225.0f - ((rpm / 10000.0f) * 270.0f) - 90.0f) * (M_PI / 180.0f);
    int tipX = (int)(centerX + 17 * cosf(needleRad));
    int tipY = (int)(centerY + 17 * sinf(needleRad));
    
    canvas.drawLine((int)centerX, (int)centerY, tipX, tipY); 
    canvas.drawLine((int)(centerX + 1 * cosf(needleRad + 1.57f)), (int)(centerY + 1 * sinf(needleRad + 1.57f)), tipX, tipY);
    canvas.drawLine((int)(centerX + 1 * cosf(needleRad - 1.57f)), (int)(centerY + 1 * sinf(needleRad - 1.57f)), tipX, tipY);
    canvas.drawDisc((int)centerX, (int)centerY, 2);
}

void drawFuelGauge(U8G2 &canvas, float centerX, float centerY, float percent) {
    // 1. Ticks
    for (int i = 0; i <= 4; i++) {
        float angle = 210.0f - (i * 60.0f); 
        float rad = (angle - 90.0f) * (M_PI / 180.0f);
        canvas.drawLine((int)(centerX + 23 * cosf(rad)), (int)(centerY + 23 * sinf(rad)), 
                        (int)(centerX + 19 * cosf(rad)), (int)(centerY + 19 * sinf(rad)));
    }

    // 2. BIG RADIAL LABELS (E, 1/2, F)
    canvas.setFont(u8g2_font_5x7_tr); 
    const char* labels[] = {"E", "", "1/2", "", "F"};
    for (int i = 0; i <= 4; i++) {
        if (strlen(labels[i]) > 0) {
            float angle = 210.0f - (i * 60.0f);
            float rad = (angle - 90.0f) * (M_PI / 180.0f);
            int tx = (int)(centerX + 10 * cosf(rad)); // Radius 10 to keep big font inside
            int ty = (int)(centerY + 10 * sinf(rad));
            int xOff = (i == 2) ? 7 : 3; 
            canvas.drawStr(tx - xOff, ty + 3, labels[i]);
        }
    }

    // 3. TAPERED NEEDLE (Matching Tacho)
    float fuelRad = (210.0f - ((percent / 100.0f) * 240.0f) - 90.0f) * (M_PI / 180.0f);
    int fTipX = (int)(centerX + 17 * cosf(fuelRad));
    int fTipY = (int)(centerY + 17 * sinf(fuelRad));
    
    canvas.drawLine((int)centerX, (int)centerY, fTipX, fTipY); 
    canvas.drawLine((int)(centerX + 1 * cosf(fuelRad + 1.57f)), (int)(centerY + 1 * sinf(fuelRad + 1.57f)), fTipX, fTipY);
    canvas.drawLine((int)(centerX + 1 * cosf(fuelRad - 1.57f)), (int)(centerY + 1 * sinf(fuelRad - 1.57f)), fTipX, fTipY);
    canvas.drawDisc((int)centerX, (int)centerY, 2);
}

void updateCompassAndClockDisplay() {
  static float visualHeading = 0.0f;
  static float visualHome = 0.0f;
  const float smoothingAlpha = 0.15f; 
  
  // REMOVED: static uint32_t missionStartTime = 0; (Using global instead)
  static bool wasArmed = false; 

  // --- 1. SMOOTHING LOGIC ---
  float deltaHeading = sharedHeading - visualHeading;
  if (deltaHeading > 180.0f)  deltaHeading -= 360.0f;
  if (deltaHeading < -180.0f) deltaHeading += 360.0f;
  visualHeading += deltaHeading * smoothingAlpha;

  float deltaHome = (float)dirToHome - visualHome;
  if (deltaHome > 180.0f)  deltaHome -= 360.0f;
  if (deltaHome < -180.0f) deltaHome += 360.0f;
  visualHome += deltaHome * smoothingAlpha;

  // --- 2. MISSION TIMER LOGIC ---
  if (isArmed) {
      if (!wasArmed) { 
          missionStartTime = millis(); // Updates the GLOBAL variable
          wasArmed = true; 
      }
      finalFlightTime = (millis() - missionStartTime); 
  } else { 
      wasArmed = false; 
  }

  // --- 3. PARAMETRIC CONSTANTS ---
  const float compassR = 22.0f; 
  const float clockR   = 16.0f; 
  const float compassX = 28.0f;
  const float clockX   = 88.0f; 
  const float centerY  = 32.0f;

  // --- 4. DRAW & SEND ---
  u8g2_CompassAndClock.clearBuffer();

  drawCompassFrame(u8g2_CompassAndClock, compassX, centerY, compassR);
  drawHomeNeedle(u8g2_CompassAndClock, compassX, centerY, compassR, visualHome);
  drawHeadingNeedle(u8g2_CompassAndClock, compassX, centerY, compassR, visualHeading);
  
  drawMissionClock(u8g2_CompassAndClock, clockX, centerY, clockR, finalFlightTime);

  u8g2_CompassAndClock.sendBuffer(); 
}

void updateEngineDisplay() {
  static float visualRpm = 0.0f;
  static float visualFuel = 100.0f;
  const float motorKv = 580.0f; 
  const float alpha = 0.12f;

  // 1. Data Processing
  float targetRpm = motorKv * vBat * sharedThrottle; 
  float maxScaleRpm = (vBat > 18.0f) ? 16000.0f : 10000.0f;
  visualRpm += (targetRpm - visualRpm) * alpha;
  visualFuel += (sharedBattery - visualFuel) * alpha;

  // 2. Render
  u8g2_Engine.clearBuffer();

  const float farLeftX = 24.0f; // Minimal clearance for 23r dial
  const float topY = 24.0f;
  const float bottomY = 101.0f;

  drawTacho(u8g2_Engine, farLeftX, topY, visualRpm, 10000.0f);
  drawFuelGauge(u8g2_Engine, 20.0f, bottomY, visualFuel);

  u8g2_Engine.sendBuffer();
}


void oled_MasterTask(void * pvParameters) {
    for(;;) {
        // Step 1: Update Screen 0x3C
        updateCompassAndClockDisplay();
        
        // Step 2: Small breather for the I2C bus hardware (optional but safe)
        vTaskDelay(pdMS_TO_TICKS(5));

        // Step 3: Update Screen 0x3D
        updateEngineDisplay();

        // Step 4: Total loop frequency (approx 20-25Hz)
        vTaskDelay(pdMS_TO_TICKS(40)); 
    }
}