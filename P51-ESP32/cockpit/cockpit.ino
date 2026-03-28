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

enum GaugeType { 
  TYPE_COMPASS,
  TYPE_CLOCK,
  TYPE_AIRSPEED,
  TYPE_ALTIMETER, 
  TYPE_TURN,
  TYPE_BANK,  
  TYPE_HORIZON, 
  TYPE_VSI, 
  TYPE_TACHO,
  TYPE_FUEL,
  TYPE_GEAR
};

enum DisplayType { SCREEN_TFT, SCREEN_U8G2 };

struct GaugeData {
  union {
    struct {
      int heading;
      int homeHeading;
    } turn;
    struct { 
      float verticalSpeed; 
    } vsi;
    struct { 
      float rotationsPerMinute; 
      float maxScaleRotationsPerMinute; 
    } tacho;
    struct { 
      float altitude; 
      float pressure; 
    } altimeter;
    struct { 
      float roll; 
      float pitch; 
    } horizon;
    struct { 
      float batteryVoltage; 
    } fuel;
    struct { 
      float mph; 
    } airspeed;
    struct { 
      float roll; 
    } bank;
  };
};

struct Gauge {
  const char* label;
  GaugeType type;
  int x, y, r;
  float labelScale;     // 1.0 = Default, 1.5 = Large, 0.8 = Tiny/Busy
  GaugeData data;
  void* displayPtr;
  DisplayType screen;
};

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

//Top line
// Gauge compassGauge = {
//   .label = "Magnetic Compass",
//   .type = TYPE_COMPASS,
//   .x = -99,
//   .y = -99,
//   .r = -99,
//   .labelScale = 1.0f,
//   .data = {.airspeed = {.mph = 0.0f}},
//   .displayPtr = (void*)&u8g2_CompassAndClock,
//   .screen = SCREEN_U8G2
// };


// Column 1 (Left)
Gauge airspeedGauge = {
  .label = "Airspeed",
  .type = TYPE_AIRSPEED,
  .x = 32,
  .y = 38,
  .r = 40,
  .labelScale = 1.0f,
  .data = {.airspeed = {.mph = 0.0f}},
  .displayPtr = (void*)&tft,
  .screen = SCREEN_TFT
};

Gauge altimeterGauge = {
  .label = "Altimeter",
  .type = TYPE_ALTIMETER,
  .x = 32,
  .y = 134,
  .r = 40,
  .labelScale = 1.0f,
  .data = {.altimeter = {.altitude = 0.0f, .pressure = 1013.25f}},
  .displayPtr = (void*)&tft,
  .screen = SCREEN_TFT
};

// Column 2 (Center)
Gauge turnGauge = {
  .label = "Turn Coordinator",
  .type = TYPE_TURN,
  .x = 137,
  .y = 38,
  .r = 36,
  .labelScale = 1.0f,
  .data = {.turn = {.heading = 0, .homeHeading = 0}},
  .displayPtr = (void*)&tft,
  .screen = SCREEN_TFT
};

Gauge bankGauge = {
  .label = "Bank Indicator",
  .type = TYPE_BANK,
  .x = 140,
  .y = 158,
  .r = 39,
  .labelScale = 1.0f,
  .data = {.bank = {.roll = 0.0f}},
  .displayPtr = (void*)&tft,
  .screen = SCREEN_TFT
};

Gauge gearGauge = {
  .label = "Gear Status",
  .type = TYPE_GEAR,
  .x = 140,
  .y = 210,
  .r = 20,
  .labelScale = 1.0f,
  .displayPtr = (void*)&tft,
  .screen = SCREEN_TFT
};

// Column 3 (Right)
Gauge horizonGauge = {
  .label = "Artificial Horizon",
  .type = TYPE_HORIZON,
  .x = 248,
  .y = 45,
  .r = 45,
  .labelScale = 1.0f,
  .data = {.horizon = {.roll = 0.0f, .pitch = 0.0f}},
  .displayPtr = (void*)&tft,
  .screen = SCREEN_TFT
};

Gauge vsiGauge = {
  .label = "VSI",
  .type = TYPE_VSI,
  .x = 254, 
  .y = 157,
  .r = 37,
  .labelScale = 1.0f,
  .data = {.vsi = {.verticalSpeed = 0.0f}},
  .displayPtr = (void*)&tft,
  .screen = SCREEN_TFT
};


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

void renderGauge(Gauge &g) {
  switch (g.type) {
    case TYPE_VSI:
      drawVSI(g); // Inside drawVSI, you use g.data.vsi.verticalSpeed
      break;
    case TYPE_HORIZON:
      // drawHorizon(g); // Inside here, you use g.data.horizon.pitch/roll
      break;
    // ... etc
  }
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
  if (cmd == MSP_RC) { // MSP_RC
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
    // --- CORRECTED THROTTLE PARSE ---
    // Your debug showed CH4 (Index 6) is the moving throttle stick.
    int thrIdx = 6; // Channel 4 = (4-1) * 2
    if (size >= (thrIdx + 2)) {
        uint16_t tRaw = payload[thrIdx] | (payload[thrIdx + 1] << 8);
        
        // Normalize 1000-2000 to 0.0-1.0
        // We use 1000.0f to ensure float division
        sharedThrottle = (float)(tRaw - 1000) / 1000.0f;
        
        // Hard constraints
        if (sharedThrottle < 0.0f) sharedThrottle = 0.0f;
        if (sharedThrottle > 1.0f) sharedThrottle = 1.0f;
        
        // Optional: add a tiny deadzone for 988-1000 jitter
        if (tRaw < 1010) sharedThrottle = 0.0f;
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
    vsi = (int16_t)(payload[4] | (payload[5] << 8));
    int32_t baroAlt = (int32_t)(payload[6] | (payload[7] << 8) | (payload[8] << 16) | (payload[9] << 24));
    alt = (estAlt == 0) ? (baroAlt / 30.48) : (estAlt / 30.48);

    if (isDebug && abs(alt - lastAltLog) > 0.1) { // 0.1ft threshold
       console.printf("DEBUG -> ALT: %.2f ft | VSI: %d\n", alt, (int)vsi);
       lastAltLog = alt;
    }
  }

  // --- BATTERY (MSP_ANALOG) ---
  else if (cmd == MSP_ANALOG && size >= 1) { 
    vBat = (payload[0] / 10.0f); // 16.4V
    
    // ignore payload[4] because it's returning Cell Count (3 or 4) instead of %
    // Manually calculate 4S percentage: 14.0V to 16.8V
    float voltPct = ((vBat - 14.0f) / 2.8f) * 100.0f;
    sharedBattery = constrain(voltPct, 0.0f, 100.0f);

    if (isDebug) {
      // This will now show ~86% instead of 3%
      console.printf("DEBUG -> VBAT: %.2fV | FUEL: %.0f%%\n", vBat, sharedBattery);
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

void drawAirspeed(Gauge &g) {
  if (g.screen != SCREEN_TFT) return;
  TFT_eSPI* tftPtr = (TFT_eSPI*)g.displayPtr;
  TFT_eSprite canvas = TFT_eSprite(tftPtr);
  
  int diameter = g.r * 2;
  if (!canvas.createSprite(diameter, diameter)) return;
  
  int cx = g.r; 
  int cy = g.r;

  // 1. Background Layers
  canvas.fillSprite(P51_CHARCOAL);
  canvas.fillSmoothCircle(cx, cy, (int)(g.r * 0.825), P51_EARTH); 
  canvas.fillSmoothCircle(cx, cy, (int)(g.r * 0.6), 0x18C3); 

  canvas.setTextColor(P51_RADIUM);
  
  // 2. Custom numbers and tick logic
  int labels[] = {10, 20, 30, 50, 70, 90};
  int labelCount = 6;
  int font = (g.r > 35) ? 2 : 1;

  for (int i = 0; i <= 100; i += 2) {
    float angleDeg = getAirspeedAngle((float)i);
    float rad = angleDeg * (PI / 180.0);
    
    bool isLabel = false;
    for(int k=0; k<labelCount; k++) { if(i == labels[k]) isLabel = true; }

    if (isLabel) {
      // Numbers - Nudged in to avoid bezel
      int tx = cx + (int)(g.r * 0.6 * cos(rad)); 
      int ty = cy + (int)(g.r * 0.6 * sin(rad));
      canvas.drawCentreString(String(i), tx, ty - 4, font);
      // Major Ticks
      canvas.drawLine(cx + (int)(g.r * 0.7 * cos(rad)), cy + (int)(g.r * 0.7 * sin(rad)), 
                      cx + (int)(g.r * 0.825 * cos(rad)), cy + (int)(g.r * 0.825 * sin(rad)), P51_RADIUM);
    } else if (i % 10 == 0 || i == 0) {
      // Mid Ticks
      canvas.drawLine(cx + (int)(g.r * 0.725 * cos(rad)), cy + (int)(g.r * 0.725 * sin(rad)), 
                      cx + (int)(g.r * 0.825 * cos(rad)), cy + (int)(g.r * 0.825 * sin(rad)), P51_RADIUM);
    } else {
      // Minor Ticks
      canvas.drawLine(cx + (int)(g.r * 0.775 * cos(rad)), cy + (int)(g.r * 0.775 * sin(rad)), 
                      cx + (int)(g.r * 0.825 * cos(rad)), cy + (int)(g.r * 0.825 * sin(rad)), P51_RADIUM);
    }
  }

  // 3. Labels
  canvas.setTextColor(P51_RADIUM);
  canvas.drawCentreString("MPH", cx, cy - (int)(g.r * 0.3), 1); 

  // 4. Needle Logic
  float speedValue = g.data.airspeed.mph;
  float needleAngle = getAirspeedAngle(constrain(speedValue, 0, 100));
  float sRad = needleAngle * (PI / 180.0);
  
  int nLen = (int)(g.r * 0.8);

  // Shadow
  canvas.drawLine(cx + 1, cy + 1, cx + 1 + (int)(nLen * cos(sRad)), cy + 1 + (int)(nLen * sin(sRad)), 0x0841);
  // Needle
  canvas.drawWideLine(cx, cy, cx + (int)(nLen * cos(sRad)), cy + (int)(nLen * sin(sRad)), 2, TFT_WHITE, 0x18C3);

  canvas.fillCircle(cx, cy, 3, TFT_BLACK);

  // 5. Final Push
  canvas.pushSprite(g.x - cx, g.y - cy);
  canvas.deleteSprite();
}

void drawPoshHorizon(Gauge &g) {
  if (g.screen != SCREEN_TFT) return;
  TFT_eSPI* tftPtr = (TFT_eSPI*)g.displayPtr;
  TFT_eSprite canvas = TFT_eSprite(tftPtr);
  
  int diameter = g.r * 2;
  if (!canvas.createSprite(diameter, diameter)) return;
  
  int cx = g.r; 
  int cy = g.r;

  // 1. COLORS
  uint16_t groundColor = 0x9442; // Saddle Brown
  uint16_t skyColor = 0x5D9B;    // Steel Blue
  
  float rll = g.data.horizon.roll;
  float ptch = g.data.horizon.pitch;
  float radRoll = rll * (PI / 180.0f);
  float tanRoll = tan(radRoll);
  float pOffset = constrain(ptch * 1.5f, -35.0f, 35.0f); // Increased sensitivity slightly

  // 2. DRAW MOVING HORIZON (Sky/Ground)
  canvas.fillSprite(skyColor);
  for (int i = -cx; i <= cx; i++) {
    float horizonY = cy - pOffset + (i * tanRoll);
    if (horizonY < diameter) {
      canvas.drawLine(cx + i, (int)constrain(horizonY, 0.0f, (float)diameter), cx + i, diameter, groundColor);
    }
  }

  // 3. PITCH LADDER (White lines on the moving horizon)
  canvas.setTextColor(TFT_WHITE);
  for (int p = -20; p <= 20; p += 5) {
    if (p == 0) continue; // The actual horizon line
    float pitchY = cy - pOffset - (p * 1.5f); // Sync with pOffset multiplier
    
    // Rotate and draw pitch lines
    int lineW = (p % 10 == 0) ? 15 : 8; // Longer lines for 10, 20
    float cosR = cos(radRoll);
    float sinR = sin(radRoll);

    int xStart = cx - (lineW * cosR);
    int yStart = pitchY - (lineW * sinR);
    int xEnd = cx + (lineW * cosR);
    int yEnd = pitchY + (lineW * sinR);

    if (pitchY > 5 && pitchY < diameter - 5) {
       canvas.drawLine(xStart, yStart, xEnd, yEnd, TFT_WHITE);
       if (p % 10 == 0) {
         canvas.setTextSize(1);
         canvas.setCursor(xEnd + 2, yEnd - 4);
         canvas.print(abs(p));
       }
    }
  }

  // 4. TOP BANK SCALE (Static)
  int angles[] = {-60, -45, -30, -20, -10, 0, 10, 20, 30, 45, 60};
  for (int i = 0; i < 11; i++) {
    float aRad = (angles[i] - 90) * (PI / 180.0f);
    int len = (angles[i] % 15 == 0) ? 5 : 3;
    canvas.drawLine(cx + (g.r-len)*cos(aRad), cy + (g.r-len)*sin(aRad), 
                    cx + g.r*cos(aRad), cy + g.r*sin(aRad), TFT_WHITE);
  }

  // 5. THE MINIATURE AIRCRAFT (Fixed Yellow "Wings" and Center Dot)
  // Center Dot
  canvas.fillCircle(cx, cy, 2, TFT_YELLOW);
  // Left Wing
  canvas.fillRect(cx - 24, cy - 1, 14, 3, TFT_YELLOW); 
  canvas.drawFastVLine(cx - 10, cy - 1, 5, TFT_YELLOW); // Down-turned tip
  // Right Wing
  canvas.fillRect(cx + 10, cy - 1, 14, 3, TFT_YELLOW);
  canvas.drawFastVLine(cx + 10, cy - 1, 5, TFT_YELLOW);

  // 6. TDC TRIANGLE (Sky Reference)
  canvas.fillTriangle(cx - 3, 5, cx + 3, 5, cx, 11, TFT_YELLOW);

  canvas.pushSprite(g.x - cx, g.y - cy);
  canvas.deleteSprite();
}

void drawHorizon(Gauge &g) {
  if (g.screen != SCREEN_TFT) return;
  TFT_eSPI* tftPtr = (TFT_eSPI*)g.displayPtr;
  TFT_eSprite canvas = TFT_eSprite(tftPtr);
  
  int diameter = g.r * 2;
  if (!canvas.createSprite(diameter, diameter)) return;
  
  int cx = g.r; 
  int cy = g.r;
  int cr = (int)(g.r * 0.9); // Indicator ring radius

  // 1. Moving World (Sky/Earth)
  canvas.fillSprite(P51_SKY);
  
  float rll = g.data.horizon.roll;
  float ptch = g.data.horizon.pitch;
  
  float radRoll = rll * (PI / 180.0f);
  float tanRoll = tan(radRoll);
  // Your exact pitch constraint and multiplier
  float pOffset = constrain(ptch * 1.2f, -30.0f, 30.0f);

  for (int i = -cx; i <= cx; i++) {
    float horizonY = cy - pOffset + (i * tanRoll);
    if (horizonY < diameter) {
      canvas.drawLine(cx + i, (int)constrain(horizonY, 0.0f, (float)diameter), cx + i, diameter, P51_EARTH);
    }
  }

  // 2. TDC TRIANGLE (Fixed at top - Yellow)
  // Scaled slightly for radius
  canvas.fillTriangle(cx - 3, 5, cx + 3, 5, cx, 9, TFT_YELLOW);

  // 3. Yellow Bank Ticks (-60, -30, 30, 60)
  for (int a = -60; a <= 60; a += 30) {
    if (a == 0) continue; 
    float tr = (a - 90.0f) * (PI / 180.0f);
    canvas.drawLine(cx + (int)((cr - 4) * cos(tr)), cy + (int)((cr - 4) * sin(tr)), 
                    cx + (int)(cr * cos(tr)), cy + (int)(cr * sin(tr)), TFT_YELLOW);
  }

  // 4. THE TRIDENT (Static Reference Wings)
  canvas.fillCircle(cx, cy - 1, 2, TFT_YELLOW); 
  // Left Wing
  canvas.fillRect(cx - 28, cy - 1, 18, 3, TFT_YELLOW); 
  // Right Wing
  canvas.fillRect(cx + 10, cy - 1, 18, 3, TFT_YELLOW);

  // 5. Text (Instrument ID)
  canvas.setTextColor(0x2104); 
  canvas.drawCentreString("AN 5736-1A", cx, diameter - 8, 1);

  // 6. Final Push to (253, 54)
  canvas.pushSprite(g.x - cx, g.y - cy);
  canvas.deleteSprite();
}

void drawAltimeter(Gauge &g) {
  if (g.screen != SCREEN_TFT) return;
  TFT_eSPI* tftPtr = (TFT_eSPI*)g.displayPtr;
  TFT_eSprite canvas = TFT_eSprite(tftPtr);
  
  int diameter = g.r * 2;
  if (!canvas.createSprite(diameter, diameter)) return;
  
  int cx = g.r; 
  int cy = g.r;

  // 1. Background Layers (Restored Aged Look)
  canvas.fillSprite(P51_CHARCOAL);
  canvas.fillSmoothCircle(cx, cy, (int)(g.r * 0.825), P51_EARTH); // Aged ring
  canvas.fillSmoothCircle(cx, cy, (int)(g.r * 0.55), 0x18C3);    // Sunken center

  // 2. Dial Markings
  canvas.setTextColor(P51_RADIUM);
  int font = (g.r * g.labelScale > 35) ? 2 : 1;
  int vPad = (font == 2) ? 4 : 2;

  for (int i = 0; i < 10; i++) {
    float angle = (i * 36 - 90) * (PI / 180.0);
    
    // Numbers
    int tx = cx + (int)(g.r * 0.625 * cos(angle));
    int ty = cy + (int)(g.r * 0.625 * sin(angle));
    canvas.drawCentreString(String(i), tx, ty - vPad, font);
    
    // Major Ticks (100ft)
    canvas.drawLine(cx + (int)(g.r * 0.725 * cos(angle)), cy + (int)(g.r * 0.725 * sin(angle)), 
                    cx + (int)(g.r * 0.825 * cos(angle)), cy + (int)(g.r * 0.825 * sin(angle)), P51_RADIUM);
    
    // Minor Ticks (20ft)
    for (int j = 1; j < 5; j++) {
      float subAngle = (i * 36 + j * 7.2 - 90) * (PI / 180.0);
      canvas.drawLine(cx + (int)(g.r * 0.775 * cos(subAngle)), cy + (int)(g.r * 0.775 * sin(subAngle)), 
                      cx + (int)(g.r * 0.825 * cos(subAngle)), cy + (int)(g.r * 0.825 * sin(subAngle)), P51_RADIUM);
    }
  }

  // 3. Kollsman Window (Parametric Position)
  int kw = (int)(g.r * 0.35);
  int kh = (int)(g.r * 0.25);
  int kx = cx + (int)(g.r * 0.4);
  int ky = cy - (int)(g.r * 0.15);
  canvas.fillRect(kx, ky, kw, kh, TFT_BLACK); 
  canvas.drawRect(kx, ky, kw, kh, 0x4228); 
  canvas.setTextColor(TFT_WHITE);
  canvas.drawCentreString("299", kx + (kw/2), ky + 1, 1); 

  // 4. Labels
  canvas.setTextColor(P51_RADIUM);
  canvas.drawCentreString("ALT", cx, cy + (int)(g.r * 0.15), 1);
  canvas.setTextColor(0x2104); 
  canvas.drawCentreString("1000 FEET", cx, cy - (int)(g.r * 0.075), 1);

  // 5. Hand Logic
  float alt = g.data.altimeter.altitude;
  float deg100   = (fmod(alt, 1000.0) / 1000.0) * 360.0;
  float deg1000  = (fmod(alt, 10000.0) / 10000.0) * 360.0;
  float deg10000 = (fmod(alt, 100000.0) / 100000.0) * 360.0;

  // 6. Draw Hands (Passing the canvas pointer to the helper)
  drawAltHand(canvas, cx, cy, deg10000, (int)(g.r * 0.775), 1, true);  // 10k Crow's Foot
  drawAltHand(canvas, cx, cy, deg1000,  (int)(g.r * 0.5),   4, false); // 1k Fat Hand
  drawAltHand(canvas, cx, cy, deg100,   (int)(g.r * 0.85),  2, false); // 100ft Long Hand

  canvas.fillCircle(cx, cy, 3, TFT_BLACK); 
  canvas.pushSprite(g.x - cx, g.y - cy);
  canvas.deleteSprite();
}

// Updated helper to take the sprite reference
void drawAltHand(TFT_eSprite &canvas, int cx, int cy, float deg, int len, int width, bool is10k) {
  float rad = (deg - 90.0) * (PI / 180.0);
  int px = cx + (int)(len * cos(rad));
  int py = cy + (int)(len * sin(rad));
  
  if (is10k) {
    canvas.drawLine(cx, cy, px, py, TFT_WHITE);
    float x1 = cx + (len - 5) * cos(rad + 0.2);
    float y1 = cy + (len - 5) * sin(rad + 0.2);
    float x2 = cx + (len - 5) * cos(rad - 0.2);
    float y2 = cy + (len - 5) * sin(rad - 0.2);
    canvas.fillTriangle(px, py, (int)x1, (int)y1, (int)x2, (int)y2, TFT_WHITE);
  } else {
    canvas.drawWideLine(cx, cy, px, py, width, TFT_WHITE, 0x18C3);
  }
}

void drawTurn(Gauge &g) {
  if (g.screen != SCREEN_TFT) return;
  TFT_eSPI* tftPtr = (TFT_eSPI*)g.displayPtr;
  TFT_eSprite canvas = TFT_eSprite(tftPtr);
  
  int diameter = g.r * 2;
  if (!canvas.createSprite(diameter, diameter)) return;
  
  int cx = g.r; 
  int cy = g.r;
  int winY = cy - 18; 
  int winH = 22;
  int winLeft = cx - 28;

  canvas.fillSprite(P51_CHARCOAL); 

  // 1. Internal Shadow
  canvas.fillRect(winLeft, winY, 56, winH, 0x0841); 

  // 2. GREEN HOME BAR LOGIC
  float heading = (float)g.data.turn.heading;
  float homeBearing = (float)g.data.turn.homeHeading;

  float homeDelta = homeBearing - heading;
  if (homeDelta > 180) homeDelta -= 360;
  if (homeDelta < -180) homeDelta += 360;

  float homeRad = homeDelta * (PI / 180.0);
  float homeX = cx + (sin(homeRad) * 45); 

  if (homeX > winLeft && homeX < winLeft + 56) {
    canvas.fillRect((int)homeX - 2, winY + 1, 4, winH - 2, 0x07E0); 
  }

  // 3. The Drum Logic
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

  // 4. Masking (Paint over the bleed)
  canvas.fillRect(0, 0, diameter, winY, P51_CHARCOAL);             
  canvas.fillRect(0, winY + winH, diameter, diameter - (winY + winH), P51_CHARCOAL); 
  canvas.fillRect(0, winY, winLeft, winH, P51_CHARCOAL);     
  canvas.fillRect(winLeft + 56, winY, diameter - (winLeft + 56), winH, P51_CHARCOAL); 

  // 5. Stationary Elements
  canvas.drawRect(winLeft, winY, 56, winH, 0x4228); 
  // Center Lubber Line (White)
  canvas.fillRect(cx - 1, winY - 3, 2, winH + 6, TFT_WHITE); 

  canvas.setTextColor(0x4228);
  canvas.drawCentreString("DIREC.GYRO", cx, cy + 10, 1);
  canvas.drawCentreString("AN 5735-1A", cx, cy + 20, 1);

  // 6. Push using the struct coordinates
  canvas.pushSprite(g.x - cx, g.y - cy);
  canvas.deleteSprite();
}


void drawBank(Gauge &g) {
  if (g.screen != SCREEN_TFT) return;
  TFT_eSPI* tftPtr = (TFT_eSPI*)g.displayPtr;
  TFT_eSprite canvas = TFT_eSprite(tftPtr);
  
  int diameter = g.r * 2;
  if (!canvas.createSprite(diameter, diameter)) return;
  
  int cx = g.r; 
  int cy = g.r;

  canvas.fillSprite(P51_CHARCOAL);

  // 1. The Background Housing (Slightly aged/browned)
  canvas.fillSmoothCircle(cx, cy, (int)(g.r * 0.8), 0x4228); 

  // 2. The "Bomb" Top Reference
  // Parametric scaling for the body
  int bombW = 5;
  int bombH = 12;
  canvas.fillSmoothRoundRect(cx - 3, cy - (int)(g.r * 0.7), bombW, bombH, 2, P51_DIRTY_W);

  // 3. The Yellow "Wings" (Static Blocks)
  int wingW = 12;
  int wingH = 10;
  canvas.fillRoundRect(cx - (int)(g.r * 0.7), cy + 2, wingW, wingH, 2, 0xCE60); 
  canvas.fillRoundRect(cx + (int)(g.r * 0.4), cy + 2, wingW, wingH, 2, 0xCE60);

  // 4. The Slip Ball Logic
  float rll = g.data.bank.roll;
  float ballX = cx + constrain(rll * 0.5f, -15.0f, 15.0f);
  
  // Ball Shadow & Body
  canvas.fillCircle(ballX, cy + 7, 6, TFT_BLACK);
  canvas.fillCircle(ballX, cy + 7, 5, 0x7BEF); 
  
  // Specular highlight for the "Steel in Liquid" look
  canvas.fillCircle(ballX - 2, cy + 5, 2, TFT_WHITE);

  // 5. Push using struct coordinates (146, 170)
  canvas.pushSprite(g.x - cx, g.y - cy);
  canvas.deleteSprite();
}

void drawVSI(Gauge &g) {
  if (g.screen != SCREEN_TFT) return;
  TFT_eSPI* tftPtr = (TFT_eSPI*)g.displayPtr;
  TFT_eSprite canvas = TFT_eSprite(tftPtr);
  
  // 1. SPRITE SETUP
  // Add 4px padding to prevent edge artifacts
  int spriteSize = (g.r * 2) + 4;
  canvas.createSprite(spriteSize, spriteSize);
  
  // Local centers (Integers only!)
  int cx = spriteSize / 2; 
  int cy = spriteSize / 2;

  // CRITICAL: Wipe the sprite memory completely
  canvas.fillSprite(TFT_BLACK); 
  
  // 2. BACKGROUNDS
  canvas.fillSmoothCircle(cx, cy, g.r, P51_CHARCOAL);
  canvas.fillSmoothCircle(cx, cy, (int)(g.r * 0.95), 0x2104); 
  canvas.fillSmoothCircle(cx, cy, (int)(g.r * 0.35), 0x0841); 

  // 3. FONT SCALING
  int fontSelection = 2;
  if (g.r * g.labelScale > 50) fontSelection = 4;
  else if (g.r * g.labelScale < 25) fontSelection = 1;
  canvas.setTextColor(P51_RADIUM);
  
  // 4. SCALE MARKS (1, 2, 4)
  float scaleValues[] = {1.0, 2.0, 4.0};
  float degreeOffsets[] = {35, 70, 140}; 
  
  int labelR = (int)(g.r * 0.55);  
  int tickIn  = (int)(g.r * 0.75);
  int tickOut = (int)(g.r * 0.95);
  int vPad = (fontSelection == 4) ? 8 : 4;

  for (int i = 0; i < 3; i++) {
    float radUp = (180 - degreeOffsets[i]) * (PI / 180.0);
    float radDn = (180 + degreeOffsets[i]) * (PI / 180.0);
    
    // Draw Numbers (Casted to Int)
    canvas.drawCentreString(String((int)scaleValues[i]), cx + (int)(labelR * cos(radUp)), cy + (int)(labelR * sin(radUp)) - vPad, fontSelection);
    canvas.drawCentreString(String((int)scaleValues[i]), cx + (int)(labelR * cos(radDn)), cy + (int)(labelR * sin(radDn)) - vPad, fontSelection);
    
    // Draw Ticks (Casted to Int)
    canvas.drawLine(cx + (int)(tickIn * cos(radUp)), cy + (int)(tickIn * sin(radUp)), cx + (int)(tickOut * cos(radUp)), cy + (int)(tickOut * sin(radUp)), P51_RADIUM);
    canvas.drawLine(cx + (int)(tickIn * cos(radDn)), cy + (int)(tickIn * sin(radDn)), cx + (int)(tickOut * cos(radDn)), cy + (int)(tickOut * sin(radDn)), P51_RADIUM);
  }

  // 5. NICHE DETAILS (The .5s and Brackets)
  canvas.drawCentreString(".5", cx - (int)(g.r * 0.7), cy - (int)(g.r * 0.5), 1);
  canvas.drawCentreString(".5", cx - (int)(g.r * 0.7), cy + (int)(g.r * 0.3), 1);
  
  // Zero and 6 markers
  canvas.drawLine(cx - g.r, cy, cx - (int)(g.r * 0.75), cy, P51_RADIUM);
  canvas.drawCentreString("6", cx + (int)(g.r * 0.65), cy - vPad, fontSelection);
  canvas.drawLine(cx + (int)(g.r * 0.85), cy, cx + (int)(g.r * 0.95), cy, P51_RADIUM);

  // 6. NEEDLE LOGIC
  float absV = abs(g.data.vsi.verticalSpeed);
  float move = 0;
  if (absV <= 1.0) move = absV * 35.0;
  else if (absV <= 2.0) move = 35.0 + (absV - 1.0) * 35.0;
  else if (absV <= 4.0) move = 70.0 + (absV - 2.0) * 35.0; 
  else move = 140.0 + (constrain(absV, 4, 6) - 4.0) * 20.0; 

  float finalDeg = (g.data.vsi.verticalSpeed >= 0) ? (180.0 - move) : (180.0 + move);
  float nRad = finalDeg * (PI / 180.0);
  int nLen = (int)(g.r * 0.85); 
  
  // Shadow + Main Needle (Casted to Int)
  canvas.drawLine(cx + 1, cy + 1, cx + 1 + (int)(nLen * cos(nRad)), cy + 1 + (int)(nLen * sin(nRad)), 0x0841);
  canvas.drawWideLine(cx, cy, cx + (int)(nLen * cos(nRad)), cy + (int)(nLen * sin(nRad)), 2, TFT_WHITE, 0x0841);
  canvas.fillCircle(cx, cy, 3, TFT_BLACK);
  
  // 7. FINAL PUSH
  // We use (int) to ensure the sprite lands exactly on a physical screen pixel
  canvas.pushSprite((int)(g.x - cx), (int)(g.y - cy));
  canvas.deleteSprite();
}



void drawGearStatus(Gauge &g) {
  if (g.screen != SCREEN_TFT) return;
  TFT_eSPI* tftPtr = (TFT_eSPI*)g.displayPtr;
  // Using your specific ucSprite name if it's a global, 
  // or localizing it to the gauge's display pointer:
  TFT_eSprite canvas = TFT_eSprite(tftPtr);
  
  // 1. SPRITE SETUP (Your 70x22 size)
  if (!canvas.createSprite(70, 22)) return;
  
  int localCX = 34; 
  int localCY = 11; 
  int spacing = 20; 

  canvas.fillSprite(P51_CHARCOAL);

  // 2. TRANSIT LOGIC (Your exact 7-second sequence)
  // Note: These need to be global variables or added to the Gauge struct
  if (gearDown != lastGearDown) {
    gearInTransit = true;
    gearTimer = millis();
    lastGearDown = gearDown;
  }
  
  if (gearInTransit && (millis() - gearTimer > 7000)) {
    gearInTransit = false;
  }

  // 3. RED LIGHT (UNSAFE/TRANSIT)
  if (gearInTransit) {
    canvas.fillCircle(localCX + spacing, localCY, 6, TFT_RED);
    canvas.drawCircle(localCX + spacing, localCY, 7, 0x8000); // Dark Red Glow
  }

  // 4. GREEN LIGHT (LOCKED DOWN)
  if (gearDown && !gearInTransit) {
    canvas.fillCircle(localCX - spacing, localCY, 6, TFT_GREEN);
    canvas.drawCircle(localCX - spacing, localCY, 7, 0x03E0); // Green Glow
  }

  // 5. PUSH (Using struct coordinates)
  canvas.pushSprite(g.x, g.y);
  canvas.deleteSprite();
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
    
    // Width scales with radius (approx 10% of r)
    float width = r * 0.10f;
    float gX = -sinH * width, gY = cosH * width; 

    float tipR = r * 0.75f;
    float bodyR = r * 0.60f;

    // Draw the parallel "rails"
    for (int side = -1; side <= 1; side += 2) {
        float rX = gX * side, rY = gY * side;
        drawSafeLine(canvas, centerX - bodyR * cosH + rX, centerY - bodyR * sinH + rY, 
                            centerX + bodyR * cosH + rX, centerY + bodyR * sinH + rY);
    }

    // Draw the Arrow Head
    float tipX = centerX + tipR * cosH, tipY = centerY + tipR * sinH;
    drawSafeLine(canvas, centerX + bodyR * cosH + gX, centerY + bodyR * sinH + gY, tipX, tipY);
    drawSafeLine(canvas, centerX + bodyR * cosH - gX, centerY + bodyR * sinH - gY, tipX, tipY);
    
    // Draw the Base Cap
    drawSafeLine(canvas, centerX - bodyR * cosH + gX, centerY - bodyR * sinH + gY, 
                        centerX - bodyR * cosH - gX, centerY - bodyR * sinH - gY);
}

void drawHomeNeedle(U8G2 &canvas, float centerX, float centerY, float r, float heading) {
    float rad = (heading - 90.0f) * (M_PI / 180.0f);
    float cosH = cosf(rad), sinH = sinf(rad);
    float pX = -sinH, pY = cosH;

    float tipR = r * 0.65f;
    float tailR = r * 0.30f;
    float tipX = centerX + tipR * cosH;
    float tipY = centerY + tipR * sinH;

    // Thickness scales: 1px for small dials, 2px+ for larger ones
    float thickness = max(1.0f, r * 0.05f); 
    float startOffset = -(thickness / 2.0f);

    for (float i = 0; i < thickness; i += 1.0f) {
        float shiftX = (startOffset + i) * pX;
        float shiftY = (startOffset + i) * pY;
        
        drawSafeLine(canvas, centerX + shiftX, centerY + shiftY, tipX + shiftX, tipY + shiftY);
        drawSafeLine(canvas, centerX + shiftX, centerY + shiftY, centerX - tailR * cosH + shiftX, centerY - tailR * sinH + shiftY);
    }

    // Cap width (T-bar) stays 15% of radius
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

void drawTacho(U8G2 &canvas, float centerX, float centerY, float r, float rpm, float maxRpm) {
    canvas.setFont(u8g2_font_u8glib_4_tf); 

    // 1. DYNAMIC TICK MARKS
    for (int i = 0; i <= 10; i++) {
        // 270 degree sweep (from 225 down to -45)
        float angle = 225.0f - (i * 27.0f); 
        float rad = (angle - 90.0f) * (M_PI / 180.0f);
        
        // Parametric radii
        float rOut = r;
        float rIn = (i % 2 == 0) ? (r * 0.78f) : (r * 0.87f); 
        
        // Ticks are static, so standard drawLine is fine here
        canvas.drawLine((int)(centerX + rOut * cosf(rad)), (int)(centerY + rOut * sinf(rad)), 
                        (int)(centerX + rIn * cosf(rad)), (int)(centerY + rIn * sinf(rad)));

        // 2. PARAMETRIC NUMBER PLACEMENT
        if (i % 2 != 0) { // i = 1, 3, 5, 7, 9
            char buf[4]; 
            // Scalable label: 10, 30, 50... or adjusted for maxRpm scale
            sprintf(buf, "%d", (int)((maxRpm / 10.0f) * i / 100.0f)); 
            
            // Numbers placed at 50% of radius
            float rText = r * 0.50f;
            int tx = (int)(centerX + rText * cosf(rad));
            int ty = (int)(centerY + rText * sinf(rad));
            
            // Center the text based on font width
            int w = canvas.getStrWidth(buf);
            canvas.drawStr(tx - (w / 2), ty + 2, buf);
        }
    }

    // 3. PARAMETRIC TAPERED NEEDLE
    // Calculate percentage of maxRpm (clamp to 0.0 - 1.0)
    float rpmPct = rpm / maxRpm;
    if (rpmPct > 1.0f) rpmPct = 1.0f;
    if (rpmPct < 0.0f) rpmPct = 0.0f;

    float needleRad = (225.0f - (rpmPct * 270.0f) - 90.0f) * (M_PI / 180.0f);
    
    // Needle tip at 75% of radius
    float tipR = r * 0.75f;
    float tipX = centerX + tipR * cosf(needleRad);
    float tipY = centerY + tipR * sinf(needleRad);
    
    // Width of needle base scales with r
    float baseWidth = max(1.0f, r * 0.08f);
    float side1X = centerX + baseWidth * cosf(needleRad + 1.57f);
    float side1Y = centerY + baseWidth * sinf(needleRad + 1.57f);
    float side2X = centerX + baseWidth * cosf(needleRad - 1.57f);
    float side2Y = centerY + baseWidth * sinf(needleRad - 1.57f);

    // Use SafeLines for moving parts
    drawSafeLine(canvas, centerX, centerY, tipX, tipY); 
    drawSafeLine(canvas, side1X, side1Y, tipX, tipY);
    drawSafeLine(canvas, side2X, side2Y, tipX, tipY);

    // 4. CENTER DISC (Scales with radius)
    canvas.drawDisc((int)centerX, (int)centerY, (int)(r * 0.12f));
}

void drawFuelGauge(U8G2 &canvas, float centerX, float centerY, float r, float percent) {
    // 1. Ticks (Using your 23/19 ratio)
    for (int i = 0; i <= 4; i++) {
        float angle = 210.0f - (i * 60.0f); 
        float rad = (angle - 90.0f) * (M_PI / 180.0f);
        
        float rOut = r;          // Was 23
        float rIn = r * 0.826f;  // Was 19 (19/23 = 0.826)
        
        canvas.drawLine((int)(centerX + rOut * cosf(rad)), (int)(centerY + rOut * sinf(rad)), 
                        (int)(centerX + rIn * cosf(rad)), (int)(centerY + rIn * sinf(rad)));
    }

    // 2. BIG RADIAL LABELS (E, 1/2, F)
    canvas.setFont(u8g2_font_5x7_tr); 
    const char* labels[] = {"E", "", "1/2", "", "F"};
    float rText = r * 0.435f; // Was 10 (10/23 = 0.435)

    for (int i = 0; i <= 4; i++) {
        if (strlen(labels[i]) > 0) {
            float angle = 210.0f - (i * 60.0f);
            float rad = (angle - 90.0f) * (M_PI / 180.0f);
            int tx = (int)(centerX + rText * cosf(rad));
            int ty = (int)(centerY + rText * sinf(rad));
            int xOff = (i == 2) ? 7 : 3; 
            canvas.drawStr(tx - xOff, ty + 3, labels[i]);
        }
    }

    // 3. TAPERED NEEDLE (Matching your WORKING logic exactly)
    float fuelRad = (210.0f - ((percent / 100.0f) * 240.0f) - 90.0f) * (M_PI / 180.0f);
    float tipR = r * 0.739f; // Was 17 (17/23 = 0.739)
    int fTipX = (int)(centerX + tipR * cosf(fuelRad));
    int fTipY = (int)(centerY + tipR * sinf(fuelRad));
    
    // Using your exact 3-line structure
    canvas.drawLine((int)centerX, (int)centerY, fTipX, fTipY); 
    canvas.drawLine((int)(centerX + 1 * cosf(fuelRad + 1.57f)), (int)(centerY + 1 * sinf(fuelRad + 1.57f)), fTipX, fTipY);
    canvas.drawLine((int)(centerX + 1 * cosf(fuelRad - 1.57f)), (int)(centerY + 1 * sinf(fuelRad - 1.57f)), fTipX, fTipY);
    
    // Center Disc (Was 2)
    canvas.drawDisc((int)centerX, (int)centerY, (int)max(2.0f, r * 0.087f));
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
  const float gaugeRadius = 23.0f; // Parametric master size for this screen

  drawTacho(u8g2_Engine, farLeftX, topY, gaugeRadius, visualRpm, maxScaleRpm);
  drawFuelGauge(u8g2_Engine, 20.0f, bottomY, gaugeRadius, visualFuel);

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

  //drawGearStatus(140,220);



  // Column 1 (Left) - Airspeed & Altimeter
  airspeedGauge.data.airspeed.mph = airSpeed;
  drawAirspeed(airspeedGauge);

  altimeterGauge.data.altimeter.altitude = alt;
  drawAltimeter(altimeterGauge);

  // Column 2 (Center) - Turn, Bank & Gear
  turnGauge.data.turn.heading = heading;
  turnGauge.data.turn.homeHeading = dirToHome;
  drawTurn(turnGauge);

  bankGauge.data.bank.roll = roll;
  drawBank(bankGauge);

  // Gear status usually uses global state or a dedicated flag
  drawGearStatus(gearGauge);

  // Column 3 (Right) - Horizon & VSI
  horizonGauge.data.horizon.roll = roll;
  horizonGauge.data.horizon.pitch = pitch;
  drawPoshHorizon(horizonGauge);

  vsiGauge.data.vsi.verticalSpeed = vsi / 100.0f; // cm/s to m/s
  drawVSI(vsiGauge);




  yield(); // Let S3 background tasks (WiFi/BT stack) breathe

}
