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

enum DisplayType { SCREEN_TFT, SCREEN_OLED_A, SCREEN_OLED_B };

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
    struct {
      int heading;
      int homeHeading;
    } compass;
    struct {
      uint32_t flightTime; // Milliseconds since mission start
    } clock;
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

TFT_eSPI tft = TFT_eSPI();

// Compass display is wide (128x64)
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2_CompassAndClock(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);
// Engine display is tall (64x128)
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2_Engine(U8G2_R3, /* reset=*/ U8X8_PIN_NONE);

//Top line
// --- COMPASS (Left Side of OLED A) ---
Gauge compassGauge = {
  .label = "Compass",
  .type = TYPE_COMPASS,
  .x = 28,
  .y = 42,
  .r = 21,
  .labelScale = 1.0f,
  .data = {
    .compass = {
      .heading = 0, 
      .homeHeading = 0
    }
  },
  .displayPtr = (void*)&u8g2_CompassAndClock,
  .screen = SCREEN_OLED_A
};

// --- MISSION CLOCK (Right Side of OLED A) ---
Gauge clockGauge = {
  .label = "Mission Clock",
  .type = TYPE_CLOCK,
  .x = 89,
  .y = 44,
  .r = 16,
  .labelScale = 0.8f,
  .data = {
    .clock = {
      .flightTime = 0
    }
  },
  .displayPtr = (void*)&u8g2_CompassAndClock,
  .screen = SCREEN_OLED_A
};


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

Gauge tachoGauge = {
  .label = "Tachometer",
  .type = TYPE_TACHO,
  .x = 28,
  .y = 28,
  .r = 26,
  .labelScale = 1.0f,
  .data = {.tacho = {.rotationsPerMinute = 0.0f, .maxScaleRotationsPerMinute = 3000.0f}},
  .displayPtr = (void*)&u8g2_Engine,
  .screen = SCREEN_OLED_B
};

Gauge fuelGauge = {
  .label = "Battery",
  .type = TYPE_FUEL,
  .x = 23,
  .y = 98,
  .r = 26,
  .labelScale = 1.0f,
  .data = {.fuel = {.batteryVoltage = 16.8f}},
  .displayPtr = (void*)&u8g2_Engine,
  .screen = SCREEN_OLED_B
};

// --- SINGLE SOURCE OF TRUTH ---
struct Telemetry {
  // Power & Engine
  float vBat = 16.8f;
  float fuelPercent = 100.0f;
  float throttle = 0.0f;
  int cellCount = 4;

  // Flight Data
  float airSpeed = 0.0f;
  float altitude = 0.0f;
  float vsi = 0.0f;
  float roll = 0.0f;
  float pitch = 0.0f;
  float heading = 0.0f;

  // Navigation
  uint16_t distToHome = 0;
  int16_t dirToHome = 0;

  // Gear & Flaps logic
  bool gearCommand;        // What the pilot wants (Switch Position)
  bool lastGearCommand;    // For detecting the moment the switch is flipped
  bool gearDown;           // The actual physical state of the wheels
  bool lastGearDown;
  bool gearInTransit;      // Is the hydraulic system running?
  uint32_t gearTimer;      // When did the movement start?
  const uint32_t GEAR_CYCLE_TIME = 5000;
  int flapPos = 0; // 0: Up, 1: Takeoff, 2: Landing

  bool lastArmState = false;
  bool isArmed = false;
  
  uint32_t armStartTime = 0;    // Snapshot of millis() when armed
  uint32_t msElapsed = 0;       // Total milliseconds since arming
  uint32_t finalMsElapsed = 0;  // Stored time after disarm

  bool isBenchMode = true;     
  uint32_t lastDataTime = 0;   
  uint32_t lastRequestTime = 0;
  float lastAltLog = 0;        
};

// The instance marked as volatile for cross-core safety
volatile Telemetry aircraft;

void updateGearPhysics() {
  if (aircraft.gearInTransit) {
    if (millis() - aircraft.gearTimer >= aircraft.GEAR_CYCLE_TIME) {
      aircraft.gearInTransit = false; // Stop the movement
      // The wheels now match whatever switch position started this mess
      aircraft.gearDown = aircraft.lastGearDown; 
    }
  }
}

void setup() {
  console.begin(115200);
  while (!console);
  delay(200);
  if (isDebug) console.println("--- P-51 COCKPIT: BARE METAL START ---");

  // --- Initialization Splash / Setup ---
  // Create a temporary local canvas for setup tasks
  TFT_eSprite canvas = TFT_eSprite(&tft); 
  canvas.createSprite(80, 80);
  // ... any code using canvas ...
  canvas.deleteSprite();

  TFT_eSprite ucSprite = TFT_eSprite(&tft);
  ucSprite.createSprite(70, 22);
  // ... any code using ucSprite ...
  ucSprite.deleteSprite();

  tft.init();
  tft.setRotation(1);
  tft.fillScreen(TFT_BLACK);
  toAndFromFC.begin(115200, SERIAL_8N1, RX_FROM_FC, TX_TO_FC);

  i2cMutex = xSemaphoreCreateMutex();

  // Handshake
  unsigned long startScan = millis();
  while (millis() - startScan < 5000) {
    sendMSPRequest(MSP_ATTITUDE);
    delay(100); 
    if (parseMSP()) {
      aircraft.isBenchMode = false;
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
  
  uint8_t payload[64]; 
  toAndFromFC.readBytes(payload, size);
  uint8_t crc = toAndFromFC.read(); 

  aircraft.lastDataTime = millis();

  // --- RC DATA (GEAR, FLAPS, ARMING) ---
  if (cmd == MSP_RC) {
    // 1. ARMING (Channel 14)
    int armIdx = (CH_ARM - 1) * 2;
    if (size >= (armIdx + 2)) {
      uint16_t aRaw = payload[armIdx] | (payload[armIdx + 1] << 8);
      bool newArmedState = (aRaw > 1500); 
      if (isDebug && (newArmedState != aircraft.isArmed)) {
        console.printf("DEBUG -> SYSTEM: %s (Raw PWM: %d)\n", newArmedState ? "!!! ARMED !!!" : "DISARMED", aRaw);
      }
      aircraft.isArmed = newArmedState;
    }

    // 2. GEAR (Channel 6)
    int gearIdx = (CH_GEAR - 1) * 2;
    if (size >= (gearIdx + 2)) {
      uint16_t newGearVal = payload[gearIdx] | (payload[gearIdx + 1] << 8);
      bool switchPositionDown = (newGearVal > 1500); 

      // If switch changed and we aren't moving, start moving!
      if (switchPositionDown != aircraft.gearDown && !aircraft.gearInTransit) {
        aircraft.gearInTransit = true;
        aircraft.gearTimer = millis();
        aircraft.lastGearDown = switchPositionDown; // Save the goal
      }
    }
    // 3. FLAPS (Channel 7)
    int flapIdx = (CH_FLAPS - 1) * 2;
    if (size >= (flapIdx + 2)) {
      uint16_t fRaw = payload[flapIdx] | (payload[flapIdx + 1] << 8);
      uint8_t newFlapPos;
      if (fRaw < 1300) newFlapPos = 0; 
      else if (fRaw < 1700) newFlapPos = 1;
      else newFlapPos = 2;

      if (isDebug && (newFlapPos != aircraft.flapPos)) {
        const char* states[] = {"CLEAN", "TAKEOFF", "LANDING"};
        console.printf("DEBUG -> FLAPS: %s (Raw PWM: %d)\n", states[newFlapPos], fRaw);
      }
      aircraft.flapPos = newFlapPos;    
    }

    // 4. THROTTLE (CH4 / Index 6)
    int thrIdx = 6; 
    if (size >= (thrIdx + 2)) {
        uint16_t tRaw = payload[thrIdx] | (payload[thrIdx + 1] << 8);
        float throttleNormalised = (float)(tRaw - 1000) / 1000.0f;
        if (throttleNormalised < 0.0f) throttleNormalised = 0.0f;
        if (throttleNormalised > 1.0f) throttleNormalised = 1.0f;
        if (tRaw < 1010) throttleNormalised = 0.0f;
        aircraft.throttle = throttleNormalised;
    }
  }

  // --- ATTITUDE ---
  else if (cmd == MSP_ATTITUDE && size >= 6) {
    aircraft.roll = (int16_t)(payload[0] | (payload[1] << 8)) / 10.0;
    aircraft.pitch = (int16_t)(payload[2] | (payload[3] << 8)) / 10.0;
    aircraft.heading = (int16_t)(payload[4] | (payload[5] << 8));
  } 

  // --- ALTITUDE ---
  else if (cmd == MSP_ALTITUDE && size >= 10) {
    int32_t estAlt = (int32_t)(payload[0] | (payload[1] << 8) | (payload[2] << 16) | (payload[3] << 24));
    aircraft.vsi = (int16_t)(payload[4] | (payload[5] << 8));
    int32_t baroAlt = (int32_t)(payload[6] | (payload[7] << 8) | (payload[8] << 16) | (payload[9] << 24));
    aircraft.altitude = (estAlt == 0) ? (baroAlt / 30.48) : (estAlt / 30.48);

    if (isDebug && abs(aircraft.altitude - aircraft.lastAltLog) > 0.1) {
       console.printf("DEBUG -> ALT: %.2f ft | VSI: %d\n", aircraft.altitude, (int)aircraft.vsi);
       aircraft.lastAltLog = aircraft.altitude;
    }
  }

  // --- BATTERY ---
  else if (cmd == MSP_ANALOG && size >= 1) { 
    aircraft.vBat = (payload[0] / 10.0f);
    // Auto-detect cells for better percentage accuracy later
    if (aircraft.cellCount == 0 && aircraft.vBat > 1.0) {
        aircraft.cellCount = (int)((aircraft.vBat / 4.2f) + 0.5f);
    }
    float voltPct = ((aircraft.vBat - (aircraft.cellCount * 3.5f)) / (aircraft.cellCount * 0.7f)) * 100.0f;
    aircraft.fuelPercent = constrain(voltPct, 0.0f, 100.0f);

    if (isDebug) {
      console.printf("DEBUG -> VBAT: %.2fV | FUEL: %.0f%%\n", aircraft.vBat, aircraft.fuelPercent);
    }
  }

  // --- GPS SPEED (RAW_GPS) ---
  else if (cmd == 106 && size >= 16) {
      uint16_t groundSpeedCMS = (uint16_t)(payload[12] | (payload[13] << 8));
      aircraft.airSpeed = groundSpeedCMS * 0.0223694;
      if (aircraft.airSpeed < 1.5) aircraft.airSpeed = 0;
  }

  // --- NAV STATUS ---
  else if (cmd == MSP_NAV_STATUS && size >= 6) {
      aircraft.distToHome = (uint16_t)(payload[3] | (payload[4] << 8));
      aircraft.dirToHome = (int16_t)(payload[5] | (payload[6] << 8));

      if (isDebug) {
        static uint16_t lastDebugDistToHome = 0; 
        static int16_t lastDebugDirectionHome = 0;
        if (abs((int)aircraft.distToHome - (int)lastDebugDistToHome) > 1 || abs((int)aircraft.dirToHome - (int)lastDebugDirectionHome) > 1) {
          console.printf("DEBUG -> FC HOME: %dm | BRG: %d deg\n", aircraft.distToHome, aircraft.dirToHome);
          lastDebugDistToHome = aircraft.distToHome; lastDebugDirectionHome = aircraft.dirToHome;
        }
      }
  }

  return true;
}


void updateMSP() {
  static int cycle = 0;
  
  // 1. SEND REQUESTS (Every 10ms to cycle through telemetry)
  // Accessing lastRequestTime via the aircraft struct
  if (millis() - aircraft.lastRequestTime > 10) {
    cycle = (cycle + 1) % 10; 
    
    switch(cycle) {
      // Prioritize Attitude (Horizon) every other cycle for smoothness
      case 0: case 2: case 4: case 6: case 8: 
        sendMSPRequest(MSP_ATTITUDE); 
        break;
      case 1: 
        sendMSPRequest(MSP_ALTITUDE); 
        break;
      case 3: 
        sendMSPRequest(MSP_ANALOG); 
        break;
      case 5: 
        sendMSPRequest(MSP_RC); 
        break; 
      case 7: 
        sendMSPRequest(106); // MSP_RAW_GPS
        break;
      case 9: 
        sendMSPRequest(MSP_NAV_STATUS); 
        break;
    }
    aircraft.lastRequestTime = millis();
  }

  // 2. PROCESS INCOMING DATA
  // Drains the Serial buffer as long as there's a potential MSP packet
  while (toAndFromFC.available() >= 6) {
    if (parseMSP()) {
      // parseMSP now updates aircraft.lastDataTime internally
    } else {
      // If parseMSP fails (wrong header), we need to clear the byte to avoid a hang
      if (toAndFromFC.available() > 0 && toAndFromFC.peek() != '$') {
        toAndFromFC.read();
      }
    }
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

  // 1. DATA & COLORS
  uint16_t groundColor = 0x9442; // Saddle Brown
  uint16_t skyColor    = 0x5D9B; // Steel Blue
  float roll = g.data.horizon.roll;
  float pitch = g.data.horizon.pitch;
  
  float radRoll = roll * (PI / 180.0f);
  float cosR = cos(radRoll);
  float sinR = sin(radRoll);
  float pOffset = pitch * 1.5f; 

  // 2. DRAW ROTATED WORLD (Sky/Ground)
  canvas.fillSprite(skyColor);
  
  // We define a massive rectangle for the ground and rotate its corners
  int boxW = diameter * 2; // Extra wide for rotation coverage
  int boxH = diameter * 2;
  
  // The 4 corners of the Ground box relative to the horizon line
  // Corner points (x, y) before rotation
  float pts[4][2] = {
    {-boxW, 0}, {boxW, 0}, {boxW, boxH}, {-boxW, boxH}
  };

  int x[4], y[4];
  for(int i = 0; i < 4; i++) {
    // Standard Rotation Matrix: x' = x*cos - y*sin | y' = x*sin + y*cos
    // We add pOffset to the 'y' component before rotating to handle pitch
    float localY = pts[i][1] + pOffset;
    x[i] = cx + (pts[i][0] * cosR - localY * sinR);
    y[i] = cy + (pts[i][0] * sinR + localY * cosR);
  }
  
  // Fill the ground polygon (using two triangles)
  canvas.fillTriangle(x[0], y[0], x[1], y[1], x[2], y[2], groundColor);
  canvas.fillTriangle(x[0], y[0], x[2], y[2], x[3], y[3], groundColor);

  // 3. PITCH LADDER (Locked to Ground, Fixed Text)
  canvas.setTextColor(TFT_WHITE);
  canvas.setTextSize(1);
  for (int p = -30; p <= 30; p += 10) {
    if (p == 0) continue; 
    
    float pY = (p * 1.5f) + pOffset; 
    int lineW = 15;

    // Calculate rotated endpoints
    int x1 = cx + (-lineW * cosR - pY * sinR);
    int y1 = cy + (-lineW * sinR + pY * cosR);
    int x2 = cx + (lineW * cosR - pY * sinR);
    int y2 = cy + (lineW * sinR + pY * cosR);

    if (y1 > 0 && y1 < diameter && y2 > 0 && y2 < diameter) {
       canvas.drawLine(x1, y1, x2, y2, TFT_WHITE);
       
       // --- FIXED TEXT LOGIC ---
       // We find which of the two points (x1 or x2) is further "Right" on the actual screen
       int rightX = (x1 > x2) ? x1 : x2;
       int rightY = (x1 > x2) ? y1 : y2;
       
       // Now offset the text slightly to the right of THAT point
       canvas.setCursor(rightX + 3, rightY - 4);
       canvas.print(abs(p));
    }
  }
  // 4. TOP BANK SCALE (Static - your original version)
  int angles[] = {-60, -45, -30, -20, -10, 0, 10, 20, 30, 45, 60};
  for (int i = 0; i < 11; i++) {
    float aRad = (angles[i] - 90) * (PI / 180.0f);
    int len = (angles[i] % 15 == 0) ? 5 : 3;
    canvas.drawLine(cx + (g.r-len)*cos(aRad), cy + (g.r-len)*sin(aRad), 
                    cx + g.r*cos(aRad), cy + g.r*sin(aRad), TFT_WHITE);
  }

  // 5. THE MINIATURE AIRCRAFT (The P-51 Yellow Wings)
  canvas.fillCircle(cx, cy, 2, TFT_YELLOW);
  canvas.fillRect(cx - 24, cy - 1, 14, 3, TFT_YELLOW); 
  canvas.drawFastVLine(cx - 10, cy - 1, 5, TFT_YELLOW); 
  canvas.fillRect(cx + 10, cy - 1, 14, 3, TFT_YELLOW);
  canvas.drawFastVLine(cx + 10, cy - 1, 5, TFT_YELLOW);

  // 6. TDC TRIANGLE
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
  TFT_eSprite canvas = TFT_eSprite(tftPtr);
  
  // Keep your original 70x22 canvas size
  if (!canvas.createSprite(70, 22)) return;
  canvas.fillSprite(P51_CHARCOAL);

  int lx = 15; // Green Light Center X
  int rx = 55; // Red Light Center X
  int cy = 11; // Vertical Center
  int r  = 11; // MAX RADIUS (Fills the 22px height exactly)

  // 1. RED LIGHT (UNSAFE / IN TRANSIT)
  if (aircraft.gearInTransit) {
    // Solid fill to the edge of the 22px height
    canvas.fillCircle(rx, cy, r, TFT_RED);
    // Subtle inner shadow to give it depth without a white ring
    canvas.drawCircle(rx, cy, r, 0x8000); 
  } else {
    // Dim "Glass" look - slightly smaller so it doesn't bleed when off
    canvas.fillCircle(rx, cy, r - 2, 0x2000); 
  }

  // 2. GREEN LIGHT (LOCKED & DOWN)
  if (aircraft.gearDown && !aircraft.gearInTransit) {
    canvas.fillCircle(lx, cy, r, TFT_GREEN);
    canvas.drawCircle(lx, cy, r, 0x03E0); 
  } else {
    canvas.fillCircle(lx, cy, r - 2, 0x0100); 
  }

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

void drawCompass(Gauge &g) {
    if (g.screen != SCREEN_OLED_A) return;
    U8G2 *canvas = (U8G2*)g.displayPtr;
    
    float cx = (float)g.x;
    float cy = (float)g.y;
    float r  = (float)g.r;

    // 1. DRAW FRAME (N, S, E, W + Ticks)
    canvas->setFont(u8g2_font_5x7_tr);
    int fAscent = canvas->getAscent();

    // Labels
    int nW = canvas->getStrWidth("N");
    canvas->drawStr(cx - (nW / 2), cy - (r * 0.85f), "N");
    int sW = canvas->getStrWidth("S");
    canvas->drawStr(cx - (sW / 2), cy + (r * 0.85f) + fAscent, "S");
    int eW = canvas->getStrWidth("E");
    canvas->drawStr(cx + (r * 0.85f) - (eW / 2), cy + (fAscent / 2), "E");
    int wW = canvas->getStrWidth("W");
    canvas->drawStr(cx - (r * 0.85f) - (wW / 2), cy + (fAscent / 2), "W");

    // Center Hub
    int hubSize = max(3, (int)(r * 0.1f));
    canvas->drawBox((int)cx - (hubSize / 2), (int)cy - (hubSize / 2), hubSize, hubSize);

    // Ticks (30 degree increments)
    for (int angle = 30; angle < 360; angle += 30) {
        if (angle % 90 == 0) continue; 
        float rad = (angle - 90.0f) * (M_PI / 180.0f);
        float tLen = r * 0.15f;
        drawSafeLine(*canvas, cx + r * cosf(rad), cy + r * sinf(rad),
                              cx + (r - tLen) * cosf(rad), cy + (r - tLen) * sinf(rad));
    }

    // 2. DRAW HOME NEEDLE (The T-Bar)
    float hRad = (g.data.turn.homeHeading - 90.0f) * (M_PI / 180.0f);
    float hCos = cosf(hRad), hSin = sinf(hRad);
    float hTipX = cx + (r * 0.65f) * hCos;
    float hTipY = cy + (r * 0.65f) * hSin;
    
    // T-Bar Cap
    float capW = r * 0.15f;
    drawSafeLine(*canvas, hTipX + (-hSin * capW), hTipY + (hCos * capW), 
                          hTipX - (-hSin * capW), hTipY - (hCos * capW));
    // Stem
    drawSafeLine(*canvas, cx, cy, hTipX, hTipY);

    // 3. DRAW HEADING NEEDLE (The Parallel Rails + Solid Mask)
    float nRad = (g.data.turn.heading - 90.0f) * (M_PI / 180.0f);
    float nCos = cosf(nRad), nSin = sinf(nRad);
    float width = r * 0.10f;
    float bodyR = r * 0.60f;
    float tipR = r * 0.75f;

    // A. THE MASK: Draw a solid black triangle/polygon underneath
    // This "erases" the home needle stem before we draw the green rails
    canvas->setDrawColor(0); // Set color to BLACK (Background)
    
    // We create a simple triangle from center to the rails and tip
    float offX = -nSin * width, offY = nCos * width; 
    canvas->drawTriangle(cx + offX, cy + offY, 
                         cx - offX, cy - offY, 
                         cx + tipR * nCos, cy + tipR * nSin);
    
    canvas->setDrawColor(1); // Set color back to WHITE/GREEN for the rails

    // B. THE RAILS (Existing logic)
    drawSafeLine(*canvas, cx + offX, cy + offY, cx + offX + bodyR * nCos, cy + offY + bodyR * nSin);
    drawSafeLine(*canvas, cx - offX, cy - offY, cx - offX + bodyR * nCos, cy - offY + bodyR * nSin);
    
    // C. ARROW HEAD (Existing logic)
    float tipX = cx + tipR * nCos, tipY = cy + tipR * nSin;
    drawSafeLine(*canvas, cx + offX + bodyR * nCos, cy + offY + bodyR * nSin, tipX, tipY);
    drawSafeLine(*canvas, cx - offX + bodyR * nCos, cy - offY + bodyR * nSin, tipX, tipY);
}


void drawMissionClock(Gauge &g) {
  if (g.screen != SCREEN_OLED_A && g.screen != SCREEN_OLED_B) return;
  U8G2 *canvas = (U8G2*)g.displayPtr;

  float cx = (float)g.x;
  float cy = (float)g.y;
  float r  = (float)g.r;
  uint32_t msTotal = g.data.clock.flightTime;

  canvas->setFont(u8g2_font_5x7_tr);
  
  // 1. LABELS (Your specific 10-7-5-2 layout)
  canvas->drawStr(cx - 5, cy - 10, "10"); 
  canvas->drawStr(cx - 3, cy + 16, "5");
  canvas->drawStr(cx + 11, cy + 3,  "2"); 
  canvas->drawStr(cx - 18, cy + 3,  "7");

  // 2. CLEAN DIAL DOTS (10-position increments)
  for (int i = 0; i < 360; i += 36) { 
    if (i == 0 || i == 90 || i == 180 || i == 270) continue;
    float rad = (i - 90.0f) * (M_PI / 180.0f);
    canvas->drawPixel((int)(cx + r * cosf(rad)), (int)(cy + r * sinf(rad)));
  }

  // 3. MISSION MINUTES HAND (10 Min per Lap)
  float minMS = (float)(msTotal % 600000);
  float minRad = (minMS * 0.0006f - 90.0f) * (M_PI / 180.0f);
  float minTipX = cx + (r * 0.65f) * cosf(minRad);
  float minTipY = cy + (r * 0.65f) * sinf(minRad);
  
  // Hand Thickness Logic
  float mPX = -sinf(minRad), mPY = cosf(minRad);
  for (float i = -0.5f; i <= 0.5f; i += 1.0f) {
    drawSafeLine(*canvas, cx + (mPX * i), cy + (mPY * i), 
                          minTipX + (mPX * i), minTipY + (mPY * i));
  }

  // 4. BALANCED SECONDS NEEDLE (1 Min per Lap - 10Hz Beat)
  uint32_t smoothSecMs = (msTotal / 100) * 100; 
  float secMS = (float)(smoothSecMs % 60000);
  float secRad = (secMS * 0.006f - 90.0f) * (M_PI / 180.0f);
  
  // Tip and Tail
  float secTipX = cx + (r - 1.0f) * cosf(secRad);
  float secTipY = cy + (r - 1.0f) * sinf(secRad);
  float secTailX = cx - (4.0f * cosf(secRad));
  float secTailY = cy - (4.0f * sinf(secRad));
  
  drawSafeLine(*canvas, secTailX, secTailY, secTipX, secTipY);

  // 5. CENTER HUB
  canvas->drawBox((int)cx - 1, (int)cy - 1, 3, 3);
}

void drawTacho(Gauge &g) {
    if (g.screen != SCREEN_OLED_B) return;
    U8G2 *canvas = (U8G2*)g.displayPtr;

    float cx = (float)g.x, cy = (float)g.y, r = (float)g.r;
    float rpmPct = constrain(g.data.tacho.rotationsPerMinute / g.data.tacho.maxScaleRotationsPerMinute, 0.0f, 1.0f);

    canvas->setFont(u8g2_font_u8glib_4_tf); 

    // 1. TICKS (Start at 210° [7:00] and ADD to move Clockwise)
    // 277.5 degree sweep takes us over the top to 5:45 (127.5° on the circle)
    float sweep = 277.5f;
    for (int i = 0; i <= 10; i++) {
        float rad = (210.0f + (i * (sweep / 10.0f)) - 90.0f) * (M_PI / 180.0f);
        float rIn = (i % 2 == 0) ? (r * 0.78f) : (r * 0.87f); 
        
        canvas->drawLine(cx + r * cosf(rad), cy + r * sinf(rad), cx + rIn * cosf(rad), cy + rIn * sinf(rad));

        if (i % 2 != 0) {
            char buf[4];
            sprintf(buf, "%d", (int)((g.data.tacho.maxScaleRotationsPerMinute / 10.0f) * i / 100.0f));
            float rText = r * 0.50f;
            canvas->drawStr((cx + rText * cosf(rad)) - (canvas->getStrWidth(buf) / 2), (cy + rText * sinf(rad)) + 2, buf);
        }
    }

    // 2. NEEDLE
    float nRad = (210.0f + (rpmPct * sweep) - 90.0f) * (M_PI / 180.0f);
    float tipX = cx + (r * 0.75f) * cosf(nRad), tipY = cy + (r * 0.75f) * sinf(nRad);
    float bw = max(1.0f, r * 0.08f); 

    drawSafeLine(*canvas, cx, cy, tipX, tipY); 
    drawSafeLine(*canvas, cx + bw * cosf(nRad + 1.57f), cy + bw * sinf(nRad + 1.57f), tipX, tipY);
    drawSafeLine(*canvas, cx + bw * cosf(nRad - 1.57f), cy + bw * sinf(nRad - 1.57f), tipX, tipY);

    canvas->drawDisc(cx, cy, r * 0.12f);
}

void drawFuel(Gauge &g) {
    if (g.screen != SCREEN_OLED_B) return;
    U8G2 *canvas = (U8G2*)g.displayPtr;

    float cx = (float)g.x, cy = (float)g.y, r = (float)g.r;
    float totalVolts = g.data.fuel.batteryVoltage;

    // 1. CELL DETECTION LOGIC
    // Detects 3S, 4S, 6S etc. (4.2V is max per cell)
    int cells = (int)((totalVolts / 4.2f) + 0.5f); 
    if (cells < 1) cells = 1; // Safety fallback

    float voltsPerCell = totalVolts / (float)cells;
    
    // Standard LiPo range: 3.5V (Empty) to 4.2V (Full)
    float pct = (voltsPerCell - 3.5f) / (4.2f - 3.5f) * 100.0f;
    pct = constrain(pct, 0.0f, 100.0f);

    // 2. TICKS & LABELS (Proven 210 -> -30 sweep)
    canvas->setFont(u8g2_font_5x7_tr);
    const char* labels[] = {"E", "", "1/2", "", "F"};
    for (int i = 0; i <= 4; i++) {
        float rad = (210.0f - (i * 60.0f) - 90.0f) * (M_PI / 180.0f);
        canvas->drawLine(cx + r * cosf(rad), cy + r * sinf(rad), 
                         cx + (r * 0.826f) * cosf(rad), cy + (r * 0.826f) * sinf(rad));
        if (labels[i][0] != '\0') {
            float rText = r * 0.435f;
            canvas->drawStr((int)(cx + rText * cosf(rad)) - ((i==2)?7:3), (int)(cy + rText * sinf(rad)) + 3, labels[i]);
        }
    }

    // 3. THE NEEDLE
    float fuelRad = (210.0f - ((pct / 100.0f) * 240.0f) - 90.0f) * (M_PI / 180.0f);
    float tipX = cx + (r * 0.739f) * cosf(fuelRad), tipY = cy + (r * 0.739f) * sinf(fuelRad);
    
    canvas->drawLine((int)cx, (int)cy, (int)tipX, (int)tipY);
    canvas->drawLine((int)(cx + cosf(fuelRad + 1.57f)), (int)(cy + sinf(fuelRad + 1.57f)), (int)tipX, (int)tipY);
    canvas->drawLine((int)(cx + cosf(fuelRad - 1.57f)), (int)(cy + sinf(fuelRad - 1.57f)), (int)tipX, (int)tipY);
    canvas->drawDisc((int)cx, (int)cy, (int)max(2.0f, r * 0.12f));
}


void updateCompassAndClockDisplay() {
  static float visualHeading = 0.0f;
  static float visualHome = 0.0f;
  const float alpha = 0.25f; // Smoothing factor

  // 1. Smooth the Heading
  float deltaH = (float)compassGauge.data.compass.heading - visualHeading;
  if (deltaH > 180.0f)  deltaH -= 360.0f;
  if (deltaH < -180.0f) deltaH += 360.0f;
  visualHeading += deltaH * alpha;

  // 2. Smooth the Home Bearing
  float deltaHome = (float)compassGauge.data.compass.homeHeading - visualHome;
  if (deltaHome > 180.0f)  deltaHome -= 360.0f;
  if (deltaHome < -180.0f) deltaHome += 360.0f;
  visualHome += deltaHome * alpha;

  // 3. Update the struct with "Visual" (Smoothed) values for the drawing function
  compassGauge.data.compass.heading = (int)visualHeading;
  compassGauge.data.compass.homeHeading = (int)visualHome;

  // 4. Render to OLED A (0x3C)
  u8g2_CompassAndClock.clearBuffer();
  
  drawCompass(compassGauge);      // The parametric function we wrote
  drawMissionClock(clockGauge);   // The analog 10-min lap clock
  
  u8g2_CompassAndClock.sendBuffer(); 
}

void updateEngineDisplay() {
  static float visualRpm = 0.0f;
  static float visualFuel = 100.0f;
  const float motorKv = 580.0f; 
  const float alpha = 0.12f;

  // 1. Snapshot and Math (Processing Truth to Visual)
  // Now pulling from the Single Source of Truth: aircraft
  float targetRpm = motorKv * aircraft.vBat * aircraft.throttle; 
  
  // Smoothing for the needles
  visualRpm += (targetRpm - visualRpm) * alpha;
  visualFuel += (aircraft.fuelPercent - visualFuel) * alpha;

  // 2. Feed the smoothed data into the Gauge Structs
  tachoGauge.data.tacho.rotationsPerMinute = visualRpm;
  
  // Dynamic scale: If voltage > 18V (6S/High 4S), use 16k scale, otherwise 10k
  tachoGauge.data.tacho.maxScaleRotationsPerMinute = (aircraft.vBat > 18.0f) ? 16000.0f : 10000.0f;
  
  // Note: We pass the smoothed percentage into the fuel gauge
  fuelGauge.data.fuel.batteryVoltage = visualFuel; 

  // 3. Render to the OLED
  u8g2_Engine.clearBuffer();
  drawTacho(tachoGauge);
  drawFuel(fuelGauge);
  u8g2_Engine.sendBuffer();
}


void updateClock() {
  // --- MISSION CLOCK LOGIC ---
  if (aircraft.isArmed) {
    if (!aircraft.lastArmState) {
      // START THE CLOCK
      aircraft.armStartTime = millis();
      aircraft.lastArmState = true;
    }
    // Calculate raw milliseconds elapsed
    aircraft.msElapsed = millis() - aircraft.armStartTime;
  } else {
    if (aircraft.lastArmState) {
      // STOP THE CLOCK
      aircraft.finalMsElapsed = aircraft.msElapsed;
      aircraft.lastArmState = false;
    }
  }
  // --- SYNC GAUGE DATA ---
  // We feed the gauge raw Milliseconds now
  clockGauge.data.clock.flightTime = (aircraft.isArmed) ? aircraft.msElapsed : aircraft.finalMsElapsed;

}

void oled_MasterTask(void * pvParameters) {
  for(;;) {
      // Step 1: Compass & Clock (0x3C)
      if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(10)) == pdPASS) {
          updateCompassAndClockDisplay(); // This will call drawCompass(gCompass)
          xSemaphoreGive(i2cMutex);
      }
      
      vTaskDelay(pdMS_TO_TICKS(5)); 

      // Step 2: Engine Vitals (0x3D)
      if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(10)) == pdPASS) {
          updateEngineDisplay(); // This will call drawFuel(gFuel), etc.
          xSemaphoreGive(i2cMutex);
      }

      // Step 3: Loop Frequency (approx 20-25Hz)
      vTaskDelay(pdMS_TO_TICKS(35)); 
  }
}

void loop() {
  // --- DATA ACQUISITION ---
  if (aircraft.isBenchMode) {
    // Simulation mode using the Source of Truth
    float t = millis() / 1000.0;
    
    aircraft.airSpeed = 45.0 + sin(t * 0.5) * 35.0;
    aircraft.altitude = 225.0 + (sin(t * 0.2) * 225.0);
    aircraft.roll     = sin(t) * 35.0;
    aircraft.pitch    = cos(t * 0.7) * 10.0;
    aircraft.vsi      = cos(t * 0.2) * 400.0; // Simulated cm/s
    
    aircraft.heading += 0.2;
    if (aircraft.heading >= 360.0) aircraft.heading = 0;
    // Simulate the plane flying in a circle around 'Home'
    aircraft.distToHome = 150 + (sin(t * 0.1) * 50); // Pulse distance between 100m and 200m
    aircraft.dirToHome = (int)(t * 10) % 360;       // Rotate the home bearing 10 degrees per second  
    aircraft.vBat = 16.8; // Constant 4S voltage for bench testing
    
    // Throttle cycling 0-100%
    aircraft.throttle = (sin(t * 0.5) * 0.5) + 0.5; 
    
    // Fuel draining 100->0 over 60 seconds
    aircraft.fuelPercent = 100.0 - (fmod(t, 60.0) * 1.66);
    
  } else {
    // Live mode: updateMSP calls parseMSP which now writes to 'aircraft'
    updateMSP(); 
  }

  // --- SYNC GAUGE DATA FROM SOURCE OF TRUTH ---
  
  // Navigation & System
  compassGauge.data.compass.heading     = (int)aircraft.heading;
  compassGauge.data.compass.homeHeading = (int)aircraft.dirToHome;

  // Column 1 (Left) - Airspeed & Altimeter
  airspeedGauge.data.airspeed.mph        = aircraft.airSpeed;
  altimeterGauge.data.altimeter.altitude = aircraft.altitude;

  // Column 2 (Center) - Turn, Bank & Gear
  turnGauge.data.turn.heading           = (int)aircraft.heading;
  turnGauge.data.turn.homeHeading       = (int)aircraft.dirToHome;
  bankGauge.data.bank.roll              = aircraft.roll;

  // Column 3 (Right) - Horizon & VSI
  horizonGauge.data.horizon.roll        = aircraft.roll;
  horizonGauge.data.horizon.pitch       = aircraft.pitch;
  
  // VSI: MSP gives cm/s. We divide by 100 to get m/s for the needle logic
  vsiGauge.data.vsi.verticalSpeed       = aircraft.vsi / 100.0f; 

  // Engine Display (OLED)
  // Logic: motorKv * current battery voltage * throttle percentage
  tachoGauge.data.tacho.rotationsPerMinute = (580.0f * aircraft.vBat * aircraft.throttle);
  fuelGauge.data.fuel.batteryVoltage       = aircraft.fuelPercent;

  updateGearPhysics();
  updateClock();

  // --- RENDER CALLS ---
  // Note: These draw functions now look at the updated .data members
  drawAirspeed(airspeedGauge);
  drawAltimeter(altimeterGauge);
  drawTurn(turnGauge);
  drawBank(bankGauge);
  drawGearStatus(gearGauge);
  drawPoshHorizon(horizonGauge);
  drawVSI(vsiGauge);

  // Background house-keeping
  yield(); 
}
