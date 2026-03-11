#include <Arduino.h>
#include <U8g2lib.h>
#include <Wire.h>

auto& console = Serial;     // USB Port to your PC (Serial Monitor)
auto& fromMaster = Serial1;   // Hardware pins 20/21 receiving from Master

// --- PIN CONFIGURATION ---
const int BUILT_IN_OLED_SCL = 6;
const int BUILT_IN_OLED_SDA = 5;     
const int EXT_OLED_SCL = 10;
const int EXT_OLED_SDA = 7;     

// --- DISPLAY CONSTRUCTORS ---
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2e1(U8G2_R0, U8X8_PIN_NONE, EXT_OLED_SCL, EXT_OLED_SDA);
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2e2(U8G2_R1, U8X8_PIN_NONE, EXT_OLED_SCL, EXT_OLED_SDA);
U8G2_SSD1306_72X40_ER_F_SW_I2C u8g2bi(U8G2_R0, U8X8_PIN_NONE, BUILT_IN_OLED_SCL, BUILT_IN_OLED_SDA);

enum GaugeType { P51_HORIZON, P51_ALTIMETER, P51_AIRSPEED };

struct Instrument {
  const char* label;
  GaugeType type;
  float val1;      // Roll (Horizon) or Feet (Altimeter)
  float val2;      // Pitch (Horizon)
  int x, y, r;     
  U8G2* screen;    
};

struct CockpitLayout {
  Instrument horizon;
  Instrument altimeter;
  Instrument airspeed;
};

// Label, Type, Val1, Val2, X, Y, Radius, Screen
CockpitLayout cockpit = {
  {"HORIZON",   P51_HORIZON,   0.0, 0.0, 64, 32, 30, &u8g2e1},
  {"ALTIMETER", P51_ALTIMETER, 0.0, 0.0, 32, 22, 26, &u8g2e2}, // TOP
  {"AIRSPEED",  P51_AIRSPEED,  0.0, 0.0, 32, 99, 26, &u8g2e2}  // BOTTOM (LIFTED)
};

float roll = 0.0, pitch = 0.0, heading = 0.0, altitude = 0.0, vBat = 0.0, currentG = 0.0, baro = 29.9, airSpeed = 0;
bool warActive = false;

// --- DRAWING FUNCTIONS ---

void drawHorizon(Instrument &inst) {
  U8G2* dev = inst.screen;
  
  // 1. MATH NUDGE (Essential for 180 deg stability)
  float rollDeg = inst.val1;
  if (fmod(abs(rollDeg), 90.0) < 0.1) rollDeg += 0.05; 
  float rollRad = rollDeg * (PI / 180.0);

  // 2. PITCH MAPPING
  int pitchOff = map(constrain(inst.val2, -30, 30), -30, 30, -20, 20);

  // 3. BANK SCALE (Internal Ticks)
  for (int a = -60; a <= 60; a += 10) {
    float rad = (a - 90) * (PI / 180.0);
    int rOut = inst.r - 1;
    int rIn = (a % 30 == 0) ? inst.r - 7 : inst.r - 4;
    dev->drawLine(inst.x + rIn * cos(rad), inst.y + rIn * sin(rad),
                  inst.x + rOut * cos(rad), inst.y + rOut * sin(rad));
  }

  // 4. MOVING HORIZON BAR
  float cosR = cos(rollRad);
  float sinR = sin(rollRad);
  
  float xL = 45.0 * cosR;
  float yL = 45.0 * sinR;

  int x1 = inst.x - xL;
  int y1 = inst.y - yL + pitchOff;
  int x2 = inst.x + xL;
  int y2 = inst.y + yL + pitchOff;

  // Draw the bar with coordinate safety to prevent the "corner-stick"
  dev->drawLine(
    constrain(x1, 0, 127), constrain(y1, 0, 63),
    constrain(x2, 0, 127), constrain(y2, 0, 63)
  );

  // 5. THE TRIDENT PEDESTAL (Replacing Mushroom/W)
  // Vertical Stem
  dev->drawVLine(inst.x, inst.y + 11, 18); 
  
  // The Curved Cradle Arms
  // Left Arm
  dev->drawLine(inst.x, inst.y + 13, inst.x - 7, inst.y + 9);
  dev->drawHLine(inst.x - 10, inst.y + 9, 3);
  
  // Right Arm
  dev->drawLine(inst.x, inst.y + 13, inst.x + 7, inst.y + 9);
  dev->drawHLine(inst.x + 7, inst.y + 9, 3);

  // 6. FIXED LEVEL PIPS (The 3 and 9 o'clock markers)
  dev->drawHLine(inst.x - inst.r, inst.y, 4);
  dev->drawHLine(inst.x + inst.r - 4, inst.y, 4);

  // 7. CENTER REFERENCE DOT (The "Nose")
  dev->drawDisc(inst.x, inst.y, 2); 

  // 8. TOP POINTER (The triangle at 12 o'clock)
  dev->drawTriangle(inst.x, inst.y - 14, inst.x - 3, inst.y - 10, inst.x + 3, inst.y - 10);
}

void drawAltimeter(Instrument &inst) {
  U8G2* dev = inst.screen;
  float alt = (inst.val1 < 0) ? 0 : inst.val1; // Clamp negative altitude
  int cx = inst.x; int cy = inst.y; int r = inst.r;
  
  // SCALED RADIUS DEFINITIONS
  int tickInnerR = r - 4;       
  int numberR = r - 9;          
  int innerDelineator = r - 14; 

  // 1. THE TICK-ZONE SHADING
  for (int y = -r; y <= r; y++) {
    for (int x = -r; x <= r; x++) {
      int distSq = x*x + y*y;
      if (distSq < (r*r) && distSq > (tickInnerR * tickInnerR)) {
        if ((x % 3 == 0) && (y % 3 == 0)) dev->drawPixel(cx + x, cy + y);
      }
    }
  }

  // 2. THE INNER BORDERS
  dev->drawCircle(cx, cy, tickInnerR);      
  dev->drawCircle(cx, cy, innerDelineator); 

  // 3. TICKS AND NUMBERS
  dev->setFont(u8g2_font_04b_03_tr);
  for (int i = 0; i < 10; i++) {
    float angle = (i * 36 - 90) * (PI / 180.0);
    dev->drawLine(cx + tickInnerR*cos(angle), cy + tickInnerR*sin(angle), 
                  cx + r*cos(angle), cy + r*sin(angle));
    
    char label[4];
    itoa(i * 5, label, 10); 
    int tx = cx + numberR*cos(angle) - (i * 5 >= 10 ? 4 : 2); 
    int ty = cy + numberR*sin(angle) + 3;
    dev->drawStr(tx, ty, label);
  }

  // --- 3.5 KOLLSMAN WINDOW ---
  int kx = cx + 6; int ky = cy - 4;  
  dev->setDrawColor(0); dev->drawBox(kx, ky, 14, 8);   
  dev->setDrawColor(1); dev->drawFrame(kx, ky, 14, 8); 
  char baroBuf[6]; dtostrf(inst.val2, 4, 1, baroBuf);
  dev->drawStr(kx + 1, ky + 6, baroBuf);

  // 4. THE HANDS

  // A. Main Precision Hand (Beefy Sword)
  // One rotation = 100 feet. 
  float a100 = (fmod(alt, 100.0) * 3.6) - 90.0;
  // Safety: Nudge 12 o'clock slightly to avoid "perfect" vertical math errors
  if (abs(fmod(a100, 360.0) + 90.0) < 0.1) a100 += 0.05; 
  
  float rad100 = a100 * (PI / 180.0);
  
  // Calculate and Clamping coordinates
  auto clampX = [](int x) { return constrain(x, 0, 127); };
  auto clampY = [](int y) { return constrain(y, 0, 63); };

  int tipX = clampX(cx + (r - 1) * cos(rad100));
  int tipY = clampY(cy + (r - 1) * sin(rad100));
  
  float baseWidth = 0.40; 
  int b1x = clampX(cx + (4.0 * cos(rad100 + baseWidth))); 
  int b1y = clampY(cy + (4.0 * sin(rad100 + baseWidth)));
  int b2x = clampX(cx + (4.0 * cos(rad100 - baseWidth))); 
  int b2y = clampY(cy + (4.0 * sin(rad100 - baseWidth)));
  
  dev->drawLine(cx, cy, tipX, tipY);
  dev->drawTriangle(tipX, tipY, b1x, b1y, b2x, b2y);

  // B. Thousands Hand (Thin Sweep)
  float a1k = (alt * 0.36) - 90.0; 
  float rad1k = a1k * (PI / 180.0);
  dev->drawLine(cx, cy, clampX(cx + (innerDelineator-2)*cos(rad1k)), clampY(cy + (innerDelineator-2)*sin(rad1k)));

  // 5. CENTER HUB
  dev->setDrawColor(0); dev->drawDisc(cx, cy, 2);
  dev->setDrawColor(1); dev->drawCircle(cx, cy, 2);
}

void drawAirspeed(Instrument &inst) {
  U8G2* dev = inst.screen;
  float speed = inst.val1; 
  int cx = inst.x; int cy = inst.y; int r = inst.r; 
  
  int tickInnerR = r - 4;
  int numberR = r - 11;      
  int innerDelineator = r - 14;

  // 1. LINEAR MAPPING (0 to 120 mph)
  // 12:30 start (-60 deg), sweeping 330 degrees around to ~11:30
  auto getSpeedAngle = [](float s) -> float {
    float startPos = -60.0; 
    float totalSweep = 330.0;
    float maxSpeed = 120.0;
    
    // Linear scale: every mph gets the same amount of "arc"
    float angle = startPos + (s / maxSpeed) * totalSweep;
    return angle * (PI / 180.0);
  };

  // 2. DRAW TICKS
  // Major ticks every 10 mph, minor ticks every 2 mph
  for (int s = 0; s <= 120; s += 2) {
    float a = getSpeedAngle((float)s);
    int tLen = (s % 10 == 0) ? 0 : 3; // 10s are full length, others are short
    dev->drawLine(cx + (tickInnerR + tLen)*cos(a), cy + (tickInnerR + tLen)*sin(a), 
                  cx + r*cos(a), cy + r*sin(a));
  }

  // 3. DRAW LABELS (Clean 10-mph increments)
  dev->setFont(u8g2_font_04b_03_tr);
  for (int s = 0; s <= 120; s += 20) { // Labels every 20mph to keep it clean
    float a = getSpeedAngle((float)s);
    char buf[4];
    itoa(s, buf, 10);
    
    int tx = cx + numberR*cos(a) - (s >= 100 ? 5 : 3); 
    int ty = cy + numberR*sin(a) + 3;
    
    // Nudge the start/end to avoid the gap
    if (s == 0)   { tx += 2; ty -= 1; }
    if (s == 120) { tx -= 3; ty -= 2; }

    dev->drawStr(tx, ty, buf);
  }

  // 4. THE BEEFY SWORD HAND
  // Draw the needle if we have any speed
  if (speed >= 2.0) { 
    float mAng = getSpeedAngle(speed);
    int tipX = cx + (r - 1) * cos(mAng);
    int tipY = cy + (r - 1) * sin(mAng);
    
    float baseWidth = 0.40; 
    float baseDist = 4.0;
    
    int b1x = cx + baseDist * cos(mAng + baseWidth);
    int b1y = cy + baseDist * sin(mAng + baseWidth);
    int b2x = cx + baseDist * cos(mAng - baseWidth);
    int b2y = cy + baseDist * sin(mAng - baseWidth);
    
    dev->drawTriangle(tipX, tipY, b1x, b1y, b2x, b2y);
    dev->drawLine(cx, cy, tipX, tipY); 
  }

  // 5. THE HUB
  dev->setDrawColor(0); dev->drawDisc(cx, cy, 3);
  dev->setDrawColor(1); dev->drawCircle(cx, cy, 3);
}

void setup() {
  Wire.begin(EXT_OLED_SDA, EXT_OLED_SCL);
  Wire.setClock(400000);
  console.begin(115200);  
  fromMaster.begin(115200, SERIAL_8N1, 20, 21);

  u8g2e1.setI2CAddress(0x3C * 2); 
  u8g2e2.setI2CAddress(0x3D * 2);
  
  u8g2bi.begin();
  u8g2e1.begin();
  u8g2e2.begin();

  u8g2bi.setFont(u8g2_font_04b_03_tr);
  u8g2bi.clearBuffer();
  u8g2bi.drawStr(0, 15, "SYSTEMS DUAL-LINK");
  u8g2bi.sendBuffer();
}

bool dataChanged = false; // The "Dirty Flag"

void readMasterData() {
  while (fromMaster.available() > 0) {
    String tag = fromMaster.readStringUntil(':'); 
    tag.trim(); 
    if (tag.length() == 0) continue;

    float value = fromMaster.parseFloat();
    dataChanged = true; // Flag that we have fresh intel
    
    if (tag == "ROL") roll = value;
    else if (tag == "PIT") pitch = value;
    else if (tag == "HED") heading = value;
    else if (tag == "ALT") altitude = value;
    else if (tag == "SPD") airSpeed = value;
    else if (tag == "BAT") vBat = value;
    else if (tag == "GFO") currentG = value;
    else if (tag == "BAR") baro = value;
    else if (tag == "WAR") warActive = (value > 0.5);
    
    // Garbage Disposal
    while (fromMaster.available() > 0 && 
          (fromMaster.peek() == ',' || fromMaster.peek() == '\n' || fromMaster.peek() == '\r')) {
      fromMaster.read();
    }
  }
}

void loop() {
  readMasterData(); 

  if (dataChanged) {
    // 1. HORIZON (Landscape)
    u8g2e1.clearBuffer();
    cockpit.horizon.val1 = roll;
    cockpit.horizon.val2 = pitch;
    drawHorizon(cockpit.horizon);
    u8g2e1.sendBuffer();

    // 2. STACKED PORTRAIT (ASI + ALT)
    u8g2e2.clearBuffer();
    
    // Pass fresh data to the struct
    cockpit.airspeed.val1 = airSpeed; 
    cockpit.altimeter.val1 = altitude;
    cockpit.altimeter.val2 = baro;

    // Draw using the struct's X and Y (32, 22 and 32, 106)
    drawAirspeed(cockpit.airspeed);
    drawAltimeter(cockpit.altimeter);
    
    u8g2e2.sendBuffer();

    dataChanged = false; 
  }
}