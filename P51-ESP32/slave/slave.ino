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

enum GaugeType { P51_HORIZON, P51_ALTIMETER, P51_AIRSPEED, P51_GYRO };

struct Instrument {
  const char* label;
  GaugeType type;
  float val1;      // Roll (Horizon) or Feet (Altimeter)
  float val2;      // Pitch (Horizon)
  int x, y, r;     
  U8G2* screen;    
};

// Update your struct to include the Gyro
struct CockpitLayout {
  Instrument horizon;   // Screen 1 (Right side of landscape)
  Instrument gyro;      // Screen 1 (Left side of landscape)
  Instrument altimeter; // Screen 2 (Top of portrait)
  Instrument airspeed;  // Screen 2 (Bottom of portrait)
};

CockpitLayout cockpit = {
  // Label, Type, Val1, Val2, X, Y, Radius, Screen
  {"HORIZON",   P51_HORIZON,   0.0, 0.0, 102, 32, 25, &u8g2e1}, // Nudged Right, R to 25
  {"GYRO",      P51_GYRO,      0.0, 0.0, 20, 34, 20, &u8g2e1},  // Far Left (X20) and Down (Y34)
  {"ALTIMETER", P51_ALTIMETER, 0.0, 0.0, 32, 22, 26, &u8g2e2}, 
  {"AIRSPEED",  P51_AIRSPEED,  0.0, 0.0, 32, 99, 26, &u8g2e2}
};


float roll = 0.0, pitch = 0.0, heading = 0.0, altitude = 0.0, vBat = 0.0, currentG = 0.0, baro = 29.9, airSpeed = 0;
bool warActive = false;

// --- DRAWING FUNCTIONS ---

void drawHorizon(Instrument &inst) {
  U8G2* dev = inst.screen;
  
  // 1. MATH NUDGE (Essential for stability at 180 deg)
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

  // 4. MOVING HORIZON BAR (2-Pixel Thickness)
  float cosR = cos(rollRad);
  float sinR = sin(rollRad);
  float xL = 45.0 * cosR;
  float yL = 45.0 * sinR;

  // Draw two lines slightly offset for thickness
  for (int i = 0; i <= 1; i++) {
    int x1 = inst.x - xL;
    int y1 = inst.y - yL + pitchOff + i;
    int x2 = inst.x + xL;
    int y2 = inst.y + yL + pitchOff + i;
    dev->drawLine(constrain(x1, 0, 127), constrain(y1, 0, 63),
                  constrain(x2, 0, 127), constrain(y2, 0, 63));
  }

  // 5. THE TRIDENT PEDESTAL (Double-Thick)
  // Vertical Stem (2 pixels wide)
  dev->drawBox(inst.x - 1, inst.y + 11, 2, 18); 
  
  // Left Arm (Double-strike)
  dev->drawLine(inst.x, inst.y + 13, inst.x - 7, inst.y + 9);
  dev->drawLine(inst.x, inst.y + 14, inst.x - 7, inst.y + 10);
  dev->drawBox(inst.x - 10, inst.y + 9, 3, 2); // Thicker horizontal tip
  
  // Right Arm (Double-strike)
  dev->drawLine(inst.x, inst.y + 13, inst.x + 7, inst.y + 9);
  dev->drawLine(inst.x, inst.y + 14, inst.x + 7, inst.y + 10);
  dev->drawBox(inst.x + 7, inst.y + 9, 3, 2); // Thicker horizontal tip

  // 6. FIXED LEVEL PIPS (Static Markers)
  dev->drawHLine(inst.x - inst.r, inst.y, 4);
  dev->drawHLine(inst.x + inst.r - 4, inst.y, 4);

  // 7. CENTER REFERENCE DOT
  dev->drawDisc(inst.x, inst.y, 2); 

  // 8. TOP POINTER (Triangle at 12 o'clock)
  dev->drawTriangle(inst.x, inst.y - 14, inst.x - 3, inst.y - 10, inst.x + 3, inst.y - 10);
}

void drawDirectionalGyro(Instrument &inst) {
  U8G2* dev = inst.screen;
  float heading = inst.val1; // 0-359
  int cx = 31; // Center of the left half (approx 62/2)
  int cy = 20; // Matches the vertical center of your horizon hole
  
  // 1. DRAW THE WINDOW FRAME
  dev->setDrawColor(1);
  dev->drawFrame(cx - 20, cy - 7, 40, 14); // The rectangular cutout
  
  // 2. DRAW THE ROTATING SCALE
  dev->setFont(u8g2_font_04b_03_tr);
  
  // We draw ticks for every 5 degrees, and numbers for every 30
  // To make it "scroll," we offset based on the heading
  for (int a = -40; a <= 40; a += 5) {
    // Calculate the actual compass degree for this tick
    int degree = (int)(heading + a + 360) % 360;
    
    // Convert the 'a' offset to pixel X position
    // Scale: 1 degree = 1.2 pixels (spreads 30 degrees over 36 pixels)
    int xPos = cx + (a * 1.2);
    
    if (xPos > cx - 18 && xPos < cx + 18) {
      if (degree % 10 == 0) {
        // Major Tick
        dev->drawVLine(xPos, cy - 4, 3);
        
        // Numbers every 30 degrees (3, 6, 9... 33, 0)
        if (degree % 30 == 0) {
          char buf[3];
          int val = degree / 10;
          itoa(val, buf, 10);
          dev->drawStr(xPos - 3, cy + 6, buf);
        }
      } else {
        // Minor Tick
        dev->drawVLine(xPos, cy - 2, 2);
      }
    }
  }

  // 3. THE LUBBER LINE (The fixed reference pointer)
  dev->setDrawColor(1);
  dev->drawVLine(cx, cy - 9, 3); // Top pointer
  dev->drawVLine(cx, cy + 6, 3); // Bottom pointer
  
  // 4. TEXT LABELS (Historical "Directional Gyro" markings)
  dev->setFont(u8g2_font_04b_03_tr);
  dev->drawStr(cx - 20, cy + 18, "DIR. GYRO");
  dev->drawStr(cx - 20, cy + 26, "AN 5735-1A");
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

void readMasterDataAndUpdateTelem() {
  while (fromMaster.available() > 0) {
    String tag = fromMaster.readStringUntil(':'); 
    tag.trim(); 
    if (tag.length() == 0) continue;

    float value = fromMaster.parseFloat();
    dataChanged = true; 
    
    // Direct assignment to the cockpit struct for neatness
    if (tag == "ROL")      cockpit.horizon.val1 = value;
    else if (tag == "PIT") cockpit.horizon.val2 = value;
    else if (tag == "HED") cockpit.gyro.val1 = value;
    else if (tag == "ALT") cockpit.altimeter.val1 = value;
    else if (tag == "BAR") cockpit.altimeter.val2 = value;
    else if (tag == "SPD") cockpit.airspeed.val1 = value;
    else if (tag == "BAT") vBat = value;
    else if (tag == "GFO") currentG = value;
    else if (tag == "WAR") warActive = (value > 0.5);
    
    // Garbage Disposal
    while (fromMaster.available() > 0 && 
          (fromMaster.peek() == ',' || fromMaster.peek() == '\n' || fromMaster.peek() == '\r')) {
      fromMaster.read();
    }
  }
}

void updateScreens() {
  // --- SCREEN 1: LANDSCAPE (Directional Gyro & Horizon) ---
  u8g2e1.clearBuffer();
  drawDirectionalGyro(cockpit.gyro); // Positioned at X=34
  drawHorizon(cockpit.horizon);      // Positioned at X=94
  u8g2e1.sendBuffer();

  // --- SCREEN 2: PORTRAIT (Altimeter & Airspeed) ---
  u8g2e2.clearBuffer();
  drawAltimeter(cockpit.altimeter);  // Positioned at Y=22
  drawAirspeed(cockpit.airspeed);    // Positioned at Y=99
  u8g2e2.sendBuffer();
}

void loop() {
  readMasterDataAndUpdateTelem(); 

  if (dataChanged) {
    updateScreens();
    dataChanged = false; 
  }
}