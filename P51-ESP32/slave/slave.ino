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
  float rollRad = inst.val1 * (PI / 180.0);
  // Pitch mapping: adjust -15, 15 to change how "sensitive" the bar is
  int pitchOff = map(constrain(inst.val2, -30, 30), -30, 30, -15, 15);

  // 1. BANK SCALE (Internal Ticks)
  // Drawn from -60 to +60 degrees
  for (int a = -60; a <= 60; a += 10) {
    float rad = (a - 90) * (PI / 180.0);
    int rOut = inst.r - 1; // Flush with your stencil edge
    int rIn = (a % 30 == 0) ? inst.r - 7 : inst.r - 4; // Long ticks at 30/60
    dev->drawLine(inst.x + rIn * cos(rad), inst.y + rIn * sin(rad),
                  inst.x + rOut * cos(rad), inst.y + rOut * sin(rad));
  }

  // 2. MOVING HORIZON BAR
  // Long enough to stay visible behind the circular mask during rolls
  float cosR = cos(rollRad);
  float sinR = sin(rollRad);
  int xL = 38 * cosR;
  int yL = 38 * sinR;
  dev->drawLine(inst.x - xL, inst.y - yL + pitchOff, 
                inst.x + xL, inst.y + yL + pitchOff);

  // 3. THE "MUSHROOM" PEDESTAL (Bottom Mask)
  // Instead of a solid triangle, we use a narrower base to look like the real gauge
  dev->drawBox(inst.x - 2, inst.y + 10, 5, 22); // The "stem"
  dev->drawDisc(inst.x, inst.y + 10, 4, U8G2_DRAW_ALL); // The rounded top of the pedestal

  // 4. FIXED AIRCRAFT REFERENCE (The "W")
  // Drawing the yellow-tipped wings and center pointer
  int ay = inst.y + 8; // Vertical position of the reference
  dev->drawHLine(inst.x - 16, ay, 10); // Left wing
  dev->drawHLine(inst.x + 6, ay, 10);  // Right wing
  dev->drawVLine(inst.x, ay - 6, 4);   // The top center "needle"
  
  // Fixed "W" Aircraft Reference
  dev->drawHLine(inst.x - 15, inst.y + 5, 10);
  dev->drawHLine(inst.x + 5, inst.y + 5, 10);
  dev->drawFrame(inst.x - 2, inst.y + 3, 5, 5);
}

void drawAltimeter(Instrument &inst) {
  U8G2* dev = inst.screen;
  float alt = inst.val1;
  int cx = inst.x; int cy = inst.y; int r = inst.r;
  
  // SCALED RADIUS DEFINITIONS
  int tickInnerR = r - 4;       // Tightened for smaller dial
  int numberR = r - 9;          // Where the numbers are centered
  int innerDelineator = r - 14; // The final circle inside the numbers

  // 1. THE TICK-ZONE SHADING (10% Dither)
  for (int y = -r; y <= r; y++) {
    for (int x = -r; x <= r; x++) {
      int distSq = x*x + y*y;
      if (distSq < (r*r) && distSq > (tickInnerR * tickInnerR)) {
        if ((x % 3 == 0) && (y % 3 == 0)) { 
          dev->drawPixel(cx + x, cy + y);
        }
      }
    }
  }

  // 2. THE INNER BORDERS
  dev->drawCircle(cx, cy, tickInnerR);      // Line separating ticks from numbers
  dev->drawCircle(cx, cy, innerDelineator); // Line separating numbers from the center

  // 3. TICKS AND NUMBERS
  dev->setFont(u8g2_font_04b_03_tr);
  for (int i = 0; i < 10; i++) {
    float angle = (i * 36 - 90) * (PI / 180.0);
    
    // Ticks (drawn over the 10% shading)
    dev->drawLine(cx + tickInnerR*cos(angle), cy + tickInnerR*sin(angle), 
                  cx + r*cos(angle), cy + r*sin(angle));
    
    // Numbers (now in a clean black "Number Ring")
    char label[2]; 
    itoa(i, label, 10);
    int tx = cx + numberR*cos(angle) - 2; 
    int ty = cy + numberR*sin(angle) + 3;
    dev->drawStr(tx, ty, label);
  }

 // --- 3.5 KOLLSMAN WINDOW (Pressure Setting) ---
  // Positioned at 3 o'clock, inside the inner delineator
  int kx = cx + 6;  // Shifted right
  int ky = cy - 4;  // Centered vertically on the 3 o'clock line
  dev->setDrawColor(0);
  dev->drawBox(kx, ky, 14, 8);   
  dev->setDrawColor(1);
  dev->drawFrame(kx, ky, 14, 8); 
  dev->setFont(u8g2_font_04b_03_tr);

  char baroBuf[6];
  // Convert val2 (the baro float) to a string: 4 wide, 1 decimal place
  dtostrf(inst.val2, 4, 1, baroBuf);
  dev->drawStr(kx + 1, ky + 6, baroBuf);  // The baro setting

  // 4. THE HANDS (P51 Complex Style)
  
  // A. 10,000ft Hand
  float a10k = (fmod(alt, 100000.0) * 0.0036) - 90.0;
  float rad10k = a10k * (PI / 180.0);
  int x10k = cx + (innerDelineator-2)*cos(rad10k);
  int y10k = cy + (innerDelineator-2)*sin(rad10k);
  dev->drawLine(cx, cy, x10k, y10k); // simplified the "thick" line for smaller pixels
  dev->drawDisc(x10k, y10k, 2);

  // B. 1,000ft Hand (Tapered Wedge)
  float a1k = (fmod(alt, 10000.0) * 0.036) - 90.0;
  float rad1k = a1k * (PI / 180.0);
  int hx = cx + (innerDelineator+1)*cos(rad1k);
  int hy = cy + (innerDelineator+1)*sin(rad1k);
  dev->drawTriangle(hx, hy, 
                    cx + (innerDelineator-3)*cos(rad1k + 0.25), cy + (innerDelineator-3)*sin(rad1k + 0.25), 
                    cx + (innerDelineator-3)*cos(rad1k - 0.25), cy + (innerDelineator-3)*sin(rad1k - 0.25));
  dev->drawLine(cx, cy, hx, hy);

  // C. 100ft Hand (Long Sweep)
  float a100 = (fmod(alt, 1000.0) * 0.36) - 90.0;
  float rad100 = a100 * (PI / 180.0);
  dev->drawLine(cx, cy, cx + (r-3)*cos(rad100), cy + (r-3)*sin(rad100));

  // 5. CENTER HUB
  dev->setDrawColor(0); dev->drawDisc(cx, cy, 2);
  dev->setDrawColor(1); dev->drawCircle(cx, cy, 2);
}

void drawAirspeed(Instrument &inst) {
  U8G2* dev = inst.screen;
  float speed = inst.val1; 
  float targetVal = inst.val2;
  int cx = inst.x; int cy = inst.y; int r = inst.r; // cy is 99
  
  int tickInnerR = r - 4;
  int numberR = r - 9;
  int innerDelineator = r - 14;

  // 1. CLEAN FRAME
//  dev->drawCircle(cx, cy, r);               
//  dev->drawCircle(cx, cy, tickInnerR);       

  // 2. SCALE MAPPING
  auto getSpeedAngle = [](float s) -> float {
    float angle;
    if (s <= 300) {
      angle = 135 + (s / 300.0) * 180.0; // 0-300 covers 180 degrees
    } else {
      angle = 315 + ((s - 300.0) / 400.0) * 90.0; // 300-700 covers 90 degrees
    }
    return angle * (PI / 180.0);
  };

  // 3. DRAW ALL TICKS (Small and Large)
  // Low speed: 10 unit increments
  for (int s = 0; s <= 300; s += 10) {
    float a = getSpeedAngle((float)s);
    int tLen = (s % 50 == 0) ? 0 : 2; // Shorter ticks for non-major units
    dev->drawLine(cx + (tickInnerR + tLen)*cos(a), cy + (tickInnerR + tLen)*sin(a), 
                  cx + r*cos(a), cy + r*sin(a));
  }
  // High speed: 50 unit increments
  for (int s = 350; s <= 700; s += 50) {
    float a = getSpeedAngle((float)s);
    dev->drawLine(cx + tickInnerR*cos(a), cy + tickInnerR*sin(a), 
                  cx + r*cos(a), cy + r*sin(a));
  }

  // 4. DRAW LABELS
  dev->setFont(u8g2_font_04b_03_tr);
  int labels [10] = {50, 100, 150, 200, 250, 300, 400, 500, 600, 700};
  for (int i = 0; i < 10; i++) {
    float a = getSpeedAngle((float)labels [i]);
    char buf [4];
    itoa(labels [i], buf, 10);
    // Offset labels slightly so they don't hit the ticks
    dev->drawStr(cx + numberR*cos(a) - 4, cy + numberR*sin(a) + 3, buf);
  }

  // 5. THE HANDS
  // Secondary Target Hand
  float sAng = getSpeedAngle(targetVal);
  dev->drawLine(cx, cy, cx + (r - 2)*cos(sAng), cy + (r - 2)*sin(sAng));

  // Main Slender Triangle Hand
  float mAng = getSpeedAngle(speed);
  int hx = cx + (r - 2)*cos(mAng);
  int hy = cy + (r - 2)*sin(mAng);
  dev->drawTriangle(hx, hy, 
                    cx + (innerDelineator-2)*cos(mAng + 0.12), cy + (innerDelineator-2)*sin(mAng + 0.12), 
                    cx + (innerDelineator-2)*cos(mAng - 0.12), cy + (innerDelineator-2)*sin(mAng - 0.12));
  dev->drawLine(cx, cy, hx, hy);

  // 6. THE HUB (Pivot only)
  dev->setDrawColor(0); dev->drawDisc(cx, cy, 2);
  dev->setDrawColor(1); dev->drawCircle(cx, cy, 2);
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