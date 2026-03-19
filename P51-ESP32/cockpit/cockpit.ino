#include <TFT_eSPI.h>

// --- P-51 VINTAGE PALETTE (16-bit RGB565) ---
#define P51_CHARCOAL 0x18C3  // Deep background
#define P51_RADIUM   0xEF55  // Aged yellowish numbers
#define P51_DIRTY_W  0xDEDB  // Weathered needle white

TFT_eSPI tft = TFT_eSPI();
TFT_eSprite canvas = TFT_eSprite(&tft); // The shared "Drawing Table"

// Bench Mode variables
float testAlt = 0;
float testSpeed = 0;

void setup() {
  tft.init();
  tft.setRotation(1);      // Landscape
  tft.fillScreen(TFT_BLACK); // Pure black for the stencil "void"

  // Create a 100x100 canvas. 
  canvas.createSprite(100, 100);
  canvas.setTextDatum(MC_DATUM); // Set text alignment to Middle Center
}

// --- SPRITE AIRSPEED (0-120 MPH) ---
void drawAirspeed(int screenX, int screenY, float speed) {
  canvas.fillSprite(P51_CHARCOAL); 
  int cx = 50, cy = 50, r = 48;

  // 1. Static Face
  canvas.drawCircle(cx, cy, r, P51_RADIUM);
  canvas.setTextColor(P51_RADIUM);
  
  // Use font 2 for the "MPH" label
  canvas.drawString("MPH", cx, cy + 20, 2);

  for (int s = 0; s <= 120; s += 10) {
    float angle = (-60.0 + (s / 120.0) * 330.0) * (PI / 180.0);
    int len = (s % 20 == 0) ? 8 : 4;
    canvas.drawLine(cx + (r-len)*cos(angle), cy + (r-len)*sin(angle), 
                    cx + r*cos(angle), cy + r*sin(angle), P51_RADIUM);
    
    // Draw numbers every 20 mph
    if (s % 20 == 0) {
      int tx = cx + (r-18)*cos(angle);
      int ty = cy + (r-18)*sin(angle);
      canvas.drawNumber(s, tx, ty); // Much cleaner than drawStr
    }
  }

  // 2. The Needle
  float nAng = (-60.0 + (speed / 120.0) * 330.0) * (PI / 180.0);
  canvas.drawLine(cx, cy, cx + (r-5)*cos(nAng), cy + (r-5)*sin(nAng), P51_DIRTY_W);
  canvas.fillCircle(cx, cy, 3, TFT_BLACK);
  canvas.drawCircle(cx, cy, 3, P51_DIRTY_W);

  // 3. Push to screen
  canvas.pushSprite(screenX, screenY);
}

// --- SPRITE ALTIMETER (0-100 FT SWEEP) ---
void drawAltimeter(int screenX, int screenY, float alt) {
  canvas.fillSprite(P51_CHARCOAL);
  int cx = 50, cy = 50, r = 48;

  // 1. Static Face
  canvas.drawCircle(cx, cy, r, P51_RADIUM);
  canvas.setTextColor(P51_RADIUM);
  canvas.drawString("FEET", cx, cy + 20, 2);

  for (int i = 0; i < 10; i++) {
    float a = (i * 36 - 90) * (PI / 180.0);
    canvas.drawLine(cx + 40*cos(a), cy + 40*sin(a), cx + 48*cos(a), cy + 48*sin(a), P51_RADIUM);
    canvas.drawNumber(i, cx + 32*cos(a), cy + 32*sin(a));
  }

  // 2. Needle (Precision 100ft hand)
  float a100 = (fmod(alt, 100.0) * 3.6) - 90.0;
  float rad = a100 * (PI / 180.0);
  canvas.drawLine(cx, cy, cx + (r-5)*cos(rad), cy + (r-5)*sin(rad), P51_DIRTY_W);
  
  // Hub
  canvas.fillCircle(cx, cy, 3, TFT_BLACK);
  canvas.drawCircle(cx, cy, 3, P51_DIRTY_W);

  canvas.pushSprite(screenX, screenY);
}

void loop() {
  // BENCH MODE MATH
  float t = millis() / 1000.0;
  testAlt = 225.0 + (sin(t * 0.2) * 225.0);
  testSpeed = 60.0 + (sin(t * 0.15) * 55.0);

  // RENDERING
  // Adjust these coordinates so they align with your stencil holes
  drawAirspeed(20, 70, testSpeed); 
  drawAltimeter(140, 70, testAlt);
}