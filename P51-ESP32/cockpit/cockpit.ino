#include <TFT_eSPI.h>

// --- P-51 VINTAGE PALETTE ---
#define P51_CHARCOAL 0x18C3
#define P51_RADIUM   0xEF55
#define P51_DIRTY_W  0xDEDB
#define P51_SKY      0x641D 
#define P51_EARTH    0x4221 

TFT_eSPI tft = TFT_eSPI();
TFT_eSprite canvas = TFT_eSprite(&tft);

float roll, pitch, alt, airSpeed, vsi, heading;

void setup() {
  tft.init();
  tft.setRotation(1);
  tft.fillScreen(TFT_BLACK);
  canvas.createSprite(80, 80); // Canvas sized for the 11.1mm gauge
  canvas.setTextDatum(MC_DATUM);
}

// --- #11 AIRSPEED (9mm -> r29) ---
void drawAirspeed(int x, int y, float s) {
  canvas.fillSprite(P51_CHARCOAL);
  int cx=40, cy=40, r=29;
  canvas.drawCircle(cx, cy, r, P51_RADIUM);
  for(int i=0; i<=100; i+=20){
    float ang = map(i, 0, 100, -210, 120) * PI/180.0;
    canvas.drawNumber(i/10, cx+20*cos(ang), cy+20*sin(ang), 1); // Shorthand 2, 4, 6
  }
  float nAng = map(constrain(s,0,100), 0, 100, -210, 120) * PI/180.0;
  canvas.drawLine(cx, cy, cx+(r-3)*cos(nAng), cy+(r-3)*sin(nAng), P51_DIRTY_W);
  canvas.pushSprite(x-40, y-40);
}

// --- #6 HORIZON (11.1mm -> r36) ---
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

  // 2. Static Bank Ticks (Top Arc)
  for (int a = -60; a <= 60; a += 30) {
    float tr = (a - 90) * (PI / 180.0);
    canvas.drawLine(cx + (cr-4)*cos(tr), cy + (cr-4)*sin(tr), 
                    cx + cr*cos(tr), cy + cr*sin(tr), P51_RADIUM);
  }

  // 3. The "Bomb" Bank Pointer (Rotates with Roll)
  float bombAng = (rll - 90) * (PI / 180.0);
  int bx = cx + (cr-8) * cos(bombAng);
  int by = cy + (cr-8) * sin(bombAng);
  canvas.fillTriangle(bx-3, by-3, bx+3, by-3, bx, by+4, TFT_WHITE); 
  canvas.drawLine(bx, by-8, bx, by-3, TFT_WHITE);

  // 4. THE TRIDENT (Aligned Exactly)
  // Horizontal Reference Line: cy
  // Wing bars: 2px thick, starting at cy
  canvas.fillRect(cx - 18, cy, 10, 2, TFT_YELLOW); // Left Wing
  canvas.fillRect(cx + 8, cy, 10, 2, TFT_YELLOW);  // Right Wing
  
  // Inverted Triangle: Point ends exactly at cy to match the wings
  // (Base at cy-6, Point at cy)
  canvas.fillTriangle(cx - 4, cy - 6, cx + 4, cy - 6, cx, cy, TFT_YELLOW); 

  // 5. Minimalist Serial Text
  canvas.setTextColor(0x2104); 
  canvas.drawCentreString("AN 5736-1A", cx, 72, 1);

  canvas.pushSprite(x - 40, y - 40);
}

// --- #8 ALTIMETER (9mm -> r29) ---
void drawAltimeter(int x, int y, float meters) {
  canvas.fillSprite(P51_CHARCOAL);
  int cx=40, cy=40, r=29;
  canvas.drawCircle(cx, cy, r, P51_RADIUM);
  for(int i=0; i<10; i++){
    float ang = (i*36-90)*PI/180.0;
    canvas.drawNumber(i, cx+21*cos(ang), cy+21*sin(ang), 1);
  }
  float a100 = (fmod(meters, 100.0) * 3.6 - 90)*PI/180.0;
  canvas.drawLine(cx, cy, cx+(r-4)*cos(a100), cy+(r-4)*sin(a100), P51_DIRTY_W);
  canvas.pushSprite(x-40, y-40);
}

// --- #7 TURN INDICATOR (8.2mm -> r27) ---
void drawTurn(int x, int y, float rll) {
  canvas.fillSprite(P51_CHARCOAL);
  int cx=40, cy=40, r=27;
  canvas.drawCircle(cx, cy, r, P51_RADIUM);
  float nAng = map(constrain(rll, -45, 45), -45, 45, 220, 320) * PI/180.0;
  canvas.drawLine(cx, cy, cx+(r-4)*cos(nAng), cy+(r-4)*sin(nAng), P51_DIRTY_W);
  canvas.pushSprite(x-40, y-40);
}

void drawBank(int x, int y, float rll) {
  canvas.fillSprite(P51_CHARCOAL);
  int cx = 40, cy = 40;
  
  // 1. The Background Housing (slightly aged/browned)
  canvas.fillSmoothCircle(cx, cy, 32, 0x4228); 

  // 2. The "Bomb" Top Reference (Nudged 1px Left)
  // Wider at the top, tapering down to the tube
  canvas.fillSmoothRoundRect(cx - 3, cy - 28, 6, 12, 2, P51_DIRTY_W); // Top body
  // Triangle moved from (cx-3, cx+3, cx) to (cx-4, cx+2, cx-1)
  canvas.fillTriangle(cx - 4, cy - 16, cx + 2, cy - 16, cx - 1, cy - 10, P51_DIRTY_W);
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

// --- #13 VSI (9mm -> r29) ---
void drawVSI(int x, int y, float v) {
  canvas.fillSprite(P51_CHARCOAL);
  int cx=40, cy=40, r=29;
  canvas.drawCircle(cx, cy, r, P51_RADIUM);
  float nAng = map(constrain(v, -10, 10), -10, 10, 225, -45)*PI/180.0;
  canvas.drawLine(cx, cy, cx+(r-4)*cos(nAng), cy+(r-4)*sin(nAng), P51_DIRTY_W);
  canvas.pushSprite(x-40, y-40);
}

void loop() {
  // 1. Get your flight data (Bench mode for now)
  float t = millis() / 1000.0;
  airSpeed = 45 + sin(t*0.5)*35;
  alt = 60 + sin(t*0.2)*55;
  roll = sin(t)*35;
  pitch = cos(t*0.7)*10;
  vsi = cos(t*0.2)*8;

// --- 2.2" S3 STENCIL ALIGNMENT (NUDGED) ---

  // TOP ROW
  drawAirspeed(33, 48, airSpeed);    // #11
  drawTurn(140, 48, roll);           // #7
  // HORIZON (#6): Moved from (245, 75) to (241, 55) 
  // This is 4px Left (5%) and 20px Up (25%)
  drawHorizon(253, 54, roll, pitch); 

  // BOTTOM ROW
  drawAltimeter(33, 144, alt);       // #8
  drawBank(146, 170, roll);          // #12
  drawVSI(253, 170, vsi);            // #13
}