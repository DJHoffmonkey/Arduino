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
  // 1. Setup Canvas
  canvas.fillSprite(P51_SKY); // Background is sky by default
  int cx = 40, cy = 40;
  int cr = 36; // Radius for ticks
  
  // 2. Draw the "Moving World" (Sky/Earth Split)
  float radRoll = rll * (PI / 180.0);
  float tanRoll = tan(radRoll);
  float pOffset = constrain(ptch * 1.5, -35, 35); // RC sensitivity boost

  // Fill the Earth (Ground) based on pitch and roll
  // We draw a large rectangle and rotate/offset it
  for (int i = -40; i <= 40; i++) {
    float horizonY = cy - pOffset + (i * tanRoll);
    if (horizonY < 80) {
      canvas.drawLine(cx + i, constrain(horizonY, 0, 80), cx + i, 80, P51_EARTH);
    }
  }

  // 3. Static Bank Ticks (White/Radium)
  // These stay fixed while the world rotates behind them
  for (int a = -60; a <= 60; a += 30) {
    float tr = (a - 90) * (PI / 180.0);
    int len = (a == 0) ? 8 : 5; // Longer tick at the 12 o'clock
    canvas.drawLine(cx + (cr-len)*cos(tr), cy + (cr-len)*sin(tr), 
                    cx + cr*cos(tr), cy + cr*sin(tr), P51_RADIUM);
  }

  // 4. Subtle Static Reference Ticks (Horizontal dashes outside the center)
  canvas.drawLine(cx - 35, cy, cx - 25, cy, P51_RADIUM); // Left outer tick
  canvas.drawLine(cx + 25, cy, cx + 35, cy, P51_RADIUM); // Right outer tick

  // 5. The Aircraft Reference (Inverted Yellow Triangle + Center Dot)
  // Triangle
  canvas.fillTriangle(cx - 4, cy - 8, cx + 4, cy - 8, cx, cy - 2, TFT_YELLOW);
  // The "W" Trident bars
  canvas.fillRect(cx - 15, cy - 1, 8, 2, TFT_YELLOW);
  canvas.fillRect(cx + 7, cy - 1, 8, 2, TFT_YELLOW);
  canvas.fillCircle(cx, cy, 1, TFT_BLACK); // Tiny center "pin"

  // 6. Subtle Serial Text (Bottom)
  canvas.setTextColor(TFT_BLACK); // Small and dark, like an engraving
  canvas.drawCentreString("AN 5736 - 1A", cx, 72, 1);

  // 7. Push to screen (No outer circle - the stencil does the work!)
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

// --- #12 SLIP BALL (9mm -> r29) ---
void drawBank(int x, int y, float rll) {
  canvas.fillSprite(P51_CHARCOAL);
  int cx=40, cy=40;
  canvas.drawSmoothArc(cx, cy, 28, 25, 160, 200, P51_RADIUM, P51_CHARCOAL);
  float ballX = cx + constrain(rll, -25, 25) * 0.6; 
  canvas.fillCircle(ballX, cy + 26, 3, TFT_WHITE);
  canvas.pushSprite(x-40, y-40);
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