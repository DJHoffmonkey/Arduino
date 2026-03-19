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
  canvas.fillSprite(P51_CHARCOAL);
  int cx = 40, cy = 40, cr = 36;
  
  // 1. Draw the "Dished" Background Window
  // This creates the shape seen in your photo where the horizon lives
  canvas.fillSmoothRoundRect(cx-28, cy-22, 56, 44, 10, P51_EARTH); 

  // 2. The Moving Horizon (Sky vs Earth)
  // We calculate the horizon line based on pitch and roll
  float radRoll = rll * (PI / 180.0);
  float tanRoll = tan(radRoll);
  float pOffset = constrain(ptch, -20, 20); // 1/7th scale sensitivity

  for (int i = -28; i <= 28; i++) {
    float horizonY = cy - pOffset + (i * tanRoll);
    // Draw the sky portion inside the window bounds
    canvas.drawLine(cx + i, cy - 20, cx + i, constrain(horizonY, cy-20, cy+20), P51_SKY);
  }

  // 3. Bank Scale Ticks (Top Arc)
  canvas.setTextColor(TFT_WHITE);
  for (int a = -60; a <= 60; a += 30) {
    float tr = (a - 90) * (PI / 180.0);
    canvas.drawLine(cx + (cr-5)*cos(tr), cy + (cr-5)*sin(tr), 
                    cx + cr*cos(tr), cy + cr*sin(tr), P51_RADIUM);
  }

  // 4. The "Fixed" Aircraft Reference (The Yellow "W")
  // Center dot
  canvas.fillCircle(cx, cy, 2, TFT_YELLOW);
  // Left and Right wing bars
  canvas.fillRoundRect(cx - 18, cy - 1, 12, 3, 1, TFT_YELLOW);
  canvas.fillRoundRect(cx + 6, cy - 1, 12, 3, 1, TFT_YELLOW);
  
  // 5. Outer Bezel & Text
  canvas.drawCircle(cx, cy, cr, P51_RADIUM); // Glass rim
  canvas.setTextColor(P51_RADIUM);
  canvas.drawString("GYRO HORIZON", cx, cy + 28, 1);

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
  float t = millis()/1000.0;
  airSpeed = 40 + sin(t*0.5)*30; 
  alt = 50 + sin(t*0.2)*45;      
  roll = sin(t)*35;              
  pitch = cos(t*0.7)*10;
  vsi = cos(t*0.2)*7;            

  // RENDER TO YOUR STENCIL CENTERS (Adjust these to match your film traces)
  drawAirspeed(55, 60, airSpeed);    // #11
  drawTurn(160, 60, roll);           // #7
  drawHorizon(265, 60, roll, pitch); // #6 (The Big One)
  
  drawAltimeter(55, 170, alt);       // #8
  drawBank(160, 170, roll);          // #12
  drawVSI(265, 170, vsi);            // #13
}