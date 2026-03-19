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
  canvas.createSprite(85, 85); 
  canvas.setTextDatum(MC_DATUM);
}

// --- #11 AIRSPEED (RC SCALE: 0-100 MPH) ---
void drawAirspeed(int x, int y, float s) {
  canvas.fillSprite(P51_CHARCOAL);
  int cx=42, cy=42, r=38;
  canvas.drawCircle(cx, cy, r, P51_RADIUM);
  for(int i=0; i<=100; i+=10){
    float ang = map(i, 0, 100, -210, 120) * PI/180.0;
    canvas.drawLine(cx+(r-5)*cos(ang), cy+(r-5)*sin(ang), cx+r*cos(ang), cy+r*sin(ang), P51_RADIUM);
    if(i%20==0) canvas.drawNumber(i, cx+25*cos(ang), cy+25*sin(ang), 1);
  }
  float nAng = map(constrain(s,0,100), 0, 100, -210, 120) * PI/180.0;
  canvas.drawLine(cx, cy, cx+(r-4)*cos(nAng), cy+(r-4)*sin(nAng), P51_DIRTY_W);
  canvas.drawString("MPH", cx, cy+12, 1);
  canvas.pushSprite(x-42, y-42);
}

// --- #6 ARTIFICIAL HORIZON (TOP RIGHT - LARGE) ---
void drawHorizon(int x, int y, float rll, float ptch) {
  canvas.fillSprite(P51_CHARCOAL);
  int cx=42, cy=42, cr=40; // Increased radius for the "Big One"
  float pOffset = constrain(ptch, -30, 30);
  canvas.fillSmoothCircle(cx, cy, cr, P51_EARTH);
  for(int i=-cr; i<cr; i++){
    float h = sqrt(cr*cr - i*i);
    float localPitch = pOffset + (i * tan(rll * PI/180.0));
    canvas.drawLine(cx+i, cy-h, cx+i, cy-localPitch, P51_SKY);
  }
  canvas.drawCircle(cx, cy, cr, P51_RADIUM);
  canvas.drawWedgeLine(cx-15, cy, cx+15, cy, 2, 2, TFT_ORANGE); // Plane Ref
  canvas.pushSprite(x-42, y-42);
}

// --- #8 ALTIMETER (METRIC RC: 100m PER SWEEP) ---
void drawAltimeter(int x, int y, float meters) {
  canvas.fillSprite(P51_CHARCOAL);
  int cx=42, cy=42, r=38;
  canvas.drawCircle(cx, cy, r, P51_RADIUM);
  for(int i=0; i<10; i++){
    float ang = (i*36-90)*PI/180.0;
    canvas.drawNumber(i, cx+28*cos(ang), cy+28*sin(ang), 1);
  }
  // Long hand: 100m per rotation
  float a100 = (fmod(meters, 100.0) * 3.6 - 90)*PI/180.0;
  canvas.drawLine(cx, cy, cx+(r-5)*cos(a100), cy+(r-5)*sin(a100), P51_DIRTY_W);
  // Short hand: 1000m per rotation (will move 1/10th as much)
  float a1k = (meters * 0.36 - 90)*PI/180.0;
  canvas.drawLine(cx, cy, cx+(r-15)*cos(a1k), cy+(r-15)*sin(a1k), TFT_WHITE);
  canvas.drawString("METRES", cx, cy+15, 1);
  canvas.pushSprite(x-42, y-42);
}

// --- #7 DIRECTIONAL GYRO (TOP MIDDLE) ---
void drawGyro(int x, int y, float hd) {
  canvas.fillSprite(P51_CHARCOAL);
  int cx=42, cy=42, r=38;
  canvas.drawCircle(cx, cy, r, P51_RADIUM);
  canvas.drawTriangle(cx, cy-r+2, cx-5, cy-r+10, cx+5, cy-r+10, TFT_RED);
  for(int i=0; i<360; i+=30){
    float ang = (i - hd - 90)*PI/180.0;
    canvas.drawLine(cx+(r-8)*cos(ang), cy+(r-8)*sin(ang), cx+r*cos(ang), cy+r*sin(ang), P51_RADIUM);
  }
  canvas.drawNumber((int)hd, cx, cy, 2);
  canvas.pushSprite(x-42, y-42);
}

// --- #12 SLIP BALL (BOTTOM MIDDLE) ---
void drawBank(int x, int y, float rll) {
  canvas.fillSprite(P51_CHARCOAL);
  int cx=42, cy=42;
  canvas.drawSmoothArc(cx, cy, 35, 32, 160, 200, P51_RADIUM, P51_CHARCOAL);
  float ballX = cx + constrain(rll, -30, 30) * 0.8; 
  canvas.fillCircle(ballX, cy + 32, 4, TFT_WHITE);
  canvas.pushSprite(x-42, y-42);
}

// --- #13 VSI (RC SCALE: +/- 10 m/s) ---
void drawVSI(int x, int y, float v) {
  canvas.fillSprite(P51_CHARCOAL);
  int cx=42, cy=42, r=38;
  canvas.drawCircle(cx, cy, r, P51_RADIUM);
  float nAng = map(constrain(v, -10, 10), -10, 10, 225, -45)*PI/180.0;
  canvas.drawLine(cx, cy, cx+(r-5)*cos(nAng), cy+(r-5)*sin(nAng), P51_DIRTY_W);
  canvas.drawString("V/S", cx, cy+15, 1);
  canvas.pushSprite(x-42, y-42);
}

void loop() {
  float t = millis()/1000.0;
  // Simulating an RC flight path
  airSpeed = 45 + sin(t*0.5)*35;   // 10 to 80 mph
  alt = 60 + sin(t*0.2)*55;        // 5 to 115 metres
  roll = sin(t)*45;                // Snappy RC rolls
  pitch = cos(t*0.7)*15;
  heading += 0.5; if(heading>360) heading=0;
  vsi = cos(t*0.2)*8;              // Climb/Dive rates

  // The 6-Pack Layout (Based on your numbering)
  drawAirspeed(55, 60, airSpeed);    // #11 TL
  drawGyro(160, 60, heading);        // #7 TM
  drawHorizon(265, 60, roll, pitch); // #6 TR (Large hole)
  
  drawAltimeter(55, 170, alt);       // #8 BL
  drawBank(160, 170, roll);          // #12 BM
  drawVSI(265, 170, vsi);            // #13 BR
}