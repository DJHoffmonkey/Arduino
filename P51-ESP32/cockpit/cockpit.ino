#include <TFT_eSPI.h>
#include <ReefwingMSP.h>

// --- P-51 VINTAGE PALETTE ---
#define P51_CHARCOAL 0x18C3
#define P51_RADIUM   0xEF55
#define P51_DIRTY_W  0xDEDB
#define P51_SKY      0x641D 
#define P51_EARTH    0x4221 

// --- CONSOLE ALIAS ---
// Change 'Serial' to 'Serial0' here if the USB port is still silent
#define console Serial0

#define RX_FROM_FC 2  // Physical Pin 2
#define TX_TO_FC 1    // Physical Pin 1

TFT_eSPI tft = TFT_eSPI();
TFT_eSprite canvas = TFT_eSprite(&tft);
HardwareSerial toAndFromFC(1);
ReefwingMSP msp;

// --- SHARED DATA ---
float roll, pitch, alt, airSpeed, vsi, heading, vBat;
bool isBenchMode = true;
bool currentlyReceiving = false;
unsigned long lastRequest = 0;
unsigned long lastDataTime = 0;

void setup() {
  console.begin(115200);

  // Critical S3 delay to let the USB handshake finish
  unsigned long startWait = millis();
  while (!console && millis() - startWait < 3000) delay(10); 

  console.println("\n--- P-51 COCKPIT: POWER ON ---");

  // 1. TFT Initialization (Using your hardware pins 10-14 via User_Setup.h)
  tft.init();
  tft.setRotation(1);
  tft.fillScreen(TFT_BLACK);
  canvas.createSprite(80, 80); // Canvas sized for the 11.1mm gauge
  canvas.setTextDatum(MC_DATUM);

  // 2. MSP / FC Initialization
  toAndFromFC.begin(115200, SERIAL_8N1, RX_FROM_FC, TX_TO_FC);
  msp.begin(toAndFromFC);
    console.println("FC UART: Initialized on Pins 1/2");
  // 3. FC Scan (5 second window to detect iNav)
  unsigned long startScan = millis();
  while (millis() - startScan < 5000) {
    msp.request(MSP_ATTITUDE, NULL, 0);
    delay(100); // Give the FC time to process    
    if (updateMSP()) { // Check if the "Promise" was returned
      isBenchMode = false;
      break;
    }
  }
  lastDataTime = millis(); // Reset this so loop doesn't immediately time out
  console.println("System Ready.");
}

bool updateMSP() {
  uint8_t msp_id;
  uint8_t payload[32];
  uint8_t size;
  console.println("--- gonna talk to MSP --- ");

  // 1. Check if the library sees a valid packet
  if (msp.recv(&msp_id, payload, sizeof(payload), &size)) {
    lastDataTime = millis();
    currentlyReceiving = true;

    console.print("--- MSP RECEIVED: ID ");
    console.print(msp_id);
    console.print(" | Size: ");
    console.println(size);

    switch (msp_id) {
      case MSP_ATTITUDE:
        roll = (int16_t)(payload[0] | (payload[1] << 8)) / 10.0;
        pitch = (int16_t)(payload[2] | (payload[3] << 8)) / 10.0;
        heading = (int16_t)(payload[4] | (payload[5] << 8));
        break;
      case MSP_ALTITUDE:
        alt = (int32_t)(payload[0] | (payload[1] << 8) | (payload[2] << 16) | (payload[3] << 24)) * 0.0328084;
        vsi = ((int16_t)(payload[4] | (payload[5] << 8)) * 1.9685) / 1000.0;
        break;
      case MSP_ANALOG:
        vBat = payload[0] / 10.0;
        console.print("VBAT: "); console.println(vBat);
        break;
    }
    return true;
  }
  return false;
}

// Updated mapping for 10mph @ 75 degrees
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

void drawAirspeed(int x, int y, float speed) {
  canvas.fillSprite(P51_CHARCOAL);
  int cx = 40, cy = 40;
  
  canvas.fillSmoothCircle(cx, cy, 33, P51_EARTH); 
  canvas.fillSmoothCircle(cx, cy, 24, 0x18C3); 

  canvas.setTextColor(P51_RADIUM);
  
  // Custom numbers as requested
  int labels[] = {10, 20, 30, 50, 70, 90};
  int labelCount = 6;

  for (int i = 0; i <= 100; i += 2) {
    float angleDeg = getAirspeedAngle((float)i);
    float rad = angleDeg * (PI / 180.0);
    
    bool isLabel = false;
    for(int k=0; k<labelCount; k++) { if(i == labels[k]) isLabel = true; }

    if (isLabel) {
      int tx = cx + 24 * cos(rad); // Nudged in 1px to avoid bezel
      int ty = cy + 24 * sin(rad);
      canvas.drawCentreString(String(i), tx, ty - 4, 2);
      canvas.drawLine(cx + 28 * cos(rad), cy + 28 * sin(rad), cx + 33 * cos(rad), cy + 33 * sin(rad), P51_RADIUM);
    } else if (i % 10 == 0 || i == 0) {
      canvas.drawLine(cx + 29 * cos(rad), cy + 29 * sin(rad), cx + 33 * cos(rad), cy + 33 * sin(rad), P51_RADIUM);
    } else {
      canvas.drawLine(cx + 31 * cos(rad), cy + 31 * sin(rad), cx + 33 * cos(rad), cy + 33 * sin(rad), P51_RADIUM);
    }
  }

  canvas.setTextColor(P51_RADIUM);
  canvas.drawCentreString("MPH", cx, cy - 12, 1); 

  float needleAngle = getAirspeedAngle(constrain(speed, 0, 100));
  float sRad = needleAngle * (PI / 180.0);
  
  // Shadow
  canvas.drawLine(cx+1, cy+1, cx+1 + 32*cos(sRad), cy+1 + 32*sin(sRad), 0x0841);
  // Needle
  canvas.drawWideLine(cx, cy, cx + 32*cos(sRad), cy + 32*sin(sRad), 2, TFT_WHITE, 0x18C3);

  canvas.fillCircle(cx, cy, 3, TFT_BLACK);
  canvas.pushSprite(x - 40, y - 40);
}

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

  // 2. JUST THE TDC TRIANGLE (Fixed at top)
  // x-coords: cx-4, cx+4, cx | y-coords: 5, 5, 13
  canvas.fillTriangle(cx - 3, 3, cx + 3, 3, cx, 3+3+1, TFT_YELLOW);

  // 3. Yellow Bank Ticks
  for (int a = -60; a <= 60; a += 30) {
    if (a == 0) continue; 
    float tr = (a - 90) * (PI / 180.0);
    canvas.drawLine(cx + (cr-4)*cos(tr), cy + (cr-4)*sin(tr), 
                    cx + cr*cos(tr), cy + cr*sin(tr), TFT_YELLOW);
  }

  // 4. THE TRIDENT (Floating Pin & Wings)
  canvas.fillCircle(cx, cy - 1, 2, TFT_YELLOW); 
  canvas.fillRect(cx - 10 - 18, cy - 1.5, 18, 3, TFT_YELLOW); // Left
  canvas.fillRect(cx + 10, cy - 1.5, 18, 3, TFT_YELLOW);  // Right

  // 5. Text
  canvas.setTextColor(0x2104); 
  canvas.drawCentreString("AN 5736-1A", cx, 72, 1);

  canvas.pushSprite(x - 40, y - 40);
}

void drawAltimeter(int x, int y, float alt) {
  canvas.fillSprite(P51_CHARCOAL);
  int cx = 40, cy = 40;
  
  // 1. Background Layers
  canvas.fillSmoothCircle(cx, cy, 33, P51_EARTH); // Aged ring
  canvas.fillSmoothCircle(cx, cy, 22, 0x18C3);    // Sunken center

  // 2. Dial Markings
  canvas.setTextColor(P51_RADIUM);
  for (int i = 0; i < 10; i++) {
    float angle = (i * 36 - 90) * (PI / 180.0);
    
    // Numbers
    int tx = cx + 25 * cos(angle);
    int ty = cy + 25 * sin(angle);
    canvas.drawCentreString(String(i), tx, ty - 4, 2);
    
    // Major Ticks (100ft)
    canvas.drawLine(cx + 29 * cos(angle), cy + 29 * sin(angle), 
                    cx + 33 * cos(angle), cy + 33 * sin(angle), P51_RADIUM);
    
    // Minor Ticks (20ft) - FIXED MATH HERE
    for (int j = 1; j < 5; j++) {
      float subAngle = (i * 36 + j * 7.2 - 90) * (PI / 180.0);
      canvas.drawLine(cx + 31 * cos(subAngle), cy + 31 * sin(subAngle), 
                      cx + 33 * cos(subAngle), cy + 33 * sin(subAngle), P51_RADIUM);
    }
  }

  // 3. Kollsman Window (Tiny 3-digit)
  int kx = cx + 16, ky = cy - 6;
  canvas.fillRect(kx, ky, 14, 10, TFT_BLACK); 
  canvas.drawRect(kx, ky, 14, 10, 0x4228); 
  canvas.setTextColor(TFT_WHITE);
  canvas.drawCentreString("299", kx + 7, ky + 1, 1); 

  // 4. Labels
  canvas.setTextColor(P51_RADIUM);
  canvas.drawCentreString("ALT", cx, cy + 6, 1);
  canvas.setTextColor(0x2104); 
  canvas.drawCentreString("1000 FEET", cx, cy - 3, 1);

  // 5. Hand Logic (Using fmod to keep rotations clean)
  float deg100   = (fmod(alt, 1000.0) / 1000.0) * 360.0;
  float deg1000  = (fmod(alt, 10000.0) / 10000.0) * 360.0;
  float deg10000 = (fmod(alt, 100000.0) / 100000.0) * 360.0;

  // 6. Draw Hands (Ordering is vital)
  drawAltHand(cx, cy, deg10000, 31, 1, true);  // 10k Crow's Foot
  drawAltHand(cx, cy, deg1000, 20, 4, false);  // 1k Fat Hand
  drawAltHand(cx, cy, deg100, 34, 2, false);   // 100ft Long Hand

  canvas.fillCircle(cx, cy, 3, TFT_BLACK); 
  canvas.pushSprite(x - 40, y - 40);
}

// Ensure this helper function is exactly as written below:
void drawAltHand(int cx, int cy, float deg, int len, int width, bool is10k) {
  float rad = (deg - 90.0) * (PI / 180.0);
  int px = cx + len * cos(rad);
  int py = cy + len * sin(rad);
  
  if (is10k) {
    canvas.drawLine(cx, cy, px, py, TFT_WHITE);
    // Crow's foot triangle tip
    float x1 = cx + (len - 5) * cos(rad + 0.2);
    float y1 = cy + (len - 5) * sin(rad + 0.2);
    float x2 = cx + (len - 5) * cos(rad - 0.2);
    float y2 = cy + (len - 5) * sin(rad - 0.2);
    canvas.fillTriangle(px, py, (int)x1, (int)y1, (int)x2, (int)y2, TFT_WHITE);
  } else {
    canvas.drawWideLine(cx, cy, px, py, width, TFT_WHITE, 0x18C3);
  }
}


void drawTurn(int x, int y, float heading) {
  canvas.fillSprite(P51_CHARCOAL); 
  int cx = 40, cy = 40;
  
  int winY = cy - 18; 
  int winH = 22;
  int winLeft = cx - 28;

  // 1. Internal Shadow
  canvas.fillRect(winLeft, winY, 56, winH, 0x0841); 

  // 2. The Drum Logic
  canvas.setTextColor(P51_RADIUM);
  
  // Find the nearest 5-degree mark to the current heading to start the loop
  float startAngle = floor(heading / 5.0) * 5.0;

  for (float a = startAngle - 40; a <= startAngle + 40; a += 5) {
    // Delta is how many degrees this tick is from the center (heading)
    float delta = a - heading;
    
    // Parallax Math
    float rad = delta * (PI / 180.0);
    float scrollX = cx + (sin(rad) * 45); 

    // Normalize angle for labeling (0-35)
    int displayAngle = ((int)(a + 3600) % 360);

    if (scrollX > winLeft - 5 && scrollX < winLeft + 61) {
      // Draw Tick for every 5 degrees
      int tickLen = (displayAngle % 10 == 0) ? 7 : 4;
      canvas.drawFastVLine((int)scrollX, winY + winH - tickLen - 2, tickLen, P51_RADIUM);

      // Draw Number if it's a 30-degree mark (3, 6, 9... 0 for 360)
      if (displayAngle % 30 == 0) {
        String label = String(displayAngle / 10);
        canvas.drawCentreString(label, (int)scrollX, winY + 2, 2);
      }
    }
  }

  // 3. Masking (Paint over the bleed)
  canvas.fillRect(0, 0, 80, winY, P51_CHARCOAL);             
  canvas.fillRect(0, winY + winH, 80, 80 - (winY+winH), P51_CHARCOAL); 
  canvas.fillRect(0, winY, winLeft, winH, P51_CHARCOAL);     
  canvas.fillRect(winLeft + 56, winY, 80 - (winLeft+56), winH, P51_CHARCOAL); 

  // 4. Stationary Elements
  canvas.drawRect(winLeft, winY, 56, winH, 0x4228); 
  canvas.fillRect(cx - 1, winY - 3, 2, winH + 6, TFT_WHITE); 

  canvas.setTextColor(0x4228); // Muted, aged grey
  canvas.drawCentreString("DIREC.GYRO", cx, cy + 10, 1); // Size 1
  canvas.drawCentreString("AN 5735-1A", cx, cy + 20, 1);
  canvas.pushSprite(x - 40, y - 40);
}

void drawBank(int x, int y, float rll) {
  canvas.fillSprite(P51_CHARCOAL);
  int cx = 40, cy = 40;
  
  // 1. The Background Housing (slightly aged/browned)
  canvas.fillSmoothCircle(cx, cy, 32, 0x4228); 

  // 2. The "Bomb" Top Reference (Nudged 1px Left)
  // Wider at the top, tapering down to the tube
  canvas.fillSmoothRoundRect(cx - 3, cy - 28, 5, 12, 2, P51_DIRTY_W); // Top body
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

void drawVSI(int x, int y, float vspd) {
  canvas.fillSprite(P51_CHARCOAL);
  int cx = 40, cy = 40;
  
  // 1. Dark Oil-Stained Background & Hub
  canvas.fillRect(0, 0, 80, 80, 0x2104); 
  canvas.fillSmoothCircle(cx, cy, 14, 0x0841); 

  canvas.setTextColor(P51_RADIUM);
  
  // 2. Expanded Scale Mapping
  // 0 is 180 deg (9 o'clock)
  // 1 is ~145 deg (roughly 10:30 / 7:30)
  // 2 is ~110 deg (roughly 11:30 / 6:30)
  // 4 is ~40 deg  (This puts it right at 2pm and 4pm)
  float scale[] = {1.0, 2.0, 4.0};
  float offsets[] = {35, 70, 140}; 
  
  for (int i = 0; i < 3; i++) {
    float rUp = (180 - offsets[i]) * (PI / 180.0);
    float rDn = (180 + offsets[i]) * (PI / 180.0);
    
    // Numbers: Tucked at 20px radius to stay clear of the bezel
    canvas.drawCentreString(String((int)scale[i]), cx + 20 * cos(rUp), cy + 20 * sin(rUp) - 4, 2);
    canvas.drawCentreString(String((int)scale[i]), cx + 20 * cos(rDn), cy + 20 * sin(rDn) - 4, 2);
    
    // Ticks: Reaching out to the edge
    canvas.drawLine(cx+28*cos(rUp), cy+28*sin(rUp), cx+40*cos(rUp), cy+40*sin(rUp), P51_RADIUM);
    canvas.drawLine(cx+28*cos(rDn), cy+28*sin(rDn), cx+40*cos(rDn), cy+40*sin(rDn), P51_RADIUM);
  }

  // 3. The Left-Side "Niche" Details
  // .5 markers and Brackets
  canvas.drawCentreString(".5", 8, cy - 20, 1);
  canvas.drawLine(2, cy - 8, 8, cy - 8, P51_RADIUM); 
  canvas.drawLine(8, cy - 12, 8, cy - 4, P51_RADIUM); 

  canvas.drawCentreString(".5", 8, cy + 12, 1);
  canvas.drawLine(2, cy + 8, 8, cy + 8, P51_RADIUM); 
  canvas.drawLine(8, cy + 4, 8, cy + 12, P51_RADIUM);

  // 4. The "Single 6" at 3 o'clock (0 degrees)
  canvas.drawCentreString("6", cx + 26, cy - 4, 2);
  canvas.drawLine(cx + 34, cy, cx + 40, cy, P51_RADIUM);

  // 5. Zero Marker (9 o'clock)
  canvas.drawLine(0, cy, 10, cy, P51_RADIUM);

  // 6. Calibrated Needle Logic
  float absV = abs(vspd);
  float move = 0;
  // Interpolation to match the new wide-sweep offsets
  if (absV <= 1.0)      move = absV * 35.0;
  else if (absV <= 2.0) move = 35.0 + (absV - 1.0) * 35.0;
  else if (absV <= 4.0) move = 70.0 + (absV - 2.0) * 35.0; // Linear 70 to 140
  else                  move = 140.0 + (constrain(absV, 4, 6) - 4.0) * 20.0; // Meets at 180 total (3 o'clock)

  float finalDeg = (vspd >= 0) ? (180.0 - move) : (180.0 + move);
  float nRad = finalDeg * (PI / 180.0);
  
  // Needle
  canvas.drawLine(cx+1, cy+1, cx+1 + 36*cos(nRad), cy+1 + 36*sin(nRad), 0x0841);
  canvas.drawWideLine(cx, cy, cx + 36*cos(nRad), cy + 36*sin(nRad), 2, TFT_WHITE, 0x0841);

  canvas.fillCircle(cx, cy, 3, TFT_BLACK);
  canvas.pushSprite(x - 40, y - 40);
}

void loop() {
 if (isBenchMode) {
    // 1. BENCH PHYSICS (Only runs if no FC was found at boot)
    float t = millis() / 1000.0;
    airSpeed = 45 + sin(t * 0.5) * 35;
    alt = 225.0 + (sin(t * 0.2) * 225.0);
    roll = sin(t) * 35;
    pitch = cos(t * 0.7) * 10;
    vsi = cos(t * 0.2) * 4.0;
    heading += 0.2;
    if (heading >= 360) heading = 0;
  } 
  else {
    // 2. FLIGHT DATA ACQUISITION
    if (millis() - lastRequest > 25) {
      static int step = 0;
      uint8_t cycle[] = {MSP_ATTITUDE, MSP_ALTITUDE, MSP_ANALOG};
      msp.request(cycle[step], NULL, 0);
      step = (step + 1) % 3;
      lastRequest = millis();
    }
    
    updateMSP();

    // 3. TELEMETRY STATUS (Replaces the 'flick back' logic)
    // currentlyReceiving will be true if we've had a packet in the last 500ms
    currentlyReceiving = (millis() - lastDataTime < 500);
  }
  

  // --- RENDER ALL GAUGES ---
  // TOP ROW

  drawAirspeed(33, 48, airSpeed); // #11
  drawTurn(140, 48, heading); // #7 Changed roll to heading for the Gyro
  drawHorizon(253, 54, roll, pitch); // #6 
  // BOTTOM ROW
  drawAltimeter(33, 144, alt); // #8
  drawBank(146, 170, roll);  // #12
  drawVSI(253, 170, vsi);  // #13



}