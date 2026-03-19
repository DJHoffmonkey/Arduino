#include <TFT_eSPI.h>
TFT_eSPI tft = TFT_eSPI();

void setup() {
  tft.init();
  tft.setRotation(1);
  tft.fillScreen(TFT_MAROON); // Your stable background
  
  // Draw a static dial circle
  tft.drawCircle(160, 120, 100, TFT_WHITE);
  tft.drawCircle(160, 120, 101, TFT_WHITE); // Thicker border
}

void loop() {
  static float angle = 0;
  int x_center = 160;
  int y_center = 120;
  int r = 90;

  // Calculate needle tip
  int x_tip = x_center + r * cos(angle);
  int y_tip = y_center + r * sin(angle);

  // Draw the new needle
  tft.drawLine(x_center, y_center, x_tip, y_tip, TFT_CYAN);
  
  delay(20); // 50fps-ish
  
  // Erase the old needle (draw over it in Maroon)
  tft.drawLine(x_center, y_center, x_tip, y_tip, TFT_MAROON);
  
  angle += 0.05;
  if (angle > 6.28) angle = 0;
}