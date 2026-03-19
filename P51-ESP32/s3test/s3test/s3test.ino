#include <Arduino_GFX_Library.h>

// THE STABLE PINS (1, 2, 40, 41, 42)
#define SCLK_PIN 2
#define MOSI_PIN 1
#define MISO_PIN -1 
#define CS_PIN   42
#define DC_PIN   41
#define RST_PIN  40

// Fixed the typo: SCLK_PIN matches here now
Arduino_DataBus *bus = new Arduino_ESP32SPI(DC_PIN, CS_PIN, SCLK_PIN, MOSI_PIN, MISO_PIN);
Arduino_GFX *gfx = new Arduino_ILI9341(bus, RST_PIN, 1 /* Rotation */, false /* IPS */);

void setup() {
  Serial.begin(115200);
  delay(2000);
  Serial.println("P-51 RECOVERY: BACK TO STABLE GFX");

  if (!gfx->begin()) {
    Serial.println("GFX Begin Failed!");
  }
  gfx->invertDisplay(false);
  // Use hex codes to avoid 'not declared' errors
  gfx->fillScreen(0xF800); 
  
  gfx->setCursor(60, 100);
  gfx->setTextColor(0xFFFF); // White
  gfx->setTextSize(3);
  gfx->println("P-51 STABLE");
  
  Serial.println("If the screen is olive green, we are back in business.");
}

void loop() {
  // Blink something or update a needle here later
  delay(1000);
}