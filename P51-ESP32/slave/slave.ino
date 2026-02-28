#include <Arduino.h>
#include <U8g2lib.h>
#include <Wire.h>

// Internal Screen Pins
const int BUILT_IN_OLED_SCL = 6;
const int BUILT_IN_OLED_SDA = 5;     

// External Screen Pins (CONFIRMED WORKING)
const int EXT_OLED_SCL = 10;
const int EXT_OLED_SDA = 7;     

// Constructors
U8G2_SSD1306_128X64_NONAME_F_SW_I2C u8g2e1(U8G2_R0, EXT_OLED_SCL, EXT_OLED_SDA, U8X8_PIN_NONE);
U8G2_SSD1306_128X64_NONAME_F_SW_I2C u8g2e2(U8G2_R0, EXT_OLED_SCL, EXT_OLED_SDA, U8X8_PIN_NONE);
U8G2_SSD1306_72X40_ER_F_HW_I2C u8g2bi(U8G2_R0, U8X8_PIN_NONE, BUILT_IN_OLED_SCL, BUILT_IN_OLED_SDA);

void setup() {
  Serial.begin(115200);
  u8g2e1.setI2CAddress(0x3C * 2); 
  u8g2e2.setI2CAddress(0x3D * 2);// Initialize Internal Screen
  u8g2bi.begin();
  u8g2bi.setFont(u8g2_font_04b_03_tr);
  Wire.setClock(400000);
  // Initialize External Screen (using the working GraphicsTest logic)
  u8g2e1.begin();
  u8g2e2.begin();
  u8g2e1.setFont(u8g2_font_6x12_tf);
  u8g2e2.setFont(u8g2_font_6x12_tf);

  // Serial1 for Master Communication (Pins 20/21)
  Serial1.begin(115200, SERIAL_8N1, 20, 21);
  
  Serial.println("Slave Setup Complete - Both Screens Ready.");
}

void loop() {
  // 1. Internal Screen Update
  u8g2bi.clearBuffer();
  u8g2bi.drawStr(0, 15, "SLAVE ACTIVE");
  u8g2bi.sendBuffer();

  // 2. Handle Data from Master
  if (Serial1.available() > 0) {
    String data = Serial1.readStringUntil('\n');
    data.trim();

    if (data.length() > 0) {
      // 3. External Screen Update
      u8g2e1.clearBuffer();
      u8g2e2.clearBuffer();
      u8g2e1.setDrawColor(1);
      u8g2e2.setDrawColor(1);
      u8g2e1.setFont(u8g2_font_ncenB14_tr);
      u8g2e2.setFont(u8g2_font_ncenB14_tr);
      
      // Print the data from Master
      u8g2e1.setCursor(0, 40);
      u8g2e2.setCursor(0, 40);
      u8g2e1.print(data); 
      u8g2e2.print(data); 
      
      // Draw a border to keep it professional
      u8g2e1.drawFrame(0, 0, 128, 64);
      u8g2e2.drawFrame(0, 0, 128, 64);
      u8g2e1.sendBuffer();
      u8g2e2.sendBuffer();
      
      // Also mirror to Serial Monitor for debugging
      Serial.print("Received from Master: ");
      Serial.println(data);
    }
  }
}