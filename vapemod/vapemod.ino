#include <Wire.h>
#include <U8g2lib.h>

#define OLED_SDA 8
#define OLED_SCL 9

U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);

void setup() {
  Serial.begin(115200);
  delay(500);

  Wire.begin(OLED_SDA, OLED_SCL);

  Serial.println("Starting u8g2.begin()...");
  bool ok = u8g2.begin();
  Serial.print("u8g2.begin() result: ");
  Serial.println(ok);
}

void loop() {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_logisoso16_tf);
  u8g2.drawStr(10, 40, "TEST OK");
  u8g2.sendBuffer();
  delay(1000);
}
