#include <Arduino.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>

#define SPI_SCK_PIN 12
#define SPI_MOSI_PIN 11
#define SPI_MISO_PIN 13
#define DISPLAY_CS_PIN 15
#define DISPLAY_DC_PIN 16
#define DISPLAY_RST_PIN 17

Adafruit_ST7735 tft(DISPLAY_CS_PIN, DISPLAY_DC_PIN, DISPLAY_RST_PIN);

void drawRotationScreen(uint8_t rotation) {
  tft.setRotation(rotation);
  tft.fillScreen(ST77XX_BLACK);

  int16_t w = tft.width();
  int16_t h = tft.height();
  int16_t barW = max<int16_t>(1, w / 4);

  tft.fillRect(0, 0, barW, h, ST77XX_RED);
  tft.fillRect(barW, 0, barW, h, ST77XX_GREEN);
  tft.fillRect(barW * 2, 0, barW, h, ST77XX_BLUE);
  tft.fillRect(barW * 3, 0, w - barW * 3, h, ST77XX_WHITE);

  tft.fillRect(0, 0, w, 28, ST77XX_BLACK);
  tft.setTextColor(ST77XX_YELLOW);
  tft.setTextSize(1);
  tft.setCursor(2, 2);
  tft.print("ST7735 128x128");
  tft.setCursor(2, 14);
  tft.print("ROT ");
  tft.print(rotation);
  tft.print(" W");
  tft.print(w);
  tft.print(" H");
  tft.print(h);
}

void setup() {
  Serial.begin(115200);
  delay(400);

  SPI.begin(SPI_SCK_PIN, SPI_MISO_PIN, SPI_MOSI_PIN);
  tft.initR(INITR_144GREENTAB);
  tft.setSPISpeed(27000000);
  Serial.println("ST7735 128x128 display test. Rotations change every 2 seconds.");
}

void loop() {
  static uint8_t rotation = 0;
  static uint32_t lastChange = 0;

  if (millis() - lastChange >= 2000) {
    lastChange = millis();
    drawRotationScreen(rotation);
    Serial.printf("rotation=%u width=%d height=%d\n", rotation, tft.width(), tft.height());
    rotation = (rotation + 1) % 4;
  }
}
