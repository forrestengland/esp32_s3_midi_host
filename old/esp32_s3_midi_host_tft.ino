#include <TFT_eSPI.h>

TFT_eSPI tft = TFT_eSPI();

void setup()
{
    Serial.begin(115200);
    delay(2000);

    Serial.println("1: Sketch started");
    Serial.println("2: Initializing TFT...");

		Serial.println("Starting TFT test");
    tft.init();

    Serial.println("3: TFT initialized");

    tft.setRotation(1);
    Serial.println("4: Rotation set");

    tft.fillScreen(TFT_BLACK);
    Serial.println("5: Screen cleared");

    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.setTextSize(2);
    tft.setCursor(20, 20);
    tft.println("ESP32-S3");

    Serial.println("6: Text drawn");
}

void loop()
{
    delay(1000);
}
