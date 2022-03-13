#include <Arduino.h>
#include "SSD1306.h"
#include "Wire.h"

#define SDA 21
#define SCL 22
SSD1306 display(0x3C, SDA, SCL);

void setup() {
    Serial.begin(115200);
    display.init();
    display.flipScreenVertically();

    display.setFont(ArialMT_Plain_10);
    display.drawString(0, 0, "Hello World");

    display.drawString(0, 20, "Hello World");

    display.display();

    delay(1000);
    display.clear();
    display.drawString(0, 49, "Hello ");
    display.display();
}

void loop() {}