#include <Arduino.h>
#include "SSD1306.h"
#include "Wire.h"

#define SDA 22
#define SCL 23

SSD1306 display(0x3c, SDA, SCL);

void setup() {
    display.init();

    display.setFont(ArialMT_Plain_10);
    display.drawString(0, 0, "Hello World");

    display.setFont(ArialMT_Plain_16);
    display.drawString(0, 10, "Hello World");

    display.setFont(ArialMT_Plain_24);
    display.drawString(0, 25, "Hello World");

    display.display();
}

void loop() {}