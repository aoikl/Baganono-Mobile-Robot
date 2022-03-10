/*====================

      1.include

====================*/
// Bluetooth
#include <BluetoothSerial.h>
BluetoothSerial BT;
char* pin = "9420";
char val;

// OLED
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Arduino.h>
#include <Wire.h>
#define SCREEN_WIDTH 128  // OLED display width, in pixels
#define SCREEN_HEIGHT 64  // OLED display height, in pixels
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

/*====================

    2.Setup & Loop

====================*/
void setup() {
    Serial.begin(115200);
    // Bluetooth
    pinMode(LED_BUILTIN, OUTPUT);
    BT.setPin(pin);
    BT.begin("BagaNono-Robot");
    byte macBT[6];
    esp_read_mac(macBT, ESP_MAC_BT);
    Serial.printf("藍芽 MAC 位址：%02X:%02X;%02X;%02X;%02X;%02X;\n", macBT[0], macBT[1], macBT[2], macBT[3], macBT[4], macBT[5]);

    if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {  // Address 0x3D for 128x64
        Serial.println(F("SSD1306 allocation failed"));
        for (;;)
            ;
    }
    delay(2000);
    display.clearDisplay();
    display.setTextSize(2);
    display.setTextColor(WHITE);
    display.setCursor(0, 10);
    // Display static text
    display.println("baganono basebase");
    display.display();
}

void loop() {
    if (BT.available()) {
        val = BT.read();
        switch (val) {
            case '0':
                display.clearDisplay();
                display.setTextSize(2);
                display.setTextColor(WHITE);
                display.setCursor(0, 10);
                display.println("0000");
                display.display();
                break;

            case '1':
                display.clearDisplay();
                display.setTextSize(2);
                display.setTextColor(WHITE);
                display.setCursor(0, 10);
                display.println("1111");
                display.display();
                break;
        }
    }
}

/*====================

      3.Function

====================*/