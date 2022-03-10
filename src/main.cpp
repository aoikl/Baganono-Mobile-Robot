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

// IR sensor
// const int sensor1 = 9;
const int sensor2 = 39;
// const int sensor3 = 11;
const int sensor4 = 36;
// const int sensor5 = 13;

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

    // OLED
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
    display.println("baganono basebase");
    display.display();
    delay(1000);
    // IR sensor
    // pinMode(sensor1, INPUT);
    pinMode(sensor2, INPUT);
    // pinMode(sensor3, INPUT);
    pinMode(sensor4, INPUT);
    // pinMode(sensor5, INPUT);
}

void loop() {
    // Bluetooth
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

    // IR sensor
    // SLL = digitalRead(sensor1);
    int SL = digitalRead(sensor2);
    // SM = digitalRead(sensor3);
    int SR = digitalRead(sensor4);
    // SRR = digitalRead(sensor5);

    //循迹
    // if (SM == HIGH) {
    //      run(45,37);
    // }
    if (SL == 1 && SR == 0) {
        display.clearDisplay();
        display.setTextSize(2);
        display.setTextColor(WHITE);
        display.setCursor(0, 40);
        display.println("right");
        display.display();
        Serial.print(SR);
        Serial.println(SL);
        delay(100);
        // left(35, 57);
    }
    // if (SLL == HIGH && SM == LOW) {
    //      left(35,57);
    // }
    else if (SR == 1 && SL == 0) {
        display.clearDisplay();
        display.setTextSize(2);
        display.setTextColor(WHITE);
        display.setCursor(0, 20);
        display.println("left");
        display.display();
        Serial.print(SR);
        Serial.println(SL);
        delay(100);
        // right(50, 32);
    } else if (SR == 1 && SL == 1) {
        display.clearDisplay();
        display.setTextSize(2);
        display.setTextColor(WHITE);
        display.setCursor(0, 10);
        display.println("forward");
        display.display();
        Serial.print(SR);
        Serial.println(SL);
        delay(100);
    } else {
        display.clearDisplay();
        display.setTextSize(2);
        display.setTextColor(WHITE);
        display.setCursor(50, 50);
        display.println("other");
        display.display();
        Serial.print(SR);
        Serial.println(SL);
        delay(100);
    }
    // if (SRR == HIGH && SM == LOW) {
    //     right(50,32);
    // }
    // if (SR == HIGH && SRR == HIGH) {
    //     right(50,32);
    // }
    // if (SL == HIGH && SLL == HIGH) {
    //     left(30,57);
    // }
    // if (SRR == HIGH && SR == HIGH && SM == HIGH && SL == HIGH) {
    //     right(50,32);
    // }
    // if (SM == HIGH && (SL == HIGH && SLL == HIGH) || (SR == HIGH && SL == HIGH) || (SR == HIGH && SRR == HIGH)) {
    //     // run(45,37);
    // }
}

/*====================

      3.Function

====================*/