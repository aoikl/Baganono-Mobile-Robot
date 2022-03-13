/*====================

    1.include

====================*/
// 1.1  Bluetooth
#include <Arduino.h>
#include <BluetoothSerial.h>
BluetoothSerial BT;
void BTinit();

// 1.2  OLED
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Wire.h>
#define SCREEN_WIDTH 128  // OLED display width, in pixels
#define SCREEN_HEIGHT 64  // OLED display height, in pixels
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
void OLEDinit();
void OLEDdisplay(String text, String text2, String text3);

// 1.3  IR sensor
#define sensor1 9
#define sensor2 39
#define sensor3 11
#define sensor4 36
#define sensor5 13
int IRread[5] = {0, 0, 0, 0, 0};
void IRinit();

// 1.4  Motor_Shield
#include "WEMOS_Motor.h"
float pwm;
Motor M1(0x30, _MOTOR_A, 1000);  // Motor LeftFront
Motor M2(0x30, _MOTOR_B, 1000);  // Motor LeftRear

/*====================

    2.Setup & Loop

====================*/
void setup() {
    BTinit();
    OLEDinit();
    IRinit();
}

void loop() {
    if (BT.available()) {
        // Mode
        switch (BT.read()) {
            case 'a':
                display.setFont();
                // OLEDdisplay("ModeAuto","--","--");
                break;

            case 'A':
                OLEDdisplay("ModeHandle", "--", "--");
                break;
        }
    }

    //循迹
    // if (SM == HIGH) {
    //      run(45,37);
    // }
    // if (SL == 1 && SR == 0) {
    //     display.clearDisplay();
    //     display.setTextSize(2);
    //     display.setTextColor(WHITE);
    //     display.setCursor(0, 40);
    //     display.println("right");
    //     display.display();
    //     Serial.print(SR);
    //     Serial.println(SL);
    //     delay(100);
    //     // left(35, 57);
    // }
    // if (SLL == HIGH && SM == LOW) {
    //      left(35,57);
    // }
    // else if (SR == 1 && SL == 0) {
    //     display.clearDisplay();
    //     display.setTextSize(2);
    //     display.setTextColor(WHITE);
    //     display.setCursor(0, 20);
    //     display.println("left");
    //     display.display();
    //     Serial.print(SR);
    //     Serial.println(SL);
    //     delay(100);
    //     // right(50, 32);
    // } else if (SR == 1 && SL == 1) {
    //     display.clearDisplay();
    //     display.setTextSize(2);
    //     display.setTextColor(WHITE);
    //     display.setCursor(0, 10);
    //     display.println("forward");
    //     display.display();
    //     Serial.print(SR);
    //     Serial.println(SL);
    //     delay(100);
    // } else {
    //     display.clearDisplay();
    //     display.setTextSize(2);
    //     display.setTextColor(WHITE);
    //     display.setCursor(50, 50);
    //     display.println("other");
    //     display.display();
    //     Serial.print(SR);
    //     Serial.println(SL);
    //     delay(100);
    // }
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
//  BT
void BTinit() {
    Serial.begin(115200);
    BT.begin("BagaNono-Robot");
    BT.setPin("899819");
    Serial.printf("BT initial ok and ready to pair. \r\n");
}

//  OLED
void OLEDinit() {
    display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
    delay(2000);
    display.clearDisplay();
    display.setFont();
    display.setTextColor(WHITE);
    display.setCursor(0, 10);
    display.println("BagaNono BaseBase");
    display.println("OLED turn On");
    display.display();
}

void OLEDdisplay(String text, String text2, String text3) {
    display.clearDisplay();
    // display.setTextSize(2);
    // display.setTextColor(WHITE);
    display.setCursor(0, 10);
    display.println(text);
    display.println(text2);
    display.println(text3);
    display.display();
}

//  IR
void IRinit() {
    pinMode(sensor1, INPUT);
    pinMode(sensor2, INPUT);
    pinMode(sensor3, INPUT);
    pinMode(sensor4, INPUT);
    pinMode(sensor5, INPUT);
}