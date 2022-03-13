/*====================

    1.include

====================*/
// 1.1  Bluetooth
#include <Arduino.h>
#include <BluetoothSerial.h>
BluetoothSerial BT;
void BTinit();

// 1.2  OLED
#include <Wire.h>
#include "SSD1306.h"
#define SDA 21
#define SCL 22
SSD1306 display(0x3C, SDA, SCL);
void OLEDinit();

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
    Serial.begin(115200);
    BTinit();
    OLEDinit();
    IRinit();
}

void loop() {
    if (BT.available()) {
        // Mode
        switch (BT.read()) {
            case 'a':
                display.clear();
                display.drawString(0, 10, "ModeAuto");
                display.display();
                delay(1000);
                break;

            case 'A':
                display.clear();
                display.drawString(0, 10, "ModeHandle");
                display.display();
                delay(1000);
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
    BT.begin("BagaNono-Robot");
    BT.setPin("899819");
    Serial.printf("BT initial ok and ready to pair. \r\n");
}

//  OLED
void OLEDinit() {
    display.init();
    display.flipScreenVertically();
    display.setFont(ArialMT_Plain_16);
    display.drawStringMaxWidth(0, 0, 128, "BagaNono BaseBase");
    display.drawStringMaxWidth(0, 34, 128, "OLED turn on");
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