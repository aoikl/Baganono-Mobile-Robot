#include <Arduino.h>
#include "WEMOS_Motor.h"
Motor MotorLeft_F(0x30, _MOTOR_A, 1000);   // LeftFront
Motor MotorLeft_R(0x30, _MOTOR_B, 1000);   // LeftRear
Motor MotorRight_F(0x2E, _MOTOR_A, 1000);  // RightFront
Motor MotorRight_R(0x2E, _MOTOR_B, 1000);  // RightRear
#include <BgNn.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Fonts/FreeSerifBoldItalic7pt7b.h>
#include <SPI.h>
#include <Wire.h>
Adafruit_SSD1306 display(128, 64, &Wire, -1);

void setup() {
    Serial.begin(115200);
    Wire.setClock(400000);
    display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
    MotorLeft_F.setmotor(_SHORT_BRAKE);
    MotorLeft_R.setmotor(_SHORT_BRAKE);
    MotorRight_F.setmotor(_SHORT_BRAKE);
    MotorRight_R.setmotor(_SHORT_BRAKE);
    OLEDinit();
    delay(1000);
    OTAinit();
    delay(1000);
    ServoDriverinit();
    delay(1000);
    IRinit();
    BTinit();
    delay(1000);
}
void loop() {
}
