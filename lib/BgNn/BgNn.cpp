#include "BgNn.h"
#include <Arduino.h>
#include <AsyncElegantOTA.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <WiFi.h>
AsyncWebServer server(80);

#include <BluetoothSerial.h>
BluetoothSerial BT;

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Fonts/FreeSerifBoldItalic7pt7b.h>
#include <SPI.h>
#include <Wire.h>
Adafruit_SSD1306 display(128, 64, &Wire, -1);

#include "WEMOS_Motor.h"
Motor MotorLeft_F(0x30, _MOTOR_A, 1000);   // LeftFront
Motor MotorLeft_R(0x30, _MOTOR_B, 1000);   // LeftRear
Motor MotorRight_F(0x2E, _MOTOR_A, 1000);  // RightFront
Motor MotorRight_R(0x2E, _MOTOR_B, 1000);  // RightRear

#include <Adafruit_PWMServoDriver.h>
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();  // default address 0x40

const int leftA_track_PIN = 32;
const int leftB_track_PIN = 35;
const int middle_track_PIN = 34;
const int righA_track_PIN = 39;
const int righB_track_PIN = 36;

int sensor[5];
float Kp = 10, Ki = 0.5, Kd = 0;
float error = 0, P = 0, I = 0, D = 0, PID_value = 0;
float previous_error = 0, previous_I = 0;
int initial_motor_speed = 60;

int pulse_wide = 0, pulse_width = 0;
int ServoMotor = 0;
/*==============================

        3.Function

==============================*/
//    3.0  AsyncElegantOTA    //

void OTAinit() {
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);
    Serial.println("");

    // Wait for connection
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("");
    Serial.print("Connected to ");
    Serial.println(ssid);
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());

    server.on("/", HTTP_GET, [](AsyncWebServerRequest* request) {
        request->send(200, "text/plain", "Hi! I am ESP32.");
    });

    AsyncElegantOTA.begin(&server);  // Start ElegantOTA
    server.begin();
    Serial.println("HTTP server started");
}

//    3.1  Bluetooth (NodeMCU-32s)    //
void BTinit() {
    BT.begin("BagaNono-Robot");
    // BT.setPin("899819");
    display.clearDisplay();
    display.setCursor(0, 14);
    display.print("BT ready to pair");
    display.display();
}

//    3.2  OLED Display 0.96" I2C 128x64 white (SSD1306)    //
void OLEDinit() {
    display.setFont(&FreeSerifBoldItalic7pt7b);
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.setCursor(0, 14);
    display.println("OLED");
    display.println("Turn on!");
    display.display();
}

//    3.3  Infrared Tracking Sensor Module 5 Channel (TCRT5000)    //
void IRinit() {
    pinMode(leftA_track_PIN, INPUT);
    pinMode(leftB_track_PIN, INPUT);
    pinMode(middle_track_PIN, INPUT);
    pinMode(righA_track_PIN, INPUT);
    pinMode(righB_track_PIN, INPUT);
}

void read_sensor_values() {
    sensor[0] = digitalRead(leftA_track_PIN);
    sensor[1] = digitalRead(leftB_track_PIN);
    sensor[2] = digitalRead(middle_track_PIN);
    sensor[3] = digitalRead(righA_track_PIN);
    sensor[4] = digitalRead(righB_track_PIN);

    if (sensor[0] == 0 && sensor[1] != 0 && sensor[2] != 0 && sensor[3] != 0 && sensor[4] != 0) {
        error = -2;
    } else if (sensor[0] == 0 && sensor[1] == 0 && sensor[2] != 0 && sensor[3] != 0 && sensor[4] != 0) {
        error = -2;
    } else if (sensor[1] == 0 && sensor[0] != 0 && sensor[2] != 0 && sensor[3] != 0 && sensor[4] != 0) {
        error = -1;
    } else if (sensor[1] == 0 && sensor[0] != 0 && sensor[2] == 0 && sensor[3] != 0 && sensor[4] != 0) {
        error = -1;
    } else if (sensor[2] == 0 && sensor[1] != 0 && sensor[0] != 0 && sensor[3] != 0 && sensor[4] != 0) {
        error = 0;
    } else if (sensor[2] == 0 && sensor[1] == 0 && sensor[0] != 0 && sensor[3] == 0 && sensor[4] != 0) {
        error = 0;
    } else if (sensor[3] == 0 && sensor[1] != 0 && sensor[2] != 0 && sensor[0] != 0 && sensor[4] != 0) {
        error = 1;
    } else if (sensor[3] == 0 && sensor[1] != 0 && sensor[2] == 0 && sensor[0] != 0 && sensor[4] != 0) {
        error = 1;
    } else if (sensor[4] == 0 && sensor[1] != 0 && sensor[2] != 0 && sensor[3] == 0 && sensor[0] != 0) {
        error = 2;
    } else if (sensor[4] == 0 && sensor[1] != 0 && sensor[2] != 0 && sensor[3] != 0 && sensor[0] != 0) {
        error = 2;
    } else {
        error = 0;
    }
}

void calc_pid() {
    P = error;
    I = I + error;
    D = error - previous_error;
    PID_value = (Kp * P) + (Ki * I) + (Kd * D);
    previous_error = error;
}

//速度设定范围(-100,100)
void motorsWrite(int speedL, int speedR) {
    if (speedR > 0) {
        MotorLeft_F.setmotor(_CCW, speedR);
        MotorLeft_R.setmotor(_CCW, speedR);

    } else {
        MotorLeft_F.setmotor(_CW, speedR);
        MotorLeft_R.setmotor(_CW, speedR);
    }

    if (speedL > 0) {
        MotorRight_F.setmotor(_CCW, speedL);
        MotorRight_R.setmotor(_CCW, speedL);
    } else {
        MotorRight_F.setmotor(_CW, speedL);
        MotorRight_R.setmotor(_CW, speedL);
    }
}

void motor_control() {
    int left_motor_speed = initial_motor_speed - PID_value;
    int right_motor_speed = initial_motor_speed + PID_value;

    if (left_motor_speed < -100) {
        left_motor_speed = -100;
    }
    if (left_motor_speed > 100) {
        left_motor_speed = 100;
    }
    motorsWrite(left_motor_speed, right_motor_speed);
    display.print("L:");
    display.print(right_motor_speed);
    display.print(",");
    display.print("R:");
    display.print(left_motor_speed);
    display.display();
    // display.setCursor(0, 56);
    // display.print("er:");
    // display.print(error);
    // display.print(",");
    // display.print("Kp:");
    // display.print(Kp);
    // display.print(",");
    // display.print("PID:");
    // display.print(PID_value);
    // display.display();
}

//    3.5  Adafruit 16-Channel Servo Driver (PCA9685)    //
void ServoDriverinit() {
    pwm.begin();
    pwm.setPWMFreq(FREQUENCY);
    display.clearDisplay();
    display.setCursor(0, 14);
    display.println("Servo Driver");
    display.println("Turn on!");
    display.display();
}
