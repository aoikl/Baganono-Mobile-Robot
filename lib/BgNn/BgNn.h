#ifndef BGNN_H
#define BGNN_H
//    1.0  AsyncElegantOTA    //
// 192.168.43.136
#include <AsyncElegantOTA.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <WiFi.h>
#include "Arduino.h"
extern const char* ssid;
extern const char* password;
void OTAinit();

//    1.1  Bluetooth (NodeMCU-32s)    //
#include <BluetoothSerial.h>
void BTinit();

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Fonts/FreeSerifBoldItalic7pt7b.h>
#include <SPI.h>
#include <Wire.h>
void OLED_init();
void IRinit();
void read_sensor_values();
void calc_pid();
void motorsWrite(int speedL, int speedR);
void motor_control();

#include <Adafruit_PWMServoDriver.h>
#include "WEMOS_Motor.h"
void ServoDriverinit();

extern const int leftA_track_PIN;
extern const int leftB_track_PIN;
extern const int middle_track_PIN;
extern const int righA_track_PIN;
extern const int righB_track_PIN;

extern int sensor;
extern float Kp, Ki, Kd;
extern float error, P, I, D, PID_value;
extern float previous_error, previous_I;
extern int initial_motor_speed;

extern int pulse_wide, pulse_width;
extern int ServoMotor;

#endif