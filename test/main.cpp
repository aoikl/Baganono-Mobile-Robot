/*==============================

        1.include

==============================*/
//    1.0  AsyncElegantOTA    //
// 192.168.43.136
// #include <Arduino.h>
// #include <AsyncElegantOTA.h>
// #include <AsyncTCP.h>
// #include <ESPAsyncWebServer.h>
// #include <WiFi.h>
// //ご注文はWIFIですか?    //111111111
// const char* ssid = "ご注文はWIFIですか?";
// const char* password = "111111111";
// AsyncWebServer server(80);
// void OTAinit();

//    1.1  Bluetooth (NodeMCU-32s)    //
#include <BluetoothSerial.h>
BluetoothSerial BT;
void BTinit();

//    1.2  OLED Display 0.96" I2C 128x64 white (SSD1306)    //
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Fonts/FreeSerifBoldItalic7pt7b.h>
#include <SPI.h>
#include <Wire.h>
Adafruit_SSD1306 display(128, 64, &Wire, -1);
void OLEDinit();

//    1.3  Infrared Tracking Sensor Module 5 Channel (TCRT5000)    //
#define leftA_track_PIN 32
#define leftB_track_PIN 35
#define middle_track_PIN 34
#define righA_track_PIN 39
#define righB_track_PIN 36
int sensor[5];                                        //= {1, 1, 1, 1, 1};
float Kp = 10, Ki = 0.5, Kd = 0;                      // pid弯道参数参数
float error = 0, P = 0, I = 0, D = 0, PID_value = 0;  // pid直道参数
float previous_error = 0, previous_I = 0;             //误差值
int initial_motor_speed = 60;
void IRinit();
void read_sensor_values();  //读取初值
void calc_pid();            //计算pid
void motorsWrite(int speedL, int speedR);
void motor_control();

//    1.4  WeMos D1 Mini Motor Drive Shield (TB6612)    //
#include "WEMOS_Motor.h"
Motor MotorLeft_F(0x30, _MOTOR_A, 1000);   // LeftFront
Motor MotorLeft_R(0x30, _MOTOR_B, 1000);   // LeftRear
Motor MotorRight_F(0x2E, _MOTOR_A, 1000);  // RightFront
Motor MotorRight_R(0x2E, _MOTOR_B, 1000);  // RightRear

//    1.5  Adafruit 16-Channel Servo Driver (PCA9685)    //
#include <Adafruit_PWMServoDriver.h>
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();  // default address 0x40
#define MIN_PULSE_WIDTH 544
#define MAX_PULSE_WIDTH 2400
#define FREQUENCY 50
int pulse_wide, pulse_width;
int ServoMotor0 = 0;  // 0~15 channels
void ServoDriverinit();

/*==============================

        2.Setup & Loop

==============================*/
void setup() {
    Serial.begin(115200);
    display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
    MotorLeft_F.setmotor(_SHORT_BRAKE);
    MotorLeft_R.setmotor(_SHORT_BRAKE);
    MotorRight_F.setmotor(_SHORT_BRAKE);
    MotorRight_R.setmotor(_SHORT_BRAKE);
    OLEDinit();
    delay(1000);
    // OTAinit();
    delay(1000);
    ServoDriverinit();
    delay(1000);
    IRinit();
    BTinit();
    delay(1000);
}

void loop() {
    if (BT.available()) {
        // Mode
        display.clearDisplay();
        display.setCursor(0, 14);
        display.print("Wait for setting Modes");
        display.display();
        switch (BT.read()) {
            case 'a':
                display.clearDisplay();
                display.setCursor(0, 14);
                display.print("ModeAuto");
                display.display();
                delay(2000);
                while (BT.read() != 's') {
                    display.clearDisplay();
                    display.setCursor(0, 14);
                    display.print("tracking.");
                    display.display();
                    delay(100);
                    display.setCursor(0, 14);
                    display.println("tracking...");
                    display.display();
                    delay(100);
                    read_sensor_values();
                    display.setCursor(0, 28);
                    display.print("sensor:");
                    display.print(sensor[0]);
                    display.print(sensor[1]);
                    display.print(sensor[2]);
                    display.print(sensor[3]);
                    display.println(sensor[4]);
                    display.display();
                    calc_pid();
                    display.setCursor(0, 42);
                    motor_control();
                    delay(500);
                }
                MotorLeft_F.setmotor(_SHORT_BRAKE);
                MotorLeft_R.setmotor(_SHORT_BRAKE);
                MotorRight_F.setmotor(_SHORT_BRAKE);
                MotorRight_R.setmotor(_SHORT_BRAKE);
                display.clearDisplay();
                display.setCursor(0, 14);
                display.print("track stop");
                display.display();
                break;
            case 'A':
                display.clearDisplay();
                display.setCursor(0, 14);
                display.print("ModeHandle");
                display.display();
                delay(1000);
                while (BT.read() != 's') {
                    display.clearDisplay();
                    display.setCursor(0, 14);
                    display.print("PressBtn2control.");
                    display.display();
                    delay(10);
                    display.setCursor(0, 14);
                    display.print("PressBtn2control..");
                    display.display();
                    // pulse_wide = map(BT.read(), 0, 1023, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
                    delay(10);
                    switch (BT.read()) {
                        case 'F':
                            MotorLeft_F.setmotor(_CCW, 50);
                            MotorLeft_R.setmotor(_CCW, 50);
                            MotorRight_F.setmotor(_CCW, 50);
                            MotorRight_R.setmotor(_CCW, 50);
                            display.setCursor(14, 28);
                            display.print("F.");
                            display.display();
                            display.setCursor(14, 28);
                            display.print("F..");
                            display.display();
                            break;
                        case 'B':
                            MotorLeft_F.setmotor(_CW, 50);
                            MotorLeft_R.setmotor(_CW, 50);
                            MotorRight_F.setmotor(_CW, 50);
                            MotorRight_R.setmotor(_CW, 50);
                            display.setCursor(14, 28);
                            display.print("B.");
                            display.display();
                            display.setCursor(14, 28);
                            display.print("B..");
                            display.display();
                            break;
                        case 'R':
                            MotorLeft_F.setmotor(_CCW, 50);
                            MotorLeft_R.setmotor(_CCW, 50);
                            MotorRight_F.setmotor(_CW, 50);
                            MotorRight_R.setmotor(_CW, 50);
                            display.setCursor(14, 28);
                            display.print("R");
                            display.display();
                            display.setCursor(14, 28);
                            display.print("R..");
                            display.display();
                            break;
                        case 'L':
                            MotorLeft_F.setmotor(_CW, 50);
                            MotorLeft_R.setmotor(_CW, 50);
                            MotorRight_F.setmotor(_CCW, 50);
                            MotorRight_R.setmotor(_CCW, 50);
                            display.setCursor(14, 28);
                            display.print("L");
                            display.display();
                            display.setCursor(14, 28);
                            display.print("L..");
                            display.display();
                            break;
                        case 'P':
                            MotorLeft_F.setmotor(_SHORT_BRAKE);
                            MotorLeft_R.setmotor(_SHORT_BRAKE);
                            MotorRight_F.setmotor(_SHORT_BRAKE);
                            MotorRight_R.setmotor(_SHORT_BRAKE);
                            display.setCursor(14, 28);
                            display.print("P");
                            display.display();
                            display.setCursor(14, 28);
                            display.print("P..");
                            display.display();
                            break;
                        default:
                            display.setCursor(14, 28);
                            display.print("Default");
                            display.display();
                            display.setCursor(14, 28);
                            display.print("Default..");
                            display.display();
                            break;
                    }
                }
                break;
        }
    }
}

/*==============================

        3.Function

==============================*/
//    3.0  AsyncElegantOTA    //

// void OTAinit() {
//     WiFi.mode(WIFI_STA);
//     WiFi.begin(ssid, password);
//     Serial.println("");

//     // Wait for connection
//     while (WiFi.status() != WL_CONNECTED) {
//         delay(500);
//         Serial.print(".");
//     }
//     Serial.println("");
//     Serial.print("Connected to ");
//     Serial.println(ssid);
//     Serial.print("IP address: ");
//     Serial.println(WiFi.localIP());

//     server.on("/", HTTP_GET, [](AsyncWebServerRequest* request) {
//         request->send(200, "text/plain", "Hi! I am ESP32.");
//     });

//     AsyncElegantOTA.begin(&server);  // Start ElegantOTA
//     server.begin();
//     Serial.println("HTTP server started");
// }

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
