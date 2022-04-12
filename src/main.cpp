/*========================================

            Include

*interface
command ending: \r
char delay: 5

*Repeat
Delay: 0
Period: 100
========================================*/
/*[------------------------------------------------]

    1.2 OLED Display 0.96" I2C 128x64 white (SSD1306)

[------------------------------------------------]*/
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
Adafruit_SSD1306 display(128, 64, &Wire, -1);
void OLEDinit() {
    display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
    display.display();
    delay(2000);
    display.clearDisplay();
    display.setTextColor(WHITE);
    display.setCursor(0, 0);
    display.ttyPrintln("OLED Init!");
    display.display();
}
/*[------------------------------------------------]

    1.0 AsyncElegantOTA
        192.168.137.110

[------------------------------------------------]*/
#include <AsyncElegantOTA.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <WiFi.h>
const char* ssid = "ご注文はWIFIですか?";
const char* password = "111111111";
// Provided by compiler at compile time.
const char compile_date[] = __DATE__ " " __TIME__;
//客戶端讀取用緩存
String readBuff = "";
String readBuff_subStr = "";
int readBuff_Int = 0;
WiFiServer TCPclient_server(443);  //声明服务器对象
WiFiClient client = TCPclient_server.available();
AsyncWebServer OTA_server(80);
void WIFIinit() {
    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);  //关闭STA模式下wifi休眠，提高响应速度
    WiFi.disconnect();
    delay(500);
    WiFi.begin(ssid, password);
    // Wait for connection
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        display.ttyPrint(".");
        display.display();
    }
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("[Connected]");
    display.println("");
    display.println("Compile timestamp: ");
    display.println(compile_date);
    display.println("");
    display.println("IP address:");
    display.print(WiFi.localIP());
    display.print(":");
    display.println(443);
    display.println("");
    display.display();

    AsyncElegantOTA.begin(&OTA_server);  // Start ElegantOTA
    OTA_server.begin();
    TCPclient_server.begin();
    TCPclient_server.setNoDelay(true);
    display.ttyPrintln("WIFI Init!");
    display.display();
}
/*[------------------------------------------------]

    1.3 Infrared Tracking Sensor Module 5 Channel (TCRT5000)

[------------------------------------------------]*/
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
void IRinit() {
    pinMode(leftA_track_PIN, INPUT);
    pinMode(leftB_track_PIN, INPUT);
    pinMode(middle_track_PIN, INPUT);
    pinMode(righA_track_PIN, INPUT);
    pinMode(righB_track_PIN, INPUT);
    display.ttyPrintln("IR Init!");
    display.display();
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
/*[------------------------------------------------]

    1.4  WeMos D1 Mini Motor Drive Shield (TB6612)

[------------------------------------------------]*/
#include "WEMOS_Motor.h"
Motor Left_Front(0x30, _MOTOR_A, 1000);    // LeftFront
Motor Left_REAR(0x30, _MOTOR_B, 1000);     // LeftRear
Motor Right_FRONT(0x2E, _MOTOR_A, 1000);   // RightFront
Motor Right_REAR(0x2E, _MOTOR_B, 1000);    // RightRear
void WheelSpeed(int speedL, int speedR) {  //速度设定范围(-100,100)
    if (speedR > 0) {
        Left_Front.setmotor(_CCW, speedR);
        Left_REAR.setmotor(_CCW, speedR);

    } else {
        Left_Front.setmotor(_CW, speedR);
        Left_REAR.setmotor(_CW, speedR);
    }

    if (speedL > 0) {
        Right_FRONT.setmotor(_CCW, speedL);
        Right_REAR.setmotor(_CCW, speedL);
    } else {
        Right_FRONT.setmotor(_CW, speedL);
        Right_REAR.setmotor(_CW, speedL);
    }
}
void Tracking() {
    int left_motor_speed = initial_motor_speed - PID_value;
    int right_motor_speed = initial_motor_speed + PID_value;

    if (left_motor_speed < -100) {
        left_motor_speed = -100;
    }
    if (left_motor_speed > 100) {
        left_motor_speed = 100;
    }
    WheelSpeed(left_motor_speed, right_motor_speed);
    display.ttyPrint("L:");
    display.ttyPrint((String)right_motor_speed);
    display.ttyPrint(",");
    display.ttyPrint("R:");
    display.ttyPrintln((String)left_motor_speed);
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
/*[-----------------------------------------------]

    1.5  Adafruit 16-Channel Servo Driver (PCA9685)

[------------------------------------------------]*/
#include <Adafruit_PWMServoDriver.h>
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();  // default address 0x40
#define MIN_PULSE_WIDTH 544
#define MAX_PULSE_WIDTH 2400
#define FREQUENCY 50
int pulse_wide, pulse_width;
int ServoMotor0 = 0;  // 0~15 channels
void ServoDriverinit() {
    pwm.begin();
    pwm.setPWMFreq(FREQUENCY);
    display.ttyPrintln("Servo Driver Init!");
    display.display();
}
/*[-----------------------------------------------]

    Mode

[------------------------------------------------]*/
void ModeAuto() {
    client.println("=>[ModeAuto]");
    display.ttyPrintln("=>[ModeAuto]");
    display.display();
    while (1) {
        if (client.available()) {
            readBuff = client.readStringUntil('\r');
            readBuff.trim();
            if (readBuff.startsWith("STOP")) {
                readBuff = "";
                break;  //跳出循跡的 while 迴圈
            }
            readBuff = "";
        }
        client.println("Tracking.");
        client.println("Tracking...");
        read_sensor_values();
        client.print("Sensor:" + sensor[0] + sensor[1] + sensor[2] + sensor[3] + sensor[4]);
        calc_pid();
        Tracking();
    }
    Left_Front.setmotor(_SHORT_BRAKE);
    Left_REAR.setmotor(_SHORT_BRAKE);
    Right_FRONT.setmotor(_SHORT_BRAKE);
    Right_REAR.setmotor(_SHORT_BRAKE);
    client.println("Track STOP");
    display.ttyPrintln("Track STOP");
    display.display();
}
void ModeHandle() {
    client.println("=>[ModeHandle]");
    display.ttyPrintln("=>[ModeHandle]");
    display.display();
    while (1) {
        if (client.available()) {  //客戶端有發送，則接收
            readBuff = client.readStringUntil('\r');
            readBuff.trim();
            client.println(readBuff);
            if (readBuff.startsWith("STOP")) {
                readBuff = "";
                break;  //若接收到STOP則跳出ModeHandle迴圈
            }
        }
        if (readBuff.startsWith("RIGHT")) {
            readBuff_subStr = readBuff.substring(5);  //尋找從索引位置到末端的字符串
            readBuff_Int = readBuff_subStr.toInt();
            if (readBuff_Int > 0) {
                Right_FRONT.setmotor(_CW, readBuff_Int);
                Right_REAR.setmotor(_CW, readBuff_Int);
            } else {
                Right_FRONT.setmotor(_CCW, readBuff_Int);
                Right_REAR.setmotor(_CCW, readBuff_Int);
            }

            display.ttyPrint("RIGHT:");
            display.ttyPrint(readBuff_subStr);
            display.display();
            display.ttyPrintln("..");
            display.display();
            readBuff = "";
        } else if (readBuff.startsWith("LEFT")) {
            Left_Front.setmotor(_CW, 50);
            Left_REAR.setmotor(_CW, 50);
            display.ttyPrint("LEFT:");
            display.ttyPrint(speedStr);
            display.display();
            display.ttyPrintln("..");
            display.display();
        }
    }
    Left_Front.setmotor(_SHORT_BRAKE);
    Left_REAR.setmotor(_SHORT_BRAKE);
    Right_FRONT.setmotor(_SHORT_BRAKE);
    Right_REAR.setmotor(_SHORT_BRAKE);
    client.println("Handle STOP");
    display.ttyPrintln("Handle STOP");
    display.display();
}
/*========================================

            Setup & Loop

========================================*/
void setup() {
    Serial.begin(115200);
    Wire.setClock(400000);
    Left_Front.setmotor(_SHORT_BRAKE);
    Left_REAR.setmotor(_SHORT_BRAKE);
    Right_FRONT.setmotor(_SHORT_BRAKE);
    Right_REAR.setmotor(_SHORT_BRAKE);
    OLEDinit();
    delay(1000);
    ServoDriverinit();
    delay(1000);
    IRinit();
    delay(1000);
    WIFIinit();
    delay(1000);
}

void loop() {
    client = TCPclient_server.available();
    if (client) {  //提示客戶端已連接
        display.ttyPrintln("[client connented]");
        display.display();
    }
    while (client.connected()) {  //如果客戶端處於連接狀態
        if (client.available()) {
            readBuff = client.readStringUntil('\r');
            readBuff.trim();  //消除多於空白
            // Mode
            if (readBuff.startsWith("AUTO")) {
                ModeAuto();
            } else if (readBuff.startsWith("HANDLE")) {
                // ModeHandle();
                client.println("Received: " + readBuff);
                display.ttyPrintln("Recieve:" + readBuff);
                display.display();
                client.println("--");
                display.ttyPrintln("--");
                display.display();
            }
        }
    }
}
