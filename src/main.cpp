/*========================================

            Include

*interface
command ending: \r
char delay: 5

*Repeat
Delay: 0
Period: 5
========================================*/
/*[------------------------------------------------]

    WeMos D1 Mini Motor Drive Shield (TB6612)

[------------------------------------------------]*/
#include <Arduino.h>
#include "WEMOS_Motor.h"
Motor Left_FRONT(0x2E, _MOTOR_A, 1000);   // LeftFront
Motor Left_REAR(0x2E, _MOTOR_B, 1000);    // LeftRear
Motor Right_FRONT(0x30, _MOTOR_B, 1000);  // RightFront
Motor Right_REAR(0x30, _MOTOR_A, 1000);   // RightRear
// Steer為Y軸陀螺儀讀取值
int Slide_Speed = 0, Steer = 0, Calculate_Speed = 0;
/*[------------------------------------------------]

    OLED Display 0.96" I2C 128x64 white (SSD1306)

[------------------------------------------------]*/
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <SPI.h>
#include <Wire.h>
Adafruit_SSD1306 display(128, 64, &Wire, -1);
void OLED_Init() {
    display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
    display.display();
    for (uint8_t i = 0; i < 10; i++) {  //強制開機時先停止馬達，避免重啟後馬達失控
        Left_FRONT.setmotor(_SHORT_BRAKE);
        Left_REAR.setmotor(_SHORT_BRAKE);
        Right_FRONT.setmotor(_SHORT_BRAKE);
        Right_REAR.setmotor(_SHORT_BRAKE);
        delay(100);
    }
    display.clearDisplay();
    display.setTextColor(WHITE);
    display.setCursor(0, 0);
    display.ttyPrintln("OLED Init!");
    display.display();
}

/*[------------------------------------------------]

    AsyncElegantOTA
    192.168.137.

[------------------------------------------------]*/
#include <AsyncElegantOTA.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <WiFi.h>
const char* ssid = "ご注文はWIFIですか?";
const char* password = "111111111";
const char compile_date[] = __DATE__ " " __TIME__;                  // Provided by compiler at compile time.
String readBuff = "", readBuff_subStr = "", readBuff_subStr2 = "";  //客戶端讀取用緩存
int readBuff_Int = 0, readBuff_Int2 = 0;
uint8_t WifiTryCount = 0;
WiFiServer TCPclient_server(443);  //聲明服務器對象
WiFiClient client = TCPclient_server.available();
AsyncWebServer OTA_server(80);
#define LED_BUTTON 2
void WIFI_Init() {
    pinMode(LED_BUTTON, OUTPUT);
    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);  //關閉STA模式下WIFI休眠，提高響應速度
    WiFi.disconnect();
    delay(250);
    WiFi.begin(ssid, password);
    // Wait for connection
    while (WiFi.status() != WL_CONNECTED) {
        delay(250);
        display.ttyPrint(".");
        display.display();
        if (WifiTryCount++ >= 20) {  //嘗試20次未連上網，重新啟動
            ESP.restart();
        }
    }
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("[Connected]");
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
    display.ttyPrintln("WaitingForConnection");
    display.display();
    while (!client) {  //  等待連接
        client = TCPclient_server.available();
        delay(300);
        display.ttyPrint("#");
        display.display();
        digitalWrite(LED_BUTTON, HIGH);
        delay(300);
        digitalWrite(LED_BUTTON, LOW);
        delay(300);
    }
    delay(100);
    digitalWrite(LED_BUTTON, LOW);
    client.println("=>[Client connented]");
    client.println("Compile timestamp: ");
    client.println(compile_date);
}
/*[-----------------------------------------------]

    Electromagnet Module

[------------------------------------------------]*/
#define Solenoid_PIN 33
void SOLENOID_Init() {
    pinMode(Solenoid_PIN, OUTPUT);
}
/*[------------------------------------------------]

    Infrared Tracking Sensor Module 5 Channel (TCRT5000)

[------------------------------------------------]*/
#define leftA_track_PIN 32
#define leftB_track_PIN 35
#define middle_track_PIN 34
#define righA_track_PIN 39
#define righB_track_PIN 36
int sensor[5];                                        //= {1, 1, 1, 1, 1};
float Kp = 43, Ki = 29, Kd = 0;                       // pid彎道參數
float error = 0, P = 0, I = 0, D = 0, PID_value = 0;  // pid直道參數
float previous_error = 0, previous_I = 0;             //誤差值
int initial_motor_speed = 28;
void IR_Init() {
    pinMode(leftA_track_PIN, INPUT);
    pinMode(leftB_track_PIN, INPUT);
    pinMode(middle_track_PIN, INPUT);
    pinMode(righA_track_PIN, INPUT);
    pinMode(righB_track_PIN, INPUT);
    display.ttyPrintln("IR Init!");
    display.display();
}
void Read_sensor_values() {
    sensor[0] = digitalRead(leftA_track_PIN);
    sensor[1] = digitalRead(leftB_track_PIN);
    sensor[2] = digitalRead(middle_track_PIN);
    sensor[3] = digitalRead(righA_track_PIN);
    sensor[4] = digitalRead(righB_track_PIN);

    if (sensor[0] == 0 && sensor[1] != 0 && sensor[2] != 0 && sensor[3] != 0 && sensor[4] != 0) {
        error = 2;  // 01111
    } else if (sensor[0] == 0 && sensor[1] == 0 && sensor[2] != 0 && sensor[3] != 0 && sensor[4] != 0) {
        error = 2;  // 00111
    } else if (sensor[1] == 0 && sensor[0] != 0 && sensor[2] != 0 && sensor[3] != 0 && sensor[4] != 0) {
        error = 1;  // 10111
    } else if (sensor[1] == 0 && sensor[0] != 0 && sensor[2] == 0 && sensor[3] != 0 && sensor[4] != 0) {
        error = 1;  // 10011
    } else if (sensor[2] == 0 && sensor[1] != 0 && sensor[0] != 0 && sensor[3] != 0 && sensor[4] != 0) {
        error = 0;  // 11011
    } else if (sensor[3] == 0 && sensor[1] != 0 && sensor[2] != 0 && sensor[0] != 0 && sensor[4] != 0) {
        error = -1;  // 11101
    } else if (sensor[3] == 0 && sensor[1] != 0 && sensor[2] == 0 && sensor[0] != 0 && sensor[4] != 0) {
        error = -1;  // 11001
    } else if (sensor[4] == 0 && sensor[1] != 0 && sensor[2] != 0 && sensor[3] == 0 && sensor[0] != 0) {
        error = -2;  // 11100
    } else if (sensor[4] == 0 && sensor[1] != 0 && sensor[2] != 0 && sensor[3] != 0 && sensor[0] != 0) {
        error = -2;  // 11110
    } else if (sensor[4] != 0 && sensor[1] != 0 && sensor[2] != 0 && sensor[3] != 0 && sensor[0] != 0) {
        error = 0;  // 11111
    }
    client.print("Sensor: ");
    client.print(sensor[0]);
    client.print(", ");
    client.print(sensor[1]);
    client.print(", ");
    client.print(sensor[2]);
    client.print(", ");
    client.print(sensor[3]);
    client.print(", ");
    client.println(sensor[4]);
}
void Calc_pid() {
    P = error;
    I = I + error;
    D = error - previous_error;
    PID_value = (Kp * P) + (Ki * I) + (Kd * D);
    previous_error = error;
}

void WheelSpeed(int speedL, int speedR) {  //速度設定範圍(-100,100)
    if (speedL > 0) {
        Left_FRONT.setmotor(_CW, speedL);
        Left_REAR.setmotor(_CW, speedL);

    } else if (speedL < 0) {
        speedL = -speedL;
        Left_FRONT.setmotor(_CCW, speedL);
        Left_REAR.setmotor(_CCW, speedL);
    } else {
        Left_FRONT.setmotor(_SHORT_BRAKE);
        Left_REAR.setmotor(_SHORT_BRAKE);
    }

    if (speedR > 0) {
        Right_FRONT.setmotor(_CCW, speedR);
        Right_REAR.setmotor(_CCW, speedR);
    } else if (speedR < 0) {
        speedR = -speedR;
        Right_FRONT.setmotor(_CW, speedR);
        Right_REAR.setmotor(_CW, speedR);
    } else {
        Right_FRONT.setmotor(_SHORT_BRAKE);
        Right_REAR.setmotor(_SHORT_BRAKE);
    }
}

void Tracking() {
    int left_motor_speed = initial_motor_speed - PID_value;
    int right_motor_speed = initial_motor_speed + PID_value;

    if (left_motor_speed > 100) {
        left_motor_speed = 100;
    }
    if (left_motor_speed < -100) {
        left_motor_speed = -100;
    }
    if (right_motor_speed > 100) {
        right_motor_speed = 100;
    }
    if (right_motor_speed < -100) {
        right_motor_speed = -100;
    }
    WheelSpeed(left_motor_speed, right_motor_speed);
    client.print("L:" + right_motor_speed);
    client.print(", ");
    client.print("R:" + left_motor_speed);
    client.print("P:");
    client.print(P);
    client.print(", ");
    client.print("I:");
    client.print(I);
    client.print(", ");
    client.print("D:");
    client.println(D);
}
/*[-----------------------------------------------]

    Adafruit 16-Channel Servo Driver (PCA9685)

[------------------------------------------------]*/
#include "HCPCA9685.h"
int ServoChannel = 0, ServoPosition = 0;
HCPCA9685 PCA9685(0x40);
/*
由於0.5/20*4096=102的寄存器值。角度如下：
0.5ms————–0度：0.5/20*4096 = 102
1.0ms————45度：1/20*4096 = 204
1.5ms————90度：1.5/20*4096 = 306
2.0ms———–135度：2/20*4096 = 408
2.5ms———–180度：2.5/20*4096 =510

因實際使用的偏差，除了0度以及180度，其他需要乘以0.915係數。最後寄存器值如下：
0.5ms————–0度：0.5/20*4096 = 102
1.0ms————45度：1/20*4096 = 204 * 0.915 = 187
1.5ms————90度：1.5/20*4096 = 306 * 0.915 = 280
2.0ms———–135度：2/20*4096 = 408 * 0.915 = 373
2.5ms———–180度：2.5/20*4096 =510
*/
void Servo_Init() {
    PCA9685.Init(SERVO_MODE);
    PCA9685.Sleep(false);
}

void ServoTurn(int _ServoChannel, int _ServoPosition) {
    _ServoPosition = map(_ServoPosition, -90, 90, 0, 420);
    switch (_ServoChannel) {
        //全部歸零
        case 0:
            for (int chan = 1; chan < 15; chan++) {
                PCA9685.Servo(chan, 210);
                delay(10);
            }
            break;
        //大球夾爪
        case 1:
            PCA9685.Servo(1, _ServoPosition);
            break;
        case 2:
            PCA9685.Servo(2, _ServoPosition);
            break;
        //小夾爪
        case 3:
            PCA9685.Servo(12, _ServoPosition);
            break;
        case 4:
            PCA9685.Servo(13, _ServoPosition);
            break;
        case 5:
            PCA9685.Servo(14, _ServoPosition);
            break;
        case 6:
            PCA9685.Servo(15, (_ServoPosition));
            break;
    }
}
/*[-----------------------------------------------]

    Mode

[------------------------------------------------]*/
void ModeAuto() {  //循跡模式
    error = P = I = D = PID_value = 0;
    previous_error = previous_I = 0;
    client.println("=>[ModeAuto]");
    display.ttyPrintln("=>[ModeAuto]");
    display.display();
    while (client.connected()) {
        if (client.available()) {
            readBuff = client.readStringUntil('\r');
            readBuff.trim();
            if (readBuff.startsWith("STOP")) {
                readBuff = "";
                break;  //跳出循跡的 while 迴圈
            }
            readBuff = "";
        }
        Read_sensor_values();
        Calc_pid();
        Tracking();
    }
    Left_FRONT.setmotor(_SHORT_BRAKE);
    Left_REAR.setmotor(_SHORT_BRAKE);
    Right_FRONT.setmotor(_SHORT_BRAKE);
    Right_REAR.setmotor(_SHORT_BRAKE);
    client.println("=>[Track STOP]");
    display.ttyPrintln("=>[Track STOP]");
    display.display();
}

void ModeHandle() {  //手動模式(加速度計)
    client.println("=>[ModeHandle]");
    display.ttyPrintln("=>[ModeHandle]");
    display.display();
    while (client.connected()) {
        if (client.available()) {
            readBuff = client.readStringUntil('\r');
            client.print(readBuff + '\r');
            if (readBuff.startsWith("STOP")) {
                readBuff = "";
                break;  //若接收到STOP則跳出ModeHandle迴圈
            } else if (readBuff.startsWith("SPEED")) {
                readBuff_subStr = readBuff.substring(5);
                Slide_Speed = readBuff_subStr.toInt();  //讀取滑桿速度值作為主速度
                readBuff = "";
            } else if (readBuff.startsWith("STEER")) {
                readBuff_subStr = readBuff.substring(5);
                Steer = readBuff_subStr.toInt();
                readBuff = "";
            } else if (readBuff.startsWith("SERVO")) {
                readBuff_subStr = readBuff.substring(5, 6);
                readBuff_subStr2 = readBuff.substring(7);
                ServoChannel = readBuff_subStr.toInt();
                ServoPosition = readBuff_subStr2.toInt();
                ServoTurn(ServoChannel, ServoPosition);
                readBuff = "";
            } else if (readBuff.startsWith("MAGNET0")) {
                digitalWrite(Solenoid_PIN, LOW);
                readBuff = "";
            } else if (readBuff.startsWith("MAGNET1")) {
                digitalWrite(Solenoid_PIN, HIGH);
                readBuff = "";
            }
            if (Steer >= 0) {  //右轉時，左輪為滑桿速度，右輪為計算速度
                Calculate_Speed = round(Slide_Speed - (Slide_Speed * Steer / 50));
                WheelSpeed(Slide_Speed, Calculate_Speed);
            } else if (Steer < 0) {  //左轉時，左輪為計算速度，右輪為滑桿速度
                Calculate_Speed = round(Slide_Speed - (Slide_Speed * (-Steer) / 50));
                WheelSpeed(Calculate_Speed, Slide_Speed);
            }
        }
    }
    Left_FRONT.setmotor(_SHORT_BRAKE);
    Left_REAR.setmotor(_SHORT_BRAKE);
    Right_FRONT.setmotor(_SHORT_BRAKE);
    Right_REAR.setmotor(_SHORT_BRAKE);
    client.println("=>[STOP Handle]");
    display.ttyPrintln("=>[STOP Handle]");
    display.display();
}
/*========================================

    Setup & Loop

========================================*/
void setup() {
    Serial.begin(115200);
    Wire.setClock(400000);
    OLED_Init();
    delay(250);
    Servo_Init();
    delay(250);
    SOLENOID_Init();
    delay(250);
    IR_Init();
    delay(250);
    WIFI_Init();
    delay(250);
}

void loop() {
    while (client.connected()) {  //如果客戶端處於連接狀態
        if (client.available()) {
            readBuff = client.readStringUntil('\r');
            if (readBuff.startsWith("AUTO")) {  // 模式切換
                ModeAuto();
            } else if (readBuff.startsWith("HANDLE")) {
                ModeHandle();
            } else if (readBuff.startsWith("KP")) {
                readBuff_subStr = readBuff.substring(2);
                Kp = readBuff_subStr.toFloat();
                client.println(Kp);
                readBuff = "";

            } else if (readBuff.startsWith("KI")) {
                readBuff_subStr = readBuff.substring(2);
                Ki = readBuff_subStr.toFloat();
                client.println(Ki);
                readBuff = "";

            } else if (readBuff.startsWith("KD")) {
                readBuff_subStr = readBuff.substring(2);
                Kd = readBuff_subStr.toFloat();
                client.println(Kd);
                readBuff = "";

            } else if (readBuff.startsWith("TSPEED")) {
                readBuff_subStr = readBuff.substring(6);
                initial_motor_speed = readBuff_subStr.toInt();
                client.println(initial_motor_speed);
                readBuff = "";
            }
        }
    }
    Left_FRONT.setmotor(_SHORT_BRAKE);
    Left_REAR.setmotor(_SHORT_BRAKE);
    Right_FRONT.setmotor(_SHORT_BRAKE);
    Right_REAR.setmotor(_SHORT_BRAKE);
    PCA9685.Sleep(true);
    display.ttyPrintln("=>[Client lost]");
    display.display();
    display.ttyPrintln("RESTART...");
    display.ttyPrintln("---");
    display.display();
    delay(250);
    ESP.restart();
}