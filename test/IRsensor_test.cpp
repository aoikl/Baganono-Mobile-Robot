//电机使用：5-6-9-10
//循迹模块使用：2-3-7-8-11

//-------------------------------------------------------------------//
//*******************************************************************//
///
//小车部分/
///

//           电机设置             //
#include <Arduino.h>
#define leftA_PIN 5
#define leftB_PIN 6
#define righA_PIN 9
#define righB_PIN 10

float Kp = 10, Ki = 0.5, Kd = 0;                      // pid弯道参数参数
float error = 0, P = 0, I = 0, D = 0, PID_value = 0;  // pid直道参数
float decide = 0;                                     //元素判断
float previous_error = 0, previous_I = 0;             //误差值
int sensor[5] = {0, 0, 0, 0, 0};                      // 5个传感器数值的数组
int initial_motor_speed = 60;                         //初始速度

void read_sensor_values(void);  //读取初值
void calc_pid(void);            //计算pid
void motor_control(void);       //电机控制

void motor_pinint();  //引脚初始化
void _stop();         //停车
///
//*******************************************************************//
//-------------------------------------------------------------------//

//-------------------------------------------------------------------//
//*******************************************************************//
///
//循迹部分///
///

//             循迹模块设置               //
#define leftA_track_PIN 3
#define leftB_track_PIN 4
#define middle_track_PIN 12
#define righA_track_PIN 7
#define righB_track_PIN 13

//----------------------------------算法部分-----------------------------
void obstacle();  //避障小车算法
//循迹小车算法/*
void read_sensor_values();
void calc_pid();
void motor_control();
//--------------------------------------------------------------------------//

void setup() {
    Serial.begin(115200);  //串口波特率115200（PC端使用）
    //track_pinint();      //循迹引脚初始化
    motor_pinint();      //电机引脚初始化
}
void loop() {
    read_sensor_values();  //循迹小车
    calc_pid();
    motor_control();
}

/*循迹模块引脚初始化*/
void track_pinint() {
    pinMode(leftA_track_PIN, INPUT);   //设置引脚为输入引脚
    pinMode(leftB_track_PIN, INPUT);   //设置引脚为输入引脚
    pinMode(middle_track_PIN, INPUT);  //设置引脚为输入引脚
    pinMode(righA_track_PIN, INPUT);   //设置引脚为输入引脚
    pinMode(righB_track_PIN, INPUT);   //设置引脚为输入引脚
}

/*电机引脚初始化*/
void motor_pinint() {
    pinMode(leftA_PIN, OUTPUT);  //设置引脚为输出引脚
    pinMode(leftB_PIN, OUTPUT);  //设置引脚为输出引脚
    pinMode(righA_PIN, OUTPUT);  //设置引脚为输出引脚
    pinMode(righB_PIN, OUTPUT);  //设置引脚为输出引脚
}

/**************************************************
stop子函数—停止子函数
函数功能：控制车停止
**************************************************/
void _stop() {
    // analogWrite(leftA_PIN, 0);
    // analogWrite(leftB_PIN, 0);  //左轮静止不动
    // analogWrite(righA_PIN, 0);
    // analogWrite(righB_PIN, 0);  //右轮静止不动
}
//速度设定范围(-255,255)
void motorsWrite(int speedL, int speedR) {
    if (speedR > 0) {
        // analogWrite(leftA_PIN, speedR);
        // analogWrite(leftB_PIN, 0);
    } else {
        // analogWrite(leftA_PIN, 0);
        //(leftB_PIN, -speedR);
    }

    if (speedL > 0) {
        // analogWrite(righA_PIN, speedL);
        // analogWrite(righB_PIN, 0);
    } else {
        // analogWrite(righA_PIN, 0);
        // analogWrite(righB_PIN, -speedL);
    }
}
//速度设定范围(-100，100)
void motorsWritePct(int speedLpct, int speedRpct) {
    // speedLpct, speedRpct ranges from -100 to 100
    motorsWrite(speedLpct * 2.55, speedRpct * 2.55);
}
void motorsStop() {
    // analogWrite(leftA_PIN, 0);
    //  analogWrite(leftB_PIN, 0);  //左轮静止不动
    //  analogWrite(righA_PIN, 0);
    //  analogWrite(righB_PIN, 0);  //右轮静止不动
}

void read_sensor_values() {
    sensor[0] = digitalRead(leftA_track_PIN);
    sensor[1] = digitalRead(leftB_track_PIN);
    sensor[2] = digitalRead(middle_track_PIN);
    sensor[3] = digitalRead(righA_track_PIN);
    sensor[4] = digitalRead(righB_track_PIN);

    if ((sensor[0] == 1) && (sensor[1] == 1) && (sensor[2] == 1) && (sensor[3] == 1) && (sensor[4] == 1)) {
        decide = 1;  //十字路口 1 1 1 1 1   直行
    } else if ((sensor[0] == 0) && (sensor[1] == 1) && (sensor[2] == 1) && (sensor[3] == 1) && (sensor[4] == 0)) {
        decide = 1;  //十字路口 0 1 1 1 0   直行
    } else if ((sensor[0] == 0) && (sensor[1] == 0) && (sensor[2] == 1) && (sensor[3] == 1) && (sensor[4] == 1)) {
        decide = 2;  // 90度路口 0 0 1 1 1    右转90度
    } else if ((sensor[0] == 0) && (sensor[1] == 0) && (sensor[2] == 1) && (sensor[3] == 1) && (sensor[4] == 0)) {
        decide = 2;  // 90度路口 0 0 1 1 0    右转90度
    } else if ((sensor[0] == 1) && (sensor[1] == 1) && (sensor[2] == 1) && (sensor[3] == 0) && (sensor[4] == 0)) {
        decide = 3;  // 90度路口 1 1 1 0 0    左转90度
    } else if ((sensor[0] == 0) && (sensor[1] == 1) && (sensor[2] == 1) && (sensor[3] == 0) && (sensor[4] == 0)) {
        decide = 3;  // 90度路口 0 1 1 0 0    左转90度
    } else if ((sensor[0] == 0) && (sensor[1] == 1) && (sensor[2] == 1) && (sensor[3] == 0) && (sensor[4] == 0)) {
        decide = 3;  //向上锐角 0 1 1 0 0    向上锐角
    } else if ((sensor[0] == 0) && (sensor[1] == 0) && (sensor[2] == 0) && (sensor[3] == 0) && (sensor[4] == 1)) {
        error = 2;  //          0 0 0 0 1
    } else if ((sensor[0] == 0) && (sensor[1] == 0) && (sensor[2] == 0) && (sensor[3] == 1) && (sensor[4] == 0)) {
        error = 1;  //          0 0 0 1 0
    } else if ((sensor[0] == 0) && (sensor[1] == 0) && (sensor[2] == 1) && (sensor[3] == 0) && (sensor[4] == 0)) {
        error = 0;  //          0 0 1 0 0
    } else if ((sensor[0] == 0) && (sensor[1] == 1) && (sensor[2] == 0) && (sensor[3] == 0) && (sensor[4] == 0)) {
        error = -1;  //         0 1 0 0 0
    } else if ((sensor[0] == 1) && (sensor[1] == 0) && (sensor[2] == 0) && (sensor[3] == 0) && (sensor[4] == 0)) {
        error = -2;  //         1 0 0 0 0
    } else if ((sensor[0] == 0) && (sensor[1] == 0) && (sensor[2] == 0) && (sensor[3] == 0) && (sensor[4] == 0)) {
        if (error == -2) {  //  0 0 0 0 0
            error = -3;
        } else {
            error = 3;
        }
    }
}
void calc_pid() {
    P = error;
    I = I + error;
    D = error - previous_error;

    PID_value = (Kp * P) + (Ki * I) + (Kd * D);

    previous_error = error;
}
void motor_control() {
    int left_motor_speed = initial_motor_speed - PID_value;
    int right_motor_speed = initial_motor_speed + PID_value;

    if (left_motor_speed < -255) {
        left_motor_speed = -255;
    }
    if (left_motor_speed > 255) {
        left_motor_speed = 255;
    }
    motorsWrite(left_motor_speed, right_motor_speed);

    Serial.print("move_A: ");
    Serial.print(left_motor_speed, OCT);
    Serial.print(" move_B: ");
    Serial.print(right_motor_speed, OCT);
    Serial.print(" error: ");
    Serial.print(error, OCT);
    Serial.print(" P: ");
    Serial.print(Kp, OCT);
    Serial.print(" PID_value: ");
    Serial.print(PID_value, OCT);
    Serial.println();
}
