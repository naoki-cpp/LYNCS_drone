/*
                    LYNCS drone project
                    2018
                    The latest version of this script is here
                    https://github.com/naoki-cpp/LYNCS_drone
*/
#include <Servo.h>
#include <MPU6050_6Axis_MotionApps20.h>
//各サーボモータに割り当てるピンの定義
#define Out1 5
#define Out2 6
#define Out3 9
#define Out4 10
//サーボモータの制御のための変数
Servo rot_1;
Servo rot_2;
Servo rot_3;
Servo rot_4;

//MPU-6050ジャイロ・加速度センサ
MPU6050 mpu;    //MPU-6050
VectorInt16 acceleration_measured;     // 加速度センサの測定値 [x,y,z]
VectorInt16 acceleration_without_gravity; // [x, y, z]            重力を除いた加速度センサの測定値
VectorFloat gravity; // [x, y, z]      gravity vector

void setup() {
    //ピンを出力に設定
    pinMode(Out1, OUTPUT);
    pinMode(Out2, OUTPUT);
    pinMode(Out3, OUTPUT);
    pinMode(Out4, OUTPUT);
    //サーボモータへのピン割当
    rot_1.attach(Out1);
    rot_2.attach(Out2);
    rot_3.attach(Out3);
    rot_4.attach(Out4);
}


void loop() {
    
    double time_micro_second = time_update();          //前回loopが呼ばれてから今loopが呼ばれるまでの時間 us単位
    double time_second = time_micro_second / 1000000;//前回loopが呼ばれてから今loopが呼ばれるまでの時間 s単位

}

double time_update(){
    static double previous_time = micros(); //前回この関数が呼ばれた時間 
    double temp_time = micros();
    double return_time = temp_time - previous_time;
    previous_time = temp_time;
    return return_time;
}