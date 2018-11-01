/*================================================================
                    LYNCS drone project
                    2018
                    The latest version of this script is here
                    https://github.com/naoki-cpp/LYNCS_drone
==================================================================*/
#include <Servo.h>
#include <I2Cdev.h>
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
MPU6050 mpu;    														//MPU-6050 object
bool dmpReady = false;  										// set true if DMP init was successful
VectorInt16 acceleration_measured;     			// [x,y,z]				加速度センサの測定値 
VectorInt16 acceleration_without_gravity; // [x, y, z]            重力を除いた加速度センサの測定値
VectorFloat gravity; 												// [x, y, z]     		gravity vector

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

	Serial.begin(115200)	//		115200bpsでポートを開く
    while (!Serial)					//		wait for Leonardo enumeration, others continue immediately
	// NOTE: 8MHz or slower host processors, like the Teensy @ 3.3v or Ardunio
    // Pro Mini running at 3.3v, cannot handle this baud rate reliably due to
    // the baud timing being too misaligned with processor ticks. You must use
    // 38400 or slower in these cases, or use some kind of external separate
    // crystal solution for the UART timer.

	//MPU-6050の初期化
	mpu.initialize();
	//MPU-6050のDMPの初期化
	if (!mpu.dmpInitialize()) {			//DMP Initialization
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
	}else{
	//supply gyro offsets
		mpu.setXGyroOffset(209);
    	mpu.setYGyroOffset(47);
    	mpu.setZGyroOffset(-21);
    	mpu.setZAccelOffset(1957);
		//enable DMP
		mpu.setDMPEnabled(true);
	}
	
}


void loop() {
    
    double time_micro_second = time_update();          //前回loopが呼ばれてから今loopが呼ばれるまでの時間. us単位
    double time_second = time_micro_second / 1000000;//前回loopが呼ばれてから今loopが呼ばれるまでの時間.  s単位
	
}

double time_update(){
    static double previous_time = micros(); //前回この関数が呼ばれた時間 
    double temp_time = micros();
    double return_time = temp_time - previous_time;
    previous_time = temp_time;
    return return_time;
}