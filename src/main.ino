/*================================================================
                    LYNCS drone project
                    2018
                    The latest version of this script is here
                    https://github.com/naoki-cpp/LYNCS_drone
==================================================================*/
#include <Servo.h>
#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h>
/*=========================================================
	===						FUNCTIONS						===
 ==========================================================*/
double delta_time();

/*=========================================================
	===						DEFINITIONS						===
===========================================================*/
//各サーボモータに割り当てるピンの定義
#define Out1 5
#define Out2 6
#define Out3 9
#define Out4 10

enum YawPitchRoll {
	YAW = 0,
	PITCH,
	ROLL
	};
/*==========================================================
	===						GLOBAL VARIABLES			===
============================================================*/
//サーボモータの制御のための変数
Servo rot_1;
Servo rot_2;
Servo rot_3;
Servo rot_4;

//MPU-6050ジャイロ・加速度センサ
MPU6050 mpu;				 //MPU-6050 object
bool dmp_ready = false;		 // set true if DMP init was successful
uint8_t mpu_interupt_status; // holds actual interrupt status byte from MPU
uint8_t dev_status;			 // return status after each device operation (0 = success, !0 = error)
//MPUのFIFO
uint16_t packet_size;	// expected DMP packet size (default is 42 bytes)
uint16_t fifo_count = 0; // count of all bytes currently in FIFO

Quaternion quaternion_mpu;				  //[x,y,z,w]				MPU-6050によって得られたquaternion
VectorInt16 acceleration_measured;		  // [x,y,z]				加速度センサの測定値
VectorInt16 acceleration_without_gravity; // [x, y, z]            重力を除いた加速度センサの測定値
VectorFloat gravity;					  // [x, y, z]     		gravity vector
VectorInt16 gyro;    // [x, y, z]      gyro vector
float yaw_pitch_roll[3];				  // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

uint8_t fifo_buffer[64];				  // FIFO storage buffer

bool mpu_interrupt = false; // dmpDataReadyが呼び出されたときにtrueになる

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================
void dmpDataReady()
{
	mpu_interrupt = true;
}
// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup()
{
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

	Serial.begin(115200); //		115200bpsでポートを開く
	while (!Serial)
	{
	} //		wait for Leonardo enumeration, others continue immediately
	// NOTE: 8MHz or slower host processors, like the Teensy @ 3.3v or Ardunio
	// Pro Mini running at 3.3v, cannot handle this baud rate reliably due to
	// the baud timing being too misaligned with processor ticks. You must use
	// 38400 or slower in these cases, or use some kind of external separate
	// crystal solution for the UART timer.

	//MPU-6050の初期化
	mpu.initialize();
	//MPU-6050のDMPの初期化
	dev_status = mpu.dmpInitialize(); //DMP Initialization
	if (dev_status == 0)
	{
		//supply gyro offsets
		mpu.setXGyroOffset(209);
		mpu.setYGyroOffset(47);
		mpu.setZGyroOffset(-21);
		mpu.setZAccelOffset(1957);

		mpu.setDMPEnabled(true);				  //		enable DMP
		attachInterrupt(2, dmpDataReady, RISING); //		pin0がLowからHighになるとdmpDataReadyが呼び出されるように設定
												  // 2はpin0を指す.
												  //see https://www.arduino.cc/reference/en/language/functions/external-interrupts/attachinterrupt/
		packet_size = mpu.dmpGetFIFOPacketSize(); //		FIFOのパケットサイズを取得
		dmp_ready = true;
	}
	else
	{
		Serial.print(F("DMP Initialization failed (code "));
		Serial.print(dev_status);
		Serial.println(F(")"));
	}
}
/*=================================================================
		====						MAIN PROGRAM LOOP			===
===================================================================*/
void loop()
{

	//================MPU_START==================================
	if (!dmp_ready)
	{ //DMPの初期化に失敗したなら以降の処理を行わない
		return;
	}
	while (!mpu_interrupt && fifo_count < packet_size)
	{ //dmpDataReadyが呼び出されるまで待機
	}
	mpu_interrupt = false;
	fifo_count = mpu.getFIFOCount();

	mpu_interupt_status = mpu.getIntStatus(); //割り込みステータスを取得.
	//	0x10:FIFO overflow
	//	0x02: DMP data ready

	// check for overflow (this should never happen unless our code is too inefficient)
	if ((mpu_interupt_status & 0x10) || fifo_count == 1024) //FIFOがoverflowした場合
	{
		mpu.resetFIFO();
		// otherwise, check for DMP data ready interrupt (this should happen frequently)
	}
	else if (mpu_interupt_status & 0x02) //DMP data ready
	{
		// wait for correct available data length, should be a VERY short wait
		while (fifo_count < packet_size)
		{
			fifo_count = mpu.getFIFOCount();
		}
		// read a packet from FIFO
		mpu.getFIFOBytes(fifo_buffer, packet_size);
		// track FIFO count here in case there is > 1 packet available
		// (this lets us immediately read more without waiting for an interrupt)
		fifo_count -= packet_size;

		mpu.dmpGetQuaternion(&quaternion_mpu, fifo_buffer); //取得データからQuaternionデータを取得
		//加速度・重力・ヨー・ピッチ・ロール・ジャイロの取得
		mpu.dmpGetAccel(&acceleration_measured);
		mpu.dmpGetGravity(&gravity, &quaternion_mpu);
		mpu.dmpGetYawPitchRoll(yaw_pitch_roll,&quaternion_mpu,&gravity);
		mpu.dmpGetGyro(&gyro,fifo_buffer);
	}
	//=======================MPU_END===================================

	double time_micro_second = delta_time();		  //前回loopが呼ばれてから今loopが呼ばれるまでの時間. us単位
	double time_second = time_micro_second / 1000000; //前回loopが呼ばれてから今loopが呼ばれるまでの時間.  s単位

}

double delta_time()
{
	static double previous_time = micros(); //前回この関数が呼ばれた時間
	double temp_time = micros();
	double return_time = temp_time - previous_time;
	previous_time = temp_time;
	return return_time;
}