/*================================================================
                    LYNCS drone project
                    2018
                    The latest version of this script is here
                    https://github.com/naoki-cpp/LYNCS_drone
==================================================================*/
#include <math.h>

#include <Servo.h>
#include <SPI.h>
#include <Wire.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
/*===========================
===       DEFINITIONS     ===
=============================*/
#define bv 0.05745024
#define cv 0.009521774

#define A_M 0.0017
#define A_m 0.0013
#define B_M 0.53
#define B_m 0.5996
#define Out1 5
#define Out2 6
#define Out3 9
#define Out4 10
#define maxi 2000
#define mini 1000
#define hhigh 1900
#define llow 1100
#define echoPin 13 // Echo Pin
#define trigPin 7  // Trigger Pin
#define VB 300
#define anga 1
#define angb 1
#define MaxP 1

#define MaxC 1 // per sec
#define MaxA 5
#define chigh 400
#define clow 0
/*==========================================================
	===						GLOBAL VARIABLES			===
============================================================*/
double vh;

double vz;
double vn;
double vn1 = 0;
double vn2 = 0;
double rvn;
double rvn1 = 0;
double rvn2 = 0;
const double we = 1600;
const double wh = 2;
const double kGravitationalAcceleration = 9.8;

MPU6050 mpu; // MPU-6050 ジャイロスコープ・加速度センサのオブジェクト

//サーボモーターのオブジェクト
Servo rot1;
Servo rot2;
Servo rot3;
Servo rot4;
double gzz0;
double gztank = 0;
double kx_m = 0;
double countx;
double ky_m = 0;
double kz_m = 0;
double vkx;
double vky;
double vkz;
double kx_a[3];
double kxa_a[3];
double ky_a[3];
double kya_a[3];
double kz_a[3];
double kza_a[3];
double kv_a[3];
double gy[3];
double gyv[3];
int land = 0;
double h_a[3];
const double h_m = 250;

double v00;
double center = 0;
double ptx = 0;
double pty = 0;
double ptz = 0;
double BPP = 520;
char buf[100];
int country = 0;
int coucou = 0;

int spi1; //ピンから送られてきた信号
int spi2;
int spi3;
int spi4;
int spi5;
int spi6;
int spi7;
int spi8;
unsigned char cspi1;
unsigned char cspi2;
volatile byte pos;
volatile boolean process_it;
//mpu cotrol /status vars. see http://invent.module143.com/mpu6050-how-to-use-it/
bool dmpReady = false;					  // set true if DMP init was successful
uint8_t mpuIntStatus;					  // holds actual interrupt status byte from MPU
uint8_t devStatus;						  // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;					  // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;						  // count of all bytes currently in FIFO
uint8_t fifoBuffer[64];					  // FIFO storage buffer
Quaternion quaternion;					  //[x,y,z,w]			quaternion
VectorInt16 acceleration_measured;		  // [x, y, z]            加速度センサの測定値
VectorInt16 acceleration_without_gravity; // [x, y, z]            重力を除いた加速度センサの測定値
VectorInt16 acceleration_world;			  // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;					  // [x, y, z]      gravity vector
VectorInt16 gyro;						  // [x, y, z]      gyro vector
float yaw_pitch_roll[3];
enum Yaw_Pitch_Roll
{
	YAW = 0,
	PITCH,
	ROLL
};
enum Coordunate
{
	COORDINATE_X = 0,
	COORDINATE_Y,
	COORDINATE_Z
};
double A[3][3];
double v;
double vv;
volatile bool mpuInterrupt = false; // indicates whether MPU interrupt pin has gone high
/*=========================================================
	===						FUNCTIONS						===
 ==========================================================*/
void cal1(double A[3][3],double f[3][3], double g[3][3]);
void cleenarray3(double array[], double newdata);
double pid(double array[], double a_m, double proportion_value, double DT, double Td, double T);
double pid_a(double array[], double a_m, double proportion_value);
void pidh(double array[], double a_m, double proportion_value, double DT, double Td, double T);
void calibration(Servo &rot1, Servo &rot2, Servo &rot3, Servo &rot4);
void flypower(int out1, int out2, int out3, int out4);
double time_update(); //前回この関数が呼ばれてからの時間 us単位
// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

void dmpDataReady()
{
	mpuInterrupt = true;
}

ISR(SPI_STC_vect)
{
	byte c = SPDR;
	// grab byte from SPI Data Register
	// add to buffer if room
	if (pos < sizeof buf)
	{
		buf[pos++] = c;
		// example: newline means time to process buffer
		if (c == '\n')
			process_it = true;
	} // end of room available
}
// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================
void setup()
{
	countx = 0;

	h_a[0] = 0;
	h_a[1] = h_m;
	h_a[2] = h_m;

	gy[YAW] = 0;
	gy[PITCH] = 0;
	gy[ROLL] = 0;

	gyv[COORDINATE_X] = 0;
	gyv[COORDINATE_Y] = 0;
	gyv[COORDINATE_Z] = 0;

	kxa_a[0] = 0;
	kxa_a[1] = 0;
	kxa_a[2] = 0;

	kya_a[0] = 0;
	kya_a[1] = 0;
	kya_a[2] = 0;

	kv_a[0] = 0;
	kv_a[1] = 0;
	kv_a[2] = 0;
	//モーターのピン割当
	rot1.attach(Out1);
	rot2.attach(Out2);
	rot3.attach(Out3);
	rot4.attach(Out4);
	//ピンモードの設定
	pinMode(Out1, OUTPUT);
	pinMode(Out2, OUTPUT);
	pinMode(Out3, OUTPUT);
	pinMode(Out4, OUTPUT);

	pinMode(echoPin, INPUT);
	pinMode(trigPin, OUTPUT);

	Wire.begin();
	Wire.setClock(400000L);

	Serial.begin(115200);
	while (!Serial)
		;

	//MPU6050の初期化
	mpu.initialize();
	devStatus = mpu.dmpInitialize();
	mpu.setXGyroOffset(209);
	mpu.setYGyroOffset(47);
	mpu.setZGyroOffset(-21);
	mpu.setZAccelOffset(1957); // 1688 factory default for my test chip
	if (devStatus == 0)
	{
		mpu.setDMPEnabled(true);
		attachInterrupt(2, dmpDataReady, RISING);
		mpuIntStatus = mpu.getIntStatus();
		dmpReady = true;
		packetSize = mpu.dmpGetFIFOPacketSize();
	}

	// 加速度/ジャイロセンサーの初期化。
	const double x = 0.0000000001;
	const double y = 0.0000000001;
	const double z = 0.0000000001;
	for (int i_r = 0; i_r < 3; i_r++)
	{
		// 重力加速度から角度を求める。
		cleenarray3(kx_a, x);
		cleenarray3(kxa_a, x);
		cleenarray3(ky_a, y);
		cleenarray3(kz_a, z);
	}

	pinMode(MISO, OUTPUT);
	// turn on SPI in slave mode
	SPCR |= _BV(SPE);
	// get ready for an interrupt
	pos = 0; // buffer empty
	process_it = false;
	// now turn on interrupts
	SPI.attachInterrupt();
	delay(5000);
	calibration(rot1, rot2, rot3, rot4);
}
/*=================================================================
		====						MAIN PROGRAM LOOP			===
===================================================================*/
void loop()
{
	//ISRが呼ばれたら実行
	if (process_it)
	{
		buf[pos] = 0;
		spi1 = *(int *)(&buf[0]);
		spi2 = *(int *)(&buf[4]);
		spi3 = *(int *)(&buf[8]);
		spi4 = *(int *)(&buf[12]);
		spi5 = *(int *)(&buf[16]);
		spi6 = *(int *)(&buf[20]);
		spi7 = *(int *)(&buf[24]);
		spi8 = *(int *)(&buf[28]);
		cspi1 = buf[32];
		cspi2 = buf[33];
		pos = 0;
		process_it = false;
	}

	if (!dmpReady)
	{
		return;
	}

	while (!mpuInterrupt && fifoCount < packetSize)
	{
	}

	mpuInterrupt = false;
	mpuIntStatus = mpu.getIntStatus();
	fifoCount = mpu.getFIFOCount();
	if ((mpuIntStatus & 0x10) || fifoCount == 1024)
	{
		mpu.resetFIFO();
	}
	else if (mpuIntStatus & 0x02)
	{
		//FIFOが適切なサイズになるまで待機
		while (fifoCount < packetSize)
		{
			fifoCount = mpu.getFIFOCount();
		}

		mpu.getFIFOBytes(fifoBuffer, packetSize);
		fifoCount -= packetSize;
		mpu.dmpGetQuaternion(&quaternion, fifoBuffer);
		mpu.dmpGetAccel(&acceleration_measured, fifoBuffer);
		mpu.dmpGetGravity(&gravity, &quaternion);

		mpu.dmpGetYawPitchRoll(yaw_pitch_roll, &quaternion, &gravity);

		gy[YAW] = (double)yaw_pitch_roll[YAW];
		gy[PITCH] = (double)yaw_pitch_roll[PITCH];
		gy[ROLL] = (double)yaw_pitch_roll[ROLL];
		double y0 = (-1) * yaw_pitch_roll[YAW];
		double y1 = (-1) * yaw_pitch_roll[PITCH];
		double y2 = yaw_pitch_roll[ROLL];
		getkgl((double)y0, (double)y1, (double)y2);

		mpu.dmpGetGyro(&gyro, fifoBuffer);

		gyv[COORDINATE_X] = (double)gyro.x;
		gyv[COORDINATE_Y] = (double)gyro.y;
		gyv[COORDINATE_Z] = (double)gyro.z;

		getkgl((double)y0, (double)y1, (double)y2);
		//感度補正

		double intaax = (double)acceleration_measured.x / 7.6;
		double intaay = (double)acceleration_measured.y / 8.0;
		double intaaz = (double)acceleration_measured.z / 10.2;

		double intypr[3];
		intypr[YAW] = yaw_pitch_roll[YAW] * 1000;
		intypr[PITCH] = yaw_pitch_roll[PITCH] * 1000;
		intypr[ROLL] = yaw_pitch_roll[ROLL] * 1000;

		//
		double aaxT = (-1) * intypr[YAW] * 1000 + 930 * intypr[PITCH] * 1000 + 3 * intypr[ROLL] * 1000 + 3 * intypr[PITCH] * intypr[PITCH] + (-4) * intypr[ROLL] * intypr[ROLL] + 10 * intypr[YAW] * intypr[PITCH] + 4 * intypr[PITCH] * intypr[ROLL] + 3 * intypr[YAW] * intypr[ROLL] + 10 * intaax * 1000;
		double aayT = 9 * intypr[YAW] * 1000 + (-20) * intypr[PITCH] * 1000 + 940 * intypr[ROLL] * 1000 + (-40) * intypr[PITCH] * intypr[PITCH] + (-30) * intypr[ROLL] * intypr[ROLL] + 30 * intypr[YAW] * intypr[PITCH] + 40 * intypr[PITCH] * intypr[ROLL] + (-30) * intypr[YAW] * intypr[ROLL] + 7 * intaay * 1000;
		double aazT = (-4) * intypr[YAW] * 1000 + 10 * intypr[PITCH] * 1000 + (-20) * intypr[ROLL] * 1000 + 5 * intypr[YAW] * intypr[YAW] + 9 * intypr[PITCH] * intypr[PITCH] + (-10) * intypr[ROLL] * intypr[ROLL] + (-30) * intypr[YAW] * intypr[PITCH] + (-30) * intypr[PITCH] * intypr[ROLL] + 9 * intypr[YAW] * intypr[ROLL] + 990 * intaaz * 1000;
		aaxT *= 0.000000001;
		aayT *= 0.000000001;
		aazT *= 0.000000001;

		//加速度の積分
		vz = A[2][0] * aaxT + A[2][1] * aayT + A[2][2] * aazT;

		double delta_time_second = time_update() / 1000000; //前回loopが呼ばれてから今loopが呼ばれるまでの時間 s単位

		rvn = rvn1 + (vz - 1) * kGravitationalAcceleration * delta_time_second;

		vn = (rvn * we - rvn2 * we - (delta_time_second / 2 * wh - 1) * (we - 2 / delta_time_second) * vn2 - ((delta_time_second / 2 * wh - 1) * (2 / delta_time_second + we) + (we - 2 / delta_time_second) * (1 + delta_time_second / 2 * wh)) * vn1) / (1 + delta_time_second / 2 * wh) / (2 / delta_time_second + we);
		vn2 = vn1;
		vn1 = vn;
		rvn2 = rvn1;
		rvn1 = rvn;

		if (country == 300)
		{
			v00 = vn;
		}

		if (spi7 == spi8)
		{
			vh = (double)spi8 / 1000;
		}
		if ((spi7 - spi8) == 256)
		{
			vh = (double)spi8 / 1000;
		}

		static double oldr;
		vh = 0.1 * vh + 0.9 * oldr;
		oldr = vh;
	}

	static double gzzz;
	gzz0 = gy[YAW];
	if ((gzzz - gy[YAW]) > M_PI)
	{
		gztank += 2.0 * M_PI;
	}
	if ((gy[YAW] - gzzz) > M_PI)
	{
		gztank += (-2.0) * M_PI;
	}
	gy[YAW] += gztank;
	gzzz = gzz0;

	if (cspi1 == cspi2)
	{
		while (cspi1 == 1)
		{
			rot1.writeMicroseconds(1000);
			rot2.writeMicroseconds(1000);
			rot3.writeMicroseconds(1000);
			rot4.writeMicroseconds(1000);
			Serial.println("END");
			delay(100000);
		}
	}

	if (coucou > 120)
	{
		while (1)
		{
			rot1.writeMicroseconds(1000);
			rot2.writeMicroseconds(1000);
			rot3.writeMicroseconds(1000);
			rot4.writeMicroseconds(1000);
			Serial.println("END");
			delay(100000);
		}
	}

	//pty = (double)w*0.01;
	if (cspi1 == cspi2)
	{
		if (cspi1 == 4)
		{
			BPP = 430;
		}
		if (cspi1 == 2)
		{
			BPP = 520;
		}
	}

	if (spi1 == spi2)
	{
		center = (double)spi1 / 1000 * MaxC;
	}
	if (cspi1 == cspi2)
	{
		if (cspi1 == 8)
		{
			center = -0.5;
			land = 1;
		}
	}
	if (land == 1)
	{
		if (vn < v00 + 0.05 && vn > v00 - 0.05)
		{
			coucou++;
		}
	}

	double centerold = 0;
	center = center * 0.2 + centerold * 0.8;
	centerold = center;

	static double ptxold = 0;
	if (spi5 == spi6)
	{
		ptx = ((double)spi5 * MaxA / 1000 + 2) / 180 * M_PI;
		ptx = ptx * 0.05 + ptxold * 0.95;
		ptxold = ptx;
	}
	static double ptyold = 0;
	if (spi3 == spi4)
	{
		pty = (double)spi3 * MaxA / 1000 / 180 * M_PI;

		pty = pty * 0.05 + ptyold * 0.95;
		ptyold = pty;
	}

	if (country > 320)
	{

		cleenarray3(kx_a, gyv[COORDINATE_X]);
		cleenarray3(ky_a, gyv[COORDINATE_Y]);
		cleenarray3(kz_a, gyv[COORDINATE_Z]);
		cleenarray3(kv_a, vn - v00);

		if (countx == 9)
		{
			cleenarray3(kxa_a, gy[ROLL]);
			cleenarray3(kya_a, -gy[PITCH]);
			cleenarray3(kza_a, -gy[YAW]);

			kx_m = pid_a(kxa_a, ptx, 180);
			ky_m = pid_a(kya_a, pty, 180);
			kz_m = pid_a(kza_a, ptz, 180);

			countx = 0;
		}
		vkx += pid(kx_a, kx_m, 0.732, 5.2286, 0.02562, 0.01);
		vky += pid(ky_a, ky_m, 0.732, 5.63, 0.024, 0.01);
		vkz += pid(kz_a, 0, 2.7, 32.5, 0, 0.01);
		pidh(kv_a, center, 80, 20, 20, 0.01);
		double vp = (v + BPP);
		if (vp > 600)
		{
			vp = 600;
		}
		if (vp < 200)
		{
			vp = 200;
		}

		flypower(vkx + vky - vkz + vp + 1100, vkx - vky + vkz + vp + 1100, -vkx + vky + vkz + vp + 1100, -vkx - vky - vkz + vp + 1100);
		countx++;
	}

	if (country <= 320)
	{
		flypower(1000 + country, 1000 + country, 1000 + country, 1000 + country);
	}

	country++;
}
void cleenarray3(double array[], double newdata)
{
	array[0] = array[1];
	array[1] = array[2];
	array[2] = newdata;
}
double time_update()
{
	static double previous_time = micros(); //前回この関数が呼ばれた時間
	double temp_time = micros();
	double return_time = temp_time - previous_time;
	previous_time = temp_time;
	return return_time;
}
double pid(double array[], double a_m, double proportion_value, double DT, double Td, double T)
{
	return proportion_value * (array[1] - array[2]) + T * DT * (a_m - array[2]) - Td / T * (array[2] - 2 * array[1] + array[0]);
}
double pid_a(double array[], double a_m, double proportion_value)
{
	return proportion_value * (a_m - array[2]);
}

void pidh(double array[], double a_m, double proportion_value, double DT, double Td, double T)
{
	vv = vv + T * DT * (a_m - array[2]) - Td / T * (array[2] - 2 * array[1] + array[0]);
	v = proportion_value * (a_m - array[2]) + vv;
}
void flypower(int out1, int out2, int out3, int out4)
{
	// 出力コード
	//out1の比較
	if (llow <= out1 <= hhigh)
	{
		rot1.writeMicroseconds(out1);
	}
	if (out1 < llow)
	{
		rot1.writeMicroseconds(llow);
	}
	if (hhigh < out1)
	{
		rot1.writeMicroseconds(hhigh);
	}
	//out2の比較
	if (llow <= out2 <= hhigh)
	{
		rot2.writeMicroseconds(out2);
	}
	if (out2 < llow)
	{
		rot2.writeMicroseconds(llow);
	}
	if (hhigh < out2)
	{
		rot2.writeMicroseconds(hhigh);
	}
	//out3の比較
	if (llow <= out3 <= hhigh)
	{
		rot3.writeMicroseconds(out3);
	}
	if (out3 < llow)
	{
		rot3.writeMicroseconds(llow);
	}
	if (hhigh < out3)
	{
		rot3.writeMicroseconds(hhigh);
	}
	//out4の比較
	if (llow <= out4 <= hhigh)
	{
		rot4.writeMicroseconds(out4);
	}
	if (out4 < llow)
	{
		rot4.writeMicroseconds(llow);
	}
	if (hhigh < out4)
	{
		rot4.writeMicroseconds(hhigh);
	}
}
void calibration(Servo &rot1, Servo &rot2, Servo &rot3, Servo &rot4)
{
	// キャリブレーションコード
	Serial.println("initiation");
	rot1.writeMicroseconds(maxi);
	rot2.writeMicroseconds(maxi);
	rot3.writeMicroseconds(maxi);
	rot4.writeMicroseconds(maxi);
	delay(2000);
	Serial.println("change");
	rot1.writeMicroseconds(mini);
	rot2.writeMicroseconds(mini);
	rot3.writeMicroseconds(mini);
	rot4.writeMicroseconds(mini);
	delay(2000);
	delay(1000);
	Serial.println("start");
}
void getkgl(double psi, double phi, double theta)
{
	double buffer1[3][3];
	double R_roll_theta[3][3] = {{1, 0, 0}, {0, cos(theta), -1 * sin(theta)}, {0, sin(theta), cos(theta)}};
	double R_pitch_phi[3][3] = {{cos(phi), 0, sin(phi)}, {0, 1, 0}, {-sin(phi), 0, cos(phi)}};
	double R_yaw_psi[3][3] = {{cos(psi), -1 * sin(psi), 0}, {sin(psi), cos(psi), 0}, {0, 0, 1}};
	cal1(A,R_yaw_psi, R_pitch_phi);
	for (int s = 0; s < 3; s++)
	{
		for (int f = 0; f < 3; f++)
		{
			buffer1[s][f] = A[s][f];
		}
	}
	cal1(A,buffer1, R_roll_theta);
}

void cal1(double A[3][3], double f[3][3], double g[3][3])
{
	for (int i = 0; i < 3; ++i)
	{
		for (int j = 0; j < 3; ++j)
		{
			A[i][j] = 0;
			for (int t = 0; t < 3; ++t)
			{
				A[i][j] += f[i][t] * g[t][j];
			}
		}
	}
}