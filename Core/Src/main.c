/*roll , pitch gain tuning////////////////////////////////////////////////////////////////////////
 *Author : Henry Ugochukwu Odoemelem
 *Description: stm32L4 Code for MAV
 *Date: 14 Oct. 2020
 */

#include <main.h>
#define ARM_MATH_CM4

#include "stm32l4xx.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>
#include "arm_math.h"                   // ARM::CMSIS:DSP
#include <string.h>
#include "POS_KalmanFilter.h"
#include "IMU_EKalmanFilter.h"
#include "Radio.h"
#include "SONAR.h"
#include "Motors.h"
#include "lqrController.h"
#include "MedianFilter.h"
#include "FreeRTOS.h"
#include "task.h"

#define   M_PI   3.14159265358979323846
#define POZYX_I2C_ADDRESS       0x4B
#define POZYX_WHO_AM_I          0x00  /* Returns the constant value 0x43 */
#define POZYX_ACCEL_X         0x54   /* Accelerometer data (in mg) */
#define POZYX_ACCEL_Y         0x56   /*  */
#define POZYX_ACCEL_Z         0x58   /*  */
#define POZYX_EULER_DIV_DEG            16.0f
#define POZYX_ACCEL_DIV_MG             1.0f
#define POZYX_GYRO_DIV_DPS             16.0f
#define POZYX_MAG_DIV_UT               16.0f
#define POZYX_EUL_HEADING       0x66   /* Euler angles heading (or yaw) */
#define POZYX_QUAT_W          0x6C   /* Weight of quaternion. */
#define POZYX_QUAT_X          0x6E   /* x of quaternion */
#define POZYX_QUAT_Y          0x70   /* y of quaternion */
#define POZYX_QUAT_Z          0x72   /* z of quaternion */
#define POZYX_QUAT_DIV        16384.0f
#define POZYX_GRAV_X          0x7A   /* x-component of gravity vector  */
#define POZYX_MAGN_X          0x5A   /* Magnemtometer data */
#define POZYX_POS_ALG         0x16   /* Algorithm used for positioning */
#define POZYX_ERRORCODE         0x4  /* Describes a possibly system error. */
#define POZYX_POS_X         0x30   /* x-coordinate of the device in mm. */
#define POZYX_POS_DIV_MM               1.0f
#define POZYX_INT_STATUS        0x5  /* Indicates the source of the interrupt. */
#define POZYX_POS_Y         0x34   /* y-coordinate of the device in mm. */
#define POZYX_POS_Z         0x38   /* z-coordinate of the device in mm. */
#define POZYX_DO_POSITIONING        0xB6   /* Initiate the positioning process.  */
#define POZYX_DO_RANGING        0xB5   /* Initiate ranging measurement */
#define POZYX_RANGE_PROTOCOL      0x21    /* The ranging protocol */
#define POZYX_DEVICE_GETRANGEINFO     0xC7   /* Get the stored range inforamation of a given pozyx device */
#define POZYX_INT_MASK          0x10   /* Indicates which interrupts are enabled. */
#define POZYX_LIA_X         0x74   /* Linear acceleration in x-direction */
#define POZYX_LIA_Y         0x76   /*  */
#define POZYX_LIA_Z         0x78   /*  */

// Ultrasonic sensor I2C addresses
#define SRF08_F   0xFA  // Front sensor
#define SRF08_R   0xEC  // Right sensor
#define SRF08_D   0xE0  // Down sensor
#define SRF08_L   0xE6  // left sensor
#define SRF08_B   0xF8  // Back sensor
#define RanginMode_in 0x50  // ranging mode - results in inches
#define RanginMode_cm 0x51  // ranging mode - results in centimeters
#define RanginMode_ms 0x52  // ranging mode - results in micro-seconds
/////////////////////////////////////////////////////////





//Main functions////////////

void SystemClock_Config(void);
void systickDelayMS(int n);
inline void systickDelayuS(int n);
void getTCPresponse(void);
void blink_LED(int delayMS);
void updateInitialPosition(void);
void updatePosition(void);
void updateIMU(void);
int updateChannels(void);
void desiredAttitude(void);
void desiredAltitude(void);
void setGains(void);
//End Main functions////////////

//string holders
char msg[100];
char radiomsg[20];
char str[100];
///End string holders //////////

//LPF variables/////////////
double to;
double f;
double rc;
double alpha;
int set_init_angles = 0; //false
//End LPF variable////////////////

//TIMER///////////////
int current_value = 0;
int period = 0; // actual an int since it holds no of counts
int toc = 0;
//End Timer//////////////////////

//i2c////////////////////////////////////////////
int32_t rdata = 1996;
//i2c1 for POzyx
void i2c_init(void);
inline void i2c_resetISR(void);
inline void i2c_readRequest(int saddr, int maddr, int nRegs);
inline void i2c_stop(void);
inline void i2c_start(int saddr, int mode, int nRegs);
inline void i2c_beginWrite(int saddr, int nRegs);
void i2c_Config(int saddr, int maddr, int wdata);
inline void i2c_beginRead(int saddr, int nRegs);
inline uint8_t i2c_readByte(void);
inline void i2c_writeByte(uint8_t data);

//i2c2 for Ultrasonic
void i2c2_init(void);
void i2c2_resetISR(void);
void i2c2_readRequest(int saddr, int maddr, int nRegs);
void i2c2_stop(void);
void i2c2_start(int saddr, int mode, int nRegs);
void i2c2_beginWrite(int saddr, int nRegs);
void i2c2_Config(int saddr, int maddr, int wdata);
void i2c2_beginRead(int saddr, int nRegs);
uint8_t i2c2_readByte(void);
void i2c2_writeByte(uint8_t data);
int sonarRange = 0;
//End i2c////////////////////////////////////////////

// 32 Max iBus packet size (2 byte header, 14 channels x 2 bytes, 2 byte checksum)
//  TX only has 10 channels
uint8_t rawChData[32] = { 0 };
uint16_t channels[10]; //0 roll,1-pitch,2-lift/throttle,3-yaw,4-SWA,7-SWD
uint16_t chHeader;
uint16_t armSWD;
uint16_t throttleCh;
uint16_t rollCh; //0 roll,1-pitch,2-lift,3-yaw
uint16_t pitchCh; //0 roll,1-pitch,2-lift,3-yaw
uint16_t rollChMax; //0 roll,1-pitch,2-lift,3-yaw
uint16_t rollChMin; //0 roll,1-pitch,2-lift,3-yaw
uint16_t pitchChMin; //0 roll,1-pitch,2-lift,3-yaw
uint16_t pitchChMax; //0 roll,1-pitch,2-lift,3-yaw
uint8_t ibus[32] = { 0 };
uint8_t IBUS_BUFFSIZE = 32;
uint8_t IBUS_MAXCHANNELS = 10;

//USART//////////////////////////////////////
// USART1 for Radio
void USART1_Init(int);

//USART2 for direct display
void USART2_Init(void);
int USART2_write(int data);
inline void USART2_TX(char *data);
char USART2_RX(void);

//USART3 for wifi
void USART3_Init(void);
inline void USART3_TX(char *data);
int USART3_RX(void);

//USART//////////////////////////////

//IMU variables////////////////////////////////////////////////
float32_t q[4] = {  1,  0,  0, 0 };
int16_t accXD, accYD, accZD;
int16_t accXDl, accYDl, accZDl;
int16_t gyroXD, gyroYD, gyroZD;
int16_t magXD, magYD, magZD;

double accXn, accYn, accZn, accnorm;
double accXnf, accYnf, accZnf, accnorm;
double accXln, accYln, accZln, accnorm;
double gyroXn, gyroYn, gyroZn;
double magXn, magYn, magZn, magnorm;

double angle_roll_accx;
double angle_pitch_accy;
double angle_rollx;
double angle_pitchy;
double yawzangle;
int16_t pozyxYawD, pozyxRollD, pozyxPitchD;
double pozyxYaw, pozyxRoll, pozyxPitch;
int16_t pozyxQWD, pozyxQXD, pozyxQYD, pozyxQZD;
double pozyxQW, pozyxQX, pozyxQY, pozyxQZ;
double rollCraft, pitchCraft, yawz;
double rollx, pitchy;
double rollRef, pitchRef, zRef;
double rollRefMax, pitchRefMax;
double rollRefMin, pitchRefMin;
double gmps2 = 9.80665; // gravity in m/s2
double gyroXnf;
double gyroYnf;
double set_init_anglesR = 0;
float rRateBuffer_7[7];
int rRateBuf_index = 0;
float pRateBuffer_7[7];
int pRateBuf_index = 0;
//End IMU variables//////////////////////////////////////////////

//Position variables///////////////////////////////////////////////
float32_t hxval[9]; // state variables
double posXn = 0;
double posYn = 0;
double posZn = 0;
double Pxk;
double Pyk;
double Pzk;
double Pvxk;
double Pvyk;
double Pvzk;
int32_t posXD, posYD, posZD;
double gmmps2 = 9806.65; // gravity in mm/s2
//Position variables//////////////////////////////////////////////

//rps
double n1;
double n2;
double n3;
double n4;

int pwm1;
int pwm2;
int pwm3;
int pwm4;

//Thrust
double F1;
double F2;
double F3;
double F4;
//////////////////////

float32_t Kdval[32] = { 1.8212E-17 }; //lqr control gain

///// roll and pitch model only gains
//double angleGain = 0.0023;
//double rateGain = 0.0004;

double angleGain = 0.0023;
double rateGain = 0.0004;
double pzGain = 3.0;
double vzGain = 2.0;
////////////////////////

void  vMainFunctionTask(void *pvParameters);
void  vUpdatePosTask(void *pvParameters);

TaskHandle_t Main_Handle,UpdatePos_Handle;
BaseType_t Main_return_Handle,UpdatePos_return_Handle;

int isUpdated = 0;
int dopos;
int main(void) {

	HAL_Init();
	SystemClock_Config();


	i2c_init(); //pozyx
	i2c2_init(); //ultrasonic
	USART2_Init(); //for direct display
	USART3_Init(); //for wifi
	USART1_Init(0x8B); //for radio data,
	Motor_init(); // init motors
	lqr_init();

	//this piece of code keeps initializing i2c until it is successful
	int16_t who_i_be = 0;
	while (who_i_be != 67) // 67 ie 0x43 is Pozyx who_am_i
	{
		i2c_init(); //pozyx
		i2c_readRequest((POZYX_I2C_ADDRESS << 1), POZYX_WHO_AM_I, 1);
		who_i_be = i2c_readByte();
		i2c_stop();
		sprintf(msg, "%d, i2c failure\n", who_i_be);
		USART2_TX(msg);
	}
	sprintf(msg, "%d,Pozyx i2c1 OK\n", who_i_be);
	USART2_TX(msg);


	/*	 1. Connect to network name
	 *   SSID:	ESP-315DA5
		 on your computer

		 2. then look for ESP-315DA5 IPv4 DNS servers(e.g 192.168.4.1), in ESP-315DA5  properties
		 3. then connect to IPv4 DNS servers(most likely 192.168.4.1),port 1234 on your TCP terminal
	 */

	USART3_TX("AT+CWMODE=3\r\n"); // client and hold mode
	getTCPresponse();
	USART3_TX("AT+CIPMUX=1\r\n"); //allow multiple connections
	getTCPresponse();
	USART3_TX("AT+CIPSERVER=1,1234\r\n"); //set esp port
	getTCPresponse();
	USART3_TX("AT+CIFSR\r\n"); // display esp APIP used by client in connection
	getTCPresponse();

	// timer to check period of loops
	//config PC6 as input of TIM3 CH1
	RCC->AHB2ENR |= (1 << 2);     // enable GPIOC clk, port C
	GPIOC->MODER = 0xFFFFEFFF;  // PC6 in alternate mode
	GPIOC->AFR[0] |= 1 << 25; // AF register low[0], AFSEL6 = AF2 TIM3 CH1 out on PC6

	//TIM3 config , count up to 65535 then reload since ARR is default
	RCC->APB1ENR1 |= (1 << 1); // enable TIM3 clk
	TIM3->PSC = 7200 - 1; // Divide Fclk(72MHz) by 7200 = 10KHz, with ARR as 65535 ie 65 sec
	TIM3->CCMR1 = 0x41;       // input mode
	TIM3->CCER = 0x03;   // enable channel 1 (CCR1), and capture at rising edges
	TIM3->CR1 = 1 << 0;        // enable timer in default upcounting  mode




	//set position ALG.
	i2c_beginWrite((POZYX_I2C_ADDRESS << 1), 1); // basically a start in write mode, write 3 bytes
	i2c_writeByte(POZYX_POS_ALG); // write/select memory addr you want to write to
	i2c_beginWrite((POZYX_I2C_ADDRESS << 1), 1); // basically a Repeat start in write mode, write 1 bytes
	//i2c_writeByte(48);    // 3D,uwb 48
	i2c_writeByte(52);    // 3D,tracking works better 52
	i2c_stop();
	systickDelayMS(1);

	updateInitialPosition();


	init_POS_KF(0.0007, 0.000000001, 10, 100, posXn / 100.0, posYn / 100.0,
			posZn / 100.0);

	init_IMU_EKF(0.0007, 20, 0.0000000011, 0.00000000011, 0.00000000036,
			182.74, 157.713,103.656, 7.69);

	USART2_TX("\n\nEnd of all initializations:)\n");

	UpdatePos_return_Handle = xTaskCreate(vUpdatePosTask,
			"Update Position Task",
			500,
			NULL,
			2,
			&UpdatePos_Handle
	);
	xTaskCreate(vMainFunctionTask,
			"main Activity Task",
			500,
			NULL,
			1,
			&Main_Handle
	);



	vTaskStartScheduler();

	while(1){}





}
void  vUpdatePosTask(void *pvParameters){



	TickType_t xLastWakeTime;
	const TickType_t xPeriod = pdMS_TO_TICKS(10);//ms
	// Initialise the xLastWakeTime variable with the current time.
	xLastWakeTime = xTaskGetTickCount();


	while(1){


		updatePosition(); //update my current location measurement


		updateIMU(); // update IMU values
		isUpdated = 1;
		TIM3->CR1 = 1 << 0;        // enable timer in default upcounting  mode
		vTaskDelayUntil( &xLastWakeTime, xPeriod );
		TIM3->CR1 = 0 << 0;        // disable timer in default upcounting  mode



	}

}
void  vMainFunctionTask(void *pvParameters)
{
	vTaskPrioritySet(UpdatePos_Handle,0);

	while (1) {

		vTaskPrioritySet(UpdatePos_Handle,0);

		/*LED GPIOA setup already done in USART2_Init()*/
		GPIOA->BSRR = 1 << (5 + 16); // LED off

		TIM3->EGR = 1 << 1;  // generate event for capture on CC1IF

		while (!(TIM3->SR & 2))
			; // wait until flag is raised, CNT value transfered to CCR1 and CNT is cleared
		current_value = (int) TIM3->CCR1;



		// Start attitude and localization Sensor Readings////////////////////////////////////////////////////
		POS_KFasymptotic(posXn, posYn, posZn, accXln, accYln, accZln,isUpdated);//Position kalman filter, asymptotic gain


		IMU_EKF(magXn, magYn, magZn, accXnf, accYnf, accZnf, gyroXnf, gyroYnf,
				gyroZn); // IMU Extended Kalman filter

		//IMU_EKFasymptotic(magXn, magYn, magZn, accXn, accYn, accZn, gyroXnf, gyroYnf,
		//		gyroZn); // IMU Extended Kalman filter, asymptotic gain

		// End  attitude and localization Sensor Readings/////////////////////////////////////////////////////

		//Start Convert quaternions from IMU_EKF to attitude ////////////////////////////////////////////////////
		rollx = atan2((2 * (q[0] * q[1] + q[2] * q[3])),
				(q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]))* ((double)(180 / M_PI));
		pitchy = -asin(2 * (q[1] * q[3] - q[0] * q[2])) * ((double)(180 / M_PI));

		//rollx = (rollx < 0 ? -(180 - (-1 * rollx)) : 180 - rollx);



		//End Convert quaternions from IMU_EKF to attitude ////////////////////////////////////////////////////

		/// Start control ////////////////////////////////////////////////////
		updateChannels();//radio rx channels update

		//		if(armSWD>1500 && throttleCh>=1050 )
		//		{   //to make sure Quadrotor should be above the ground before pitch or roll
		//			//armSWD used as switch for roll and pitch sticks
		//
		//
		//			desiredAttitude();//update channels before calling this fxn to update rollRef,pitchRef
		//			lqr(0,0,  pitchy,gyroYnf, 0,0, 1,0  ,  0,0,  pitchRef,1E-30, 1E-30,1E-30, 1,0);// update desiredAttitude before calling lqr control
		//
		//			set_Motors(F1,F2,F3,F4);// update F1,F2,F3,F4 using lqr before setting motor values
		//
		//
		//		}else{
		//
		//			desiredAttitude();//update channels before calling this fxn to update rollRef,pitchRef
		//			//lqr(rollx,gyroXnf, pitchy,gyroYnf,rollRef,0,pitchRef,0);// update desiredAttitude before calling lqr control
		//			//testing pitch only
		//			//lqr(0,0, pitchy,gyroYnf,0,0,pitchRef,0);// update desiredAttitude before calling lqr control
		//			turnMotorsOff();//Turn of motors with 0N thrust
		//			setGains();// set gains only when motors are not running
		//
		//		}

		desiredAltitude();//testing hover now
		/// end control//////////////////////////////////////////////////////////

		////////////Display///////////////////////////////////////////

		////display imu and pos
		//		pozyxYaw = (double)pozyxYawD/POZYX_EULER_DIV_DEG ;
		//		pozyxRoll = (double)pozyxRollD/POZYX_EULER_DIV_DEG;
		//		pozyxPitch = (double)pozyxPitchD/POZYX_EULER_DIV_DEG ;
		//		sprintf(msg,"%li,%li,%li,%li,%li,%li\n",(int32_t)rollx,(int32_t)pitchy,(int32_t)yawz,(int32_t)pozyxRoll,(int32_t)pozyxPitch,(int32_t)pozyxYaw);
		//			sprintf(msg,"%i,%i,%i,%i,%i,%i\n",(int32_t)Pxk,(int32_t)Pyk,(int32_t)Pzk,(int32_t)posXn,(int32_t)posYn,(int32_t)posZn);
		//		sprintf(msg, "%li,%li,%li,%li,%li,%li\n", (int32_t) Pxk, (int32_t) Pyk,
		//				(int32_t) Pzk, (int32_t) rollx, (int32_t) pitchy,
		//				(int32_t) yawz);
		//		sprintf(msg, "%li,%li,%li,%li,%li,%li\n", (int32_t) Pxk, (int32_t) Pyk,
		//						(int32_t) Pzk, (int32_t) Pvxk, (int32_t) Pvyk,
		//						(int32_t) Pvzk);

		//sprintf(msg, "%li,%li\n",(int32_t) throttleCh, (int32_t) pwm4);
		//sprintf(msg, "%li,%li,%li,%li,%li,%li,%li\n", (int32_t)rollx,(int32_t)pitchy,(int32_t)pitchRef,(int32_t)pwm1,(int32_t)pwm2,(int32_t) pwm3,(int32_t) pwm4);
		//sprintf(msg, "%li,%li,%li,%li,%.4f,%.4f\n", (int32_t)rollx,(int32_t)pitchy,(int32_t)pitchRef,(int32_t)pwm1,angleGain,rateGain);

		//		sprintf(msg, "%li,%li,%li,%li,%li,%li,%li,%.4f,%.4f\n",
		//				(int32_t)Pzk,(int32_t)Pvzk ,(int32_t)zRef,(int32_t)pwm1,(int32_t)pwm2,(int32_t) pwm3,(int32_t) pwm4,pzGain,rateGain);

		//sprintf(msg, "%.4f,%.4f,%i,%i\n\n",pitchy,pitchRef,pwm1,(int)((10E3)/period));

		//		sprintf(msg, "%li,%li,%li,  %li,%.2lf,  %.4f,%.4f\n",
		//					(int32_t)Pzk,(int32_t)Pvzk ,(int32_t)zRef,(int32_t)pwm1,F1,pzGain,vzGain);

		//sprintf(msg, "%.4f,%.4f,%i,%i,%i,%i,%i\n", rollx,pitchy,(int) Pxk, (int) Pyk,(int) Pzk,(int)pwm1,(int)((10E3)/period));
		//(double)(accZnf/gmps2)
		sprintf(msg, "%f,%f,%i\n", rollx,pitchy,(int)((10E3)/period));
		//sprintf(msg, "%.2lf,%.2lf,%.2lf,%.2lf\n", gyroXn,gyroXnf , gyroYn,gyroYnf);
		//systickDelayMS(5);
		//sprintf(msg, "%.4f,%.4f,%i\n", rollx, pitchy, (int) ((10E3) / period));

		//sprintf(msg, "%li,%li\n", (int32_t)throttleCh,(int32_t)pwm3);
		USART2_TX(msg);		//direct visiualization

		//		////TCP//////
		//		sprintf(str, "AT+CIPSEND=0,%i\r\n", strlen(msg));
		//		USART3_TX(str);
		//		systickDelayuS(700);		//micro seconds
		//		USART3_TX(msg);
		//		////end tcp////

		//		sprintf(msg, "%s %li,%li\n","desired:", (int32_t) rollRef, (int32_t) pitchRef);
		//		USART2_TX(msg);//direct visiualization
		//				sprintf(msg, "%s %.1lf,%.1lf,%.1lf,%.1lf\n\n","Force:",  F1,  F2,
		//						F3,  F4);
		//      USART2_TX(msg);//direct visiualization
		//		sprintf(msg, "%s %li,%li,%li,%li\n","Speed:", (int32_t) n1, (int32_t) n2,
		//				(int32_t) n3, (int32_t) n4);
		//		USART2_TX(msg);//direct visiualization
		//		sprintf(msg, "%s %li,%li,%li,%li\n\n\n","pwm:", (int32_t) pwm1, (int32_t)pwm2,(int32_t) pwm3,(int32_t) pwm4);
		//		USART2_TX(msg);//direct visiualization

		//		//ultrasonic
		//		startSonarRanging(RanginMode_cm);
		//		systickDelayMS(75);	//wait for sonar unless the code will hange!!, resolve issue of hanging
		//
		//		sonarRange = processSonar(SRF08_F);
		//		sprintf(msg, "\nsonar RANGER:%i\n", sonarRange);
		//		USART2_TX(msg);		//direct visualization
		//////////////// End of Display////////////////////////////////////

		//Period of this while loop, sampling time is (period/10000.0) in sec, where 1000.0 is timer freq
		isUpdated = 0;
		period = current_value - toc; // no. of counts btw captures
		toc = current_value;
		vTaskPrioritySet(UpdatePos_Handle,2);

	}
}


void setGains(void) {

	//	///update gains
	//	angleGain =  (0 + ((channels[8] - 1000.0)*(0.1 - 0))/(2000.0 - 1000.0));//VRA for angle gains
	//	rateGain =   (0 + ((channels[9] - 1000.0)*(0.1 - 0))/(2000.0 - 1000.0));//VRB for rate gains
	//
	//	Kdval[4]= angleGain;  //roll gain
	//	Kdval[5]= rateGain;  //roll rate gain
	//	Kdval[10]= angleGain; //pitch gain
	//	Kdval[11]= rateGain; //pitch rate gain
	//	//////////////

	///update gains
	pzGain = (0 + ((channels[8] - 1000.0) * (20.0 - 0)) / (2000.0 - 1000.0));//VRA for angle gains
	vzGain = (0 + ((channels[9] - 1000.0) * (20.0 - 0)) / (2000.0 - 1000.0));//VRB for rate gains

	Kdval[6] = pzGain;  //roll gain
	Kdval[7] = vzGain;  //roll gain
	Kdval[8] = angleGain;  //roll gain
	Kdval[9] = rateGain;  //roll rate gain
	Kdval[18] = angleGain; //pitch gain
	Kdval[19] = rateGain; //pitch rate gain
	Kdval[28] = angleGain; //yaw  gain
	Kdval[29] = rateGain; //yaw rate gain
	//////////////

}

void desiredAltitude(void) {

	if (channels[4] > 1500) {	// SWA

		throttleCh = channels[2];

		//		int zRefMax = 700;
		//		int zRefMin = 190;
		//		int zChMax = 2000;
		//		int zChMin = 1000;
		//
		//		zRef =  (zRefMin + ((throttleCh - zChMin)*(zRefMax - zRefMin))/(zChMax - zChMin));
		//
		//		if(Pzk <= zRefMin){Pzk = zRefMin;}
		//		if(Pzk <= 0){Pzk = 0;}
		//		if(Pvzk <= 0){Pvzk = 0;}
		//
		//		if(Pzk >= (zRef -50) && Pzk <= (zRef + 50)){
		//			Pzk = zRef;
		//			Pvzk = 0;
		//
		//		}
		//
		//
		////		if(Pzk == zRef){
		////			hover();
		////		}else
		//
		//		if(Pzk <= (zRefMax) ){
		//			//lqr(rollx,gyroXnf,  pitchy,gyroYnf, 0,0, (Pzk/1000.0),(Pvzk/1000.0),   0,0, 0,0, 0,0, (zRef/1000.0),0);// update desiredAttitude before calling lqr control
		//			lqr(rollx,gyroXnf,  0,0, 0,0, (Pzk/1000.0),(Pvzk/1000.0) ,  0,0, 0,0, 0,0, (zRef/1000.0),0);// update desiredAttitude before calling lqr control
		//
		//		}else if(Pzk >  (zRefMax) ){//bring it down
		//
		//			//lqr(rollx,gyroXnf,  pitchy,gyroYnf, 0,0, (Pzk/1000.0),0 ,  0,0, 0,0, 0,0, ((zRefMax-50)/1000.0),0);// update desiredAttitude before calling lqr control
		//			lqr(rollx,gyroXnf,  0,0, 0,0, (Pzk/1000.0),0 ,  0,0, 0,0, 0,0, ((zRefMax-100)/1000.0),0);// update desiredAttitude before calling lqr control
		//		}
		//
		//
		//
		////		else
		////		{
		////
		////	     	lqr(rollx,gyroXnf, 0,0, 0,0, (Pzk/1000.0),(Pvzk/1000.0) ,  0,0, 0,0, 0,0, (zRef/1000.0),0);// update desiredAttitude before calling lqr control
		////		}
		//
		//		set_Motors(F1,F2,F3,F4);// update F1,F2,F3,F4 using lqr before setting motor values

		set_Motors(throttleCh, throttleCh, throttleCh, throttleCh);	// update F1,F2,F3,F4 using lqr before setting motor values

	} else {
		turnMotorsOff();
		setGains();		// set gains only when motors are not running

	}

}
void desiredAttitude(void) {

	// Rx channel val. in microsec:     1000   1490    1510     2000
	// Dseaired Degress range:          -15     0       0        15

	//for desired roll
	rollCh = channels[0];
	if (rollCh >= 1510) {	//roll ccw +ve rotation
		rollRefMax = 15;
		rollRefMin = 0;
		rollChMax = 2000;
		rollChMin = 1510;
		rollRef = (rollRefMin
				+ ((rollCh - rollChMin) * (rollRefMax - rollRefMin))
				/ (rollChMax - rollChMin));

	} else if (rollCh <= 1490) {	//roll cw -ve rotation

		rollRefMax = 0;
		rollRefMin = -15;
		rollChMax = 1490;
		rollChMin = 1000;
		rollRef = (rollRefMin
				+ ((rollCh - rollChMin) * (rollRefMax - rollRefMin))
				/ (rollChMax - rollChMin));

	} else if (rollCh > 1490 && rollCh < 1510) {
		rollRef = 0; //No roll
	} else {
		rollRef = 0; //No roll, unknown zone
	}

	//for desired pitch
	pitchCh = channels[1];
	if (pitchCh >= 1510) { //pitch ccw +ve rotation
		pitchRefMax = 15;
		pitchRefMin = 0;
		pitchChMax = 2000;
		pitchChMin = 1510;
		pitchRef = (pitchRefMin
				+ ((pitchCh - pitchChMin) * (pitchRefMax - pitchRefMin))
				/ (pitchChMax - pitchChMin));

	} else if (pitchCh <= 1490) { //pitch cw -ve rotation

		pitchRefMax = 0;
		pitchRefMin = -15;
		pitchChMax = 1490;
		pitchChMin = 1000;
		pitchRef = (pitchRefMin
				+ ((pitchCh - pitchChMin) * (pitchRefMax - pitchRefMin))
				/ (pitchChMax - pitchChMin));

	} else if (pitchCh > 1490 && pitchCh < 1510) {
		pitchRef = 0; //No pitch
	} else {
		pitchRef = 0; //No pitch, unknown zone
	}

}

void updateInitialPosition(void) {

	int startcount = 0;
	while (startcount < 99) {
		i2c_readRequest((POZYX_I2C_ADDRESS << 1), POZYX_DO_POSITIONING, 1);
		int dopos = i2c_readByte();
		i2c_stop();

		systickDelayMS(1);
		i2c_readRequest((POZYX_I2C_ADDRESS << 1), POZYX_POS_X, 12);
		posXD = i2c_readByte() | i2c_readByte() << 8 | i2c_readByte() << 16
				| i2c_readByte() << 24;
		posYD = i2c_readByte() | i2c_readByte() << 8 | i2c_readByte() << 16
				| i2c_readByte() << 24;
		posZD = i2c_readByte() | i2c_readByte() << 8 | i2c_readByte() << 16
				| i2c_readByte() << 24;
		i2c_stop();

		posXn = posXn + (double) posXD / POZYX_POS_DIV_MM;
		posYn = posYn + (double) posYD / POZYX_POS_DIV_MM;
		posZn = posZn + (double) posZD / POZYX_POS_DIV_MM;

		startcount++;

	}

}

void updatePosition(void) {
	dopos = 0;



	//	//Do position
	i2c_readRequest((POZYX_I2C_ADDRESS << 1), POZYX_DO_POSITIONING, 1);
	dopos = i2c_readByte();
	i2c_stop();


	//	get Position
	i2c_readRequest((POZYX_I2C_ADDRESS << 1), POZYX_POS_X, 12);
	posXD = i2c_readByte() | i2c_readByte() << 8 | i2c_readByte() << 16
			| i2c_readByte() << 24;
	posYD = i2c_readByte() | i2c_readByte() << 8 | i2c_readByte() << 16
			| i2c_readByte() << 24;
	posZD = i2c_readByte() | i2c_readByte() << 8 | i2c_readByte() << 16
			| i2c_readByte() << 24;
	i2c_stop();

	posXn = (double) (posXD / POZYX_POS_DIV_MM);
	posYn = (double) (posYD / POZYX_POS_DIV_MM);
	posZn = (double) (posZD / POZYX_POS_DIV_MM);

	//Linear acc.
	i2c_readRequest((POZYX_I2C_ADDRESS << 1), POZYX_LIA_X, 6);
	// Little Endian: combine the bytes as  LSByte|MSByte<<8
	accXDl = i2c_readByte() | i2c_readByte() << 8;
	accYDl = i2c_readByte() | i2c_readByte() << 8;
	accZDl = i2c_readByte() | i2c_readByte() << 8;
	i2c_stop();

	// convert to to mm/s2
	accXln = (double) ((accXDl / POZYX_ACCEL_DIV_MG) / 1000.0) * gmmps2; //accxl
	accYln = (double) ((accYDl / POZYX_ACCEL_DIV_MG) / 1000.0) * gmmps2; //accyl
	accZln = (double) ((accZDl / POZYX_ACCEL_DIV_MG) / 1000.0) * gmmps2; //acczl

}

void updateIMU(void) {

	////////////////for IMU
	i2c_readRequest((POZYX_I2C_ADDRESS << 1), POZYX_GRAV_X, 6);
	// Little Endian: combine the bytes as  LSByte|MSByte<<8
	accXD = i2c_readByte() | i2c_readByte() << 8;
	accYD = i2c_readByte() | i2c_readByte() << 8;
	accZD = i2c_readByte() | i2c_readByte() << 8;
	i2c_stop();

	i2c_readRequest((POZYX_I2C_ADDRESS << 1), POZYX_MAGN_X, 12);
	magXD = i2c_readByte() | i2c_readByte() << 8;
	magYD = i2c_readByte() | i2c_readByte() << 8;
	magZD = i2c_readByte() | i2c_readByte() << 8;
	gyroXD = i2c_readByte() | i2c_readByte() << 8;
	gyroYD = i2c_readByte() | i2c_readByte() << 8;
	gyroZD = i2c_readByte() | i2c_readByte() << 8;
	//	pozyxYawD = i2c_readByte()|i2c_readByte()<<8;
	//	pozyxRollD = i2c_readByte()|i2c_readByte()<<8;
	//	pozyxPitchD = i2c_readByte()|i2c_readByte()<<8;
	i2c_stop();

	//as is
	//	 // convert of deg/s
	gyroXn = (double) gyroXD / POZYX_GYRO_DIV_DPS;
	gyroYn = (double) gyroYD / POZYX_GYRO_DIV_DPS;
	gyroZn = (double) gyroZD / POZYX_GYRO_DIV_DPS;

	// convert to m/s2
	accXn = (double) ((accXD / POZYX_ACCEL_DIV_MG) / 1000.0) * gmps2;
	accYn = (double) ((accYD / POZYX_ACCEL_DIV_MG) / 1000.0) * gmps2;
	accZn = (double) ((accZD / POZYX_ACCEL_DIV_MG) / 1000.0) * gmps2;

	// measuered y values actually not normailzsed
	// convert to uT
	magXn = (double) (magXD / POZYX_MAG_DIV_UT);
	magYn = (double) (magYD / POZYX_MAG_DIV_UT);
	magZn = (double) (magZD / POZYX_MAG_DIV_UT);


	//updateIMU(); // update IMU values
	/*	Median filter for rates */
	////for roll rate
	rRateBuffer_7[rRateBuf_index % 7] = gyroXn;
	++rRateBuf_index;
	// reset counter
	if (rRateBuf_index == 100) {
		rRateBuf_index = 0;
	} // reset buffer index
	/// filter rates readings
	gyroXnf = MedianFilter_7(rRateBuffer_7);
	//for pitch rate
	pRateBuffer_7[pRateBuf_index % 7] = gyroYn;
	++pRateBuf_index;
	// reset counter
	if (pRateBuf_index == 100) {
		pRateBuf_index = 0;
	} // reset buffer index
	/// filter rates readings
	gyroYnf = MedianFilter_7(pRateBuffer_7);
	/* end of median filters*/

	if (set_init_angles) {
		alpha = 0.95;

		accXnf = (accXnf- accXn) * alpha + accXn;
		accYnf = (accYnf- accYn) * alpha + accYn;
		accZnf = (accZnf- accZn) * alpha + accZn;

	} else {
		accXnf = accXn;
		accYnf = accYn;
		accZnf = accZn;
		set_init_angles = 1;        // true
	}

}

int updateChannels(void) {

	if ((DMA1->ISR & (1 << 17))) {

		Process_ibus();

		///for roll and pitch arming
		armSWD = channels[7];	//arm roll and pitch
		throttleCh = channels[2];

		/////Display
		//		for (int i = 2, ch = 0; ch < IBUS_MAXCHANNELS; i = i + 2, ch++) {
		//			sprintf(radiomsg, "%i ", channels[ch]);
		//			USART2_TX(radiomsg);
		//
		//		}
		//
		//		USART2_TX("\n");
		DMA1->IFCR = 1 << 17; //clear transfer complete flag
	}

	return 1;

}

void i2c_init(void) {

	RCC->AHB2ENR |= 1 << 1;         //enable gpiob clock rm pg251
	RCC->APB1ENR1 |= 1 << 21;       //enable i2c1 clock rm pg253
	systickDelayMS(1);

	GPIOB->MODER = 0xEBAAFEBF; //set pb8and9 and pb10and11(used in usart3 below),PB13,14 for 12c2 to alternative function rm pg305
	//GPIOB->AFR[1] =0x04400044;   // AF4 12C on pb8 and 9 rm pg310, uCds pg93
	GPIOB->AFR[1] = 0x04407744; // enable usart3 on PB10, and PB11,// AF4 12C on pb8,9 and pb13,14 rm pg310, uCds pg93
	GPIOB->OTYPER = 0x6300;       // pb8,9 and p13,14 output open drain rm pg306
	GPIOB->PUPDR = 0x50100;      // pb8 and 9 pull-up, rm pg307

	i2c_resetISR();
}

void i2c_resetISR(void) {

	//I2C1->TIMINGR = 0x00610611;  // I2C at 400kHz cube
	I2C1->TIMINGR = 0x10911E24;  // I2C at 100kHz cube for pozyx to work well
	//I2C1->TIMINGR = 0x10320309;  // I2C at 400kHz ds
	//I2C1->TIMINGR = 0x30420F13;    // I2C at 100kHz ds
	I2C1->CR1 = 0;                 // PE cleared, SWresest rm pg1325
	while (I2C1->CR1 & 1 << 0)
		;       // wait for at least 3 APB clock cycle b4 set rm pg 1325
	I2C1->CR1 |= 0x01;              // set PE ie enable i2c

}

void i2c_start(int saddr, int rwmode, int nRegs) {
	I2C1->CR2 = saddr;      // slave address,sw end mode, write mode(0) on reset
	I2C1->CR2 |= nRegs << 16;      // read/write n number of registers(byte)
	I2C1->CR2 |= rwmode << 10; // change to read mode if rwmode = 1, else remain in write mode
	I2C1->CR2 |= 1 << 13;           // start condition rm pg1327
	while ((I2C1->CR2 & 1 << 13))
		;   // wait until start bit is reset, meaning address sent
}

void i2c_beginRead(int saddr, int nRegs) {
	i2c_start(saddr, 1, nRegs); // call i2c start with 1 for read mode
}

void i2c_beginWrite(int saddr, int nRegs) {
	i2c_start(saddr, 0, nRegs);  // call i2c start with 0 for write mode
}

void i2c_writeByte(uint8_t data) {

	// Transmit/send data to slave device
	while (!(I2C1->ISR & 1 << 0))
		;  // wait until TXE is set, ie trans buffer empty
	I2C1->TXDR = data;          // transmit register address
	GPIOA->BSRR = (1 << 5);      //LED ON
}

uint8_t i2c_readByte(void) {

	//read data from slave device
	while (!(I2C1->ISR & 1 << 2)) {

	}    // RXNE wait until receive buffer has data

	rdata = (uint8_t) (I2C1->RXDR); // read the rx buffer

	GPIOA->BSRR = (1 << 5);         //LED ON
	return rdata;
}

void i2c_stop() {

	I2C1->CR2 |= 1 << 14;          // stop condition rm pg1327

	while ((I2C1->CR2 & 1 << 14)) {


	} // wait until stop bit is set, and reset by PE being reset at stop condition


}

void i2c_readRequest(int saddr, int maddr, int nRegs) {

	i2c_beginWrite(saddr, 1);  // basically a start in write mode
	i2c_writeByte(maddr);         // write memory addr to read from
	i2c_beginRead(saddr, nRegs); // basically do repeat start and read from the memory addr
}

void i2c_Config(int saddr, int maddr, int wdata) {
	// saddr|(1<<25); slave address,Auto end mode
	i2c_beginWrite((saddr | (1 << 25)), 2); // basically a start in write mode, write 1 bytes
	i2c_writeByte(maddr);    // write/select memory addr you want to write to
	i2c_writeByte(wdata);    // write/select memory addr you want to write to

}

///////////12c2//////////////////////
void i2c2_init(void) {

	//pb13, pb14
	RCC->AHB2ENR |= 1 << 1;         //enable gpiob clock rm pg251
	RCC->APB1ENR1 |= 1 << 22;       //enable i2c2 clock rm pg253
	systickDelayMS(1);

	GPIOB->MODER = 0xEBAAFEBF; //set pb8and9 and pb10and11(used in usart3 below),PB13,14 for 12c2 to alternative function rm pg305
	//GPIOB->AFR[1] =0x04400044;   // AF4 12C on pb8 and 9 rm pg310, uCds pg93
	GPIOB->AFR[1] = 0x04407744; // enable usart3 on PB10, and PB11,// AF4 12C on pb8,9 and pb13,14 rm pg310, uCds pg93
	GPIOB->OTYPER = 0x6300;       // pb8,9 and pb13,14output open drain rm pg306

	i2c2_resetISR();
}

void i2c2_resetISR(void) {

	//I2C2->TIMINGR = 0x00610611;  // I2C at 400kHz cube
	I2C2->TIMINGR = 0x10911E24;  // I2C at 100kHz cube for pozyx to work well
	//I2C2->TIMINGR = 0x10320309;  // I2C at 400kHz ds
	//I2C2->TIMINGR = 0x30420F13;    // I2C at 100kHz ds
	I2C2->CR1 = 0;                 // PE cleared, SWresest rm pg1325
	while (I2C2->CR1 & 1 << 0)
		;       // wait for at least 3 APB clock cycle b4 set rm pg 1325
	I2C2->CR1 |= 0x01;              // set PE ie enable i2c

}

void i2c2_start(int saddr, int rwmode, int nRegs) {
	I2C2->CR2 = saddr;      // slave address,sw end mode, write mode(0) on reset
	I2C2->CR2 |= nRegs << 16;      // read/write n number of registers(byte)
	I2C2->CR2 |= rwmode << 10; // change to read mode if rwmode = 1, else remain in write mode
	I2C2->CR2 |= 1 << 13;           // start condition rm pg1327
	while ((I2C2->CR2 & 1 << 13))
		;   // wait until start bit is reset, meaning address sent
}

void i2c2_beginRead(int saddr, int nRegs) {
	i2c2_start(saddr, 1, nRegs); // call i2c start with 1 for read mode
}

void i2c2_beginWrite(int saddr, int nRegs) {
	i2c2_start(saddr, 0, nRegs);  // call i2c start with 0 for write mode
}

//void i2c_writeByte(int data)
void i2c2_writeByte(uint8_t data) {

	// Transmit/send data to slave device
	while (!(I2C2->ISR & 1 << 0))
		;  // wait until TXE is set, ie trans buffer empty
	I2C2->TXDR = data;          // transmit register address
	GPIOA->BSRR = (1 << 5);      //LED ON
}

uint8_t i2c2_readByte(void) {

	//read data from slave device
	while (!(I2C2->ISR & 1 << 2)) {

	}    // RXNE wait until receive buffer has data

	GPIOA->BSRR = (1 << 5);         //LED ON
	rdata = (uint8_t) (I2C2->RXDR); // read the rx buffer
	return rdata;
	//return (uint8_t)(I2C2->RXDR);
}

void i2c2_stop() {

	I2C2->CR2 |= 1 << 14;          // stop condition rm pg1327

	while (I2C2->CR2 & (1 << 14)) {

	} // wait until stop bit is set, and reset by PE being reset at stop condition


}

void i2c2_readRequest(int saddr, int maddr, int nRegs) {

	i2c2_beginWrite(saddr, 1);  // basically a start in write mode
	i2c2_writeByte(maddr);         // write memory addr to read from
	i2c2_beginRead(saddr, nRegs); // basically do repeat start and read from the memory addr
}

void i2c2_Config(int saddr, int maddr, int wdata) {
	// saddr|(1<<25); slave address,Auto end mode
	i2c2_beginWrite((saddr | (1 << 25)), 2); // basically a start in write mode, write 1 bytes
	i2c2_writeByte(maddr);    // write/select memory addr you want to write to
	i2c2_writeByte(wdata);    // write/select memory addr you want to write to
}

void blink_LED(int delayMS) {
	GPIOA->BSRR = (1 << 5);  //led on
	systickDelayMS(delayMS);
	GPIOA->BSRR = 1 << (5 + 16);  //led off
	systickDelayMS(delayMS);
}

void USART2_Init(void) {

	//PA2 TX
	//PA3 Rx
	RCC->APB1ENR1 |= 1 << 17;     // enable USART2 clk source BIT 17
	RCC->AHB2ENR |= (1 << 0);     // enable GPIOA clk, port A
	systickDelayMS(1);

	GPIOA->AFR[0] |= 0x7700;    // enable usart2 on PA3, and PA2
	GPIOA->MODER = 0xABABF7AA; // PA2 and PA3 in alternate mode, PA5 output,
	//PA10 alternate mode (for usart1), PA11 alt
	//PA9 alt,PA0 and PA1 alt

	USART2->BRR = 0x27; // 36MHz/921600 = 0x27 hex,  pg 1356/1395 of refernce manual

	USART2->CR1 = (1 << 2) |     // enable RX mode of USART RE (BIT 2)
			(1 << 3);      // enable TX mode of USART TE (BIT 3)
	USART2->CR1 |= 1 << 0; // enable USART UE BIT 0 (do after all USART config.)
}

void USART2_TX(char *data) {
	unsigned int i = 0;
	while (data[i]) {
		while (!(USART2->ISR & (1 << 7)))
			;     // wait for TX buffer(BIT 7) to be empty
		USART2->TDR = data[i] & 0xFF;       // Transmit element i of data array.
		i++;                            // Increment variable for array address.
	}

}
int USART2_write(int data) {
	while (!(USART2->ISR & (1 << 7)))
		;     // wait for TX buffer(BIT 7) to be empty
	USART2->TDR = data & 0xFF;             // Transmit element i of data array.
	return data;
}

char USART2_RX(void) {
	while (!(USART2->ISR & (1 << 5)))
		;     // wait for data to be available on RX buffer
	return (char) USART2->RDR;
}

//for radio, note receiver works at 115200 baud
void USART1_Init(int val) {
	//PA10 Rx
	RCC->APB2ENR |= 1 << 14;     // enable USART1 clk source BIT 14
	RCC->AHB2ENR |= (1 << 0);     // enable GPIOA clk, port A
	systickDelayMS(1);

	GPIOA->AFR[1] |= 0x0700;    // enable afternate mode on PA10
	GPIOA->MODER = 0xABABF7AA; // PA2 and PA3 in alternate mode, PA5 output,
	//PA10 alternate mode (for usart1), PA11 alt
	//PA9 alt,PA0 and PA1 alt
	RCC->AHB1ENR |= 1 << 0; //DMA1 clck enable
	DMA1_Channel5->CCR = 0; //clear config reg
	DMA1_CSELR->CSELR |= 1 << 17; //usart1 rx,ch5 sel.

	USART1->CR3 |= (1 << 6) | (1 << 12);  //DMAR//disable overrun ditection

	DMA1_Channel5->CPAR = (uint32_t) &USART1->RDR;
	DMA1_Channel5->CMAR = (uint32_t) ibus;

	DMA1_Channel5->CNDTR = (uint32_t) IBUS_BUFFSIZE; //number of byte to rx
	DMA1_Channel5->CCR |= (1 << 7) | (1 << 5) | (1 << 13); //minc,circ,high priority

	USART1->BRR = 0x139; // 36MHz/115200 = 139 hex,  pg 1356/1395 of refernce manual

	USART1->CR1 = (1 << 2) |     // enable RX mode of USART RE (BIT 2)
			(1 << 3);      // enable TX mode of USART TE (BIT 3)
	USART1->CR1 |= 1 << 0; // enable USART UE BIT 0 (do after all USART config.)

	DMA1_Channel5->CCR |= 1 << 0;        //enable
}

// USART3 for ESP AT commands
void USART3_Init(void) {

	//PB10 TX
	//PB11 Rx
	RCC->APB1ENR1 |= 1 << 18;     // enable USART3 clk source BIT 18
	systickDelayMS(1);

	GPIOB->AFR[1] = 0x04407744; // enable usart3 on PB10, and PB11,// AF4 12C on pb8,9 and pb13,14 rm pg310, uCds pg93
	GPIOB->MODER = 0xEBAAFEBF; //set pb8and9 and pb10and11(used in usart3 below),PB13,14 for 12c2 to alternative function rm pg305

	USART3->CR3 = (1 << 12);   //Disable overrun detection
	USART3->BRR = 0x27; // 36MHz/921600 = 0x27 hex,  pg 1356/1395 of refernce manual

	USART3->CR1 = (1 << 2) |     // enable RX mode of USART RE (BIT 2)
			(1 << 3);      // enable TX mode of USART TE (BIT 3)
	USART3->CR1 |= 1 << 0; // enable USART UE BIT 0 (do after all USART config.)
}

void USART3_TX(char *data) {
	unsigned int i = 0;
	while (data[i]) {

		while (!(USART3->ISR & (1 << 7)))
			;     // wait for TX buffer(BIT 7) to be empty
		USART3->TDR = data[i] & 0xFF;       // Transmit element i of data array.
		i++;                            // Increment variable for array address.
	}

}

int USART3_RX(void) {
	int count = 0;
	while (!(USART3->ISR & (1 << 5))) // wait for data to be available on RX buffer
	{

		if (count >= 500)     //Do not remove, TCP setup may hang
		{

			break;
		}
		count++;
	}

	return (USART3->RDR) & 0xFF;

}

void getTCPresponse(void) {

	for (int i = 0; i <= 10; i++) {
		int c = -1;
		c = USART3_RX();
		USART2_write(c);
		while (c != '\n') {
			if ((USART3->ISR & (1 << 5))) { // wait for data to be available on RX buffer

				c = (USART3->RDR) & 0xFF;
				USART2_write(c);
			}

		}

	}
}

void systickDelayMS(int n) {
	// Based  on 72MHz clk
	SysTick->LOAD = 72000 - 1; // 1m sec event @72MHz clk
	SysTick->VAL = 0;          // Clear current value reg.
	SysTick->CTRL = 0x5;        // Enable Systick

	for (int i = 0; i < n; i++) {
		while ((SysTick->CTRL & 0x10000) == 0)
			;
	}
	SysTick->CTRL = 0; // Disable SysTick
}

void systickDelayuS(int n) {
	// Based  on 72MHz clk
	SysTick->LOAD = 72 - 1;    // 1u sec event @72MHz clk
	SysTick->VAL = 0;          // Clear current value reg.
	SysTick->CTRL = 0x5;        // Enable Systick

	for (int i = 0; i < n; i++) {
		while ((SysTick->CTRL & 0x10000) == 0)
			;
	}
	SysTick->CTRL = 0; // Disable SysTick
}


/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
	RCC_OscInitTypeDef RCC_OscInitStruct = {0};
	RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLM = 1;
	RCC_OscInitStruct.PLL.PLLN = 9;//72mHz
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
	RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
	RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
			|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
	{
		Error_Handler();
	}
	/** Configure the main internal regulator output voltage
	 */
	if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
	{
		Error_Handler();
	}
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  Period elapsed callback in non blocking mode
 * @note   This function is called  when TIM7 interrupt took place, inside
 * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
 * a global variable "uwTick" used as application time base.
 * @param  htim : TIM handle
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	/* USER CODE BEGIN Callback 0 */

	/* USER CODE END Callback 0 */
	if (htim->Instance == TIM7) {
		HAL_IncTick();
	}
	/* USER CODE BEGIN Callback 1 */

	/* USER CODE END Callback 1 */
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */

	/* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
	/* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
