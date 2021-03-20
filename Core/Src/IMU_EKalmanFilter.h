///*
// * IMU_EKalmanFilter.h
// *
// *  Created on: 27 Dec 2020
// *      Author: HENRY ODOEMELEM
// */
//
//#ifndef SRC_IMU_EKALMANFILTER_H_
//#define SRC_IMU_EKALMANFILTER_H_
//
//
//
//#endif /* SRC_IMU_EKALMANFILTER_H_ */
//
//#include "stm32l4xx.h"                  // Device header
//#define ARM_MATH_CM4
//#include "arm_math.h"                   // ARM::CMSIS:DSP
//#include <stdlib.h>
//
//uint32_t srcRows, srcColumns;	/* Temporary variables */
//
//////for IMU
//float32_t Fval[16];
//float32_t Feval[16];
//float32_t FTval[16];
//float32_t Gval[12];
//float32_t GTval[12];
//float32_t Hval[24];
//float32_t HTval[24];
//float32_t yhatval[6];
//float32_t qhatTEMPval[4];
//extern float32_t q[4];  // start zero deg.
//float32_t qt[4];
//float32_t qtildeval[4];
//float32_t qtildevalT[4];
//float32_t qtilde2val[16];
//float32_t PTEMPval[16];
//float32_t Ptval[16];
//float32_t Ptildeval[16];
//float32_t FPval[16];
//float32_t FPFTval[16];
//float32_t GQval[16];
//float32_t Qtval[16];
//float32_t HPval[24];
//float32_t HPHTval[36];
//float32_t PHTval[24];
//float32_t Sval[36];
//float32_t SIval[36];
//
//float32_t KTval[24];
//float32_t Erval[6];
//float32_t KErval[4];
//float32_t KSval[24];
//float32_t KSKTval[16];
//float32_t KHPtval[16];
//float32_t KHval[16];
//float32_t Jval[16];
//float32_t JTval[16];
//float32_t  JPtildeval[16];
//
//
//double deltaT;
//float32_t dt;
//
//float32_t qtildenorm;
//float32_t qtildenorm3;
//extern double gmps2;
/////////for imu
//arm_status status;
//arm_matrix_instance_f32 S;		/* Matrix A Instance */
//arm_matrix_instance_f32 SI;		/* Matrix A Instance */
//
//
//
//
//
//
//
//
//void  init_IMU_EKF(double idt,double iPn,double iRaxn,double iRayn,double iRazn,double iRmxn,double iRmyn,double iRmzn,double iQn);
//void IMU_EKF(double magX,double  magY,double magZ,double accX,double accY,double accZ,double gyroX,double gyroY,double gyroZ);
//void IMU_EKFasymptotic(double magX,double  magY,double magZ,double accX,double accY,double accZ,double gyroX,double gyroY,double gyroZ);
//void check_IMUcalc(int i);
//





///////////////with arm math
/*
 * IMU_EKalmanFilter.h
 *
 *  Created on: 27 Dec 2020
 *      Author: HENRY ODOEMELEM
 */

#ifndef SRC_IMU_EKALMANFILTER_H_
#define SRC_IMU_EKALMANFILTER_H_



#endif /* SRC_IMU_EKALMANFILTER_H_ */

#include "stm32l4xx.h"                  // Device header
#define ARM_MATH_CM4
#include "arm_math.h"                   // ARM::CMSIS:DSP
#include <stdlib.h>

uint32_t srcRows, srcColumns;	/* Temporary variables */

////for IMU
float32_t Fval[16];
float32_t Feval[16];
float32_t FTval[16];
float32_t Gval[12];
float32_t GTval[12];
float32_t Hval[24];
float32_t HTval[24];
float32_t yhatval[6];
float32_t qhatTEMPval[4];
extern float32_t q[4];  // start zero deg.
float32_t qt[4];
float32_t qtildeval[4];
float32_t qtildevalT[4];
float32_t qtilde2val[16];
float32_t PTEMPval[16];
float32_t Ptval[16];
float32_t Ptildeval[16];
float32_t FPval[16];
float32_t FPFTval[16];
float32_t GQval[16];
float32_t Qtval[16];
float32_t HPval[24];
float32_t HPHTval[36];
float32_t PHTval[24];
float32_t Sval[36];
float32_t SIval[36];
float32_t Kval[24];
float32_t KTval[24];
float32_t Erval[6];
float32_t KErval[4];
float32_t KSval[24];
float32_t KSKTval[16];
float32_t KHPtval[16];
float32_t KHval[16];
float32_t Jval[16];
float32_t JTval[16];
float32_t  JPtildeval[16];


double deltaT;
float32_t dt;

float32_t qtildenorm;
float32_t qtildenorm3;
extern double gmps2;
///////for imu
arm_status status;
arm_matrix_instance_f32 F;		/* Matrix A Instance */
arm_matrix_instance_f32 Fe;		/* Matrix A Instance */
arm_matrix_instance_f32 FT;		/* Matrix A Instance */
arm_matrix_instance_f32 G;		/* Matrix A Instance */
arm_matrix_instance_f32 GT;		/* Matrix A Instance */
arm_matrix_instance_f32 H;		/* Matrix A Instance */
arm_matrix_instance_f32 HT;		/* Matrix A Instance */
arm_matrix_instance_f32 yhat;		/* Matrix A Instance */
arm_matrix_instance_f32 Q;		/* Matrix A Instance */
arm_matrix_instance_f32 R;		/* Matrix A Instance */
arm_matrix_instance_f32 y;		/* Matrix A Instance */
arm_matrix_instance_f32 qhat;		/* Matrix A Instance */
arm_matrix_instance_f32 qhatt;		/* Matrix A Instance */
arm_matrix_instance_f32 qtilde;		/* Matrix A Instance */
arm_matrix_instance_f32 qtildeT;		/* Matrix A Instance */
arm_matrix_instance_f32 qtilde2;		/* Matrix A Instance */
arm_matrix_instance_f32 qhatTEMP;
arm_matrix_instance_f32 P;		/* Matrix A Instance */
arm_matrix_instance_f32 PTEMP;		/* Matrix A Instance */
arm_matrix_instance_f32 Pt;		/* Matrix A Instance */
arm_matrix_instance_f32 Ptilde;		/* Matrix A Instance */
arm_matrix_instance_f32 FP;		/* Matrix A Instance */
arm_matrix_instance_f32 FPFT;		/* Matrix A Instance */
arm_matrix_instance_f32 HP;		/* Matrix A Instance */
arm_matrix_instance_f32 HPHT;		/* Matrix A Instance */
arm_matrix_instance_f32 PHT;		/* Matrix A Instance */
arm_matrix_instance_f32 S;		/* Matrix A Instance */
arm_matrix_instance_f32 SI;		/* Matrix A Instance */
arm_matrix_instance_f32 K;		/* Matrix A Instance */
arm_matrix_instance_f32 KT;		/* Matrix A Instance */
arm_matrix_instance_f32 KS;		/* Matrix A Instance */
arm_matrix_instance_f32 KSKT;		/* Matrix A Instance */
arm_matrix_instance_f32 KH;
arm_matrix_instance_f32 KHPt;
arm_matrix_instance_f32 GQ;		/* Matrix A Instance */
arm_matrix_instance_f32 Qt;		/* Matrix A Instance */
arm_matrix_instance_f32 Er;		/* Matrix A Instance */
arm_matrix_instance_f32 KEr;		/* Matrix A Instance */
arm_matrix_instance_f32 J;		/* Matrix A Instance */
arm_matrix_instance_f32 JT;		/* Matrix A Instance */
arm_matrix_instance_f32 JPtilde;
arm_matrix_instance_f32 magraw;
arm_matrix_instance_f32 magb;
arm_matrix_instance_f32 magrawb;
arm_matrix_instance_f32 magA;
arm_matrix_instance_f32 magCalib;

/////////////////








void  init_IMU_EKF(double idt,double iPn,double iRaxn,double iRayn,double iRazn,double iRmxn,double iRmyn,double iRmzn,double iQn);
void IMU_EKF(double magX,double  magY,double magZ,double accX,double accY,double accZ,double gyroX,double gyroY,double gyroZ);
void check_IMUcalc(int i);




