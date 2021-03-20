/*
 * lqrController.h
 *
 *  Created on: 26 Jan 2021
 *      Author: HENRY ODOEMELEM
 */

#ifndef SRC_LQRCONTROLLER_H_
#define SRC_LQRCONTROLLER_H_



#endif /* SRC_LQRCONTROLLER_H_ */

#include "stm32l4xx.h"                  // Device header
#define ARM_MATH_CM4
#include "arm_math.h"                   // ARM::CMSIS:DSP
#include <stdlib.h>

uint32_t srcRows, srcColumns;	/* Temporary variables */

//rps
extern double n1;
extern double n2;
extern double n3;
extern double n4;
//////////////////////

//rps
extern double F1;
extern double F2;
extern double F3;
extern double F4;
//////////////////////

extern float32_t Kdval[32] ;
extern double angleGain;
extern double rateGain;
extern double pzGain;
extern double vzGain;
void lqr_init(void);
void lqr(double roll,double rollRate, double pitch,double pitchRate,double yaw,double yawRate,double Z,double ZRate,
		double rolld,double rollRated, double pitchd,double pitchRated,double yawd,double yawRated,double Zd,double ZRated);



///*
// * lqrController.h
// *
// *  Created on: 26 Jan 2021
// *      Author: HENRY ODOEMELEM
// */
//
//#ifndef SRC_LQRCONTROLLER_H_
//#define SRC_LQRCONTROLLER_H_
//
//
//
//#endif /* SRC_LQRCONTROLLER_H_ */
//
//#include "stm32l4xx.h"                  // Device header
//#define ARM_MATH_CM4
//#include "arm_math.h"                   // ARM::CMSIS:DSP
//#include <stdlib.h>
//
//uint32_t srcRows, srcColumns;	/* Temporary variables */
//
////rps
//extern double n1;
//extern double n2;
//extern double n3;
//extern double n4;
////////////////////////
//
////rps
//extern double F1;
//extern double F2;
//extern double F3;
//extern double F4;
////////////////////////
//
//extern float32_t Kdval[16] ;
//extern double angleGain;
//extern double rateGain;
//
//void lqr_init(void);
//void lqr(double roll,double rollRate, double pitch,double pitchRate,double rolld,double rollRated, double pitchd,double pitchRated);
