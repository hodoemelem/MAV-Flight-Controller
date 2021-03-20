/*
 * POS_KalmanFilter.h
 *
 *  Created on: 26 Dec 2020
 *      Author: HENRY ODOEMELEM
 */

#ifndef SRC_POS_KALMANFILTER_H_
#define SRC_POS_KALMANFILTER_H_



#endif /* SRC_POS_KALMANFILTER_H_ */

#include "stm32l4xx.h"                  // Device header
#define ARM_MATH_CM4
#include "arm_math.h"                   // ARM::CMSIS:DSP
#include <stdlib.h>


extern double Pxk;
extern double Pyk;
extern double Pzk;
extern double Pvxk;
extern double Pvyk;
extern double Pvzk;

uint32_t srcRows, srcColumns;	/* Temporary variables */



float32_t hFval[81];
float32_t hFTval[81];
float32_t hFxval[9];
float32_t hGval[9];
float32_t hGTval[9];
float32_t hGGTval[81];
float32_t hHval[27];
float32_t hHTval[27];
float32_t hyhatval[3];
float32_t hyval[3];
extern float32_t hxval[9];// state variables
float32_t hxtval[9];
float32_t hxtildeval[9];
float32_t hPval[81];
float32_t hPtval[81];
float32_t hPtildeval[81];
float32_t hFPval[81];
float32_t hFPFTval[81];
float32_t hQtval[81];
float32_t scale;
float32_t hHPval[27];
float32_t hHPHTval[9];
float32_t hPHTval[27];
float32_t hSval[9];
float32_t hSIval[9];
float32_t hKval[27];
float32_t hKTval[27];
float32_t hKSval[27];
float32_t hKSKTval[81];
float32_t hErval[3];
float32_t hKErval[9];
float32_t hRval[9];

/////////////////

arm_status status;
////////for POS
arm_matrix_instance_f32 hF;
arm_matrix_instance_f32 hFT;
arm_matrix_instance_f32 hFx;		/* Matrix A Instance */
arm_matrix_instance_f32 hG;
arm_matrix_instance_f32 hGT;
arm_matrix_instance_f32 hGGT;
arm_matrix_instance_f32 hH;
arm_matrix_instance_f32 hHT;
arm_matrix_instance_f32 hyhat;
arm_matrix_instance_f32 hy;
arm_matrix_instance_f32 hx;  // start zero deg.
arm_matrix_instance_f32 hxt;
arm_matrix_instance_f32 hxtilde;
arm_matrix_instance_f32 hP;
arm_matrix_instance_f32 hPt;
arm_matrix_instance_f32 hPtilde;
arm_matrix_instance_f32 hR;
arm_matrix_instance_f32 hFP;
arm_matrix_instance_f32 hFPFT;
arm_matrix_instance_f32 hQt;
arm_matrix_instance_f32 hHP;
arm_matrix_instance_f32 hHPHT;
arm_matrix_instance_f32 hPHT;
arm_matrix_instance_f32 hS;
arm_matrix_instance_f32 hSI;
arm_matrix_instance_f32 hK;
arm_matrix_instance_f32 hKT;
arm_matrix_instance_f32 hKS;
arm_matrix_instance_f32 hKSKT;
arm_matrix_instance_f32 hEr;
arm_matrix_instance_f32 hKEr;

////////////////////////////









void init_POS_KF(double hdt,double processScale,double hPn,double hRn,double initX,double initY,double initZ);
void POS_KFasymptotic(double posX,double posY,double posZ,double accXl,double accYl,double accZl,int isUpdatedFk);
void POS_KF(double posX,double posY,double posZ,double accXl,double accYl,double accZl,int isUpdatedFk);
void check_POScalc(int i);
