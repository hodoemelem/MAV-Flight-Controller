/////////////////////////////////with arm math
/*
 * IMU_EKalmanFilter.c
 *
 *  Created on: 27 Dec 2020
 *      Author: HENRY ODOEMELEM
 */


#include "IMU_EKalmanFilter.h"
float32_t w[3] = {0,0,0};
float32_t we[3] = {0,0,0};
float32_t yval[6] = {0,0,0,0,0,0};

float32_t Pval[16] =
{
		1,      0,       0,       0,
		0,      1,       0,       0,
		0,      0,       1,       0,
		0,      0,       0,       1
};

// gyro, acc and mag var are assigned in code in place of the 1
float32_t Qval[9] =
{
		1,             0,                0,
		0,             1,                0,
		0,                      0,        1

};
float32_t Rval[36] =
{
		1,                0,          0,             0,         0,           0,
		0,                 1,          0,             0,         0,            0,
		0,                  0,        1,             0,         0,          0,
		0,                  0,        0,            1,         0,             0,
		0,                  0,        0,            0,         1,            0,
		0,                  0,        0,            0,         0,           1

};

float32_t magAval[9] =
{

		1.1552 ,  -0.0381,  -0.1380,
		-0.0381 ,   1.2091 ,  -0.0557,
		-0.1380 ,  -0.0557 ,   0.7362

};

float32_t magbval[3] = { 7.0275  ,  1.4082 ,  29.9378};


float32_t m[3] = { 19.639 ,   0.952 , 44.979}; // x forward direction

float32_t yAnorm = 1.0; // because the m vector only needs to be normilized
float32_t yMnorm = 1.0;

float32_t magrawbval[3];
float32_t magrawval[3];
float32_t magCalibval[3];

//gryo bais in dps
double wbx = 2.28E-4;
double wby = 1.99E-5;
double wbz = 1.33E-4;


void  init_IMU_EKF(double idt,double iPn,double iRaxn,double iRayn,double iRazn,double iRmxn,double iRmyn,double iRmzn,double iQn)
{
	/////for IMU
	srcRows = 4;
	srcColumns = 4;
	arm_mat_init_f32(&F, srcRows, srcColumns, (float32_t *)Fval);
	srcRows = 4;
	srcColumns = 4;
	arm_mat_init_f32(&FT, srcRows, srcColumns, FTval);
	srcRows = 4;
	srcColumns = 4;
	arm_mat_init_f32(&Fe, srcRows, srcColumns, (float32_t *)Feval);
	srcRows = 4;
	srcColumns = 3;
	arm_mat_init_f32(&G, srcRows, srcColumns, (float32_t *)Gval);
	srcRows = 3;
	srcColumns = 4;
	arm_mat_init_f32(&GT, srcRows, srcColumns, GTval);
	srcRows = 6;
	srcColumns = 4;
	arm_mat_init_f32(&H, srcRows, srcColumns, (float32_t *)Hval);
	srcRows = 4;
	srcColumns = 6;
	arm_mat_init_f32(&HT, srcRows, srcColumns, HTval);
	srcRows = 6;
	srcColumns = 1;
	arm_mat_init_f32(&yhat, srcRows, srcColumns, (float32_t *)yhatval);
	srcRows = 3;
	srcColumns = 3;
	arm_mat_init_f32(&Q, srcRows, srcColumns, (float32_t *)Qval);
	srcRows = 6;
	srcColumns = 6;
	arm_mat_init_f32(&R, srcRows, srcColumns, (float32_t *)Rval);
	srcRows = 6;
	srcColumns = 1;
	arm_mat_init_f32(&y, srcRows, srcColumns, (float32_t *)yval);
	srcRows = 4;
	srcColumns = 1;
	arm_mat_init_f32(&qhat, srcRows, srcColumns, (float32_t *)q);
	srcRows = 4;
	srcColumns = 1;
	arm_mat_init_f32(&qhatTEMP, srcRows, srcColumns, qhatTEMPval);
	srcRows = 4;
	srcColumns = 1;
	arm_mat_init_f32(&qhatt, srcRows, srcColumns, qt);
	srcRows = 4;
	srcColumns = 1;
	arm_mat_init_f32(&qtilde, srcRows, srcColumns, qtildeval);
	srcRows = 1;
	srcColumns = 4;
	arm_mat_init_f32(&qtildeT, srcRows, srcColumns, qtildevalT);
	srcRows = 4;
	srcColumns = 4;
	arm_mat_init_f32(&qtilde2, srcRows, srcColumns, qtilde2val);

	srcRows = 4;
	srcColumns = 4;
	arm_mat_init_f32(&P, srcRows, srcColumns, (float32_t *)Pval);
	srcRows = 4;
	srcColumns = 4;
	arm_mat_init_f32(&PTEMP, srcRows, srcColumns, (float32_t *)PTEMPval);
	srcRows = 4;
	srcColumns =4;
	arm_mat_init_f32(&Pt, srcRows, srcColumns, Ptval);
	srcRows = 4;
	srcColumns = 4;
	arm_mat_init_f32(&Ptilde, srcRows, srcColumns, Ptildeval);

	srcRows = 4;
	srcColumns = 4;
	arm_mat_init_f32(&FP, srcRows, srcColumns, FPval);
	srcRows = 4;
	srcColumns = 4;
	arm_mat_init_f32(&FPFT, srcRows, srcColumns, FPFTval);
	srcRows = 4;
	srcColumns = 3;
	arm_mat_init_f32(&GQ, srcRows, srcColumns, GQval);
	srcRows = 4;
	srcColumns = 4;
	arm_mat_init_f32(&Qt, srcRows, srcColumns, Qtval);

	srcRows = 6;
	srcColumns = 4;
	arm_mat_init_f32(&HP, srcRows, srcColumns, HPval);
	srcRows = 6;
	srcColumns = 6;
	arm_mat_init_f32(&HPHT, srcRows, srcColumns, HPHTval);
	srcRows = 4;
	srcColumns = 6;
	arm_mat_init_f32(&PHT, srcRows, srcColumns, PHTval);

	srcRows = 6;
	srcColumns = 6;
	arm_mat_init_f32(&S, srcRows, srcColumns, Sval);
	srcRows = 6;
	srcColumns = 6;
	arm_mat_init_f32(&SI, srcRows, srcColumns, SIval);

	srcRows = 4;
	srcColumns = 6;
	arm_mat_init_f32(&K, srcRows, srcColumns, Kval);
	srcRows = 6;
	srcColumns = 4;
	arm_mat_init_f32(&KT, srcRows, srcColumns, KTval);
	srcRows = 4;
	srcColumns = 6;
	arm_mat_init_f32(&KS, srcRows, srcColumns, KSval);
	srcRows = 4;
	srcColumns = 4;
	arm_mat_init_f32(&KSKT, srcRows, srcColumns, KSKTval);
	srcRows = 4;
	srcColumns = 4;
	arm_mat_init_f32(&KHPt, srcRows, srcColumns, KHPtval);
	srcRows = 4;
	srcColumns = 4;
	arm_mat_init_f32(&KH, srcRows, srcColumns, KHval);
	srcRows = 6;
	srcColumns = 1;
	arm_mat_init_f32(&Er, srcRows, srcColumns, Erval);

	srcRows = 4;
	srcColumns = 1;
	arm_mat_init_f32(&KEr, srcRows, srcColumns, KErval);

	srcRows = 4;
	srcColumns = 4;
	arm_mat_init_f32(&J, srcRows, srcColumns, Jval);

	srcRows = 4;
	srcColumns = 4;
	arm_mat_init_f32(&JT, srcRows, srcColumns, JTval);

	srcRows = 4;
	srcColumns = 4;
	arm_mat_init_f32(&JPtilde, srcRows, srcColumns, JPtildeval);

	srcRows = 1;
	srcColumns = 3;
	arm_mat_init_f32(&magCalib, srcRows, srcColumns, (float32_t *)magCalibval);
	srcRows = 3;
	srcColumns = 3;
	arm_mat_init_f32(&magA, srcRows, srcColumns, (float32_t *)magAval);
	srcRows = 1;
	srcColumns = 3;
	arm_mat_init_f32(&magb, srcRows, srcColumns, (float32_t *)magbval);
	srcRows = 1;
	srcColumns = 3;
	arm_mat_init_f32(&magraw, srcRows, srcColumns, (float32_t *)magrawval);
	srcRows = 1;
	srcColumns = 3;
	arm_mat_init_f32(&magrawb, srcRows, srcColumns, (float32_t *)magrawbval);
	/////////////////////////////////




	///For IMU
	deltaT = idt;// >>100 hertz
	dt = (deltaT)/2.0;

	//constant part of Fval

	Fval[0]=   1; Fval[5]=      1 ;  Fval[10]=      1 ;  Fval[15]=      1 ;
	//convergence of P if System is started as q(1,0,0,0) and sys is a 0 deg pitch and roll
	//	Pval[0] =	 2.8E-5;
	//	Pval[1] =	 3.655E-3;
	//	Pval[2] =	 -2.35E-2;
	//	Pval[3] =	-2.3E-5;

	Pval[0] =	 iPn;
	Pval[5] =	 iPn;
	Pval[10] =	 iPn;
	Pval[15] =	 iPn;


	//	 //	 //	 //gyro var
	Qval[0] = pow(iQn/sqrt(deltaT),2);//gx
	Qval[1] = pow(iQn/sqrt(deltaT),2);//gy
	Qval[2] = pow(iQn/sqrt(deltaT),2);//gz

	//	 //gravity var is lower than acc var
	Rval[0] = pow(iRaxn/sqrt(deltaT),2);//ax
	Rval[7] = pow(iRayn/sqrt(deltaT),2);//ay
	Rval[14] = pow(iRazn/sqrt(deltaT),2);//az

	//	 // mag var
	Rval[21] = pow(iRmxn/sqrt(deltaT),2);//mx
	Rval[28] = pow(iRmyn/sqrt(deltaT),2);//my
	Rval[35] = pow(iRmzn/sqrt(deltaT),2);//mz

	////////////////////////////////////////////////

}

void IMU_EKF(double magX,double  magY,double magZ,double accX,double accY,double accZ,double gyroX,double gyroY,double gyroZ)
{


	//magnetometer calibration magCorreted = (magraw- b)A
	magrawval[0]=  magX;
	magrawval[1]=  magY;
	magrawval[2]=  magZ;
	status = arm_mat_sub_f32(&magraw, &magb, &magrawb);
	status = arm_mat_mult_f32(&magrawb, &magA, &magCalib);

	yval[0] = accX;
	yval[1] = accY;
	yval[2] = accZ;
	yval[3] = magCalibval[0];
	yval[4] = magCalibval[1];
	yval[5] = magCalibval[2];

	//	/* Fval Buffer */

	//   w[0] = (gyroX  <0? -(abs(gyroX) - abs(wbx) ): (abs(gyroX) - abs(wbx)));
	//	 w[1] = (gyroY  <0? -(abs(gyroY) - abs(wby) ): (abs(gyroY) - abs(wby)));
	//	 w[2] = (gyroZ  <0? -(abs(gyroZ) - abs(wbz) ): (abs(gyroZ) - abs(wbz)));
	w[0] = gyroX;
	w[1] = gyroY;
	w[2] = gyroZ;
	//from schon
	Fval[1]=  -w[0]*dt;    Fval[2]=  -w[1]*dt;   Fval[3]=   -w[2]*dt;
	Fval[4]= w[0]*dt;      Fval[6]=  -w[2]*dt;   Fval[7]=   w[1]*dt;
	Fval[8]= w[1]*dt;  Fval[9]=  w[2]*dt;        Fval[11]=  -w[0]*dt;
	Fval[12]= w[2]*dt; Fval[13]=  -w[1]*dt;   Fval[14]=   w[0]*dt;

	status = arm_mat_trans_f32(&F, &FT);
	//check_calc(1);

	//  /* Gval Buffer */

	Gval[0] = q[1]*dt; Gval[1] =   q[2]*dt; Gval[2] =   q[3]*dt;
	Gval[3] = -q[0]*dt; Gval[4] =   q[3]*dt; Gval[5] =   -q[2]*dt;
	Gval[6] = -q[3]*dt; Gval[7] = -q[0]*dt;  Gval[8] = q[1]*dt;
	Gval[9] =q[2]*dt;  Gval[10] = -q[1]*dt;  Gval[11] = -q[0]*dt;

	status = arm_mat_trans_f32(&G, &GT);
	//check_calc(2);




	/* calculation of F Multiply with q */
	status = arm_mat_mult_f32(&F, &qhat, &qhatt);
	//check_calc(4);


	//q[0]=qt[0];q[1]=qt[1];q[2]=qt[2];q[3]=qt[3];


	//  sprintf(msg,"%s,%f,%f,%f,%f\n","qhat",qt[0],qt[1],qt[2],qt[3]);
	//  USART2_TX(msg);

	// output equation  6x1
	yhatval[0] = -2*(qt[1]*qt[3]-qt[0]*qt[2])*gmps2;
	yhatval[1] = -2*(qt[2]*qt[3]+qt[0]*qt[1])*gmps2;
	yhatval[2] = -1*(qt[0]*qt[0]-qt[1]*qt[1]-qt[2]*qt[2]+qt[3]*qt[3])*gmps2;
	yhatval[3] = ((qt[0]*qt[0]+qt[1]*qt[1]-qt[2]*qt[2]-qt[3]*qt[3])*m[0]  +   (2*(qt[1]*qt[2]+qt[0]*qt[3]))*m[1] + 2*(qt[1]*qt[3]-qt[0]*qt[2])*m[2]) ;
	yhatval[4] = (2*(qt[1]*qt[2]-qt[0]*qt[3])*m[0] + (qt[0]*qt[0]-qt[1]*qt[1]+qt[2]*qt[2]-qt[3]*qt[3])*m[1] +  (2*(qt[2]*qt[3]+qt[0]*qt[1]))*m[2]) ;
	yhatval[5] = (2*(qt[1]*qt[3]+qt[0]*qt[2])*m[0] + (2*(qt[2]*qt[3]-qt[0]*qt[1]))*m[1] +  (qt[0]*qt[0]-qt[1]*qt[1]-qt[2]*qt[2]+qt[3]*qt[3])*m[2]) ;

	//  H jacobain 6x4
	Hval[0] =	 (qt[2]*2)*gmps2;   Hval[1] =  (-qt[3]*2 )*gmps2;  Hval[2] =  (qt[0]*2 )*gmps2;  Hval[3] = (-qt[1]*2 )*gmps2;//r1
	Hval[4] =	(-qt[1]*2)*gmps2;   Hval[5] =  (-qt[0]*2 )*gmps2;  Hval[6] = (-qt[3]*2 )*gmps2;  Hval[7] = (-qt[2]*2 )*gmps2;//r2
	Hval[8] =	(-qt[0]*2)*gmps2;   Hval[9] =   (qt[1]*2 )*gmps2;  Hval[10] =  (qt[2]*2  )*gmps2; Hval[11] = (-qt[3]*2 )*gmps2;//r3
	//r4
	Hval[12] = (qt[0]*2*m[0]+	qt[3]*2*m[1]-qt[2]*2*m[2]) ; Hval[13] = (qt[1]*2*m[0]+qt[2]*2*m[1]+qt[3]*2*m[2]) ; Hval[14] = (-qt[2]*2*m[0]+  qt[1]*2*m[1]-qt[0]*2*m[2]) ;  Hval[15] = (-qt[3]*2*m[0]+ qt[0]*2*m[1]+qt[1]*2*m[2]) ;
	//r5
	Hval[16] = (-qt[3]*2*m[0]+	qt[0]*2*m[1]+qt[1]*2*m[2]) ; Hval[17] = (qt[2]*2*m[0]-qt[1]*2*m[1]+qt[0]*2*m[2])  ; Hval[18] = (qt[1]*2*m[0]+ qt[2]*2*m[1]+qt[3]*2*m[2]) ; Hval[19] = (-qt[0]*2*m[0] -qt[3]*2*m[1]+qt[2]*2*m[2]) ;
	//r6
	Hval[20] = (qt[2]*2*m[0]-qt[1]*2*m[1]+qt[0]*2*m[2]) ; Hval[21] =    (qt[3]*2*m[0]-qt[0]*2*m[1]-qt[1]*2*m[2]) ; Hval[22] = (qt[0]*2*m[0]+ qt[3]*2*m[1]-qt[2]*2*m[2]) ;  Hval[23] =  (qt[1]*2*m[0]+  qt[2]*2*m[1]+qt[3]*2*m[2]) ;
	// end  H jacobain

	status = arm_mat_trans_f32(&H, &HT);


	/* calculation of P*/
	status = arm_mat_mult_f32(&F, &P, &FP); //check_calc(5);
	status = arm_mat_mult_f32(&FP, &FT, &FPFT); //check_calc(6);
	status = arm_mat_mult_f32(&G, &Q, &GQ);     //check_calc(7);
	status = arm_mat_mult_f32(&GQ, &GT, &Qt);   //check_calc(8);
	status = arm_mat_add_f32(&FPFT, &Qt, &Pt);  // check_calc(9);

	//	//Pval[0]=Ptval[0];Pval[1]=Ptval[1];Pval[2]=Ptval[2];Pval[3]=Ptval[3];
	//	 //sprintf(msg,"%s,%f,%f,%f,%f\n","Fval",Fval[0],Fval[1],Fval[2],Fval[3]);
	//	 sprintf(msg,"%s,%f,%f,%f,%f\n","Qtval",Qtval[0],Qtval[5],Qtval[10],Qtval[15]);
	//	  USART2_TX(msg);
	// sprintf(msg,"%s,%f,%f,%f,%f\n","Ptval",Ptval[0],Ptval[5],Ptval[10],Ptval[15]);
	//sprintf(msg,"%s,%f,%f,%f,%f\n","FPFT",FPFTval[4],FPFTval[5],FPFTval[6],FPFTval[7]);
	//USART2_TX(msg);
	//

	/* calculation of K */
	status = arm_mat_mult_f32(&H, &Pt, &HP);    //check_calc(10);
	status = arm_mat_mult_f32(&HP, &HT, &HPHT); //  check_calc(11);
	status = arm_mat_add_f32(&HPHT, &R, &S);   // check_calc(12);
	status = arm_mat_inverse_f32(&S, &SI);     // check_calc(13);
	status = arm_mat_mult_f32(&Pt, &HT, &PHT); //  check_calc(14);
	status = arm_mat_mult_f32(&PHT, &SI, &K);  // check_calc(15);
	status = arm_mat_trans_f32(&K, &KT);  //check_calc(16);



	//  sprintf(msg,"%s,%f,%f,%f,%f,%f,%f\n","PHTval",PHTval[7],PHTval[12],PHTval[17],PHTval[22],PHTval[18],PHTval[23]);
	//  USART2_TX(msg);



	////	/* calculation of Er */
	status = arm_mat_sub_f32(&y, &yhat, &Er); // check_calc(17);


	//	sprintf(msg,"%s,%f,%f,%f,%f,%f,%f\n","yc",yhatval[0],yhatval[1],yhatval[2],yhatval[3],yhatval[4],yhatval[5]);
	//  USART2_TX(msg);
	//	sprintf(msg,"%s,%f,%f,%f,%f,%f,%f\n\n","Er",Erval[0],Erval[1],Erval[2],Erval[3],Erval[4],Erval[5]);
	//  USART2_TX(msg);
	//	sprintf(msg,"%s,%f,%f,%f,%f,%f,%f\n","Kval",Kval[0],Kval[1],Kval[2],Kval[3],Kval[4],Kval[5]);
	//  USART2_TX(msg);



	/* calculation  q update */
	status = arm_mat_mult_f32(&K, &Er, &KEr);      //    check_calc(181);
	status = arm_mat_add_f32(&qhatt, &KEr, &qtilde);  // check_calc(182);
	status = arm_mat_trans_f32(&qtilde, &qtildeT); // check_calc(19);


	//q[0]=qtildeval[0];q[1]=qtildeval[1];q[2]=qtildeval[2];q[3]=qtildeval[3];

	//		sprintf(msg,"%s,%f,%f,%f,%f,%f,%f\n","KErval",KErval[0],KErval[1],KErval[2],KErval[3],KErval[4],KErval[5]);
	//  USART2_TX(msg);
	//
	//	//sprintf(msg,"%s,%f,%f,%f,%f\n","qupdate",q[0],q[1],q[2],q[3]);
	//	sprintf(msg,"%s,%f,%f,%f,%f\n","qupdate",qtildeval[0],qtildeval[1],qtildeval[2],qtildeval[3]);
	//   USART2_TX(msg);


	/* calculation of Pt update */

	status = arm_mat_mult_f32(&K, &S, &KS);  // check_calc(20);
	status = arm_mat_mult_f32(&KS, &KT, &KSKT); // check_calc(21);
	status = arm_mat_sub_f32(&Pt, &KSKT, &Ptilde); // check_calc(22);



	/* renormalize q */
	qtildenorm = sqrt(qtildeval[0]*qtildeval[0]+qtildeval[1]*qtildeval[1]+qtildeval[2]*qtildeval[2]+qtildeval[3]*qtildeval[3]);
	status = arm_mat_scale_f32(&qtilde, (1/qtildenorm), &qhatTEMP); // new quaternion state
	//	check_calc(23);
	q[0]=qhatTEMPval[0];q[1]=qhatTEMPval[1];q[2]=qhatTEMPval[2];q[3]=qhatTEMPval[3];// new quaternion state

	//	sprintf(msg,"%s,%f,%f,%f,%f\n","qupdatenorm",q[0],q[1],q[2],q[3]);
	//	USART2_TX(msg);
	//	sprintf(msg,"%s,%f\n","qtildenorm",qtildenorm);
	//  USART2_TX(msg);

	/* renormalize P */
	qtildenorm3 = qtildenorm*qtildenorm*qtildenorm;
	status = arm_mat_mult_f32(&qtilde, &qtildeT, &qtilde2); // check_calc(24);
	status = arm_mat_scale_f32(&qtilde2, (1/qtildenorm3), &J);//  check_calc(25);
	status = arm_mat_trans_f32(&J, &JT);                       // check_calc(26);
	status = arm_mat_mult_f32(&J, &Ptilde,&JPtilde);          // check_calc(27);
	status = arm_mat_mult_f32(&JPtilde, &JT,&PTEMP); // check_calc(28); // new system covariance P
	//Pval[0]=PTEMPval[0];Pval[1]=PTEMPval[1];Pval[2]=PTEMPval[2];Pval[3]=PTEMPval[3];//// new system covariance P
	//Pval[0]=PTEMPval[0];
	//Pval[1]=PTEMPval[1];Pval[2]=PTEMPval[2];Pval[3]=PTEMPval[3];//// new system covariance P
	//Pval[5]=PTEMPval[5];Pval[10]=PTEMPval[10];Pval[15]=PTEMPval[15];//// new system covariance P

	for(int i=0;i<=15;i++)
	{
		Pval[i]=PTEMPval[i];

	}
	/* end renormalize P */


}
void check_IMUcalc(int i)
{
	//		if( status == ARM_MATH_SUCCESS)
	//	{
	//			sprintf(msg,"%d,%d\n",i,1);
	//      USART2_TX(msg);
	//	}
	//	else
	//	{
	//			sprintf(msg,"%d,%d\n",i,0);
	//     USART2_TX(msg);
	//	}
	//	status = ARM_MATH_TEST_FAILURE;
}

















///*
// * IMU_EKalmanFilter.c
// *
// *  Created on: 27 Dec 2020
// *      Author: HENRY ODOEMELEM
// */
//
//
//#include "IMU_EKalmanFilter.h"
//float32_t w[3] = {0,0,0};
//float32_t we[3] = {0,0,0};
//float32_t yval[6] = {0,0,0,0,0,0};
//
//float32_t Pval[16] =
//{
//		0.01,      0,       0,       0,
//		0,      0.01,       0,       0,
//		0,      0,       0.01,       0,
//		0,      0,       0,       0.01
//};
//
//// gyro, acc and mag var are assigned in code in place of the 1
//float32_t Qval[9] =
//{
//		1,             0,                0,
//		0,             1,                0,
//		0,                      0,        1
//
//};
//float32_t Rval[36] =
//{
//		1,                0,          0,             0,         0,           0,
//		0,                 1,          0,             0,         0,            0,
//		0,                  0,        1,             0,         0,          0,
//		0,                  0,        0,            1,         0,             0,
//		0,                  0,        0,            0,         1,            0,
//		0,                  0,        0,            0,         0,           1
//
//};
//
//float32_t magAval[9] =
//{
//
//		1.1552 ,  -0.0381,  -0.1380,
//		-0.0381 ,   1.2091 ,  -0.0557,
//		-0.1380 ,  -0.0557 ,   0.7362
//
//};
//
//float32_t magbval[3] = { 7.0275  ,  1.4082 ,  29.9378};
//
//
//float32_t m[3] = { 19.639 ,   0.952 , 44.979}; // x forward direction
//
//float32_t yAnorm = 1.0; // because the m vector only needs to be normilized
//float32_t yMnorm = 1.0;
//
//float32_t magrawbval[3];
//float32_t magrawval[3];
//float32_t magCalibval[3];
//
////gryo bais in dps
//double wbx = 2.28E-4;
//double wby = 1.99E-5;
//double wbz = 1.33E-4;
//
//
//
//float32_t Kval[24] = {1E-3};
//float32_t  gainAcc = 1E-3;
//float32_t  gainMAg = 1E-3;
//
//void  init_IMU_EKF(double idt,double iPn,double iRaxn,double iRayn,double iRazn,double iRmxn,double iRmyn,double iRmzn,double iQn)
//{
//
//
//	//	Kval[0]= gainAcc ;  Kval[1]= gainAcc ;  Kval[2]= gainAcc ; Kval[3]=gainMAg;Kval[4]= gainMAg;Kval[5]=gainMAg;
//	//
//	//	Kval[6]= gainAcc ;  Kval[7]= gainAcc ;  Kval[8]= gainAcc ; Kval[9]= gainMAg; Kval[10]= gainMAg; Kval[11]= gainMAg;
//	//	Kval[12]= gainAcc ; Kval[13]= gainAcc ; Kval[14]= gainAcc ; Kval[15]= gainMAg;Kval[16]= gainMAg; Kval[17]= gainMAg;
//	//
//	//	Kval[18]= gainAcc ; Kval[19]= gainAcc ; Kval[20]= gainAcc ; Kval[21]= gainMAg; Kval[22]= gainMAg; Kval[23]= gainMAg;
//
//	srcRows = 6;
//	srcColumns = 6;
//	arm_mat_init_f32(&S, srcRows, srcColumns, Sval);
//	srcRows = 6;
//	srcColumns = 6;
//	arm_mat_init_f32(&SI, srcRows, srcColumns, SIval);
//
//
//
//	///For IMU
//	deltaT = idt;// >>100 hertz
//	dt = (double)((deltaT)/2.0);
//
//	//constant part of Fval
//
//	Fval[0]=   1; Fval[5]=      1 ;  Fval[10]=      1 ;  Fval[15]=      1 ;
//	//convergence of P if System is started as q(1,0,0,0) and sys is a 0 deg pitch and roll
//	//	Pval[0] =	 2.8E-5;
//	//	Pval[1] =	 3.655E-3;
//	//	Pval[2] =	 -2.35E-2;
//	//	Pval[3] =	-2.3E-5;
//
//	Pval[0] =	 iPn;
//
//
//	Pval[5] =	 iPn;
//	Pval[10] =	 iPn;
//	Pval[15] =	 iPn;
//
//
//	//	 //	 //	 //gyro var
//	Qval[0] = iQn;//pow((double)(iQn/sqrt(deltaT)),2);//gx
//	Qval[1] = iQn;//pow((double)(iQn/sqrt(deltaT)),2);//gy
//	Qval[2] = iQn;//pow((double)(iQn/sqrt(deltaT)),2);//gz
//
//	//	 //gravity var is lower than acc var
//	Rval[0] = iRaxn;// pow((double)(iRaxn/sqrt(deltaT)),2);//ax
//	Rval[7] = iRayn; //pow((double)(iRayn/sqrt(deltaT)),2);//ay
//	Rval[14] = iRazn; //pow((double)(iRazn/sqrt(deltaT)),2);//az
//
//	//	 // mag var
//	Rval[21] = iRmxn; //pow((double)(iRmxn/sqrt(deltaT)),2);//mx
//	Rval[28] = iRmyn; //pow((double)(iRmyn/sqrt(deltaT)),2);//my
//	Rval[35] = iRmzn; //pow((double)(iRmzn/sqrt(deltaT)),2);//mz
//
//	////////////////////////////////////////////////
//	//
//	//	Kval[0]= -0.035957; Kval[1]= 0.02912; Kval[2]= 0.000140; Kval[3]=0.000308;Kval[4]=0.000066;Kval[5]=0.000037;
//	//	Kval[6]= -0.236523; Kval[7]= 0.099266; Kval[8]= -0.040512; Kval[9]= -0.017970; Kval[10]= -0.003876; Kval[11]= -0.002140;
//	//	Kval[12]= 0.186398; Kval[13]= -0.079144; Kval[14]= -0.032963; Kval[15]= 0.014202;Kval[16]= 0.003063; Kval[17]= 0.001692;
//	//	Kval[18]= 0.031433; Kval[19]= 0.040074; Kval[20]= -0.000689; Kval[21]= -0.000005; Kval[22]= -0.000001; Kval[23]= -0.000001;
//
//
//
//}
//
//void IMU_EKFasymptotic(double magX,double  magY,double magZ,double accX,double accY,double accZ,double gyroX,double gyroY,double gyroZ)
//{
//
//	//	gainAcc = 1E-3;
//	//	gainMAg = 1E-3;
//	//	Kval[0]= gainAcc ;  Kval[1]= gainAcc ;  Kval[2]= gainAcc ; Kval[3]=gainMAg;Kval[4]= gainMAg;Kval[5]=gainMAg;
//	//
//	//	Kval[6]= gainAcc ;  Kval[7]= gainAcc ;  Kval[8]= gainAcc ; Kval[9]= gainMAg; Kval[10]= gainMAg; Kval[11]= gainMAg;
//	//	Kval[12]= gainAcc ; Kval[13]= gainAcc ; Kval[14]= gainAcc ; Kval[15]= gainMAg;Kval[16]= gainMAg; Kval[17]= gainMAg;
//	//
//	//	Kval[18]= gainAcc ; Kval[19]= gainAcc ; Kval[20]= gainAcc ; Kval[21]= gainMAg; Kval[22]= gainMAg; Kval[23]= gainMAg;
//
//
//	//	//Prediction
//	w[0] = gyroX;
//	w[1] = gyroY;
//	w[2] = gyroZ;
//
//	//Fval 4x4 from schoen
//	Fval[1]=  -w[0]*dt;    Fval[2]=  -w[1]*dt;       Fval[3]=   -w[2]*dt;  //r1
//	Fval[4]= w[0]*dt;      Fval[6]=  -w[2]*dt;       Fval[7]=   w[1]*dt;   //r2
//	Fval[8]= w[1]*dt;      Fval[9]=  w[2]*dt;        Fval[11]=  -w[0]*dt;  //r3
//	Fval[12]= w[2]*dt;     Fval[13]=  -w[1]*dt;      Fval[14]=   w[0]*dt;     //r4
//	//end Fval
//
//
//
//	/* calculation of estimated qt; F Multiply with q */
//	//q 4x1
//	qt[0]= Fval[0]*q[0]+Fval[1]*q[1]+Fval[2]*q[2]+Fval[3]*q[3];//r1
//	qt[1]= Fval[4]*q[0]+Fval[5]*q[1]+Fval[6]*q[2]+Fval[7]*q[3];//r2
//	qt[2]= Fval[8]*q[0]+Fval[9]*q[1]+Fval[10]*q[2]+Fval[11]*q[3];//r3
//	qt[3]= Fval[12]*q[0]+Fval[13]*q[1]+Fval[14]*q[2]+Fval[15]*q[3];//r4
//	//end q
//	/* calculation of estimated qt; F Multiply with q */
//
//
//
//
//
//	///////////Correction
//
//	//magnetometer calibration magCorreted = (magraw- b)A
//	magrawval[0]=  magX;
//	magrawval[1]=  magY;
//	magrawval[2]=  magZ;
//
//
//	//magrawbval 1x3
//	magrawbval[0]= magrawval[0] - magbval[0]; magrawbval[1]= magrawval[1] - magbval[1];magrawbval[2]= magrawval[2] - magbval[2];//r1
//	//end magrawbval 1x3
//
//	//magCalibval 1x3
//	magCalibval[0]= magrawbval[0]*magAval[0]+magrawbval[1]*magAval[3]+magrawbval[2]*magAval[6];//c1
//	magCalibval[1]= magrawbval[0]*magAval[1]+magrawbval[1]*magAval[4]+magrawbval[2]*magAval[7];//c2
//	magCalibval[2]= magrawbval[0]*magAval[2]+magrawbval[1]*magAval[5]+magrawbval[2]*magAval[8];//c3
//	//end magCalibval
//
//	//measuered output yval 6x1
//	yval[0] = accX;
//	yval[1] = accY;
//	yval[2] = accZ;
//	yval[3] = magCalibval[0];
//	yval[4] = magCalibval[1];
//	yval[5] = magCalibval[2];
//	//end yval
//
//	//calc. output equation  6x1
//	yhatval[0] = -2*(qt[1]*qt[3]-qt[0]*qt[2])*gmps2;
//	yhatval[1] = -2*(qt[2]*qt[3]+qt[0]*qt[1])*gmps2;
//	yhatval[2] = -1*(qt[0]*qt[0]-qt[1]*qt[1]-qt[2]*qt[2]+qt[3]*qt[3])*gmps2;
//	yhatval[3] = ((qt[0]*qt[0]+qt[1]*qt[1]-qt[2]*qt[2]-qt[3]*qt[3])*m[0]  +   (2*(qt[1]*qt[2]+qt[0]*qt[3]))*m[1] + 2*(qt[1]*qt[3]-qt[0]*qt[2])*m[2]) ;
//	yhatval[4] = (2*(qt[1]*qt[2]-qt[0]*qt[3])*m[0] + (qt[0]*qt[0]-qt[1]*qt[1]+qt[2]*qt[2]-qt[3]*qt[3])*m[1] +  (2*(qt[2]*qt[3]+qt[0]*qt[1]))*m[2]) ;
//	yhatval[5] = (2*(qt[1]*qt[3]+qt[0]*qt[2])*m[0] + (2*(qt[2]*qt[3]-qt[0]*qt[1]))*m[1] +  (qt[0]*qt[0]-qt[1]*qt[1]-qt[2]*qt[2]+qt[3]*qt[3])*m[2]) ;
//
//	////	/* calculation of Er */
//	//Erval  6x1
//	Erval[0]= yval[0]-yhatval[0];//r1
//	Erval[1]= yval[1]-yhatval[1];//r2
//	Erval[2]= yval[2]-yhatval[2];//r3
//	Erval[3]= yval[3]-yhatval[3];//r4
//	Erval[4]= yval[4]-yhatval[4];//r5
//	Erval[5]= yval[5]-yhatval[5];//r6
//	//end Erval  6x1
//	/* end calculation of Er */
//
//	/* calculation  q update */
//	//KErval  4x1
//	KErval[0]=   Kval[0]* Erval[0]+Kval[1]* Erval[1]+Kval[2]* Erval[2]+Kval[3]* Erval[3]+Kval[4]* Erval[4]+Kval[5]* Erval[5]; //r1
//	KErval[1]=   Kval[6]* Erval[0]+Kval[7]* Erval[1]+Kval[8]* Erval[2]+Kval[9]* Erval[3]+Kval[10]*Erval[4]+Kval[11]*Erval[5]; //r2
//	KErval[2]=   Kval[12]*Erval[0]+Kval[13]*Erval[1]+Kval[14]*Erval[2]+Kval[15]*Erval[3]+Kval[16]*Erval[4]+Kval[17]*Erval[5]; //r3
//	KErval[3]=   Kval[18]*Erval[0]+Kval[19]*Erval[1]+Kval[20]*Erval[2]+Kval[21]*Erval[3]+Kval[22]*Erval[4]+Kval[23]*Erval[5]; //r4
//	//end KErval
//
//
//	//qtildeval  4x1
//	qtildeval[0]= qt[0]+KErval[0];//r1
//	qtildeval[1]= qt[1]+KErval[1];//r2
//	qtildeval[2]= qt[2]+KErval[2];//r3
//	qtildeval[3]= qt[3]+KErval[3];//r4
//	//end qtildeval
//	/*end  calculation  q update */
//
//	/* renormalize q */
//	qtildenorm = sqrt(qtildeval[0]*qtildeval[0]+qtildeval[1]*qtildeval[1]+qtildeval[2]*qtildeval[2]+qtildeval[3]*qtildeval[3]);
//	// new quaternion state
//	q[0]=(float32_t)(qtildeval[0]/qtildenorm);//r1
//	q[1]=(float32_t)(qtildeval[1]/qtildenorm);//r2
//	q[2]=(float32_t)(qtildeval[2]/qtildenorm);//r3
//	q[3]=(float32_t)(qtildeval[3]/qtildenorm);//r4
//	/* end renormalize q */
//
//}
//
//
//
//void IMU_EKF(double magX,double  magY,double magZ,double accX,double accY,double accZ,double gyroX,double gyroY,double gyroZ)
//{
//
//
//
//
//	//magnetometer calibration magCorreted = (magraw- b)A
//	magrawval[0]=  magX;
//	magrawval[1]=  magY;
//	magrawval[2]=  magZ;
//
//
//	//magrawbval 1x3
//	magrawbval[0]= magrawval[0] - magbval[0]; magrawbval[1]= magrawval[1] - magbval[1];magrawbval[2]= magrawval[2] - magbval[2];//r1
//	//end magrawbval 1x3
//
//	//magCalibval 1x3
//	magCalibval[0]= magrawbval[0]*magAval[0]+magrawbval[1]*magAval[3]+magrawbval[2]*magAval[6];//c1
//	magCalibval[1]= magrawbval[0]*magAval[1]+magrawbval[1]*magAval[4]+magrawbval[2]*magAval[7];//c2
//	magCalibval[2]= magrawbval[0]*magAval[2]+magrawbval[1]*magAval[5]+magrawbval[2]*magAval[8];//c3
//	//end magCalibval
//
//	//yval 6x1
//	yval[0] = accX;
//	yval[1] = accY;
//	yval[2] = accZ;
//	yval[3] = magCalibval[0];
//	yval[4] = magCalibval[1];
//	yval[5] = magCalibval[2];
//	//end yval
//
//	//	/* Fval Buffer */
//
//	//   w[0] = (gyroX  <0? -(abs(gyroX) - abs(wbx) ): (abs(gyroX) - abs(wbx)));
//	//	 w[1] = (gyroY  <0? -(abs(gyroY) - abs(wby) ): (abs(gyroY) - abs(wby)));
//	//	 w[2] = (gyroZ  <0? -(abs(gyroZ) - abs(wbz) ): (abs(gyroZ) - abs(wbz)));
//	w[0] = gyroX;
//	w[1] = gyroY;
//	w[2] = gyroZ;
//
//	//Fval 4x4 from schoen
//	Fval[1]=  -w[0]*dt;    Fval[2]=  -w[1]*dt;   Fval[3]=   -w[2]*dt;  //r1
//	Fval[4]= w[0]*dt;      Fval[6]=  -w[2]*dt;   Fval[7]=   w[1]*dt;   //r2
//	Fval[8]= w[1]*dt;  Fval[9]=  w[2]*dt;        Fval[11]=  -w[0]*dt;  //r3
//	Fval[12]= w[2]*dt; Fval[13]=  -w[1]*dt;   Fval[14]=   w[0]*dt;     //r4
//	//end Fval
//
//	//Fval 4x4
//	FTval[0]=  Fval[0];    FTval[1]=  Fval[4];  FTval[2]=  Fval[8];   FTval[3]=  Fval[12];  //r1
//	FTval[4]=  Fval[1];    FTval[5]=  Fval[5];  FTval[6]=  Fval[9];   FTval[7]=  Fval[13];  //r2
//	FTval[8]=  Fval[2];    FTval[9]=  Fval[6];  FTval[10]= Fval[10];  FTval[11]= Fval[14];  //r3
//	FTval[12]= Fval[3];    FTval[13]= Fval[7];  FTval[14]= Fval[11];  FTval[15]= Fval[15];  //r4
//	//end Fval 4x4
//
//
//
//
//	/* Gval Buffer 4x3*/
//
//	Gval[0] = q[1]*dt; Gval[1] =   q[2]*dt; Gval[2] =   q[3]*dt;    //r1
//	Gval[3] = -q[0]*dt; Gval[4] =   q[3]*dt; Gval[5] =   -q[2]*dt;  //r2
//	Gval[6] = -q[3]*dt; Gval[7] = -q[0]*dt;  Gval[8] = q[1]*dt;     //r3
//	Gval[9] =q[2]*dt;  Gval[10] = -q[1]*dt;  Gval[11] = -q[0]*dt;   //r4
//	/*end  Gval Buffer */
//
//	//Gval 3x4
//	GTval[0]=  Gval[0];    GTval[1]=  Gval[3];  GTval[2]=  Gval[6];  GTval[3]=  Gval[9];   //r1
//	GTval[4]=  Gval[1];    GTval[5]=  Gval[4];  GTval[6]=  Gval[7];  GTval[7]=  Gval[10];  //r2
//	GTval[8]=  Gval[2];    GTval[9]=  Gval[5];  GTval[10]= Gval[8];  GTval[11]= Gval[11];  //r3
//	//end Gval 3x4
//
//	/* calculation of estimated qt; F Multiply with q */
//	//q 4x1
//	qt[0]= Fval[0]*q[0]+Fval[1]*q[1]+Fval[2]*q[2]+Fval[3]*q[3];//r1
//	qt[1]= Fval[4]*q[0]+Fval[5]*q[1]+Fval[6]*q[2]+Fval[7]*q[3];//r2
//	qt[2]= Fval[8]*q[0]+Fval[9]*q[1]+Fval[10]*q[2]+Fval[11]*q[3];//r3
//	qt[3]= Fval[12]*q[0]+Fval[13]*q[1]+Fval[14]*q[2]+Fval[15]*q[3];//r4
//	//end q
//	/* calculation of estimated qt; F Multiply with q */
//
//	// output equation  6x1
//	yhatval[0] = -2*(qt[1]*qt[3]-qt[0]*qt[2])*gmps2;
//	yhatval[1] = -2*(qt[2]*qt[3]+qt[0]*qt[1])*gmps2;
//	yhatval[2] = -1*(qt[0]*qt[0]-qt[1]*qt[1]-qt[2]*qt[2]+qt[3]*qt[3])*gmps2;
//	yhatval[3] = ((qt[0]*qt[0]+qt[1]*qt[1]-qt[2]*qt[2]-qt[3]*qt[3])*m[0]  +   (2*(qt[1]*qt[2]+qt[0]*qt[3]))*m[1] + 2*(qt[1]*qt[3]-qt[0]*qt[2])*m[2]) ;
//	yhatval[4] = (2*(qt[1]*qt[2]-qt[0]*qt[3])*m[0] + (qt[0]*qt[0]-qt[1]*qt[1]+qt[2]*qt[2]-qt[3]*qt[3])*m[1] +  (2*(qt[2]*qt[3]+qt[0]*qt[1]))*m[2]) ;
//	yhatval[5] = (2*(qt[1]*qt[3]+qt[0]*qt[2])*m[0] + (2*(qt[2]*qt[3]-qt[0]*qt[1]))*m[1] +  (qt[0]*qt[0]-qt[1]*qt[1]-qt[2]*qt[2]+qt[3]*qt[3])*m[2]) ;
//
//	//  H jacobain 6x4
//	Hval[0] =	 (qt[2]*2)*gmps2;   Hval[1] =  (-qt[3]*2 )*gmps2;  Hval[2] =  (qt[0]*2 )*gmps2;  Hval[3] = (-qt[1]*2 )*gmps2;//r1
//	Hval[4] =	(-qt[1]*2)*gmps2;   Hval[5] =  (-qt[0]*2 )*gmps2;  Hval[6] = (-qt[3]*2 )*gmps2;  Hval[7] = (-qt[2]*2 )*gmps2;//r2
//	Hval[8] =	(-qt[0]*2)*gmps2;   Hval[9] =   (qt[1]*2 )*gmps2;  Hval[10] =  (qt[2]*2  )*gmps2; Hval[11] = (-qt[3]*2 )*gmps2;//r3
//	//r4
//	Hval[12] = (qt[0]*2*m[0]+	qt[3]*2*m[1]-qt[2]*2*m[2]) ; Hval[13] = (qt[1]*2*m[0]+qt[2]*2*m[1]+qt[3]*2*m[2]) ; Hval[14] = (-qt[2]*2*m[0]+  qt[1]*2*m[1]-qt[0]*2*m[2]) ;  Hval[15] = (-qt[3]*2*m[0]+ qt[0]*2*m[1]+qt[1]*2*m[2]) ;
//	//r5
//	Hval[16] = (-qt[3]*2*m[0]+	qt[0]*2*m[1]+qt[1]*2*m[2]) ; Hval[17] = (qt[2]*2*m[0]-qt[1]*2*m[1]+qt[0]*2*m[2])  ; Hval[18] = (qt[1]*2*m[0]+ qt[2]*2*m[1]+qt[3]*2*m[2]) ; Hval[19] = (-qt[0]*2*m[0] -qt[3]*2*m[1]+qt[2]*2*m[2]) ;
//	//r6
//	Hval[20] = (qt[2]*2*m[0]-qt[1]*2*m[1]+qt[0]*2*m[2]) ; Hval[21] =    (qt[3]*2*m[0]-qt[0]*2*m[1]-qt[1]*2*m[2]) ; Hval[22] = (qt[0]*2*m[0]+ qt[3]*2*m[1]-qt[2]*2*m[2]) ;  Hval[23] =  (qt[1]*2*m[0]+  qt[2]*2*m[1]+qt[3]*2*m[2]) ;
//	// end  H jacobain
//
//	//HTval 4x6
//	HTval[0]=  Hval[0];    HTval[1]=  Hval[4];  HTval[2]=  Hval[8];  HTval[3]=  Hval[12];  HTval[4]=  Hval[16];  HTval[5]=  Hval[20];//r1
//	HTval[6]=  Hval[1];    HTval[7]=  Hval[5];  HTval[8]=  Hval[9];  HTval[9]=  Hval[13];  HTval[10]= Hval[17];  HTval[11]= Hval[21];//r2
//	HTval[12]= Hval[2];    HTval[13]= Hval[6];  HTval[14]= Hval[10];  HTval[15]= Hval[14];  HTval[16]= Hval[18];  HTval[17]= Hval[22];//r3
//	HTval[18]= Hval[3];    HTval[19]= Hval[7];  HTval[20]= Hval[11];  HTval[21]= Hval[15];  HTval[22]= Hval[19];  HTval[23]= Hval[23];//r4
//	//end HTval 4x6
//
//
//
//
//
//
//	//	// output equation  6x1
//	//	yhatval[0] = -2*(q[1]*q[3]-q[0]*q[2])*gmps2;
//	//	yhatval[1] = -2*(q[2]*q[3]+q[0]*q[1])*gmps2;
//	//	yhatval[2] = -1*(q[0]*q[0]-q[1]*q[1]-q[2]*q[2]+q[3]*q[3])*gmps2;
//	//	yhatval[3] = ((q[0]*q[0]+q[1]*q[1]-q[2]*q[2]-q[3]*q[3])*m[0]  +   (2*(q[1]*q[2]+q[0]*q[3]))*m[1] + 2*(q[1]*q[3]-q[0]*q[2])*m[2]) ;
//	//	yhatval[4] = (2*(q[1]*q[2]-q[0]*q[3])*m[0] + (q[0]*q[0]-q[1]*q[1]+q[2]*q[2]-q[3]*q[3])*m[1] +  (2*(q[2]*q[3]+q[0]*q[1]))*m[2]) ;
//	//	yhatval[5] = (2*(q[1]*q[3]+q[0]*q[2])*m[0] + (2*(q[2]*q[3]-q[0]*q[1]))*m[1] +  (q[0]*q[0]-q[1]*q[1]-q[2]*q[2]+q[3]*q[3])*m[2]) ;
//	//
//	//	//  H jacobain 6x4
//	//	Hval[0] =	 (q[2]*2)*gmps2;   Hval[1] =  (-q[3]*2 )*gmps2;  Hval[2] =  (q[0]*2 )*gmps2;  Hval[3] = (-q[1]*2 )*gmps2;//r1
//	//	Hval[4] =	(-q[1]*2)*gmps2;   Hval[5] =  (-q[0]*2 )*gmps2;  Hval[6] = (-q[3]*2 )*gmps2;  Hval[7] = (-q[2]*2 )*gmps2;//r2
//	//	Hval[8] =	(-q[0]*2)*gmps2;   Hval[9] =   (q[1]*2 )*gmps2;  Hval[10] =  (q[2]*2  )*gmps2; Hval[11] = (-q[3]*2 )*gmps2;//r3
//	//	//r4
//	//	Hval[12] = (q[0]*2*m[0]+	q[3]*2*m[1]-q[2]*2*m[2]) ; Hval[13] = (q[1]*2*m[0]+q[2]*2*m[1]+q[3]*2*m[2]) ; Hval[14] = (-q[2]*2*m[0]+  q[1]*2*m[1]-q[0]*2*m[2]) ;  Hval[15] = (-q[3]*2*m[0]+ q[0]*2*m[1]+q[1]*2*m[2]) ;
//	//	//r5
//	//	Hval[16] = (-q[3]*2*m[0]+	q[0]*2*m[1]+q[1]*2*m[2]) ; Hval[17] = (q[2]*2*m[0]-q[1]*2*m[1]+q[0]*2*m[2])  ; Hval[18] = (q[1]*2*m[0]+ q[2]*2*m[1]+q[3]*2*m[2]) ; Hval[19] = (-q[0]*2*m[0] -q[3]*2*m[1]+q[2]*2*m[2]) ;
//	//	//r6
//	//	Hval[20] = (q[2]*2*m[0]-q[1]*2*m[1]+q[0]*2*m[2]) ; Hval[21] =    (q[3]*2*m[0]-q[0]*2*m[1]-q[1]*2*m[2]) ; Hval[22] = (q[0]*2*m[0]+ q[3]*2*m[1]-q[2]*2*m[2]) ;  Hval[23] =  (q[1]*2*m[0]+  q[2]*2*m[1]+q[3]*2*m[2]) ;
//	//	// end  H jacobain
//	//
//	//	//HTval 4x6
//	//	HTval[0]=  Hval[0];    HTval[1]=  Hval[4];  HTval[2]=  Hval[8];  HTval[3]=  Hval[12];  HTval[4]=  Hval[16];  HTval[5]=  Hval[20];//r1
//	//	HTval[6]=  Hval[1];    HTval[7]=  Hval[5];  HTval[8]=  Hval[9];  HTval[9]=  Hval[13];  HTval[10]= Hval[17];  HTval[11]= Hval[21];//r2
//	//	HTval[12]= Hval[2];    HTval[13]= Hval[6];  HTval[14]= Hval[10];  HTval[15]= Hval[14];  HTval[16]= Hval[18];  HTval[17]= Hval[22];//r3
//	//	HTval[18]= Hval[3];    HTval[19]= Hval[7];  HTval[20]= Hval[11];  HTval[21]= Hval[15];  HTval[22]= Hval[19];  HTval[23]= Hval[23];//r4
//	//	//end HTval 4x6
//	//
//	//
//	//	/* calculation of estimated qt; F Multiply with q */
//	//	//q 4x1
//	//	qt[0]= Fval[0]*q[0]+Fval[1]*q[1]+Fval[2]*q[2]+Fval[3]*q[3];//r1
//	//	qt[1]= Fval[4]*q[0]+Fval[5]*q[1]+Fval[6]*q[2]+Fval[7]*q[3];//r2
//	//	qt[2]= Fval[8]*q[0]+Fval[9]*q[1]+Fval[10]*q[2]+Fval[11]*q[3];//r3
//	//	qt[3]= Fval[12]*q[0]+Fval[13]*q[1]+Fval[14]*q[2]+Fval[15]*q[3];//r4
//	//	//end q
//	//	/* calculation of estimated qt; F Multiply with q */
//
//
//	/* calculation of P*/
//	//FP 4x4
//	//r1
//	FPval[0]=  Fval[0]*Pval[0]+Fval[1]*Pval[4]+Fval[2]*Pval[8]+Fval[3]*Pval[12];      FPval[1]=  Fval[0]*Pval[1]+Fval[1]*Pval[5]+Fval[2]*Pval[9]+Fval[3]*Pval[13];
//	FPval[2]=  Fval[0]*Pval[2]+Fval[1]*Pval[6]+Fval[2]*Pval[10]+Fval[3]*Pval[14];     FPval[3]=  Fval[0]*Pval[3]+Fval[1]*Pval[7]+Fval[2]*Pval[11]+Fval[3]*Pval[15];
//	//r2
//	FPval[4]=  Fval[4]*Pval[0]+Fval[5]*Pval[4]+Fval[6]*Pval[8]+Fval[7]*Pval[12];      FPval[5]=  Fval[4]*Pval[1]+Fval[5]*Pval[5]+Fval[6]*Pval[9]+Fval[7]*Pval[13];
//	FPval[6]=  Fval[4]*Pval[2]+Fval[5]*Pval[6]+Fval[6]*Pval[10]+Fval[7]*Pval[14];     FPval[7]=  Fval[4]*Pval[3]+Fval[5]*Pval[7]+Fval[6]*Pval[11]+Fval[7]*Pval[15];
//	//r3
//	FPval[8]=  Fval[8]*Pval[0]+Fval[9]*Pval[4]+Fval[10]*Pval[8]+Fval[11]*Pval[12];    FPval[9]=  Fval[8]*Pval[1]+Fval[9]*Pval[5]+Fval[10]*Pval[9]+Fval[11]*Pval[13];
//	FPval[10]= Fval[8]*Pval[2]+Fval[9]*Pval[6]+Fval[10]*Pval[10]+Fval[11]*Pval[14];   FPval[11]= Fval[8]*Pval[3]+Fval[9]*Pval[7]+Fval[10]*Pval[11]+Fval[11]*Pval[15];
//	//r4
//	FPval[12]= Fval[12]*Pval[0]+Fval[13]*Pval[4]+Fval[14]*Pval[8]+Fval[15]*Pval[12];  FPval[13]= Fval[12]*Pval[1]+Fval[13]*Pval[5]+Fval[14]*Pval[9]+Fval[15]*Pval[13];
//	FPval[14]= Fval[12]*Pval[2]+Fval[13]*Pval[6]+Fval[14]*Pval[10]+Fval[15]*Pval[14]; FPval[15]= Fval[12]*Pval[3]+Fval[13]*Pval[7]+Fval[14]*Pval[11]+Fval[15]*Pval[15];
//	///End FP
//
//	//FPFT4x4
//	//r1
//	FPFTval[0]=  FPval[0]*FTval[0]+FPval[1]*FTval[4]+FPval[2]*FTval[8]+FPval[3]*FTval[12];      FPFTval[1]=  FPval[0]*FTval[1]+FPval[1]*FTval[5]+FPval[2]*FTval[9]+FPval[3]*FTval[13];
//	FPFTval[2]=  FPval[0]*FTval[2]+FPval[1]*FTval[6]+FPval[2]*FTval[10]+FPval[3]*FTval[14];     FPFTval[3]=  FPval[0]*FTval[3]+FPval[1]*FTval[7]+FPval[2]*FTval[11]+FPval[3]*FTval[15];
//	//r2
//	FPFTval[4]=  FPval[4]*FTval[0]+FPval[5]*FTval[4]+Fval[6]*FTval[8]+FPval[7]*FTval[12];       FPFTval[5]=  FPval[4]*FTval[1]+FPval[5]*FTval[5]+FPval[6]*FTval[9]+FPval[7]*FTval[13];
//	FPFTval[6]=  FPval[4]*FTval[2]+FPval[5]*FTval[6]+FPval[6]*FTval[10]+FPval[7]*FTval[14];     FPFTval[7]=  FPval[4]*FTval[3]+FPval[5]*FTval[7]+FPval[6]*FTval[11]+FPval[7]*FTval[15];
//	//r3
//	FPFTval[8]=  FPval[8]*FTval[0]+FPval[9]*FTval[4]+FPval[10]*FTval[8]+FPval[11]*FTval[12];    FPFTval[9]=  FPval[8]*FTval[1]+FPval[9]*FTval[5]+FPval[10]*FTval[9]+FPval[11]*FTval[13];
//	FPFTval[10]= FPval[8]*FTval[2]+FPval[9]*FTval[6]+FPval[10]*FTval[10]+FPval[11]*FTval[14];   FPFTval[11]= FPval[8]*FTval[3]+FPval[9]*FTval[7]+FPval[10]*FTval[11]+FPval[11]*FTval[15];
//	//r4
//	FPFTval[12]= FPval[12]*FTval[0]+FPval[13]*FTval[4]+FPval[14]*FTval[8]+FPval[15]*FTval[12];  FPFTval[13]= FPval[12]*FTval[1]+FPval[13]*FTval[5]+FPval[14]*FTval[9]+FPval[15]*FTval[13];
//	FPFTval[14]= FPval[12]*FTval[2]+FPval[13]*FTval[6]+FPval[14]*FTval[10]+FPval[15]*FTval[14]; FPFTval[15]= FPval[12]*FTval[3]+FPval[13]*FTval[7]+FPval[14]*FTval[11]+FPval[15]*FTval[15];
//	////end FPFT
//
//	//GQ 4x3
//	//r1
//	GQval[0]=  Gval[0]*Qval[0]+Gval[1]*Qval[3]+Gval[2]*Qval[6];     GQval[1]=  Gval[0]*Qval[1]+Gval[1]*Qval[4]+Gval[2]*Qval[7];
//	GQval[2]=  Gval[0]*Qval[2]+Gval[1]*Qval[5]+Gval[2]*Qval[8];
//	//r2
//	GQval[3]=  Gval[3]*Qval[0]+Gval[4]*Qval[3]+Gval[5]*Qval[6];     GQval[4]=  Gval[3]*Qval[1]+Gval[4]*Qval[4]+Gval[5]*Qval[7];
//	GQval[5]=  Gval[3]*Qval[2]+Gval[4]*Qval[5]+Gval[5]*Qval[8];
//	//r3
//	GQval[6]=  Gval[6]*Qval[0]+Gval[7]*Qval[3]+Gval[8]*Qval[6];     GQval[7]=  Gval[6]*Qval[1]+Gval[7]*Qval[4]+Gval[8]*Qval[7];
//	GQval[8]=  Gval[6]*Qval[2]+Gval[7]*Qval[5]+Gval[8]*Qval[8];
//	//r4
//	GQval[9]=  Gval[9]*Qval[0]+Gval[10]*Qval[3]+Gval[11]*Qval[6];   GQval[10]=  Gval[9]*Qval[1]+Gval[10]*Qval[4]+Gval[11]*Qval[7];
//	GQval[11]=  Gval[9]*Qval[2]+Gval[10]*Qval[5]+Gval[11]*Qval[8];
//	/// End GQ
//
//	//Qt 4x4
//	//r1
//	Qtval[0]=   GQval[0]*GTval[0]+GQval[1]*GTval[4]+GQval[2]*GTval[8];     Qtval[1]=  GQval[0]*GTval[1]+GQval[1]*GTval[5]+GQval[2]*GTval[9];
//	Qtval[2]=   GQval[0]*GTval[2]+GQval[1]*GTval[6]+GQval[2]*GTval[10];    Qtval[3]=  GQval[0]*GTval[3]+GQval[1]*GTval[7]+GQval[2]*GTval[11];
//	//r2
//	Qtval[4]=   GQval[3]*GTval[0]+GQval[4]*GTval[4]+GQval[5]*GTval[8];     Qtval[5]=  GQval[3]*GTval[1]+GQval[4]*GTval[5]+GQval[5]*GTval[9];
//	Qtval[6]=   GQval[3]*GTval[2]+GQval[4]*GTval[6]+GQval[5]*GTval[10];    Qtval[7]=  GQval[3]*GTval[3]+GQval[4]*GTval[7]+GQval[5]*GTval[11];
//	//r3
//	Qtval[8]=   GQval[6]*GTval[0]+GQval[7]*GTval[4]+GQval[8]*GTval[8];     Qtval[9]=  GQval[6]*GTval[1]+GQval[7]*GTval[5]+GQval[8]*GTval[9];
//	Qtval[10]=  GQval[6]*GTval[2]+GQval[7]*GTval[6]+GQval[8]*GTval[10];    Qtval[11]= GQval[6]*GTval[3]+GQval[7]*GTval[7]+GQval[8]*GTval[11];
//	//r4
//	Qtval[12]=  GQval[9]*GTval[0]+GQval[10]*GTval[4]+GQval[11]*GTval[8];   Qtval[13]= GQval[9]*GTval[1]+GQval[10]*GTval[5]+GQval[11]*GTval[9];
//	Qtval[14]=  GQval[9]*GTval[2]+GQval[10]*GTval[6]+GQval[11]*GTval[10];  Qtval[15]= GQval[9]*GTval[3]+GQval[10]*GTval[7]+GQval[11]*GTval[11];
//	/// End Qt
//
//
//	//Ptval 4x4
//	Ptval[0]=  FPFTval[0]+Qtval[0];      Ptval[1]=  FPFTval[1]+Qtval[1];     Ptval[2]=  FPFTval[2]+Qtval[2];     Ptval[3]=  FPFTval[3]+Qtval[3];//r1
//	Ptval[4]=  FPFTval[4]+Qtval[4];      Ptval[5]=  FPFTval[5]+Qtval[5];     Ptval[6]=  FPFTval[6]+Qtval[6];     Ptval[7]=  FPFTval[7]+Qtval[7];//r2
//	Ptval[8]=  FPFTval[8]+Qtval[8];      Ptval[9]=  FPFTval[9]+Qtval[9];     Ptval[10]= FPFTval[10]+Qtval[10];   Ptval[11]= FPFTval[11]+Qtval[11];//r3
//	Ptval[12]= FPFTval[12]+Qtval[12];    Ptval[13]= FPFTval[13]+Qtval[13];   Ptval[14]= FPFTval[14]+Qtval[14];   Ptval[15]= FPFTval[15]+Qtval[15];//r4
//	//end Ptval
//	/*end  calculation of P*/
//
//
//	/* calculation of K */
//	//HPval 6x4
//	//r1
//	HPval[0]=   Hval[0]*Ptval[0]+Hval[1]*Ptval[4]+Hval[2]*Ptval[8]+Hval[3]*Ptval[12];      HPval[1]=   Hval[0]*Ptval[1]+Hval[1]*Ptval[5]+Hval[2]*Ptval[9]+Hval[3]*Ptval[13];
//	HPval[2]=   Hval[0]*Ptval[2]+Hval[1]*Ptval[6]+Hval[2]*Ptval[10]+Hval[3]*Ptval[14];     HPval[3]=   Hval[0]*Ptval[3]+Hval[1]*Ptval[7]+Hval[2]*Ptval[11]+Hval[3]*Ptval[15];
//	//r2
//	HPval[4]=   Hval[4]*Ptval[0]+Hval[5]*Ptval[4]+Hval[6]*Ptval[8]+Hval[7]*Ptval[12];      HPval[5]=   Hval[4]*Ptval[1]+Hval[5]*Ptval[5]+Hval[6]*Ptval[9]+Hval[7]*Ptval[13];
//	HPval[6]=   Hval[4]*Ptval[2]+Hval[5]*Ptval[6]+Hval[6]*Ptval[10]+Hval[7]*Ptval[14];     HPval[7]=   Hval[4]*Ptval[3]+Hval[5]*Ptval[7]+Hval[6]*Ptval[11]+Hval[7]*Ptval[15];
//	//r3
//	HPval[8]=   Hval[8]*Ptval[0]+Hval[9]*Ptval[4]+Hval[10]*Ptval[8]+Hval[11]*Ptval[12];    HPval[9]=   Hval[8]*Ptval[1]+Hval[9]*Ptval[5]+Hval[10]*Ptval[9]+Hval[11]*Ptval[13];
//	HPval[10]=  Hval[8]*Ptval[2]+Hval[9]*Ptval[6]+Hval[10]*Ptval[10]+Hval[11]*Ptval[14];   HPval[11]=  Hval[8]*Ptval[3]+Hval[9]*Ptval[7]+Hval[10]*Ptval[11]+Hval[11]*Ptval[15];
//	//r4
//	HPval[12]=  Hval[12]*Ptval[0]+Hval[13]*Ptval[4]+Hval[14]*Ptval[8]+Hval[15]*Ptval[12];  HPval[13]=  Hval[12]*Ptval[1]+Hval[13]*Ptval[5]+Hval[14]*Ptval[9]+Hval[15]*Ptval[13];
//	HPval[14]=  Hval[12]*Ptval[2]+Hval[13]*Ptval[6]+Hval[14]*Ptval[10]+Hval[15]*Ptval[14]; HPval[15]=  Hval[12]*Ptval[3]+Hval[13]*Ptval[7]+Hval[14]*Ptval[11]+Hval[15]*Ptval[15];
//	//r5
//	HPval[16]=  Hval[16]*Ptval[0]+Hval[17]*Ptval[4]+Hval[18]*Ptval[8]+Hval[19]*Ptval[12];  HPval[17]=  Hval[16]*Ptval[1]+Hval[17]*Ptval[5]+Hval[18]*Ptval[9]+Hval[19]*Ptval[13];
//	HPval[18]=  Hval[16]*Ptval[2]+Hval[17]*Ptval[6]+Hval[18]*Ptval[10]+Hval[19]*Ptval[14]; HPval[19]=  Hval[16]*Ptval[3]+Hval[17]*Ptval[7]+Hval[18]*Ptval[11]+Hval[19]*Ptval[15];
//	//r3
//	HPval[20]=  Hval[20]*Ptval[0]+Hval[21]*Ptval[4]+Hval[22]*Ptval[8]+Hval[23]*Ptval[12];  HPval[21]=  Hval[20]*Ptval[1]+Hval[21]*Ptval[5]+Hval[22]*Ptval[9]+Hval[23]*Ptval[13];
//	HPval[22]=  Hval[20]*Ptval[2]+Hval[21]*Ptval[6]+Hval[22]*Ptval[10]+Hval[23]*Ptval[14]; HPval[23]=  Hval[20]*Ptval[3]+Hval[21]*Ptval[7]+Hval[22]*Ptval[11]+Hval[23]*Ptval[15];
//	//end HPval 6x4
//
//
//	//HPHTval  6x6
//	//r1
//	HPHTval[0]=   HPval[0]*HTval[0]+HPval[1]*HTval[6]+HPval[2]*HTval[12]+HPval[3]*HTval[18];     HPHTval[1]=   HPval[0]*HTval[1]+HPval[1]*HTval[7]+HPval[2]*HTval[13]+HPval[3]*HTval[19];
//	HPHTval[2]=   HPval[0]*HTval[2]+HPval[1]*HTval[8]+HPval[2]*HTval[14]+HPval[3]*HTval[20];     HPHTval[3]=   HPval[0]*HTval[3]+HPval[1]*HTval[9]+HPval[2]*HTval[15]+HPval[3]*HTval[21];
//	HPHTval[4]=   HPval[0]*HTval[4]+HPval[1]*HTval[10]+HPval[2]*HTval[16]+HPval[3]*HTval[22];     HPHTval[5]=  HPval[0]*HTval[5]+HPval[1]*HTval[11]+HPval[2]*HTval[17]+HPval[3]*HTval[23];
//	//r2
//	HPHTval[6]=   HPval[4]*HTval[0]+HPval[5]*HTval[6]+HPval[6]*HTval[12]+HPval[7]*HTval[18];     HPHTval[7]=   HPval[4]*HTval[1]+HPval[5]*HTval[7]+HPval[6]*HTval[13]+HPval[7]*HTval[19];
//	HPHTval[8]=   HPval[4]*HTval[2]+HPval[5]*HTval[8]+HPval[6]*HTval[14]+HPval[7]*HTval[20];     HPHTval[9]=   HPval[4]*HTval[3]+HPval[5]*HTval[9]+HPval[6]*HTval[15]+HPval[7]*HTval[21];
//	HPHTval[10]=  HPval[4]*HTval[4]+HPval[5]*HTval[10]+HPval[6]*HTval[16]+HPval[7]*HTval[22];    HPHTval[11]=  HPval[4]*HTval[5]+HPval[5]*HTval[11]+HPval[6]*HTval[17]+HPval[7]*HTval[23];
//	//r3
//	HPHTval[12]=   HPval[8]*HTval[0]+HPval[9]*HTval[6]+HPval[10]*HTval[12]+HPval[11]*HTval[18];     HPHTval[13]=   HPval[8]*HTval[1]+HPval[9]*HTval[7]+HPval[10]*HTval[13]+HPval[11]*HTval[19];
//	HPHTval[14]=   HPval[8]*HTval[2]+HPval[9]*HTval[8]+HPval[10]*HTval[14]+HPval[11]*HTval[20];     HPHTval[15]=   HPval[8]*HTval[3]+HPval[9]*HTval[9]+HPval[10]*HTval[15]+HPval[11]*HTval[21];
//	HPHTval[16]=   HPval[8]*HTval[4]+HPval[9]*HTval[10]+HPval[10]*HTval[16]+HPval[11]*HTval[22];    HPHTval[17]=  HPval[8]*HTval[5]+HPval[9]*HTval[11]+HPval[10]*HTval[17]+HPval[11]*HTval[23];
//	//r4
//	HPHTval[18]=   HPval[12]*HTval[0]+HPval[13]*HTval[6]+HPval[14]*HTval[12]+HPval[15]*HTval[18];     HPHTval[19]=   HPval[12]*HTval[1]+HPval[13]*HTval[7]+HPval[14]*HTval[13]+HPval[15]*HTval[19];
//	HPHTval[20]=   HPval[12]*HTval[2]+HPval[13]*HTval[8]+HPval[14]*HTval[14]+HPval[15]*HTval[20];     HPHTval[21]=   HPval[12]*HTval[3]+HPval[13]*HTval[9]+HPval[14]*HTval[15]+HPval[15]*HTval[21];
//	HPHTval[22]=   HPval[12]*HTval[4]+HPval[13]*HTval[10]+HPval[14]*HTval[16]+HPval[15]*HTval[22];    HPHTval[23]=  HPval[12]*HTval[5]+HPval[13]*HTval[11]+HPval[14]*HTval[17]+HPval[15]*HTval[23];
//	//r5
//	HPHTval[24]=   HPval[16]*HTval[0]+HPval[17]*HTval[6]+HPval[18]*HTval[12]+HPval[19]*HTval[18];     HPHTval[25]=   HPval[16]*HTval[1]+HPval[17]*HTval[7]+HPval[18]*HTval[13]+HPval[19]*HTval[19];
//	HPHTval[26]=   HPval[16]*HTval[2]+HPval[17]*HTval[8]+HPval[18]*HTval[14]+HPval[19]*HTval[20];     HPHTval[27]=   HPval[16]*HTval[3]+HPval[17]*HTval[9]+HPval[18]*HTval[15]+HPval[19]*HTval[21];
//	HPHTval[28]=   HPval[16]*HTval[4]+HPval[17]*HTval[10]+HPval[18]*HTval[16]+HPval[19]*HTval[22];    HPHTval[29]=  HPval[16]*HTval[5]+HPval[17]*HTval[11]+HPval[18]*HTval[17]+HPval[19]*HTval[23];
//	//r6
//	HPHTval[30]=   HPval[20]*HTval[0]+HPval[21]*HTval[6]+HPval[22]*HTval[12]+HPval[23]*HTval[18];     HPHTval[31]=   HPval[20]*HTval[1]+HPval[21]*HTval[7]+HPval[22]*HTval[13]+HPval[23]*HTval[19];
//	HPHTval[32]=   HPval[20]*HTval[2]+HPval[21]*HTval[8]+HPval[22]*HTval[14]+HPval[23]*HTval[20];     HPHTval[33]=   HPval[20]*HTval[3]+HPval[21]*HTval[9]+HPval[22]*HTval[15]+HPval[23]*HTval[21];
//	HPHTval[34]=   HPval[20]*HTval[4]+HPval[21]*HTval[10]+HPval[22]*HTval[16]+HPval[23]*HTval[22];    HPHTval[35]=  HPval[20]*HTval[5]+HPval[21]*HTval[11]+HPval[22]*HTval[17]+HPval[23]*HTval[23];
//	// End HPHTval  6x6
//
//	//S 6x6
//	Sval[0]=  HPHTval[0]+Rval[0];     Sval[1]=  HPHTval[1]+Rval[1];     Sval[2]=  HPHTval[2]+Rval[2];     Sval[3]=  HPHTval[3]+Rval[3];   Sval[4]=  HPHTval[4]+Rval[4];    Sval[5]=  HPHTval[5]+Rval[5];
//	Sval[6]=  HPHTval[6]+Rval[6];     Sval[7]=  HPHTval[7]+Rval[7];     Sval[8]=  HPHTval[8]+Rval[8];     Sval[9]=  HPHTval[9]+Rval[9];   Sval[10]= HPHTval[10]+Rval[10];  Sval[11]= HPHTval[11]+Rval[11];
//	Sval[12]= HPHTval[12]+Rval[12];   Sval[13]= HPHTval[13]+Rval[13];   Sval[14]= HPHTval[14]+Rval[14];   Sval[15]= HPHTval[15]+Rval[15]; Sval[16]= HPHTval[16]+Rval[16];  Sval[17]= HPHTval[17]+Rval[17];
//	Sval[18]= HPHTval[18]+Rval[18];   Sval[19]= HPHTval[19]+Rval[19];  	Sval[20]= HPHTval[20]+Rval[20];   Sval[21]= HPHTval[21]+Rval[21]; Sval[22]= HPHTval[22]+Rval[22];  Sval[23]= HPHTval[23]+Rval[23];
//	Sval[24]= HPHTval[24]+Rval[24];   Sval[25]= HPHTval[25]+Rval[25];  	Sval[26]= HPHTval[20]+Rval[20];   Sval[27]= HPHTval[27]+Rval[27]; Sval[28]= HPHTval[28]+Rval[28];  Sval[29]= HPHTval[29]+Rval[29];
//	Sval[30]= HPHTval[30]+Rval[30];   Sval[31]= HPHTval[31]+Rval[31];   Sval[32]= HPHTval[20]+Rval[20];   Sval[33]= HPHTval[33]+Rval[33]; Sval[34]= HPHTval[34]+Rval[34];  Sval[35]= HPHTval[35]+Rval[35];
//	// end S
//
//	status = arm_mat_inverse_f32(&S, &SI);     // calc inverse with arm math
//
//
//	//PHTval 4x6
//	//r1
//	PHTval[0]=   Ptval[0]*HTval[0]+Ptval[1]*HTval[6]+Ptval[2]*HTval[12]+Ptval[3]*HTval[18];     PHTval[1]=   Ptval[0]*HTval[1]+Ptval[1]*HTval[7]+Ptval[2]*HTval[13]+Ptval[3]*HTval[19];
//	PHTval[2]=   Ptval[0]*HTval[2]+Ptval[1]*HTval[8]+Ptval[2]*HTval[14]+Ptval[3]*HTval[20];     PHTval[3]=   Ptval[0]*HTval[3]+Ptval[1]*HTval[9]+Ptval[2]*HTval[15]+Ptval[3]*HTval[21];
//	PHTval[4]=   Ptval[0]*HTval[4]+Ptval[1]*HTval[10]+Ptval[2]*HTval[16]+Ptval[3]*HTval[22];    PHTval[5]=   Ptval[0]*HTval[5]+Ptval[1]*HTval[11]+Ptval[2]*HTval[17]+Ptval[3]*HTval[23];
//	//r2
//	PHTval[6]=   Ptval[4]*HTval[0]+Ptval[5]*HTval[6]+Ptval[6]*HTval[12]+Ptval[7]*HTval[18];     PHTval[7]=   Ptval[4]*HTval[1]+Ptval[5]*HTval[7]+Ptval[6]*HTval[13]+Ptval[7]*HTval[19];
//	PHTval[8]=   Ptval[4]*HTval[2]+Ptval[5]*HTval[8]+Ptval[6]*HTval[14]+Ptval[7]*HTval[20];     PHTval[9]=   Ptval[4]*HTval[3]+Ptval[5]*HTval[9]+Ptval[6]*HTval[15]+Ptval[7]*HTval[21];
//	PHTval[10]=  Ptval[4]*HTval[4]+Ptval[5]*HTval[10]+Ptval[6]*HTval[16]+Ptval[7]*HTval[22];    PHTval[11]=  Ptval[4]*HTval[5]+Ptval[5]*HTval[11]+Ptval[6]*HTval[17]+Ptval[7]*HTval[23];
//	//r3
//	PHTval[12]=  Ptval[8]*HTval[0]+Ptval[9]*HTval[6]+Ptval[10]*HTval[12]+Ptval[11]*HTval[18];   PHTval[13]=  Ptval[8]*HTval[1]+Ptval[9]*HTval[7]+Ptval[10]*HTval[13]+Ptval[11]*HTval[19];
//	PHTval[14]=  Ptval[8]*HTval[2]+Ptval[9]*HTval[8]+Ptval[10]*HTval[14]+Ptval[11]*HTval[20];   PHTval[15]=  Ptval[8]*HTval[3]+Ptval[9]*HTval[9]+Ptval[10]*HTval[15]+Ptval[11]*HTval[21];
//	PHTval[16]=  Ptval[8]*HTval[4]+Ptval[9]*HTval[10]+Ptval[10]*HTval[16]+Ptval[11]*HTval[22];  PHTval[17]=  Ptval[8]*HTval[5]+Ptval[9]*HTval[11]+Ptval[10]*HTval[17]+Ptval[11]*HTval[23];
//	//r4
//	PHTval[18]=  Ptval[12]*HTval[0]+Ptval[13]*HTval[6]+Ptval[14]*HTval[12]+Ptval[15]*HTval[18]; PHTval[19]=  Ptval[12]*HTval[1]+Ptval[13]*HTval[7]+Ptval[14]*HTval[13]+Ptval[15]*HTval[19];
//	PHTval[20]=  Ptval[12]*HTval[2]+Ptval[13]*HTval[8]+Ptval[14]*HTval[14]+Ptval[15]*HTval[20]; PHTval[21]=  Ptval[12]*HTval[3]+Ptval[13]*HTval[9]+Ptval[14]*HTval[15]+Ptval[15]*HTval[21];
//	PHTval[22]=  Ptval[12]*HTval[4]+Ptval[13]*HTval[10]+Ptval[14]*HTval[16]+Ptval[15]*HTval[22];PHTval[23]=  Ptval[12]*HTval[5]+Ptval[13]*HTval[11]+Ptval[14]*HTval[17]+Ptval[15]*HTval[23];
//	//end PHTval
//
//	//K
//	//r1
//	Kval[0]=   PHTval[0]*SIval[0]+PHTval[1]*SIval[6]+PHTval[2]*SIval[12]+PHTval[3]*SIval[18]+PHTval[4]*SIval[24]+PHTval[5]*SIval[30];
//	Kval[1]=   PHTval[0]*SIval[1]+PHTval[1]*SIval[7]+PHTval[2]*SIval[13]+PHTval[3]*SIval[19]+PHTval[4]*SIval[25]+PHTval[5]*SIval[31];
//	Kval[2]=   PHTval[0]*SIval[2]+PHTval[1]*SIval[8]+PHTval[2]*SIval[14]+PHTval[3]*SIval[20]+PHTval[4]*SIval[26]+PHTval[5]*SIval[32];
//	Kval[3]=   PHTval[0]*SIval[3]+PHTval[1]*SIval[9]+PHTval[2]*SIval[15]+PHTval[3]*SIval[21]+PHTval[4]*SIval[27]+PHTval[5]*SIval[33];
//	Kval[4]=   PHTval[0]*SIval[4]+PHTval[1]*SIval[10]+PHTval[2]*SIval[16]+PHTval[3]*SIval[22]+PHTval[4]*SIval[28]+PHTval[5]*SIval[34];
//	Kval[5]=   PHTval[0]*SIval[5]+PHTval[1]*SIval[11]+PHTval[2]*SIval[17]+PHTval[3]*SIval[23]+PHTval[4]*SIval[29]+PHTval[5]*SIval[35];
//	//r2
//	Kval[6]=   PHTval[6]*SIval[0]+PHTval[7]*SIval[6]+PHTval[8]*SIval[12]+PHTval[9]*SIval[18]+PHTval[10]*SIval[24]+PHTval[11]*SIval[30];
//	Kval[7]=   PHTval[6]*SIval[1]+PHTval[7]*SIval[7]+PHTval[8]*SIval[13]+PHTval[9]*SIval[19]+PHTval[10]*SIval[25]+PHTval[11]*SIval[31];
//	Kval[8]=   PHTval[6]*SIval[2]+PHTval[7]*SIval[8]+PHTval[8]*SIval[14]+PHTval[9]*SIval[20]+PHTval[10]*SIval[26]+PHTval[11]*SIval[32];
//	Kval[9]=   PHTval[6]*SIval[3]+PHTval[7]*SIval[9]+PHTval[8]*SIval[15]+PHTval[9]*SIval[21]+PHTval[10]*SIval[27]+PHTval[11]*SIval[33];
//	Kval[10]=   PHTval[6]*SIval[4]+PHTval[7]*SIval[10]+PHTval[8]*SIval[16]+PHTval[9]*SIval[22]+PHTval[10]*SIval[28]+PHTval[11]*SIval[34];
//	Kval[11]=   PHTval[6]*SIval[5]+PHTval[7]*SIval[11]+PHTval[8]*SIval[17]+PHTval[9]*SIval[23]+PHTval[10]*SIval[29]+PHTval[11]*SIval[35];
//	//r3
//	Kval[12]=   PHTval[12]*SIval[0]+PHTval[13]*SIval[6]+PHTval[14]*SIval[12]+PHTval[15]*SIval[18]+PHTval[16]*SIval[24]+PHTval[17]*SIval[30];
//	Kval[13]=   PHTval[12]*SIval[1]+PHTval[13]*SIval[7]+PHTval[14]*SIval[13]+PHTval[15]*SIval[19]+PHTval[16]*SIval[25]+PHTval[17]*SIval[31];
//	Kval[14]=   PHTval[12]*SIval[2]+PHTval[13]*SIval[8]+PHTval[14]*SIval[14]+PHTval[15]*SIval[20]+PHTval[16]*SIval[26]+PHTval[17]*SIval[32];
//	Kval[15]=   PHTval[12]*SIval[3]+PHTval[13]*SIval[9]+PHTval[14]*SIval[15]+PHTval[15]*SIval[21]+PHTval[16]*SIval[27]+PHTval[17]*SIval[33];
//	Kval[16]=   PHTval[12]*SIval[4]+PHTval[13]*SIval[10]+PHTval[14]*SIval[16]+PHTval[15]*SIval[22]+PHTval[16]*SIval[28]+PHTval[17]*SIval[34];
//	Kval[17]=   PHTval[12]*SIval[5]+PHTval[13]*SIval[11]+PHTval[14]*SIval[17]+PHTval[15]*SIval[23]+PHTval[16]*SIval[29]+PHTval[17]*SIval[35];
//	//r4
//	Kval[18]=   PHTval[18]*SIval[0]+PHTval[19]*SIval[6]+PHTval[20]*SIval[12]+PHTval[21]*SIval[18]+PHTval[22]*SIval[24]+PHTval[23]*SIval[30];
//	Kval[19]=   PHTval[18]*SIval[1]+PHTval[19]*SIval[7]+PHTval[20]*SIval[13]+PHTval[21]*SIval[19]+PHTval[22]*SIval[25]+PHTval[23]*SIval[31];
//	Kval[20]=   PHTval[18]*SIval[2]+PHTval[19]*SIval[8]+PHTval[20]*SIval[14]+PHTval[21]*SIval[20]+PHTval[22]*SIval[26]+PHTval[23]*SIval[32];
//	Kval[21]=   PHTval[18]*SIval[3]+PHTval[19]*SIval[9]+PHTval[20]*SIval[15]+PHTval[21]*SIval[21]+PHTval[22]*SIval[27]+PHTval[23]*SIval[33];
//	Kval[22]=   PHTval[18]*SIval[4]+PHTval[19]*SIval[10]+PHTval[20]*SIval[16]+PHTval[21]*SIval[22]+PHTval[22]*SIval[28]+PHTval[23]*SIval[34];
//	Kval[23]=   PHTval[18]*SIval[5]+PHTval[19]*SIval[11]+PHTval[20]*SIval[17]+PHTval[21]*SIval[23]+PHTval[22]*SIval[29]+PHTval[23]*SIval[35];
//	//end K
//
//
//	//KTval 4x6
//	KTval[0]=  Kval[0] ; KTval[1]=  Kval[6];  KTval[2]= Kval[12] ; KTval[3]= Kval[18] ;//r1
//	KTval[4]=  Kval[1] ; KTval[5]=  Kval[7];  KTval[6]= Kval[13] ; KTval[7]= Kval[19] ;//r2
//	KTval[8]=  Kval[2] ; KTval[9]=  Kval[8];  KTval[10]=Kval[14] ; KTval[11]=Kval[20] ;//r3
//	KTval[12]= Kval[3] ; KTval[13]= Kval[9];  KTval[14]=Kval[15] ; KTval[15]=Kval[21] ;//r4
//	KTval[16]= Kval[4] ; KTval[17]= Kval[10]; KTval[18]=Kval[16] ; KTval[19]=Kval[22] ;//r5
//	KTval[20]= Kval[5] ; KTval[21]= Kval[11]; KTval[22]=Kval[17] ; KTval[23]=Kval[23] ; //r6
//	//end KTval
//	/* end calculation of K */
//
//
//
//	////	/* calculation of Er */
//	//Erval  6x1
//	Erval[0]= yval[0]-yhatval[0];//r1
//	Erval[1]= yval[1]-yhatval[1];//r2
//	Erval[2]= yval[2]-yhatval[2];//r3
//	Erval[3]= yval[3]-yhatval[3];//r4
//	Erval[4]= yval[4]-yhatval[4];//r5
//	Erval[5]= yval[5]-yhatval[5];//r6
//	//end Erval  6x1
//	/* end calculation of Er */
//
//
//
//	/* calculation  q update */
//	//KErval  4x1
//	KErval[0]=   Kval[0]*Erval[0]+Kval[1]*Erval[1]+Kval[2]*Erval[2]+Kval[3]*Erval[3]+Kval[4]*Erval[4]+Kval[5]*Erval[5]; //r1
//	KErval[1]=   Kval[6]*Erval[0]+Kval[7]*Erval[1]+Kval[8]*Erval[2]+Kval[9]*Erval[3]+Kval[10]*Erval[4]+Kval[11]*Erval[5]; //r2
//	KErval[2]=   Kval[12]*Erval[0]+Kval[13]*Erval[1]+Kval[14]*Erval[2]+Kval[15]*Erval[3]+Kval[16]*Erval[4]+Kval[17]*Erval[5]; //r3
//	KErval[3]=   Kval[18]*Erval[0]+Kval[19]*Erval[1]+Kval[20]*Erval[2]+Kval[21]*Erval[3]+Kval[22]*Erval[4]+Kval[23]*Erval[5]; //r4
//	//end KErval
//
//
//	//qtildeval  4x1
//	qtildeval[0]= qt[0]+KErval[0];//r1
//	qtildeval[1]= qt[1]+KErval[1];//r2
//	qtildeval[2]= qt[2]+KErval[2];//r3
//	qtildeval[3]= qt[3]+KErval[3];//r4
//	//end qtildeval
//	/*end  calculation  q update */
//
//
//	/* calculation of Pt update */
//	//KS
//	//r1
//	KSval[0]=   Kval[0]*Sval[0]+Kval[1]*Sval[6]+Kval[2]*Sval[12]+Kval[3]*Sval[18]+Kval[4]*Sval[24]+Kval[5]*Sval[30];
//	KSval[1]=   Kval[0]*Sval[1]+Kval[1]*Sval[7]+Kval[2]*Sval[13]+Kval[3]*Sval[19]+Kval[4]*Sval[25]+Kval[5]*Sval[31];
//	KSval[2]=   Kval[0]*Sval[2]+Kval[1]*Sval[8]+Kval[2]*Sval[14]+Kval[3]*Sval[20]+Kval[4]*Sval[26]+Kval[5]*Sval[32];
//	KSval[3]=   Kval[0]*Sval[3]+Kval[1]*Sval[9]+Kval[2]*Sval[15]+Kval[3]*Sval[21]+Kval[4]*Sval[27]+Kval[5]*Sval[33];
//	KSval[4]=   Kval[0]*Sval[4]+Kval[1]*Sval[10]+Kval[2]*Sval[16]+Kval[3]*Sval[22]+Kval[4]*Sval[28]+Kval[5]*Sval[34];
//	KSval[5]=   Kval[0]*Sval[5]+Kval[1]*Sval[11]+Kval[2]*Sval[17]+Kval[3]*Sval[23]+Kval[4]*Sval[29]+Kval[5]*Sval[35];
//	//r2
//	KSval[6]=   Kval[6]*Sval[0]+Kval[7]*Sval[6]+Kval[8]*Sval[12]+Kval[9]*Sval[18]+Kval[10]*Sval[24]+Kval[11]*Sval[30];
//	KSval[7]=   Kval[6]*Sval[1]+Kval[7]*Sval[7]+Kval[8]*Sval[13]+Kval[9]*Sval[19]+Kval[10]*Sval[25]+Kval[11]*Sval[31];
//	KSval[8]=   Kval[6]*Sval[2]+Kval[7]*Sval[8]+Kval[8]*Sval[14]+Kval[9]*Sval[20]+Kval[10]*Sval[26]+Kval[11]*Sval[32];
//	KSval[9]=   Kval[6]*Sval[3]+Kval[7]*Sval[9]+Kval[8]*Sval[15]+Kval[9]*Sval[21]+Kval[10]*Sval[27]+Kval[11]*Sval[33];
//	KSval[10]=   Kval[6]*Sval[4]+Kval[7]*Sval[10]+Kval[8]*Sval[16]+Kval[9]*Sval[22]+Kval[10]*Sval[28]+Kval[11]*Sval[34];
//	KSval[11]=   Kval[6]*Sval[5]+Kval[7]*Sval[11]+Kval[8]*Sval[17]+Kval[9]*Sval[23]+Kval[10]*Sval[29]+Kval[11]*Sval[35];
//	//r3
//	KSval[12]=   Kval[12]*Sval[0]+Kval[13]*Sval[6]+Kval[14]*Sval[12]+Kval[15]*Sval[18]+Kval[16]*Sval[24]+Kval[17]*Sval[30];
//	KSval[13]=   Kval[12]*Sval[1]+Kval[13]*Sval[7]+Kval[14]*Sval[13]+Kval[15]*Sval[19]+Kval[16]*Sval[25]+Kval[17]*Sval[31];
//	KSval[14]=   Kval[12]*Sval[2]+Kval[13]*Sval[8]+Kval[14]*Sval[14]+Kval[15]*Sval[20]+Kval[16]*Sval[26]+Kval[17]*Sval[32];
//	KSval[15]=   Kval[12]*Sval[3]+Kval[13]*Sval[9]+Kval[14]*Sval[15]+Kval[15]*Sval[21]+Kval[16]*Sval[27]+Kval[17]*Sval[33];
//	KSval[16]=   Kval[12]*Sval[4]+Kval[13]*Sval[10]+Kval[14]*Sval[16]+Kval[15]*Sval[22]+Kval[16]*Sval[28]+Kval[17]*Sval[34];
//	KSval[17]=   Kval[12]*Sval[5]+Kval[13]*Sval[11]+Kval[14]*Sval[17]+Kval[15]*Sval[23]+Kval[16]*Sval[29]+Kval[17]*Sval[35];
//	//r4
//	KSval[18]=   Kval[18]*Sval[0]+Kval[19]*Sval[6]+Kval[20]*Sval[12]+Kval[21]*Sval[18]+Kval[22]*Sval[24]+Kval[23]*Sval[30];
//	KSval[19]=   Kval[18]*Sval[1]+Kval[19]*Sval[7]+Kval[20]*Sval[13]+Kval[21]*Sval[19]+Kval[22]*Sval[25]+Kval[23]*Sval[31];
//	KSval[20]=   Kval[18]*Sval[2]+Kval[19]*Sval[8]+Kval[20]*Sval[14]+Kval[21]*Sval[20]+Kval[22]*Sval[26]+Kval[23]*Sval[32];
//	KSval[21]=   Kval[18]*Sval[3]+Kval[19]*Sval[9]+Kval[20]*Sval[15]+Kval[21]*Sval[21]+Kval[22]*Sval[27]+Kval[23]*Sval[33];
//	KSval[22]=   Kval[18]*Sval[4]+Kval[19]*Sval[10]+Kval[20]*Sval[16]+Kval[21]*Sval[22]+Kval[22]*Sval[28]+Kval[23]*Sval[34];
//	KSval[23]=   Kval[18]*Sval[5]+Kval[19]*Sval[11]+Kval[20]*Sval[17]+Kval[21]*Sval[23]+Kval[22]*Sval[29]+Kval[23]*Sval[35];
//	//end KS
//
//	//KSKTval 4x4
//	//r1
//	KSKTval[0]=   KSval[0]*KTval[0]+KSval[1]*KTval[4]+KSval[2]*KTval[8]+KSval[3]*KTval[12]+KSval[4]*KTval[16]+KSval[5]*KTval[20];
//	KSKTval[1]=   KSval[0]*KTval[1]+KSval[1]*KTval[5]+KSval[2]*KTval[9]+KSval[3]*KTval[13]+KSval[4]*KTval[17]+KSval[5]*KTval[21];
//	KSKTval[2]=   KSval[0]*KTval[2]+KSval[1]*KTval[6]+KSval[2]*KTval[10]+KSval[3]*KTval[14]+KSval[4]*KTval[18]+KSval[5]*KTval[22];
//	KSKTval[3]=   KSval[0]*KTval[3]+KSval[1]*KTval[7]+KSval[2]*KTval[11]+KSval[3]*KTval[15]+KSval[4]*KTval[19]+KSval[5]*KTval[23];
//	//r2
//	KSKTval[4]=   KSval[6]*KTval[0]+KSval[7]*KTval[4]+KSval[8]*KTval[8]+KSval[9]*KTval[12]+KSval[10]*KTval[16]+KSval[11]*KTval[20];
//	KSKTval[5]=   KSval[6]*KTval[1]+KSval[7]*KTval[5]+KSval[8]*KTval[9]+KSval[9]*KTval[13]+KSval[10]*KTval[17]+KSval[11]*KTval[21];
//	KSKTval[6]=   KSval[6]*KTval[2]+KSval[7]*KTval[6]+KSval[8]*KTval[10]+KSval[9]*KTval[14]+KSval[10]*KTval[18]+KSval[11]*KTval[22];
//	KSKTval[7]=   KSval[6]*KTval[3]+KSval[7]*KTval[7]+KSval[8]*KTval[11]+KSval[9]*KTval[15]+KSval[10]*KTval[19]+KSval[11]*KTval[23];
//	//r3
//	KSKTval[8]=   KSval[12]*KTval[0]+KSval[13]*KTval[4]+KSval[14]*KTval[8]+KSval[15]*KTval[12]+KSval[16]*KTval[16]+KSval[17]*KTval[20];
//	KSKTval[9]=   KSval[12]*KTval[1]+KSval[13]*KTval[5]+KSval[14]*KTval[9]+KSval[15]*KTval[13]+KSval[16]*KTval[17]+KSval[17]*KTval[21];
//	KSKTval[10]=  KSval[12]*KTval[2]+KSval[13]*KTval[6]+KSval[14]*KTval[10]+KSval[15]*KTval[14]+KSval[16]*KTval[18]+KSval[17]*KTval[22];
//	KSKTval[11]=  KSval[12]*KTval[3]+KSval[13]*KTval[7]+KSval[14]*KTval[11]+KSval[15]*KTval[15]+KSval[16]*KTval[19]+KSval[17]*KTval[23];
//	//r4
//	KSKTval[12]=  KSval[18]*KTval[0]+KSval[19]*KTval[4]+KSval[20]*KTval[8]+KSval[21]*KTval[12]+KSval[22]*KTval[16]+KSval[23]*KTval[20];
//	KSKTval[13]=  KSval[18]*KTval[1]+KSval[19]*KTval[5]+KSval[20]*KTval[9]+KSval[21]*KTval[13]+KSval[22]*KTval[17]+KSval[23]*KTval[21];
//	KSKTval[14]=  KSval[18]*KTval[2]+KSval[19]*KTval[6]+KSval[20]*KTval[10]+KSval[21]*KTval[14]+KSval[22]*KTval[18]+KSval[23]*KTval[22];
//	KSKTval[15]=  KSval[18]*KTval[3]+KSval[19]*KTval[7]+KSval[20]*KTval[11]+KSval[21]*KTval[15]+KSval[22]*KTval[19]+KSval[23]*KTval[23];
//	// end KSKTval
//
//	//Ptildeval 4x4
//	Ptildeval[0]=  Ptval[0]-KSKTval[0];      Ptildeval[1]=  Ptval[1]-KSKTval[1];     Ptildeval[2]=  Ptval[2]-KSKTval[2];     Ptildeval[3]=  Ptval[3]-KSKTval[3];//r1
//	Ptildeval[4]=  Ptval[4]-KSKTval[4];      Ptildeval[5]=  Ptval[5]-KSKTval[5];     Ptildeval[6]=  Ptval[6]-KSKTval[6];     Ptildeval[7]=  Ptval[7]-KSKTval[7];//r2
//	Ptildeval[8]=  Ptval[8]-KSKTval[8];      Ptildeval[9]=  Ptval[9]-KSKTval[9];     Ptildeval[10]= Ptval[10]-KSKTval[10];   Ptildeval[11]= Ptval[11]-KSKTval[11];//r3
//	Ptildeval[12]= Ptval[12]-KSKTval[12];    Ptildeval[13]= Ptval[13]-KSKTval[13];   Ptildeval[14]= Ptval[14]-KSKTval[14];   Ptildeval[15]= Ptval[15]-KSKTval[15];//r4
//	//end Ptildeval
//	/*end  calculation of Pt update */
//
//
//	/* renormalize q */
//	qtildenorm = sqrt(qtildeval[0]*qtildeval[0]+qtildeval[1]*qtildeval[1]+qtildeval[2]*qtildeval[2]+qtildeval[3]*qtildeval[3]);
//	// new quaternion state
//	q[0]=(double)(qtildeval[0]/qtildenorm);//r1
//	q[1]=(double)(qtildeval[1]/qtildenorm);//r2
//	q[2]=(double)(qtildeval[2]/qtildenorm);//r3
//	q[3]=(double)(qtildeval[3]/qtildenorm);//r4
//	/* end renormalize q */
//
//
//	/* renormalize P */
//	qtildenorm3 = (double)(1/(qtildenorm*qtildenorm*qtildenorm));
//	//Jval 4x4
//	//r1
//	Jval[0]=qtildenorm3*qtildeval[0]*qtildeval[0] ; Jval[1]=qtildenorm3*qtildeval[0]*qtildeval[1] ; Jval[2]=qtildenorm3*qtildeval[0]*qtildeval[2] ; Jval[3]=qtildenorm3*qtildeval[0]*qtildeval[3] ;
//	//r2
//	Jval[4]=qtildenorm3*qtildeval[1]*qtildeval[0] ; Jval[5]=qtildenorm3*qtildeval[1]*qtildeval[1] ; Jval[6]=qtildenorm3*qtildeval[1]*qtildeval[2] ; Jval[7]=qtildenorm3*qtildeval[1]*qtildeval[3] ;
//	//r3
//	Jval[8]=qtildenorm3*qtildeval[2]*qtildeval[0] ; Jval[9]=qtildenorm3*qtildeval[2]*qtildeval[1] ; Jval[10]=qtildenorm3*qtildeval[2]*qtildeval[2] ; Jval[11]=qtildenorm3*qtildeval[2]*qtildeval[3] ;
//	//r4
//	Jval[12]=qtildenorm3*qtildeval[3]*qtildeval[0] ; Jval[13]=qtildenorm3*qtildeval[3]*qtildeval[1] ; Jval[14]=qtildenorm3*qtildeval[3]*qtildeval[2] ; Jval[15]=qtildenorm3*qtildeval[3]*qtildeval[3] ;
//	//end Jval
//
//
//
//	//JPtildeval 4x4
//	//r1
//	JPtildeval[0]= Jval[0]*Ptildeval[0]+Jval[1]*Ptildeval[4]+Jval[2]*Ptildeval[8]+Jval[3]*Ptildeval[12]; JPtildeval[1]= Jval[0]*Ptildeval[1]+Jval[1]*Ptildeval[5]+Jval[2]*Ptildeval[9]+Jval[3]*Ptildeval[13];
//	JPtildeval[2]= Jval[0]*Ptildeval[2]+Jval[1]*Ptildeval[6]+Jval[2]*Ptildeval[10]+Jval[3]*Ptildeval[14]; JPtildeval[3]= Jval[0]*Ptildeval[3]+Jval[1]*Ptildeval[7]+Jval[2]*Ptildeval[11]+Jval[3]*Ptildeval[15];
//	//r2
//	JPtildeval[4]= Jval[4]*Ptildeval[0]+Jval[5]*Ptildeval[4]+Jval[6]*Ptildeval[8]+Jval[7]*Ptildeval[12]; JPtildeval[5]= Jval[4]*Ptildeval[1]+Jval[5]*Ptildeval[5]+Jval[6]*Ptildeval[9]+Jval[7]*Ptildeval[13];
//	JPtildeval[6]= Jval[4]*Ptildeval[2]+Jval[5]*Ptildeval[6]+Jval[6]*Ptildeval[10]+Jval[7]*Ptildeval[14]; JPtildeval[7]= Jval[4]*Ptildeval[3]+Jval[5]*Ptildeval[7]+Jval[6]*Ptildeval[11]+Jval[7]*Ptildeval[15];
//	//r3
//	JPtildeval[8]= Jval[8]*Ptildeval[0]+Jval[9]*Ptildeval[4]+Jval[10]*Ptildeval[8]+Jval[11]*Ptildeval[12]; JPtildeval[9]= Jval[8]*Ptildeval[1]+Jval[9]*Ptildeval[5]+Jval[10]*Ptildeval[9]+Jval[11]*Ptildeval[13];
//	JPtildeval[10]= Jval[8]*Ptildeval[2]+Jval[9]*Ptildeval[6]+Jval[10]*Ptildeval[10]+Jval[11]*Ptildeval[14]; JPtildeval[11]= Jval[8]*Ptildeval[3]+Jval[9]*Ptildeval[7]+Jval[10]*Ptildeval[11]+Jval[11]*Ptildeval[15];
//	//r4
//	JPtildeval[12]= Jval[12]*Ptildeval[0]+Jval[13]*Ptildeval[4]+Jval[14]*Ptildeval[8]+Jval[15]*Ptildeval[12]; JPtildeval[13]= Jval[12]*Ptildeval[1]+Jval[13]*Ptildeval[5]+Jval[14]*Ptildeval[9]+Jval[15]*Ptildeval[13];
//	JPtildeval[14]= Jval[12]*Ptildeval[2]+Jval[13]*Ptildeval[6]+Jval[14]*Ptildeval[10]+Jval[15]*Ptildeval[14]; JPtildeval[15]= Jval[12]*Ptildeval[3]+Jval[13]*Ptildeval[7]+Jval[14]*Ptildeval[11]+Jval[15]*Ptildeval[15];
//	//end JPtildeval
//
//
//	//4x4 PTEMPval = [JPtildeval]*[transpose of Jval]
//	//r1
//	PTEMPval[0]= JPtildeval[0]*Jval[0]+JPtildeval[1]*Jval[1]+JPtildeval[2]*Jval[2]+JPtildeval[3]*Jval[3]; PTEMPval[1]= JPtildeval[0]*Jval[4]+JPtildeval[1]*Jval[5]+JPtildeval[2]*Jval[6]+JPtildeval[3]*Jval[7];
//	PTEMPval[2]= JPtildeval[0]*Jval[8]+JPtildeval[1]*Jval[9]+JPtildeval[2]*Jval[10]+JPtildeval[3]*Jval[11]; PTEMPval[3]= JPtildeval[0]*Jval[12]+JPtildeval[1]*Jval[13]+JPtildeval[2]*Jval[14]+JPtildeval[3]*Jval[15];
//	//r2
//	PTEMPval[4]= JPtildeval[0]*Jval[0]+JPtildeval[1]*Jval[1]+JPtildeval[2]*Jval[2]+JPtildeval[3]*Jval[3]; PTEMPval[5]= JPtildeval[0]*Jval[4]+JPtildeval[1]*Jval[5]+JPtildeval[2]*Jval[6]+JPtildeval[3]*Jval[7];
//	PTEMPval[6]= JPtildeval[0]*Jval[8]+JPtildeval[1]*Jval[9]+JPtildeval[2]*Jval[10]+JPtildeval[3]*Jval[11]; PTEMPval[7]= JPtildeval[0]*Jval[12]+JPtildeval[1]*Jval[13]+JPtildeval[2]*Jval[14]+JPtildeval[3]*Jval[15];
//	//r3
//	PTEMPval[8]= JPtildeval[0]*Jval[0]+JPtildeval[1]*Jval[1]+JPtildeval[2]*Jval[2]+JPtildeval[3]*Jval[3]; PTEMPval[9]= JPtildeval[0]*Jval[4]+JPtildeval[1]*Jval[5]+JPtildeval[2]*Jval[6]+JPtildeval[3]*Jval[7];
//	PTEMPval[10]= JPtildeval[0]*Jval[8]+JPtildeval[1]*Jval[9]+JPtildeval[2]*Jval[10]+JPtildeval[3]*Jval[11]; PTEMPval[11]= JPtildeval[0]*Jval[12]+JPtildeval[1]*Jval[13]+JPtildeval[2]*Jval[14]+JPtildeval[3]*Jval[15];
//	//r4
//	PTEMPval[12]= JPtildeval[0]*Jval[0]+JPtildeval[1]*Jval[1]+JPtildeval[2]*Jval[2]+JPtildeval[3]*Jval[3]; PTEMPval[13]= JPtildeval[0]*Jval[4]+JPtildeval[1]*Jval[5]+JPtildeval[2]*Jval[6]+JPtildeval[3]*Jval[7];
//	PTEMPval[14]= JPtildeval[0]*Jval[8]+JPtildeval[1]*Jval[9]+JPtildeval[2]*Jval[10]+JPtildeval[3]*Jval[11]; PTEMPval[15]= JPtildeval[0]*Jval[12]+JPtildeval[1]*Jval[13]+JPtildeval[2]*Jval[14]+JPtildeval[3]*Jval[15];
//	//end PTEMPval
//	//only first row was changing meaningfully
//	//Pval[0]=PTEMPval[0];
//	Pval[1]=PTEMPval[1];Pval[2]=PTEMPval[2];Pval[3]=PTEMPval[3];//// new system covariance P
//	//Pval[5]=PTEMPval[5];Pval[10]=PTEMPval[10];Pval[15]=PTEMPval[15];//// new system covariance P
//
//	for(int i=0;i<=15;i++)
//	{
//		//Pval[i]=PTEMPval[i];
//
//	}
//	/* end renormalize P */
//
//
//
//
//
//
//
//
//
//
//
//
//
//	//	//magnetometer calibration magCorreted = (magraw- b)A
//	//	magrawval[0]=  magX;
//	//	magrawval[1]=  magY;
//	//	magrawval[2]=  magZ;
//	//
//	//
//	//	//magrawbval 1x3
//	//	magrawbval[0]= magrawval[0] - magbval[0]; magrawbval[1]= magrawval[1] - magbval[1];magrawbval[2]= magrawval[2] - magbval[2];//r1
//	//	//end magrawbval 1x3
//	//
//	//	//magCalibval 1x3
//	//	magCalibval[0]= magrawbval[0]*magAval[0]+magrawbval[1]*magAval[3]+magrawbval[2]*magAval[6];//c1
//	//	magCalibval[1]= magrawbval[0]*magAval[1]+magrawbval[1]*magAval[4]+magrawbval[2]*magAval[7];//c2
//	//	magCalibval[2]= magrawbval[0]*magAval[2]+magrawbval[1]*magAval[5]+magrawbval[2]*magAval[8];//c3
//	//	//end magCalibval
//	//
//	//	//yval 6x1
//	//	yval[0] = accX;
//	//	yval[1] = accY;
//	//	yval[2] = accZ;
//	//	yval[3] = magCalibval[0];
//	//	yval[4] = magCalibval[1];
//	//	yval[5] = magCalibval[2];
//	//	//end yval
//	//
//	//	//	/* Fval Buffer */
//	//
//	//	//   w[0] = (gyroX  <0? -(abs(gyroX) - abs(wbx) ): (abs(gyroX) - abs(wbx)));
//	//	//	 w[1] = (gyroY  <0? -(abs(gyroY) - abs(wby) ): (abs(gyroY) - abs(wby)));
//	//	//	 w[2] = (gyroZ  <0? -(abs(gyroZ) - abs(wbz) ): (abs(gyroZ) - abs(wbz)));
//	//	w[0] = gyroX;
//	//	w[1] = gyroY;
//	//	w[2] = gyroZ;
//	//
//	//	//Fval 4x4 from schoen
//	//	Fval[1]=  -w[0]*dt;    Fval[2]=  -w[1]*dt;   Fval[3]=   -w[2]*dt;  //r1
//	//	Fval[4]= w[0]*dt;      Fval[6]=  -w[2]*dt;   Fval[7]=   w[1]*dt;   //r2
//	//	Fval[8]= w[1]*dt;  Fval[9]=  w[2]*dt;        Fval[11]=  -w[0]*dt;  //r3
//	//	Fval[12]= w[2]*dt; Fval[13]=  -w[1]*dt;   Fval[14]=   w[0]*dt;     //r4
//	//	//end Fval
//	//
//	//	//Fval 4x4
//	//	FTval[0]=  Fval[0];    FTval[1]=  Fval[4];  FTval[2]=  Fval[8];   FTval[3]=  Fval[12];  //r1
//	//	FTval[4]=  Fval[1];    FTval[5]=  Fval[5];  FTval[6]=  Fval[9];   FTval[7]=  Fval[13];  //r2
//	//	FTval[8]=  Fval[2];    FTval[9]=  Fval[6];  FTval[10]= Fval[10];  FTval[11]= Fval[14];  //r3
//	//	FTval[12]= Fval[3];    FTval[13]= Fval[7];  FTval[14]= Fval[11];  FTval[15]= Fval[15];  //r4
//	//	//end Fval 4x4
//	//
//	//
//	//
//	//
//	//	/* Gval Buffer 4x3*/
//	//
//	//	Gval[0] = q[1]*dt; Gval[1] =   q[2]*dt; Gval[2] =   q[3]*dt;    //r1
//	//	Gval[3] = -q[0]*dt; Gval[4] =   q[3]*dt; Gval[5] =   -q[2]*dt;  //r2
//	//	Gval[6] = -q[3]*dt; Gval[7] = -q[0]*dt;  Gval[8] = q[1]*dt;     //r3
//	//	Gval[9] =q[2]*dt;  Gval[10] = -q[1]*dt;  Gval[11] = -q[0]*dt;   //r4
//	//	/*end  Gval Buffer */
//	//
//	//	//Gval 3x4
//	//	GTval[0]=  Gval[0];    GTval[1]=  Gval[3];  GTval[2]=  Gval[6];  GTval[3]=  Gval[9];   //r1
//	//	GTval[4]=  Gval[1];    GTval[5]=  Gval[4];  GTval[6]=  Gval[7];  GTval[7]=  Gval[10];  //r2
//	//	GTval[8]=  Gval[2];    GTval[9]=  Gval[5];  GTval[10]= Gval[8];  GTval[11]= Gval[11];  //r3
//	//	//end Gval 3x4
//	//
//	//
//	//
//	//
//	//	// output equation  6x1
//	//	yhatval[0] = -2*(q[1]*q[3]-q[0]*q[2])*gmps2;
//	//	yhatval[1] = -2*(q[2]*q[3]+q[0]*q[1])*gmps2;
//	//	yhatval[2] = -1*(q[0]*q[0]-q[1]*q[1]-q[2]*q[2]+q[3]*q[3])*gmps2;
//	//	yhatval[3] = ((q[0]*q[0]+q[1]*q[1]-q[2]*q[2]-q[3]*q[3])*m[0]  +   (2*(q[1]*q[2]+q[0]*q[3]))*m[1] + 2*(q[1]*q[3]-q[0]*q[2])*m[2]) ;
//	//	yhatval[4] = (2*(q[1]*q[2]-q[0]*q[3])*m[0] + (q[0]*q[0]-q[1]*q[1]+q[2]*q[2]-q[3]*q[3])*m[1] +  (2*(q[2]*q[3]+q[0]*q[1]))*m[2]) ;
//	//	yhatval[5] = (2*(q[1]*q[3]+q[0]*q[2])*m[0] + (2*(q[2]*q[3]-q[0]*q[1]))*m[1] +  (q[0]*q[0]-q[1]*q[1]-q[2]*q[2]+q[3]*q[3])*m[2]) ;
//	//
//	//	//  H jacobain 6x4
//	//	Hval[0] =	 (q[2]*2)*gmps2;   Hval[1] =  (-q[3]*2 )*gmps2;  Hval[2] =  (q[0]*2 )*gmps2;  Hval[3] = (-q[1]*2 )*gmps2;//r1
//	//	Hval[4] =	(-q[1]*2)*gmps2;   Hval[5] =  (-q[0]*2 )*gmps2;  Hval[6] = (-q[3]*2 )*gmps2;  Hval[7] = (-q[2]*2 )*gmps2;//r2
//	//	Hval[8] =	(-q[0]*2)*gmps2;   Hval[9] =   (q[1]*2 )*gmps2;  Hval[10] =  (q[2]*2  )*gmps2; Hval[11] = (-q[3]*2 )*gmps2;//r3
//	//	//r4
//	//	Hval[12] = (q[0]*2*m[0]+	q[3]*2*m[1]-q[2]*2*m[2]) ; Hval[13] = (q[1]*2*m[0]+q[2]*2*m[1]+q[3]*2*m[2]) ; Hval[14] = (-q[2]*2*m[0]+  q[1]*2*m[1]-q[0]*2*m[2]) ;  Hval[15] = (-q[3]*2*m[0]+ q[0]*2*m[1]+q[1]*2*m[2]) ;
//	//	//r5
//	//	Hval[16] = (-q[3]*2*m[0]+	q[0]*2*m[1]+q[1]*2*m[2]) ; Hval[17] = (q[2]*2*m[0]-q[1]*2*m[1]+q[0]*2*m[2])  ; Hval[18] = (q[1]*2*m[0]+ q[2]*2*m[1]+q[3]*2*m[2]) ; Hval[19] = (-q[0]*2*m[0] -q[3]*2*m[1]+q[2]*2*m[2]) ;
//	//	//r6
//	//	Hval[20] = (q[2]*2*m[0]-q[1]*2*m[1]+q[0]*2*m[2]) ; Hval[21] =    (q[3]*2*m[0]-q[0]*2*m[1]-q[1]*2*m[2]) ; Hval[22] = (q[0]*2*m[0]+ q[3]*2*m[1]-q[2]*2*m[2]) ;  Hval[23] =  (q[1]*2*m[0]+  q[2]*2*m[1]+q[3]*2*m[2]) ;
//	//	// end  H jacobain
//	//
//	//	//HTval 4x6
//	//	HTval[0]=  Hval[0];    HTval[1]=  Hval[4];  HTval[2]=  Hval[8];  HTval[3]=  Hval[12];  HTval[4]=  Hval[16];  HTval[5]=  Hval[20];//r1
//	//	HTval[6]=  Hval[1];    HTval[7]=  Hval[5];  HTval[8]=  Hval[9];  HTval[9]=  Hval[13];  HTval[10]= Hval[17];  HTval[11]= Hval[21];//r2
//	//	HTval[12]= Hval[2];    HTval[13]= Hval[6];  HTval[14]= Hval[10];  HTval[15]= Hval[14];  HTval[16]= Hval[18];  HTval[17]= Hval[22];//r3
//	//	HTval[18]= Hval[3];    HTval[19]= Hval[7];  HTval[20]= Hval[11];  HTval[21]= Hval[15];  HTval[22]= Hval[19];  HTval[23]= Hval[23];//r4
//	//	//end HTval 4x6
//	//
//	//
//	//	/* calculation of estimated qt; F Multiply with q */
//	//	//q 4x1
//	//	qt[0]= Fval[0]*q[0]+Fval[1]*q[1]+Fval[2]*q[2]+Fval[3]*q[3];//r1
//	//	qt[1]= Fval[4]*q[0]+Fval[5]*q[1]+Fval[6]*q[2]+Fval[7]*q[3];//r2
//	//	qt[2]= Fval[8]*q[0]+Fval[9]*q[1]+Fval[10]*q[2]+Fval[11]*q[3];//r3
//	//	qt[3]= Fval[12]*q[0]+Fval[13]*q[1]+Fval[14]*q[2]+Fval[15]*q[3];//r4
//	//	//end q
//	//	/* calculation of estimated qt; F Multiply with q */
//	//
//	//
//	//
//	//
//	//
//	//
//	//
//	//
//	//
//	//
//	//	/* calculation of P*/
//	//	//FP 4x4
//	//	//r1
//	//	FPval[0]=  Fval[0]*Pval[0]+Fval[1]*Pval[4]+Fval[2]*Pval[8]+Fval[3]*Pval[12];      FPval[1]=  Fval[0]*Pval[1]+Fval[1]*Pval[5]+Fval[2]*Pval[9]+Fval[3]*Pval[13];
//	//	FPval[2]=  Fval[0]*Pval[2]+Fval[1]*Pval[6]+Fval[2]*Pval[10]+Fval[3]*Pval[14];     FPval[3]=  Fval[0]*Pval[3]+Fval[1]*Pval[7]+Fval[2]*Pval[11]+Fval[3]*Pval[15];
//	//	//r2
//	//	FPval[4]=  Fval[4]*Pval[0]+Fval[5]*Pval[4]+Fval[6]*Pval[8]+Fval[7]*Pval[12];      FPval[5]=  Fval[4]*Pval[1]+Fval[5]*Pval[5]+Fval[6]*Pval[9]+Fval[7]*Pval[13];
//	//	FPval[6]=  Fval[4]*Pval[2]+Fval[5]*Pval[6]+Fval[6]*Pval[10]+Fval[7]*Pval[14];     FPval[7]=  Fval[4]*Pval[3]+Fval[5]*Pval[7]+Fval[6]*Pval[11]+Fval[7]*Pval[15];
//	//	//r3
//	//	FPval[8]=  Fval[8]*Pval[0]+Fval[9]*Pval[4]+Fval[10]*Pval[8]+Fval[11]*Pval[12];    FPval[9]=  Fval[8]*Pval[1]+Fval[9]*Pval[5]+Fval[10]*Pval[9]+Fval[11]*Pval[13];
//	//	FPval[10]= Fval[8]*Pval[2]+Fval[9]*Pval[6]+Fval[10]*Pval[10]+Fval[11]*Pval[14];   FPval[11]= Fval[8]*Pval[3]+Fval[9]*Pval[7]+Fval[10]*Pval[11]+Fval[11]*Pval[15];
//	//	//r4
//	//	FPval[12]= Fval[12]*Pval[0]+Fval[13]*Pval[4]+Fval[14]*Pval[8]+Fval[15]*Pval[12];  FPval[13]= Fval[12]*Pval[1]+Fval[13]*Pval[5]+Fval[14]*Pval[9]+Fval[15]*Pval[13];
//	//	FPval[14]= Fval[12]*Pval[2]+Fval[13]*Pval[6]+Fval[14]*Pval[10]+Fval[15]*Pval[14]; FPval[15]= Fval[12]*Pval[3]+Fval[13]*Pval[7]+Fval[14]*Pval[11]+Fval[15]*Pval[15];
//	//	///End FP
//	//
//	//	//FPFT4x4
//	//	//r1
//	//	FPFTval[0]=  FPval[0]*FTval[0]+FPval[1]*FTval[4]+FPval[2]*FTval[8]+FPval[3]*FTval[12];      FPFTval[1]=  FPval[0]*FTval[1]+FPval[1]*FTval[5]+FPval[2]*FTval[9]+FPval[3]*FTval[13];
//	//	FPFTval[2]=  FPval[0]*FTval[2]+FPval[1]*FTval[6]+FPval[2]*FTval[10]+FPval[3]*FTval[14];     FPFTval[3]=  FPval[0]*FTval[3]+FPval[1]*FTval[7]+FPval[2]*FTval[11]+FPval[3]*FTval[15];
//	//	//r2
//	//	FPFTval[4]=  FPval[4]*FTval[0]+FPval[5]*FTval[4]+Fval[6]*FTval[8]+FPval[7]*FTval[12];       FPFTval[5]=  FPval[4]*FTval[1]+FPval[5]*FTval[5]+FPval[6]*FTval[9]+FPval[7]*FTval[13];
//	//	FPFTval[6]=  FPval[4]*FTval[2]+FPval[5]*FTval[6]+FPval[6]*FTval[10]+FPval[7]*FTval[14];     FPFTval[7]=  FPval[4]*FTval[3]+FPval[5]*FTval[7]+FPval[6]*FTval[11]+FPval[7]*FTval[15];
//	//	//r3
//	//	FPFTval[8]=  FPval[8]*FTval[0]+FPval[9]*FTval[4]+FPval[10]*FTval[8]+FPval[11]*FTval[12];    FPFTval[9]=  FPval[8]*FTval[1]+FPval[9]*FTval[5]+FPval[10]*FTval[9]+FPval[11]*FTval[13];
//	//	FPFTval[10]= FPval[8]*FTval[2]+FPval[9]*FTval[6]+FPval[10]*FTval[10]+FPval[11]*FTval[14];   FPFTval[11]= FPval[8]*FTval[3]+FPval[9]*FTval[7]+FPval[10]*FTval[11]+FPval[11]*FTval[15];
//	//	//r4
//	//	FPFTval[12]= FPval[12]*FTval[0]+FPval[13]*FTval[4]+FPval[14]*FTval[8]+FPval[15]*FTval[12];  FPFTval[13]= FPval[12]*FTval[1]+FPval[13]*FTval[5]+FPval[14]*FTval[9]+FPval[15]*FTval[13];
//	//	FPFTval[14]= FPval[12]*FTval[2]+FPval[13]*FTval[6]+FPval[14]*FTval[10]+FPval[15]*FTval[14]; FPFTval[15]= FPval[12]*FTval[3]+FPval[13]*FTval[7]+FPval[14]*FTval[11]+FPval[15]*FTval[15];
//	//	////end FPFT
//	//
//	//	//GQ 4x3
//	//	//r1
//	//	GQval[0]=  Gval[0]*Qval[0]+Gval[1]*Qval[3]+Gval[2]*Qval[6];     GQval[1]=  Gval[0]*Qval[1]+Gval[1]*Qval[4]+Gval[2]*Qval[7];
//	//	GQval[2]=  Gval[0]*Qval[2]+Gval[1]*Qval[5]+Gval[2]*Qval[8];
//	//	//r2
//	//	GQval[3]=  Gval[3]*Qval[0]+Gval[4]*Qval[3]+Gval[5]*Qval[6];     GQval[4]=  Gval[3]*Qval[1]+Gval[4]*Qval[4]+Gval[5]*Qval[7];
//	//	GQval[5]=  Gval[3]*Qval[2]+Gval[4]*Qval[5]+Gval[5]*Qval[8];
//	//	//r3
//	//	GQval[6]=  Gval[6]*Qval[0]+Gval[7]*Qval[3]+Gval[8]*Qval[6];     GQval[7]=  Gval[6]*Qval[1]+Gval[7]*Qval[4]+Gval[8]*Qval[7];
//	//	GQval[8]=  Gval[6]*Qval[2]+Gval[7]*Qval[5]+Gval[8]*Qval[8];
//	//	//r4
//	//	GQval[9]=  Gval[9]*Qval[0]+Gval[10]*Qval[3]+Gval[11]*Qval[6];   GQval[10]=  Gval[9]*Qval[1]+Gval[10]*Qval[4]+Gval[11]*Qval[7];
//	//	GQval[11]=  Gval[9]*Qval[2]+Gval[10]*Qval[5]+Gval[11]*Qval[8];
//	//	/// End GQ
//	//
//	//	//Qt 4x4
//	//	//r1
//	//	Qtval[0]=   GQval[0]*GTval[0]+GQval[1]*GTval[4]+GQval[2]*GTval[8];     Qtval[1]=  GQval[0]*GTval[1]+GQval[1]*GTval[5]+GQval[2]*GTval[9];
//	//	Qtval[2]=   GQval[0]*GTval[2]+GQval[1]*GTval[6]+GQval[2]*GTval[10];    Qtval[3]=  GQval[0]*GTval[3]+GQval[1]*GTval[7]+GQval[2]*GTval[11];
//	//	//r2
//	//	Qtval[4]=   GQval[3]*GTval[0]+GQval[4]*GTval[4]+GQval[5]*GTval[8];     Qtval[5]=  GQval[3]*GTval[1]+GQval[4]*GTval[5]+GQval[5]*GTval[9];
//	//	Qtval[6]=   GQval[3]*GTval[2]+GQval[4]*GTval[6]+GQval[5]*GTval[10];    Qtval[7]=  GQval[3]*GTval[3]+GQval[4]*GTval[7]+GQval[5]*GTval[11];
//	//	//r3
//	//	Qtval[8]=   GQval[6]*GTval[0]+GQval[7]*GTval[4]+GQval[8]*GTval[8];     Qtval[9]=  GQval[6]*GTval[1]+GQval[7]*GTval[5]+GQval[8]*GTval[9];
//	//	Qtval[10]=  GQval[6]*GTval[2]+GQval[7]*GTval[6]+GQval[8]*GTval[10];    Qtval[11]= GQval[6]*GTval[3]+GQval[7]*GTval[7]+GQval[8]*GTval[11];
//	//	//r4
//	//	Qtval[12]=  GQval[9]*GTval[0]+GQval[10]*GTval[4]+GQval[11]*GTval[8];   Qtval[13]= GQval[9]*GTval[1]+GQval[10]*GTval[5]+GQval[11]*GTval[9];
//	//	Qtval[14]=  GQval[9]*GTval[2]+GQval[10]*GTval[6]+GQval[11]*GTval[10];  Qtval[15]= GQval[9]*GTval[3]+GQval[10]*GTval[7]+GQval[11]*GTval[11];
//	//	/// End Qt
//	//
//	//
//	//	//Ptval 4x4
//	//	Ptval[0]=  FPFTval[0]+Qtval[0];      Ptval[1]=  FPFTval[1]+Qtval[1];     Ptval[2]=  FPFTval[2]+Qtval[2];     Ptval[3]=  FPFTval[3]+Qtval[3];//r1
//	//	Ptval[4]=  FPFTval[4]+Qtval[4];      Ptval[5]=  FPFTval[5]+Qtval[5];     Ptval[6]=  FPFTval[6]+Qtval[6];     Ptval[7]=  FPFTval[7]+Qtval[7];//r2
//	//	Ptval[8]=  FPFTval[8]+Qtval[8];      Ptval[9]=  FPFTval[9]+Qtval[9];     Ptval[10]= FPFTval[10]+Qtval[10];   Ptval[11]= FPFTval[11]+Qtval[11];//r3
//	//	Ptval[12]= FPFTval[12]+Qtval[12];    Ptval[13]= FPFTval[13]+Qtval[13];   Ptval[14]= FPFTval[14]+Qtval[14];   Ptval[15]= FPFTval[15]+Qtval[15];//r4
//	//	//end Ptval
//	//	/*end  calculation of P*/
//	//
//	//
//	//	/* calculation of K */
//	//	//HPval 6x4
//	//	//r1
//	//	HPval[0]=   Hval[0]*Ptval[0]+Hval[1]*Ptval[4]+Hval[2]*Ptval[8]+Hval[3]*Ptval[12];      HPval[1]=   Hval[0]*Ptval[1]+Hval[1]*Ptval[5]+Hval[2]*Ptval[9]+Hval[3]*Ptval[13];
//	//	HPval[2]=   Hval[0]*Ptval[2]+Hval[1]*Ptval[6]+Hval[2]*Ptval[10]+Hval[3]*Ptval[14];     HPval[3]=   Hval[0]*Ptval[3]+Hval[1]*Ptval[7]+Hval[2]*Ptval[11]+Hval[3]*Ptval[15];
//	//	//r2
//	//	HPval[4]=   Hval[4]*Ptval[0]+Hval[5]*Ptval[4]+Hval[6]*Ptval[8]+Hval[7]*Ptval[12];      HPval[5]=   Hval[4]*Ptval[1]+Hval[5]*Ptval[5]+Hval[6]*Ptval[9]+Hval[7]*Ptval[13];
//	//	HPval[6]=   Hval[4]*Ptval[2]+Hval[5]*Ptval[6]+Hval[6]*Ptval[10]+Hval[7]*Ptval[14];     HPval[7]=   Hval[4]*Ptval[3]+Hval[5]*Ptval[7]+Hval[6]*Ptval[11]+Hval[7]*Ptval[15];
//	//	//r3
//	//	HPval[8]=   Hval[8]*Ptval[0]+Hval[9]*Ptval[4]+Hval[10]*Ptval[8]+Hval[11]*Ptval[12];    HPval[9]=   Hval[8]*Ptval[1]+Hval[9]*Ptval[5]+Hval[10]*Ptval[9]+Hval[11]*Ptval[13];
//	//	HPval[10]=  Hval[8]*Ptval[2]+Hval[9]*Ptval[6]+Hval[10]*Ptval[10]+Hval[11]*Ptval[14];   HPval[11]=  Hval[8]*Ptval[3]+Hval[9]*Ptval[7]+Hval[10]*Ptval[11]+Hval[11]*Ptval[15];
//	//	//r4
//	//	HPval[12]=  Hval[12]*Ptval[0]+Hval[13]*Ptval[4]+Hval[14]*Ptval[8]+Hval[15]*Ptval[12];  HPval[13]=  Hval[12]*Ptval[1]+Hval[13]*Ptval[5]+Hval[14]*Ptval[9]+Hval[15]*Ptval[13];
//	//	HPval[14]=  Hval[12]*Ptval[2]+Hval[13]*Ptval[6]+Hval[14]*Ptval[10]+Hval[15]*Ptval[14]; HPval[15]=  Hval[12]*Ptval[3]+Hval[13]*Ptval[7]+Hval[14]*Ptval[11]+Hval[15]*Ptval[15];
//	//	//r5
//	//	HPval[16]=  Hval[16]*Ptval[0]+Hval[17]*Ptval[4]+Hval[18]*Ptval[8]+Hval[19]*Ptval[12];  HPval[17]=  Hval[16]*Ptval[1]+Hval[17]*Ptval[5]+Hval[18]*Ptval[9]+Hval[19]*Ptval[13];
//	//	HPval[18]=  Hval[16]*Ptval[2]+Hval[17]*Ptval[6]+Hval[18]*Ptval[10]+Hval[19]*Ptval[14]; HPval[19]=  Hval[16]*Ptval[3]+Hval[17]*Ptval[7]+Hval[18]*Ptval[11]+Hval[19]*Ptval[15];
//	//	//r3
//	//	HPval[20]=  Hval[20]*Ptval[0]+Hval[21]*Ptval[4]+Hval[22]*Ptval[8]+Hval[23]*Ptval[12];  HPval[21]=  Hval[20]*Ptval[1]+Hval[21]*Ptval[5]+Hval[22]*Ptval[9]+Hval[23]*Ptval[13];
//	//	HPval[22]=  Hval[20]*Ptval[2]+Hval[21]*Ptval[6]+Hval[22]*Ptval[10]+Hval[23]*Ptval[14]; HPval[23]=  Hval[20]*Ptval[3]+Hval[21]*Ptval[7]+Hval[22]*Ptval[11]+Hval[23]*Ptval[15];
//	//	//end HPval 6x4
//	//
//	//
//	//	//HPHTval  6x6
//	//	//r1
//	//	HPHTval[0]=   HPval[0]*HTval[0]+HPval[1]*HTval[6]+HPval[2]*HTval[12]+HPval[3]*HTval[18];     HPHTval[1]=   HPval[0]*HTval[1]+HPval[1]*HTval[7]+HPval[2]*HTval[13]+HPval[3]*HTval[19];
//	//	HPHTval[2]=   HPval[0]*HTval[2]+HPval[1]*HTval[8]+HPval[2]*HTval[14]+HPval[3]*HTval[20];     HPHTval[3]=   HPval[0]*HTval[3]+HPval[1]*HTval[9]+HPval[2]*HTval[15]+HPval[3]*HTval[21];
//	//	HPHTval[4]=   HPval[0]*HTval[4]+HPval[1]*HTval[10]+HPval[2]*HTval[16]+HPval[3]*HTval[22];     HPHTval[5]=  HPval[0]*HTval[5]+HPval[1]*HTval[11]+HPval[2]*HTval[17]+HPval[3]*HTval[23];
//	//	//r2
//	//	HPHTval[6]=   HPval[4]*HTval[0]+HPval[5]*HTval[6]+HPval[6]*HTval[12]+HPval[7]*HTval[18];     HPHTval[7]=   HPval[4]*HTval[1]+HPval[5]*HTval[7]+HPval[6]*HTval[13]+HPval[7]*HTval[19];
//	//	HPHTval[8]=   HPval[4]*HTval[2]+HPval[5]*HTval[8]+HPval[6]*HTval[14]+HPval[7]*HTval[20];     HPHTval[9]=   HPval[4]*HTval[3]+HPval[5]*HTval[9]+HPval[6]*HTval[15]+HPval[7]*HTval[21];
//	//	HPHTval[10]=  HPval[4]*HTval[4]+HPval[5]*HTval[10]+HPval[6]*HTval[16]+HPval[7]*HTval[22];    HPHTval[11]=  HPval[4]*HTval[5]+HPval[5]*HTval[11]+HPval[6]*HTval[17]+HPval[7]*HTval[23];
//	//	//r3
//	//	HPHTval[12]=   HPval[8]*HTval[0]+HPval[9]*HTval[6]+HPval[10]*HTval[12]+HPval[11]*HTval[18];     HPHTval[13]=   HPval[8]*HTval[1]+HPval[9]*HTval[7]+HPval[10]*HTval[13]+HPval[11]*HTval[19];
//	//	HPHTval[14]=   HPval[8]*HTval[2]+HPval[9]*HTval[8]+HPval[10]*HTval[14]+HPval[11]*HTval[20];     HPHTval[15]=   HPval[8]*HTval[3]+HPval[9]*HTval[9]+HPval[10]*HTval[15]+HPval[11]*HTval[21];
//	//	HPHTval[16]=   HPval[8]*HTval[4]+HPval[9]*HTval[10]+HPval[10]*HTval[16]+HPval[11]*HTval[22];    HPHTval[17]=  HPval[8]*HTval[5]+HPval[9]*HTval[11]+HPval[10]*HTval[17]+HPval[11]*HTval[23];
//	//	//r4
//	//	HPHTval[18]=   HPval[12]*HTval[0]+HPval[13]*HTval[6]+HPval[14]*HTval[12]+HPval[15]*HTval[18];     HPHTval[19]=   HPval[12]*HTval[1]+HPval[13]*HTval[7]+HPval[14]*HTval[13]+HPval[15]*HTval[19];
//	//	HPHTval[20]=   HPval[12]*HTval[2]+HPval[13]*HTval[8]+HPval[14]*HTval[14]+HPval[15]*HTval[20];     HPHTval[21]=   HPval[12]*HTval[3]+HPval[13]*HTval[9]+HPval[14]*HTval[15]+HPval[15]*HTval[21];
//	//	HPHTval[22]=   HPval[12]*HTval[4]+HPval[13]*HTval[10]+HPval[14]*HTval[16]+HPval[15]*HTval[22];    HPHTval[23]=  HPval[12]*HTval[5]+HPval[13]*HTval[11]+HPval[14]*HTval[17]+HPval[15]*HTval[23];
//	//	//r5
//	//	HPHTval[24]=   HPval[16]*HTval[0]+HPval[17]*HTval[6]+HPval[18]*HTval[12]+HPval[19]*HTval[18];     HPHTval[25]=   HPval[16]*HTval[1]+HPval[17]*HTval[7]+HPval[18]*HTval[13]+HPval[19]*HTval[19];
//	//	HPHTval[26]=   HPval[16]*HTval[2]+HPval[17]*HTval[8]+HPval[18]*HTval[14]+HPval[19]*HTval[20];     HPHTval[27]=   HPval[16]*HTval[3]+HPval[17]*HTval[9]+HPval[18]*HTval[15]+HPval[19]*HTval[21];
//	//	HPHTval[28]=   HPval[16]*HTval[4]+HPval[17]*HTval[10]+HPval[18]*HTval[16]+HPval[19]*HTval[22];    HPHTval[29]=  HPval[16]*HTval[5]+HPval[17]*HTval[11]+HPval[18]*HTval[17]+HPval[19]*HTval[23];
//	//	//r6
//	//	HPHTval[30]=   HPval[20]*HTval[0]+HPval[21]*HTval[6]+HPval[22]*HTval[12]+HPval[23]*HTval[18];     HPHTval[31]=   HPval[20]*HTval[1]+HPval[21]*HTval[7]+HPval[22]*HTval[13]+HPval[23]*HTval[19];
//	//	HPHTval[32]=   HPval[20]*HTval[2]+HPval[21]*HTval[8]+HPval[22]*HTval[14]+HPval[23]*HTval[20];     HPHTval[33]=   HPval[20]*HTval[3]+HPval[21]*HTval[9]+HPval[22]*HTval[15]+HPval[23]*HTval[21];
//	//	HPHTval[34]=   HPval[20]*HTval[4]+HPval[21]*HTval[10]+HPval[22]*HTval[16]+HPval[23]*HTval[22];    HPHTval[35]=  HPval[20]*HTval[5]+HPval[21]*HTval[11]+HPval[22]*HTval[17]+HPval[23]*HTval[23];
//	//	// End HPHTval  6x6
//	//
//	//	//S 6x6
//	//	Sval[0]=  HPHTval[0]+Rval[0];     Sval[1]=  HPHTval[1]+Rval[1];     Sval[2]=  HPHTval[2]+Rval[2];     Sval[3]=  HPHTval[3]+Rval[3];   Sval[4]=  HPHTval[4]+Rval[4];    Sval[5]=  HPHTval[5]+Rval[5];
//	//	Sval[6]=  HPHTval[6]+Rval[6];     Sval[7]=  HPHTval[7]+Rval[7];     Sval[8]=  HPHTval[8]+Rval[8];     Sval[9]=  HPHTval[9]+Rval[9];   Sval[10]= HPHTval[10]+Rval[10];  Sval[11]= HPHTval[11]+Rval[11];
//	//	Sval[12]= HPHTval[12]+Rval[12];   Sval[13]= HPHTval[13]+Rval[13];   Sval[14]= HPHTval[14]+Rval[14];   Sval[15]= HPHTval[15]+Rval[15]; Sval[16]= HPHTval[16]+Rval[16];  Sval[17]= HPHTval[17]+Rval[17];
//	//	Sval[18]= HPHTval[18]+Rval[18];   Sval[19]= HPHTval[19]+Rval[19];  	Sval[20]= HPHTval[20]+Rval[20];   Sval[21]= HPHTval[21]+Rval[21]; Sval[22]= HPHTval[22]+Rval[22];  Sval[23]= HPHTval[23]+Rval[23];
//	//	Sval[24]= HPHTval[24]+Rval[24];   Sval[25]= HPHTval[25]+Rval[25];  	Sval[26]= HPHTval[20]+Rval[20];   Sval[27]= HPHTval[27]+Rval[27]; Sval[28]= HPHTval[28]+Rval[28];  Sval[29]= HPHTval[29]+Rval[29];
//	//	Sval[30]= HPHTval[30]+Rval[30];   Sval[31]= HPHTval[31]+Rval[31];   Sval[32]= HPHTval[20]+Rval[20];   Sval[33]= HPHTval[33]+Rval[33]; Sval[34]= HPHTval[34]+Rval[34];  Sval[35]= HPHTval[35]+Rval[35];
//	//	// end S
//	//
//	//	status = arm_mat_inverse_f32(&S, &SI);     // calc inverse with arm math
//	//
//	//
//	//	//PHTval 4x6
//	//	//r1
//	//	PHTval[0]=   Ptval[0]*HTval[0]+Ptval[1]*HTval[6]+Ptval[2]*HTval[12]+Ptval[3]*HTval[18];     PHTval[1]=   Ptval[0]*HTval[1]+Ptval[1]*HTval[7]+Ptval[2]*HTval[13]+Ptval[3]*HTval[19];
//	//	PHTval[2]=   Ptval[0]*HTval[2]+Ptval[1]*HTval[8]+Ptval[2]*HTval[14]+Ptval[3]*HTval[20];     PHTval[3]=   Ptval[0]*HTval[3]+Ptval[1]*HTval[9]+Ptval[2]*HTval[15]+Ptval[3]*HTval[21];
//	//	PHTval[4]=   Ptval[0]*HTval[4]+Ptval[1]*HTval[10]+Ptval[2]*HTval[16]+Ptval[3]*HTval[22];    PHTval[5]=   Ptval[0]*HTval[5]+Ptval[1]*HTval[11]+Ptval[2]*HTval[17]+Ptval[3]*HTval[23];
//	//	//r2
//	//	PHTval[6]=   Ptval[4]*HTval[0]+Ptval[5]*HTval[6]+Ptval[6]*HTval[12]+Ptval[7]*HTval[18];     PHTval[7]=   Ptval[4]*HTval[1]+Ptval[5]*HTval[7]+Ptval[6]*HTval[13]+Ptval[7]*HTval[19];
//	//	PHTval[8]=   Ptval[4]*HTval[2]+Ptval[5]*HTval[8]+Ptval[6]*HTval[14]+Ptval[7]*HTval[20];     PHTval[9]=   Ptval[4]*HTval[3]+Ptval[5]*HTval[9]+Ptval[6]*HTval[15]+Ptval[7]*HTval[21];
//	//	PHTval[10]=  Ptval[4]*HTval[4]+Ptval[5]*HTval[10]+Ptval[6]*HTval[16]+Ptval[7]*HTval[22];    PHTval[11]=  Ptval[4]*HTval[5]+Ptval[5]*HTval[11]+Ptval[6]*HTval[17]+Ptval[7]*HTval[23];
//	//	//r3
//	//	PHTval[12]=  Ptval[8]*HTval[0]+Ptval[9]*HTval[6]+Ptval[10]*HTval[12]+Ptval[11]*HTval[18];   PHTval[13]=  Ptval[8]*HTval[1]+Ptval[9]*HTval[7]+Ptval[10]*HTval[13]+Ptval[11]*HTval[19];
//	//	PHTval[14]=  Ptval[8]*HTval[2]+Ptval[9]*HTval[8]+Ptval[10]*HTval[14]+Ptval[11]*HTval[20];   PHTval[15]=  Ptval[8]*HTval[3]+Ptval[9]*HTval[9]+Ptval[10]*HTval[15]+Ptval[11]*HTval[21];
//	//	PHTval[16]=  Ptval[8]*HTval[4]+Ptval[9]*HTval[10]+Ptval[10]*HTval[16]+Ptval[11]*HTval[22];  PHTval[17]=  Ptval[8]*HTval[5]+Ptval[9]*HTval[11]+Ptval[10]*HTval[17]+Ptval[11]*HTval[23];
//	//	//r4
//	//	PHTval[18]=  Ptval[12]*HTval[0]+Ptval[13]*HTval[6]+Ptval[14]*HTval[12]+Ptval[15]*HTval[18]; PHTval[19]=  Ptval[12]*HTval[1]+Ptval[13]*HTval[7]+Ptval[14]*HTval[13]+Ptval[15]*HTval[19];
//	//	PHTval[20]=  Ptval[12]*HTval[2]+Ptval[13]*HTval[8]+Ptval[14]*HTval[14]+Ptval[15]*HTval[20]; PHTval[21]=  Ptval[12]*HTval[3]+Ptval[13]*HTval[9]+Ptval[14]*HTval[15]+Ptval[15]*HTval[21];
//	//	PHTval[22]=  Ptval[12]*HTval[4]+Ptval[13]*HTval[10]+Ptval[14]*HTval[16]+Ptval[15]*HTval[22];PHTval[23]=  Ptval[12]*HTval[5]+Ptval[13]*HTval[11]+Ptval[14]*HTval[17]+Ptval[15]*HTval[23];
//	//	//end PHTval
//	//
//	//	//K
//	//	//r1
//	//	Kval[0]=   PHTval[0]*SIval[0]+PHTval[1]*SIval[6]+PHTval[2]*SIval[12]+PHTval[3]*SIval[18]+PHTval[4]*SIval[24]+PHTval[5]*SIval[30];
//	//	Kval[1]=   PHTval[0]*SIval[1]+PHTval[1]*SIval[7]+PHTval[2]*SIval[13]+PHTval[3]*SIval[19]+PHTval[4]*SIval[25]+PHTval[5]*SIval[31];
//	//	Kval[2]=   PHTval[0]*SIval[2]+PHTval[1]*SIval[8]+PHTval[2]*SIval[14]+PHTval[3]*SIval[20]+PHTval[4]*SIval[26]+PHTval[5]*SIval[32];
//	//	Kval[3]=   PHTval[0]*SIval[3]+PHTval[1]*SIval[9]+PHTval[2]*SIval[15]+PHTval[3]*SIval[21]+PHTval[4]*SIval[27]+PHTval[5]*SIval[33];
//	//	Kval[4]=   PHTval[0]*SIval[4]+PHTval[1]*SIval[10]+PHTval[2]*SIval[16]+PHTval[3]*SIval[22]+PHTval[4]*SIval[28]+PHTval[5]*SIval[34];
//	//	Kval[5]=   PHTval[0]*SIval[5]+PHTval[1]*SIval[11]+PHTval[2]*SIval[17]+PHTval[3]*SIval[23]+PHTval[4]*SIval[29]+PHTval[5]*SIval[35];
//	//	//r2
//	//	Kval[6]=   PHTval[6]*SIval[0]+PHTval[7]*SIval[6]+PHTval[8]*SIval[12]+PHTval[9]*SIval[18]+PHTval[10]*SIval[24]+PHTval[11]*SIval[30];
//	//	Kval[7]=   PHTval[6]*SIval[1]+PHTval[7]*SIval[7]+PHTval[8]*SIval[13]+PHTval[9]*SIval[19]+PHTval[10]*SIval[25]+PHTval[11]*SIval[31];
//	//	Kval[8]=   PHTval[6]*SIval[2]+PHTval[7]*SIval[8]+PHTval[8]*SIval[14]+PHTval[9]*SIval[20]+PHTval[10]*SIval[26]+PHTval[11]*SIval[32];
//	//	Kval[9]=   PHTval[6]*SIval[3]+PHTval[7]*SIval[9]+PHTval[8]*SIval[15]+PHTval[9]*SIval[21]+PHTval[10]*SIval[27]+PHTval[11]*SIval[33];
//	//	Kval[10]=   PHTval[6]*SIval[4]+PHTval[7]*SIval[10]+PHTval[8]*SIval[16]+PHTval[9]*SIval[22]+PHTval[10]*SIval[28]+PHTval[11]*SIval[34];
//	//	Kval[11]=   PHTval[6]*SIval[5]+PHTval[7]*SIval[11]+PHTval[8]*SIval[17]+PHTval[9]*SIval[23]+PHTval[10]*SIval[29]+PHTval[11]*SIval[35];
//	//	//r3
//	//	Kval[12]=   PHTval[12]*SIval[0]+PHTval[13]*SIval[6]+PHTval[14]*SIval[12]+PHTval[15]*SIval[18]+PHTval[16]*SIval[24]+PHTval[17]*SIval[30];
//	//	Kval[13]=   PHTval[12]*SIval[1]+PHTval[13]*SIval[7]+PHTval[14]*SIval[13]+PHTval[15]*SIval[19]+PHTval[16]*SIval[25]+PHTval[17]*SIval[31];
//	//	Kval[14]=   PHTval[12]*SIval[2]+PHTval[13]*SIval[8]+PHTval[14]*SIval[14]+PHTval[15]*SIval[20]+PHTval[16]*SIval[26]+PHTval[17]*SIval[32];
//	//	Kval[15]=   PHTval[12]*SIval[3]+PHTval[13]*SIval[9]+PHTval[14]*SIval[15]+PHTval[15]*SIval[21]+PHTval[16]*SIval[27]+PHTval[17]*SIval[33];
//	//	Kval[16]=   PHTval[12]*SIval[4]+PHTval[13]*SIval[10]+PHTval[14]*SIval[16]+PHTval[15]*SIval[22]+PHTval[16]*SIval[28]+PHTval[17]*SIval[34];
//	//	Kval[17]=   PHTval[12]*SIval[5]+PHTval[13]*SIval[11]+PHTval[14]*SIval[17]+PHTval[15]*SIval[23]+PHTval[16]*SIval[29]+PHTval[17]*SIval[35];
//	//	//r4
//	//	Kval[18]=   PHTval[18]*SIval[0]+PHTval[19]*SIval[6]+PHTval[20]*SIval[12]+PHTval[21]*SIval[18]+PHTval[22]*SIval[24]+PHTval[23]*SIval[30];
//	//	Kval[19]=   PHTval[18]*SIval[1]+PHTval[19]*SIval[7]+PHTval[20]*SIval[13]+PHTval[21]*SIval[19]+PHTval[22]*SIval[25]+PHTval[23]*SIval[31];
//	//	Kval[20]=   PHTval[18]*SIval[2]+PHTval[19]*SIval[8]+PHTval[20]*SIval[14]+PHTval[21]*SIval[20]+PHTval[22]*SIval[26]+PHTval[23]*SIval[32];
//	//	Kval[21]=   PHTval[18]*SIval[3]+PHTval[19]*SIval[9]+PHTval[20]*SIval[15]+PHTval[21]*SIval[21]+PHTval[22]*SIval[27]+PHTval[23]*SIval[33];
//	//	Kval[22]=   PHTval[18]*SIval[4]+PHTval[19]*SIval[10]+PHTval[20]*SIval[16]+PHTval[21]*SIval[22]+PHTval[22]*SIval[28]+PHTval[23]*SIval[34];
//	//	Kval[23]=   PHTval[18]*SIval[5]+PHTval[19]*SIval[11]+PHTval[20]*SIval[17]+PHTval[21]*SIval[23]+PHTval[22]*SIval[29]+PHTval[23]*SIval[35];
//	//	//end K
//	//
//	//
//	//	//KTval 4x6
//	//	KTval[0]=  Kval[0] ; KTval[1]=  Kval[6];  KTval[2]= Kval[12] ; KTval[3]= Kval[18] ;//r1
//	//	KTval[4]=  Kval[1] ; KTval[5]=  Kval[7];  KTval[6]= Kval[13] ; KTval[7]= Kval[19] ;//r2
//	//	KTval[8]=  Kval[2] ; KTval[9]=  Kval[8];  KTval[10]=Kval[14] ; KTval[11]=Kval[20] ;//r3
//	//	KTval[12]= Kval[3] ; KTval[13]= Kval[9];  KTval[14]=Kval[15] ; KTval[15]=Kval[21] ;//r4
//	//	KTval[16]= Kval[4] ; KTval[17]= Kval[10]; KTval[18]=Kval[16] ; KTval[19]=Kval[22] ;//r5
//	//	KTval[20]= Kval[5] ; KTval[21]= Kval[11]; KTval[22]=Kval[17] ; KTval[23]=Kval[23] ; //r6
//	//	//end KTval
//	//	/* end calculation of K */
//	//
//	//
//	//
//	//	////	/* calculation of Er */
//	//	//Erval  6x1
//	//	Erval[0]= yval[0]-yhatval[0];//r1
//	//	Erval[1]= yval[1]-yhatval[1];//r2
//	//	Erval[2]= yval[2]-yhatval[2];//r3
//	//	Erval[3]= yval[3]-yhatval[3];//r4
//	//	Erval[4]= yval[4]-yhatval[4];//r5
//	//	Erval[5]= yval[5]-yhatval[5];//r6
//	//	//end Erval  6x1
//	//	/* end calculation of Er */
//	//
//	//
//	//
//	//	/* calculation  q update */
//	//	//KErval  4x1
//	//	KErval[0]=   Kval[0]*Erval[0]+Kval[1]*Erval[1]+Kval[2]*Erval[2]+Kval[3]*Erval[3]+Kval[4]*Erval[4]+Kval[5]*Erval[5]; //r1
//	//	KErval[1]=   Kval[6]*Erval[0]+Kval[7]*Erval[1]+Kval[8]*Erval[2]+Kval[9]*Erval[3]+Kval[10]*Erval[4]+Kval[11]*Erval[5]; //r2
//	//	KErval[2]=   Kval[12]*Erval[0]+Kval[13]*Erval[1]+Kval[14]*Erval[2]+Kval[15]*Erval[3]+Kval[16]*Erval[4]+Kval[17]*Erval[5]; //r3
//	//	KErval[3]=   Kval[18]*Erval[0]+Kval[19]*Erval[1]+Kval[20]*Erval[2]+Kval[21]*Erval[3]+Kval[22]*Erval[4]+Kval[23]*Erval[5]; //r4
//	//	//end KErval
//	//
//	//
//	//	//qtildeval  4x1
//	//	qtildeval[0]= qt[0]+KErval[0];//r1
//	//	qtildeval[1]= qt[1]+KErval[1];//r2
//	//	qtildeval[2]= qt[2]+KErval[2];//r3
//	//	qtildeval[3]= qt[3]+KErval[3];//r4
//	//	//end qtildeval
//	//	/*end  calculation  q update */
//	//
//	//
//	//	/* calculation of Pt update */
//	//	//KS
//	//	//r1
//	//	KSval[0]=   Kval[0]*Sval[0]+Kval[1]*Sval[6]+Kval[2]*Sval[12]+Kval[3]*Sval[18]+Kval[4]*Sval[24]+Kval[5]*Sval[30];
//	//	KSval[1]=   Kval[0]*Sval[1]+Kval[1]*Sval[7]+Kval[2]*Sval[13]+Kval[3]*Sval[19]+Kval[4]*Sval[25]+Kval[5]*Sval[31];
//	//	KSval[2]=   Kval[0]*Sval[2]+Kval[1]*Sval[8]+Kval[2]*Sval[14]+Kval[3]*Sval[20]+Kval[4]*Sval[26]+Kval[5]*Sval[32];
//	//	KSval[3]=   Kval[0]*Sval[3]+Kval[1]*Sval[9]+Kval[2]*Sval[15]+Kval[3]*Sval[21]+Kval[4]*Sval[27]+Kval[5]*Sval[33];
//	//	KSval[4]=   Kval[0]*Sval[4]+Kval[1]*Sval[10]+Kval[2]*Sval[16]+Kval[3]*Sval[22]+Kval[4]*Sval[28]+Kval[5]*Sval[34];
//	//	KSval[5]=   Kval[0]*Sval[5]+Kval[1]*Sval[11]+Kval[2]*Sval[17]+Kval[3]*Sval[23]+Kval[4]*Sval[29]+Kval[5]*Sval[35];
//	//	//r2
//	//	KSval[6]=   Kval[6]*Sval[0]+Kval[7]*Sval[6]+Kval[8]*Sval[12]+Kval[9]*Sval[18]+Kval[10]*Sval[24]+Kval[11]*Sval[30];
//	//	KSval[7]=   Kval[6]*Sval[1]+Kval[7]*Sval[7]+Kval[8]*Sval[13]+Kval[9]*Sval[19]+Kval[10]*Sval[25]+Kval[11]*Sval[31];
//	//	KSval[8]=   Kval[6]*Sval[2]+Kval[7]*Sval[8]+Kval[8]*Sval[14]+Kval[9]*Sval[20]+Kval[10]*Sval[26]+Kval[11]*Sval[32];
//	//	KSval[9]=   Kval[6]*Sval[3]+Kval[7]*Sval[9]+Kval[8]*Sval[15]+Kval[9]*Sval[21]+Kval[10]*Sval[27]+Kval[11]*Sval[33];
//	//	KSval[10]=   Kval[6]*Sval[4]+Kval[7]*Sval[10]+Kval[8]*Sval[16]+Kval[9]*Sval[22]+Kval[10]*Sval[28]+Kval[11]*Sval[34];
//	//	KSval[11]=   Kval[6]*Sval[5]+Kval[7]*Sval[11]+Kval[8]*Sval[17]+Kval[9]*Sval[23]+Kval[10]*Sval[29]+Kval[11]*Sval[35];
//	//	//r3
//	//	KSval[12]=   Kval[12]*Sval[0]+Kval[13]*Sval[6]+Kval[14]*Sval[12]+Kval[15]*Sval[18]+Kval[16]*Sval[24]+Kval[17]*Sval[30];
//	//	KSval[13]=   Kval[12]*Sval[1]+Kval[13]*Sval[7]+Kval[14]*Sval[13]+Kval[15]*Sval[19]+Kval[16]*Sval[25]+Kval[17]*Sval[31];
//	//	KSval[14]=   Kval[12]*Sval[2]+Kval[13]*Sval[8]+Kval[14]*Sval[14]+Kval[15]*Sval[20]+Kval[16]*Sval[26]+Kval[17]*Sval[32];
//	//	KSval[15]=   Kval[12]*Sval[3]+Kval[13]*Sval[9]+Kval[14]*Sval[15]+Kval[15]*Sval[21]+Kval[16]*Sval[27]+Kval[17]*Sval[33];
//	//	KSval[16]=   Kval[12]*Sval[4]+Kval[13]*Sval[10]+Kval[14]*Sval[16]+Kval[15]*Sval[22]+Kval[16]*Sval[28]+Kval[17]*Sval[34];
//	//	KSval[17]=   Kval[12]*Sval[5]+Kval[13]*Sval[11]+Kval[14]*Sval[17]+Kval[15]*Sval[23]+Kval[16]*Sval[29]+Kval[17]*Sval[35];
//	//	//r4
//	//	KSval[18]=   Kval[18]*Sval[0]+Kval[19]*Sval[6]+Kval[20]*Sval[12]+Kval[21]*Sval[18]+Kval[22]*Sval[24]+Kval[23]*Sval[30];
//	//	KSval[19]=   Kval[18]*Sval[1]+Kval[19]*Sval[7]+Kval[20]*Sval[13]+Kval[21]*Sval[19]+Kval[22]*Sval[25]+Kval[23]*Sval[31];
//	//	KSval[20]=   Kval[18]*Sval[2]+Kval[19]*Sval[8]+Kval[20]*Sval[14]+Kval[21]*Sval[20]+Kval[22]*Sval[26]+Kval[23]*Sval[32];
//	//	KSval[21]=   Kval[18]*Sval[3]+Kval[19]*Sval[9]+Kval[20]*Sval[15]+Kval[21]*Sval[21]+Kval[22]*Sval[27]+Kval[23]*Sval[33];
//	//	KSval[22]=   Kval[18]*Sval[4]+Kval[19]*Sval[10]+Kval[20]*Sval[16]+Kval[21]*Sval[22]+Kval[22]*Sval[28]+Kval[23]*Sval[34];
//	//	KSval[23]=   Kval[18]*Sval[5]+Kval[19]*Sval[11]+Kval[20]*Sval[17]+Kval[21]*Sval[23]+Kval[22]*Sval[29]+Kval[23]*Sval[35];
//	//	//end KS
//	//
//	//	//KSKTval 4x4
//	//	//r1
//	//	KSKTval[0]=   KSval[0]*KTval[0]+KSval[1]*KTval[4]+KSval[2]*KTval[8]+KSval[3]*KTval[12]+KSval[4]*KTval[16]+KSval[5]*KTval[20];
//	//	KSKTval[1]=   KSval[0]*KTval[1]+KSval[1]*KTval[5]+KSval[2]*KTval[9]+KSval[3]*KTval[13]+KSval[4]*KTval[17]+KSval[5]*KTval[21];
//	//	KSKTval[2]=   KSval[0]*KTval[2]+KSval[1]*KTval[6]+KSval[2]*KTval[10]+KSval[3]*KTval[14]+KSval[4]*KTval[18]+KSval[5]*KTval[22];
//	//	KSKTval[3]=   KSval[0]*KTval[3]+KSval[1]*KTval[7]+KSval[2]*KTval[11]+KSval[3]*KTval[15]+KSval[4]*KTval[19]+KSval[5]*KTval[23];
//	//	//r2
//	//	KSKTval[4]=   KSval[6]*KTval[0]+KSval[7]*KTval[4]+KSval[8]*KTval[8]+KSval[9]*KTval[12]+KSval[10]*KTval[16]+KSval[11]*KTval[20];
//	//	KSKTval[5]=   KSval[6]*KTval[1]+KSval[7]*KTval[5]+KSval[8]*KTval[9]+KSval[9]*KTval[13]+KSval[10]*KTval[17]+KSval[11]*KTval[21];
//	//	KSKTval[6]=   KSval[6]*KTval[2]+KSval[7]*KTval[6]+KSval[8]*KTval[10]+KSval[9]*KTval[14]+KSval[10]*KTval[18]+KSval[11]*KTval[22];
//	//	KSKTval[7]=   KSval[6]*KTval[3]+KSval[7]*KTval[7]+KSval[8]*KTval[11]+KSval[9]*KTval[15]+KSval[10]*KTval[19]+KSval[11]*KTval[23];
//	//	//r3
//	//	KSKTval[8]=   KSval[12]*KTval[0]+KSval[13]*KTval[4]+KSval[14]*KTval[8]+KSval[15]*KTval[12]+KSval[16]*KTval[16]+KSval[17]*KTval[20];
//	//	KSKTval[9]=   KSval[12]*KTval[1]+KSval[13]*KTval[5]+KSval[14]*KTval[9]+KSval[15]*KTval[13]+KSval[16]*KTval[17]+KSval[17]*KTval[21];
//	//	KSKTval[10]=  KSval[12]*KTval[2]+KSval[13]*KTval[6]+KSval[14]*KTval[10]+KSval[15]*KTval[14]+KSval[16]*KTval[18]+KSval[17]*KTval[22];
//	//	KSKTval[11]=  KSval[12]*KTval[3]+KSval[13]*KTval[7]+KSval[14]*KTval[11]+KSval[15]*KTval[15]+KSval[16]*KTval[19]+KSval[17]*KTval[23];
//	//	//r4
//	//	KSKTval[12]=  KSval[18]*KTval[0]+KSval[19]*KTval[4]+KSval[20]*KTval[8]+KSval[21]*KTval[12]+KSval[22]*KTval[16]+KSval[23]*KTval[20];
//	//	KSKTval[13]=  KSval[18]*KTval[1]+KSval[19]*KTval[5]+KSval[20]*KTval[9]+KSval[21]*KTval[13]+KSval[22]*KTval[17]+KSval[23]*KTval[21];
//	//	KSKTval[14]=  KSval[18]*KTval[2]+KSval[19]*KTval[6]+KSval[20]*KTval[10]+KSval[21]*KTval[14]+KSval[22]*KTval[18]+KSval[23]*KTval[22];
//	//	KSKTval[15]=  KSval[18]*KTval[3]+KSval[19]*KTval[7]+KSval[20]*KTval[11]+KSval[21]*KTval[15]+KSval[22]*KTval[19]+KSval[23]*KTval[23];
//	//	// end KSKTval
//	//
//	//	//Ptildeval 4x4
//	//	Ptildeval[0]=  Ptval[0]-KSKTval[0];      Ptildeval[1]=  Ptval[1]-KSKTval[1];     Ptildeval[2]=  Ptval[2]-KSKTval[2];     Ptildeval[3]=  Ptval[3]-KSKTval[3];//r1
//	//	Ptildeval[4]=  Ptval[4]-KSKTval[4];      Ptildeval[5]=  Ptval[5]-KSKTval[5];     Ptildeval[6]=  Ptval[6]-KSKTval[6];     Ptildeval[7]=  Ptval[7]-KSKTval[7];//r2
//	//	Ptildeval[8]=  Ptval[8]-KSKTval[8];      Ptildeval[9]=  Ptval[9]-KSKTval[9];     Ptildeval[10]= Ptval[10]-KSKTval[10];   Ptildeval[11]= Ptval[11]-KSKTval[11];//r3
//	//	Ptildeval[12]= Ptval[12]-KSKTval[12];    Ptildeval[13]= Ptval[13]-KSKTval[13];   Ptildeval[14]= Ptval[14]-KSKTval[14];   Ptildeval[15]= Ptval[15]-KSKTval[15];//r4
//	//	//end Ptildeval
//	//	/*end  calculation of Pt update */
//	//
//	//
//	//	/* renormalize q */
//	//	qtildenorm = sqrt(qtildeval[0]*qtildeval[0]+qtildeval[1]*qtildeval[1]+qtildeval[2]*qtildeval[2]+qtildeval[3]*qtildeval[3]);
//	//	// new quaternion state
//	//	q[0]=(float32_t)(qtildeval[0]/qtildenorm);//r1
//	//	q[1]=(float32_t)(qtildeval[1]/qtildenorm);//r2
//	//	q[2]=(float32_t)(qtildeval[2]/qtildenorm);//r3
//	//	q[3]=(float32_t)(qtildeval[3]/qtildenorm);//r4
//	//	/* end renormalize q */
//	//
//	//
//	//	/* renormalize P */
//	//	qtildenorm3 = (float32_t)(1/(qtildenorm*qtildenorm*qtildenorm));
//	//	//Jval 4x4
//	//	//r1
//	//	Jval[0]=qtildenorm3*qtildeval[0]*qtildeval[0] ; Jval[1]=qtildenorm3*qtildeval[0]*qtildeval[1] ; Jval[2]=qtildenorm3*qtildeval[0]*qtildeval[2] ; Jval[3]=qtildenorm3*qtildeval[0]*qtildeval[3] ;
//	//	//r2
//	//	Jval[4]=qtildenorm3*qtildeval[1]*qtildeval[0] ; Jval[5]=qtildenorm3*qtildeval[1]*qtildeval[1] ; Jval[6]=qtildenorm3*qtildeval[1]*qtildeval[2] ; Jval[7]=qtildenorm3*qtildeval[1]*qtildeval[3] ;
//	//	//r3
//	//	Jval[8]=qtildenorm3*qtildeval[2]*qtildeval[0] ; Jval[9]=qtildenorm3*qtildeval[2]*qtildeval[1] ; Jval[10]=qtildenorm3*qtildeval[2]*qtildeval[2] ; Jval[11]=qtildenorm3*qtildeval[2]*qtildeval[3] ;
//	//	//r4
//	//	Jval[12]=qtildenorm3*qtildeval[3]*qtildeval[0] ; Jval[13]=qtildenorm3*qtildeval[3]*qtildeval[1] ; Jval[14]=qtildenorm3*qtildeval[3]*qtildeval[2] ; Jval[15]=qtildenorm3*qtildeval[3]*qtildeval[3] ;
//	//	//end Jval
//	//
//	//
//	//
//	//	//JPtildeval 4x4
//	//	//r1
//	//	JPtildeval[0]= Jval[0]*Ptildeval[0]+Jval[1]*Ptildeval[4]+Jval[2]*Ptildeval[8]+Jval[3]*Ptildeval[12]; JPtildeval[1]= Jval[0]*Ptildeval[1]+Jval[1]*Ptildeval[5]+Jval[2]*Ptildeval[9]+Jval[3]*Ptildeval[13];
//	//	JPtildeval[2]= Jval[0]*Ptildeval[2]+Jval[1]*Ptildeval[6]+Jval[2]*Ptildeval[10]+Jval[3]*Ptildeval[14]; JPtildeval[3]= Jval[0]*Ptildeval[3]+Jval[1]*Ptildeval[7]+Jval[2]*Ptildeval[11]+Jval[3]*Ptildeval[15];
//	//	//r2
//	//	JPtildeval[4]= Jval[4]*Ptildeval[0]+Jval[5]*Ptildeval[4]+Jval[6]*Ptildeval[8]+Jval[7]*Ptildeval[12]; JPtildeval[5]= Jval[4]*Ptildeval[1]+Jval[5]*Ptildeval[5]+Jval[6]*Ptildeval[9]+Jval[7]*Ptildeval[13];
//	//	JPtildeval[6]= Jval[4]*Ptildeval[2]+Jval[5]*Ptildeval[6]+Jval[6]*Ptildeval[10]+Jval[7]*Ptildeval[14]; JPtildeval[7]= Jval[4]*Ptildeval[3]+Jval[5]*Ptildeval[7]+Jval[6]*Ptildeval[11]+Jval[7]*Ptildeval[15];
//	//	//r3
//	//	JPtildeval[8]= Jval[8]*Ptildeval[0]+Jval[9]*Ptildeval[4]+Jval[10]*Ptildeval[8]+Jval[11]*Ptildeval[12]; JPtildeval[9]= Jval[8]*Ptildeval[1]+Jval[9]*Ptildeval[5]+Jval[10]*Ptildeval[9]+Jval[11]*Ptildeval[13];
//	//	JPtildeval[10]= Jval[8]*Ptildeval[2]+Jval[9]*Ptildeval[6]+Jval[10]*Ptildeval[10]+Jval[11]*Ptildeval[14]; JPtildeval[11]= Jval[8]*Ptildeval[3]+Jval[9]*Ptildeval[7]+Jval[10]*Ptildeval[11]+Jval[11]*Ptildeval[15];
//	//	//r4
//	//	JPtildeval[12]= Jval[12]*Ptildeval[0]+Jval[13]*Ptildeval[4]+Jval[14]*Ptildeval[8]+Jval[15]*Ptildeval[12]; JPtildeval[13]= Jval[12]*Ptildeval[1]+Jval[13]*Ptildeval[5]+Jval[14]*Ptildeval[9]+Jval[15]*Ptildeval[13];
//	//	JPtildeval[14]= Jval[12]*Ptildeval[2]+Jval[13]*Ptildeval[6]+Jval[14]*Ptildeval[10]+Jval[15]*Ptildeval[14]; JPtildeval[15]= Jval[12]*Ptildeval[3]+Jval[13]*Ptildeval[7]+Jval[14]*Ptildeval[11]+Jval[15]*Ptildeval[15];
//	//	//end JPtildeval
//	//
//	//
//	//	//4x4 PTEMPval = [JPtildeval]*[transpose of Jval]
//	//	//r1
//	//	PTEMPval[0]= JPtildeval[0]*Jval[0]+JPtildeval[1]*Jval[1]+JPtildeval[2]*Jval[2]+JPtildeval[3]*Jval[3]; PTEMPval[1]= JPtildeval[0]*Jval[4]+JPtildeval[1]*Jval[5]+JPtildeval[2]*Jval[6]+JPtildeval[3]*Jval[7];
//	//	PTEMPval[2]= JPtildeval[0]*Jval[8]+JPtildeval[1]*Jval[9]+JPtildeval[2]*Jval[10]+JPtildeval[3]*Jval[11]; PTEMPval[3]= JPtildeval[0]*Jval[12]+JPtildeval[1]*Jval[13]+JPtildeval[2]*Jval[14]+JPtildeval[3]*Jval[15];
//	//	//r2
//	//	PTEMPval[4]= JPtildeval[0]*Jval[0]+JPtildeval[1]*Jval[1]+JPtildeval[2]*Jval[2]+JPtildeval[3]*Jval[3]; PTEMPval[5]= JPtildeval[0]*Jval[4]+JPtildeval[1]*Jval[5]+JPtildeval[2]*Jval[6]+JPtildeval[3]*Jval[7];
//	//	PTEMPval[6]= JPtildeval[0]*Jval[8]+JPtildeval[1]*Jval[9]+JPtildeval[2]*Jval[10]+JPtildeval[3]*Jval[11]; PTEMPval[7]= JPtildeval[0]*Jval[12]+JPtildeval[1]*Jval[13]+JPtildeval[2]*Jval[14]+JPtildeval[3]*Jval[15];
//	//	//r3
//	//	PTEMPval[8]= JPtildeval[0]*Jval[0]+JPtildeval[1]*Jval[1]+JPtildeval[2]*Jval[2]+JPtildeval[3]*Jval[3]; PTEMPval[9]= JPtildeval[0]*Jval[4]+JPtildeval[1]*Jval[5]+JPtildeval[2]*Jval[6]+JPtildeval[3]*Jval[7];
//	//	PTEMPval[10]= JPtildeval[0]*Jval[8]+JPtildeval[1]*Jval[9]+JPtildeval[2]*Jval[10]+JPtildeval[3]*Jval[11]; PTEMPval[11]= JPtildeval[0]*Jval[12]+JPtildeval[1]*Jval[13]+JPtildeval[2]*Jval[14]+JPtildeval[3]*Jval[15];
//	//	//r4
//	//	PTEMPval[12]= JPtildeval[0]*Jval[0]+JPtildeval[1]*Jval[1]+JPtildeval[2]*Jval[2]+JPtildeval[3]*Jval[3]; PTEMPval[13]= JPtildeval[0]*Jval[4]+JPtildeval[1]*Jval[5]+JPtildeval[2]*Jval[6]+JPtildeval[3]*Jval[7];
//	//	PTEMPval[14]= JPtildeval[0]*Jval[8]+JPtildeval[1]*Jval[9]+JPtildeval[2]*Jval[10]+JPtildeval[3]*Jval[11]; PTEMPval[15]= JPtildeval[0]*Jval[12]+JPtildeval[1]*Jval[13]+JPtildeval[2]*Jval[14]+JPtildeval[3]*Jval[15];
//	//	//end PTEMPval
//	//	//only first row was changing meaningfully
//	//	Pval[0]=PTEMPval[0];Pval[1]=PTEMPval[1];Pval[2]=PTEMPval[2];Pval[3]=PTEMPval[3];//// new system covariance P
//	//
//	//	//Pval[5]=PTEMPval[5];Pval[10]=PTEMPval[10];Pval[15]=PTEMPval[15];//// new system covariance P
//	//	/* end renormalize P */
//
//
//
//}
//













