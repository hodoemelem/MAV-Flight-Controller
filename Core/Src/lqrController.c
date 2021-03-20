/*
 * lqrController.c
 *
 *  Created on: 26 Jan 2021
 *      Author: HENRY ODOEMELEM
 */


#include "lqrController.h"
//F=c*n^2; thrust
//M=b*n^2; thrust
float32_t mass = 1.280;//Kg
float32_t c = 1.8409E-4;//lift constant
float32_t b = 2.6788E-6;//drag constant
float32_t L = 0.1167;// L= armlength*cos(45), L is distance for motors to quadrotor center in X configuration.
float32_t g = 9.81;//acc. due to gravity
float32_t rps2Hover; //the square rps for hover condition
float32_t hoverThrust;//thrust needed for hover

float32_t Mmval[16] = {0};
float32_t Mmival[16] = {0};
float32_t errorval[8] = {0};
float32_t Kderrorval[4]= {0};
float32_t Uval[4] = {0};
float32_t Ussval[4] = {0};
float32_t rps2val[4] = {0};
float32_t ThrustMin = 2.38;//... pwm count
float32_t ThrustMax = 3.90;//... pwm count



//working gain


////working gain
//float32_t Kdval[16] = {
//
//		0,               0,                  0,                   0,
//		0.006,           0.0004,           3.7983E-16,         2.2767E-17,
//		-3.9947E-16,    1.8212E-17,         0.006,             0.0004,
//		0,                0,                 0,                   0
//
//};


arm_status status;
arm_matrix_instance_f32 Mm;		/* Matrix A Instance */
arm_matrix_instance_f32 Mmi;		/* Matrix A Instance */
arm_matrix_instance_f32 Kd;		/* Matrix A Instance */
arm_matrix_instance_f32 error;		/* Matrix A Instance */
arm_matrix_instance_f32 Kderror;
arm_matrix_instance_f32 U;		/* Matrix A Instance */
arm_matrix_instance_f32 Uss;		/* Matrix A Instance */
arm_matrix_instance_f32 rps2;		/* Matrix A Instance */
void lqr_init()
{
	hoverThrust  = (mass*g)/4.0;

	////
	Kdval[6]= pzGain;  //posz gain
	Kdval[7]= vzGain;  //vz gain
	Kdval[8]= angleGain;  //roll gain
	Kdval[9]= rateGain;  //roll rate gain
	Kdval[18]= angleGain; //pitch gain
	Kdval[19]= rateGain; //pitch rate gain
	Kdval[28]= angleGain; //yaw  gain
	Kdval[29]= rateGain; //yaw rate gain
	//////////////

	srcRows = 4;
	srcColumns = 4;
	arm_mat_init_f32(&Mm, srcRows, srcColumns, (float32_t *)Mmval);
	srcRows = 4;
	srcColumns = 4;
	arm_mat_init_f32(&Mmi, srcRows, srcColumns, (float32_t *)Mmival);
	srcRows =4;
	srcColumns = 8;
	arm_mat_init_f32(&Kd, srcRows, srcColumns, (float32_t *)Kdval);
	srcRows = 8;
	srcColumns = 1;
	arm_mat_init_f32(&error, srcRows, srcColumns, (float32_t *)errorval);
	srcRows = 4;
	srcColumns = 1;
	arm_mat_init_f32(&U, srcRows, srcColumns, (float32_t *)Uval);
	srcRows = 4;
	srcColumns = 1;
	arm_mat_init_f32(&rps2, srcRows, srcColumns, (float32_t *)rps2val);

	//motor mixing matirx Mmval
	Mmval[0] = c;  Mmval[1]= c;   Mmval[2]= c;   Mmval[3]= c;
	Mmval[4] = L*c;  Mmval[5]= -L*c;   Mmval[6]= -L*c;   Mmval[7]= L*c;
	Mmval[8]= -L*c;   Mmval[9]= -L*c;   Mmval[10]= L*c;   Mmval[11]= L*c;
	Mmval[12]= b;   Mmval[13]= -b;   Mmval[14]= b;   Mmval[15]= -b;

	//inverse of motor mixing matrix Mmival
	status = arm_mat_inverse_f32(&Mm, &Mmi);
}

void lqr(double roll,double rollRate, double pitch,double pitchRate,double yaw,double yawRate,double Z,double ZRate,
		double rolld,double rollRated, double pitchd,double pitchRated,double yawd,double yawRated,double Zd,double ZRated){





	errorval[0] = (rolld - roll);
	errorval[1] = (rollRated - rollRate);
	errorval[2] = (pitchd - pitch);
	errorval[3] = (pitchRated - pitchRate);
	errorval[4] = (yawd - yaw);
	errorval[5] = (yawRated - yawRate);
	errorval[6] = (Zd - Z);
	errorval[7] = (ZRated - ZRate);



	status = arm_mat_mult_f32(&Kd, &error, &U);
	status = arm_mat_mult_f32(&Mmi, &U, &rps2);

	//return to hoverthrust when rps2 form lqr is zero, that is the desired state is reached.
	F1= hoverThrust + c*rps2val[0];
	F2= hoverThrust + c*rps2val[1];
	F3= hoverThrust + c*rps2val[2];
	F4= hoverThrust + c*rps2val[3];

	//	//thrust clipping
	//	F1 =  fmax(F1, ThrustMin);
	//	F1 =  fmin(F1, ThrustMax);
	//
	//	F2 =  fmax(F2, ThrustMin);
	//	F2=   fmin(F2, ThrustMax);
	//
	//	F3 =  fmax(F3, ThrustMin);
	//	F3 =  fmin(F3, ThrustMax);
	//
	//	F4 =  fmax(F4, ThrustMin);
	//	F4 =  fmin(F4, ThrustMax);





	//    //rps values
	//   	n1 = sqrt(F1/c);
	//   	n2 = sqrt(F2/c);
	//   	n3 = sqrt(F3/c);
	//   	n4 = sqrt(F4/c);



}



///*
// * lqrController.c
// *
// *  Created on: 26 Jan 2021
// *      Author: HENRY ODOEMELEM
// */
//
//
//#include "lqrController.h"
////F=c*n^2; thrust
////M=b*n^2; thrust
//float32_t mass = 1.295;//Kg
//float32_t c = 1.8409E-4;//lift constant
//float32_t b = 2.6788E-6;//drag constant
//float32_t L = 0.1167;// L= armlength*cos(45), L is distance for motors to quadrotor center in X configuration.
//float32_t g = 9.81;//acc. due to gravity
//float32_t rps2Hover; //the square rps for hover condition
//float32_t hoverThrust;//thrust needed for hover
//
//float32_t Mmval[16] = {0};
//float32_t Mmival[16] = {0};
//float32_t errorval[4] = {0};
//float32_t Kderrorval[4]= {0};
//float32_t Uval[4] = {0};
//float32_t Ussval[4] = {0};
//float32_t rps2val[4] = {0};
//float32_t ThrustMin = 2.65;//... pwm count
//float32_t ThrustMax = 3.70;//... pwm count
//
//
//
////working gain
//
//
//////working gain
////float32_t Kdval[16] = {
////
////		0,               0,                  0,                   0,
////		0.006,           0.0004,           3.7983E-16,         2.2767E-17,
////		-3.9947E-16,    1.8212E-17,         0.006,             0.0004,
////		0,                0,                 0,                   0
////
////};
//
//
//arm_status status;
//arm_matrix_instance_f32 Mm;		/* Matrix A Instance */
//arm_matrix_instance_f32 Mmi;		/* Matrix A Instance */
//arm_matrix_instance_f32 Kd;		/* Matrix A Instance */
//arm_matrix_instance_f32 error;		/* Matrix A Instance */
//arm_matrix_instance_f32 Kderror;
//arm_matrix_instance_f32 U;		/* Matrix A Instance */
//arm_matrix_instance_f32 Uss;		/* Matrix A Instance */
//arm_matrix_instance_f32 rps2;		/* Matrix A Instance */
//void lqr_init()
//{
//	hoverThrust  = (mass*g)/4.0;
//	//Ussval[0] = (mass*g);
//	Kdval[4]= angleGain;  //roll gain
//	Kdval[5]= rateGain;  //roll rate gain
//	Kdval[10]= angleGain; //pitch gain
//	Kdval[11]= rateGain; //pitch rate gain
//
//	srcRows = 4;
//	srcColumns = 4;
//	arm_mat_init_f32(&Mm, srcRows, srcColumns, (float32_t *)Mmval);
//	srcRows = 4;
//	srcColumns = 4;
//	arm_mat_init_f32(&Mmi, srcRows, srcColumns, (float32_t *)Mmival);
//	srcRows = 4;
//	srcColumns = 4;
//	arm_mat_init_f32(&Kd, srcRows, srcColumns, (float32_t *)Kdval);
//	srcRows = 4;
//	srcColumns = 1;
//	arm_mat_init_f32(&error, srcRows, srcColumns, (float32_t *)errorval);
//	srcRows = 4;
//	srcColumns = 1;
//	arm_mat_init_f32(&Kderror, srcRows, srcColumns, (float32_t *)Kderrorval);
//	srcRows = 4;
//	srcColumns = 1;
//	arm_mat_init_f32(&U, srcRows, srcColumns, (float32_t *)Uval);
//	srcRows = 4;
//	srcColumns = 1;
//	arm_mat_init_f32(&Uss, srcRows, srcColumns, (float32_t *)Ussval);
//	srcRows = 4;
//	srcColumns = 1;
//	arm_mat_init_f32(&rps2, srcRows, srcColumns, (float32_t *)rps2val);
//
//	//motor mixing matirx Mmval
//	Mmval[0] = c;  Mmval[1]= c;   Mmval[2]= c;   Mmval[3]= c;
//	Mmval[4] = L*c;  Mmval[5]= -L*c;   Mmval[6]= -L*c;   Mmval[7]= L*c;
//	Mmval[8]= -L*c;   Mmval[9]= -L*c;   Mmval[10]= L*c;   Mmval[11]= L*c;
//	Mmval[12]= b;   Mmval[13]= -b;   Mmval[14]= b;   Mmval[15]= -b;
//
//	//inverse of motor mixing matrix Mmival
//	status = arm_mat_inverse_f32(&Mm, &Mmi);
//}
//
//void lqr(double roll,double rollRate, double pitch,double pitchRate,double rolld,double rollRated, double pitchd,double pitchRated){
//
//
//
//
//
//	errorval[0] = (rolld - roll);
//	errorval[1] = (rollRated - rollRate);
//	errorval[2] = (pitchd - pitch);
//	errorval[3] = (pitchRated - pitchRate);
//
////	if( fabs(errorval[2])<=1){
////
////		errorval[2] = 0;
////		errorval[3] =0;
////	}
////	if(fabs(errorval[0])<=1){
////
////		 errorval[0] = 0;
////		 errorval[1]=0;
////
////	}
//
//	//	status = arm_mat_mult_f32(&Kd, &error, &Kderror);
//	//	status = arm_mat_add_f32(&Kderror, &Uss, &U);
//	//	status = arm_mat_mult_f32(&Mmi, &U, &rps2);
//
//	status = arm_mat_mult_f32(&Kd, &error, &U);
//	status = arm_mat_mult_f32(&Mmi, &U, &rps2);
//
//	//return to hoverthrust when rps2 form lqr is zero, that is the desired state is reached.
//	F1= hoverThrust + c*rps2val[0];
//	F2= hoverThrust + c*rps2val[1];
//	F3= hoverThrust + c*rps2val[2];
//	F4= hoverThrust + c*rps2val[3];
//
//	//thrust clipping
//	F1 =  fmax(F1, ThrustMin);
//	F1 =  fmin(F1, ThrustMax);
//
//	F2 =  fmax(F2, ThrustMin);
//	F2=   fmin(F2, ThrustMax);
//
//	F3 =  fmax(F3, ThrustMin);
//	F3 =  fmin(F3, ThrustMax);
//
//	F4 =  fmax(F4, ThrustMin);
//	F4 =  fmin(F4, ThrustMax);
//
//
//
//
//
//	//    //rps values
//	//   	n1 = sqrt(F1/c);
//	//   	n2 = sqrt(F2/c);
//	//   	n3 = sqrt(F3/c);
//	//   	n4 = sqrt(F4/c);
//
//
//
//}
