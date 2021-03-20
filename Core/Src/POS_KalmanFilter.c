/*
 * POS_KalmanFilter.c
 *
 *  Created on: 26 Dec 2020
 *      Author: HENRY ODOEMELEM
 */




#include "POS_KalmanFilter.h"


double hdeltaT;//////////////////////////////////////////////////////////
void  init_POS_KF(double hdt,double processScale,double hPn,double hRn,double initX,double initY,double initZ)
{


	///For POS
	srcRows = 9;
	srcColumns = 9;
	arm_mat_init_f32(&hF, srcRows, srcColumns, (float32_t *)hFval);
	srcRows = 9;
	srcColumns = 9;
	arm_mat_init_f32(&hFT, srcRows, srcColumns, hFTval);
	srcRows = 9;
	srcColumns = 1;
	arm_mat_init_f32(&hFx, srcRows, srcColumns, (float32_t *)hFxval);

	srcRows = 9;
	srcColumns = 1;
	arm_mat_init_f32(&hG, srcRows, srcColumns, (float32_t *)hGval);
	srcRows = 1;
	srcColumns = 9;
	arm_mat_init_f32(&hGT, srcRows, srcColumns, (float32_t *)hGTval);
	srcRows = 9;
	srcColumns = 9;
	arm_mat_init_f32(&hGGT, srcRows, srcColumns, (float32_t *)hGGTval);

	srcRows = 3;
	srcColumns = 9;
	arm_mat_init_f32(&hH, srcRows, srcColumns, (float32_t *)hHval);
	srcRows = 9;
	srcColumns = 3;
	arm_mat_init_f32(&hHT, srcRows, srcColumns, hHTval);

	srcRows = 3;
	srcColumns = 1;
	arm_mat_init_f32(&hyhat, srcRows, srcColumns, (float32_t *)hyhatval);
	srcRows = 3;
	srcColumns = 1;
	arm_mat_init_f32(&hy, srcRows, srcColumns, (float32_t *)hyval);

	srcRows = 9;
	srcColumns = 1;
	arm_mat_init_f32(&hx, srcRows, srcColumns, (float32_t *)hxval);
	srcRows = 9;
	srcColumns = 1;
	arm_mat_init_f32(&hxt, srcRows, srcColumns, hxtval);
	srcRows = 9;
	srcColumns = 1;
	arm_mat_init_f32(&hxtilde, srcRows, srcColumns, hxtildeval);


	srcRows = 9;
	srcColumns = 9;
	arm_mat_init_f32(&hP, srcRows, srcColumns, (float32_t *)hPval);


	srcRows = 9;
	srcColumns =9;
	arm_mat_init_f32(&hPt, srcRows, srcColumns, hPtval);
	srcRows = 9;
	srcColumns =9;
	arm_mat_init_f32(&hPtilde, srcRows, srcColumns, hPtildeval);


	srcRows = 9;
	srcColumns = 9;
	arm_mat_init_f32(&hFP, srcRows, srcColumns, hFPval);
	srcRows = 9;
	srcColumns = 9;
	arm_mat_init_f32(&hFPFT, srcRows, srcColumns, hFPFTval);

	srcRows = 9;
	srcColumns = 9;
	arm_mat_init_f32(&hQt, srcRows, srcColumns, hQtval);


	srcRows = 3;
	srcColumns = 9;
	arm_mat_init_f32(&hHP, srcRows, srcColumns, hHPval);
	srcRows = 3;
	srcColumns = 3;
	arm_mat_init_f32(&hHPHT, srcRows, srcColumns, hHPHTval);


	srcRows = 9;
	srcColumns = 3;
	arm_mat_init_f32(&hPHT, srcRows, srcColumns, hPHTval);

	srcRows = 3;
	srcColumns = 3;
	arm_mat_init_f32(&hS, srcRows, srcColumns, hSval);
	srcRows = 3;
	srcColumns = 3;
	arm_mat_init_f32(&hSI, srcRows, srcColumns, hSIval);

	srcRows = 9;
	srcColumns = 3;
	arm_mat_init_f32(&hK, srcRows, srcColumns, hKval);
	srcRows = 3;
	srcColumns = 9;
	arm_mat_init_f32(&hKT, srcRows, srcColumns, hKTval);


	srcRows = 9;
	srcColumns = 3;
	arm_mat_init_f32(&hKS, srcRows, srcColumns, hKSval);
	srcRows = 9;
	srcColumns = 9;
	arm_mat_init_f32(&hKSKT, srcRows, srcColumns, hKSKTval);


	srcRows = 3;
	srcColumns = 1;
	arm_mat_init_f32(&hEr, srcRows, srcColumns,hErval);

	srcRows = 9;
	srcColumns = 1;
	arm_mat_init_f32(&hKEr, srcRows, srcColumns, hKErval);
	srcRows = 3;
	srcColumns = 3;
	arm_mat_init_f32(&hR, srcRows, srcColumns, (float32_t *)hRval);
	///////////////////////////////////////////////////////////

	//////for POS
	hdeltaT = hdt;//125 hertz


	//Initialize pos,vel and acc.
	hxval[0] = initX; hxval[1] = 0; hxval[2] = 0;//x comp.
	hxval[3] = initY; hxval[4] =0;  hxval[5] = 0;//y comp.
	hxval[6] = initZ; hxval[7] = 0;  hxval[8] = 0;//z comp.

	hPval[0]=hPn;hPval[10]=hPn;hPval[20]=hPn;hPval[30]=hPn;
	hPval[40]=hPn;hPval[50]=hPn;hPval[60]=hPn;hPval[70]=hPn;hPval[80]=hPn;

	//How well to trust the model, high values means trust model less
	scale = processScale;//pow((12.2129)/sqrt(hdeltaT),2);//acc var;//scale factor for Qt




	// pos var
	//How well to trust the measurement
	hRval[0] = hRn;//px
	hRval[4] = hRn;//py
	hRval[8] = hRn;//pz


	//F diagonal
	hFval[0]=1;hFval[10]=1;hFval[20]=1;hFval[30]=1;hFval[40]=1;hFval[50]=1;hFval[60]=1;hFval[70]=1;hFval[80]=1;
	//F sample time comp.
	hFval[1]=hdeltaT;hFval[2]=0.5*hdeltaT*hdeltaT;hFval[11]=0.5*hdeltaT;
	hFval[31]=hdeltaT;hFval[32]=0.5*hdeltaT*hdeltaT;hFval[41]=0.5*hdeltaT;
	hFval[61]=hdeltaT;hFval[62]=0.5*hdeltaT*hdeltaT;hFval[71]=0.5*hdeltaT;

	status = arm_mat_trans_f32(&hF, &hFT);

	hGval[0]= 0.5*hdeltaT*hdeltaT;     hGval[3]=hdeltaT;
	hGval[3]=0.5*hdeltaT*hdeltaT; hGval[4]= hdeltaT;
	hGval[6]=0.5*hdeltaT*hdeltaT; hGval[7]= hdeltaT;

	status = arm_mat_trans_f32(&hG, &hGT);
	status = arm_mat_mult_f32(&hG, &hGT, &hGGT);
	status = arm_mat_scale_f32(&hGGT, scale, &hQt);

	hHval[0]=1;hHval[12]=1;hHval[24]=1;
	status = arm_mat_trans_f32(&hH, &hHT);

	//asymptotic k gain, k increase means trust measurement more
//	double posgain = 10/100.0;	double velgain =0.1/100.0;	double accgain =1/100.0;
	double posgain = (double) (1/100.0);
	double velgain = (double) (0.1/100.0);
	double accgain = (double) (0.1/100.0);
	hKval[0] = posgain;  hKval[3] = velgain; hKval[6] = accgain;
	hKval[10] = posgain; hKval[13] = velgain; hKval[16] = accgain;
	hKval[20] = posgain; hKval[23] = velgain; hKval[26] = accgain;

}

void POS_KFasymptotic(double posX,double posY,double posZ,double accXl,double accYl,double accZl,int isUpdatedFk)
{


		//raw acc
		hxval[2] = accXl;
		hxval[5] = accYl;
		hxval[8] = accZl;


		//prediction
		hxtval[0] = hxval[0] + 0.5*hdeltaT*hxval[1]  + 0.5*hdeltaT*hdeltaT*hxval[2];
		hxtval[1] = hxval[1] + 0.5*hdeltaT*hxval[2];
		hxtval[2] = hxval[2];
		hxtval[3] = hxval[3] + 0.5*hdeltaT*hxval[4]  + 0.5*hdeltaT*hdeltaT*hxval[5];
		hxtval[4] = hxval[4] + 0.5*hdeltaT*hxval[5];
		hxtval[5] = hxval[5];
		hxtval[6] = hxval[6] + 0.5*hdeltaT*hxval[7]  + 0.5*hdeltaT*hdeltaT*hxval[8];
		hxtval[7] = hxval[7] + 0.5*hdeltaT*hxval[8];
		hxtval[8] = hxval[8];

		//correction
		hErval[0] = posX - hxtval[0];
		hErval[1] = posY - hxtval[3];
		hErval[2] = posZ - hxtval[6];



		hxtildeval[0] = hxtval[0] + hKval[0]*hErval[0];
		hxtildeval[1] = hxtval[1] + hKval[3]*hErval[0];
		hxtildeval[2] = hxtval[2] + hKval[6]*hErval[0];
		hxtildeval[3] = hxtval[3] + hKval[10]*hErval[1];
		hxtildeval[4] = hxtval[4] + hKval[13]*hErval[1];
		hxtildeval[5] = hxtval[5] + hKval[16]*hErval[1];
		hxtildeval[6] = hxtval[6] + hKval[20]*hErval[2];
		hxtildeval[7] = hxtval[7] + hKval[23]*hErval[2];
		hxtildeval[8] = hxtval[8] + hKval[26]*hErval[2];

		//update states pos,vel,acc
		hxval[0]=hxtildeval[0]; hxval[1]=hxtildeval[1];hxval[2]=hxtildeval[2];// x pos,vel,acc
		hxval[3]=hxtildeval[3]; hxval[4]=hxtildeval[4]; hxval[5]=hxtildeval[5];// y pos,vel,acc
		hxval[6]=hxtildeval[6]; hxval[7]=hxtildeval[7]; hxval[8]=hxtildeval[8];// z pos,vel,acc

		Pxk= hxval[0]; Pyk= hxval[3]; Pzk= hxval[6];
		Pvxk= hxval[1]; Pvyk= hxval[4]; Pvzk= hxval[7];



}

void POS_KF(double posX,double posY,double posZ,double accXl,double accYl,double accZl,int isUpdatedFk )
{

		//raw acc
		hxval[2] = accXl;
		hxval[5] = accYl;
		hxval[8] = accZl;

		//	//raw pos
		hyval[0] = posX;
		hyval[1] = posY;
		hyval[2] = posZ;

		//calc new state
		status = arm_mat_mult_f32(&hF, &hx, &hxt);

		//predicated pos
		hyhatval[0] = hxtval[0];
		hyhatval[1] = hxtval[3];
		hyhatval[2] = hxtval[6];



		/* calculation of P*/
		status = arm_mat_mult_f32(&hF, &hP, &hFP); //check_calc(5);
		status = arm_mat_mult_f32(&hFP, &hFT, &hFPFT); //check_calc(6);
		status = arm_mat_add_f32(&hFPFT, &hQt, &hPt);  // check_calc(9);

		/* calculation of K */
		status = arm_mat_mult_f32(&hH, &hPt, &hHP);    //check_calc(10);
		status = arm_mat_mult_f32(&hHP, &hHT, &hHPHT); //  check_calc(11);
		status = arm_mat_add_f32(&hHPHT, &hR, &hS);   // check_calc(12);
		status = arm_mat_inverse_f32(&hS, &hSI);     // check_calc(13);
		status = arm_mat_mult_f32(&hPt, &hHT, &hPHT); //  check_calc(14);
		status = arm_mat_mult_f32(&hPHT, &hSI, &hK);  // check_calc(15);
		status = arm_mat_trans_f32(&hK, &hKT);




		//	/* calculation of Er */
		status = arm_mat_sub_f32(&hy, &hyhat, &hEr); // check_calc(17);


		/* calculation  x update */
		status = arm_mat_mult_f32(&hK, &hEr, &hKEr);      //    check_calc(181);
		status = arm_mat_add_f32(&hxt, &hKEr, &hxtilde);  // check_calc(182);

		/* calculation of Pt update */
		status = arm_mat_mult_f32(&hK, &hS, &hKS);  // check_calc(20);
		status = arm_mat_mult_f32(&hKS, &hKT, &hKSKT); // check_calc(21);
		status = arm_mat_sub_f32(&hPt, &hKSKT, &hPtilde); // check_calc(22);

		//sprintf(msg,"%s,%f,%f,%f,%f\n","hPtilde",hPtildeval[0],hPtildeval[1],hPtildeval[2],hPtildeval[3]);
		// USART2_TX(msg);

		//update states pos,vel,acc
		hxval[0]=hxtildeval[0]; hxval[1]=hxtildeval[1];hxval[2]=hxtildeval[2];// x pos,vel,acc
		hxval[3]=hxtildeval[3];hxval[4]=hxtildeval[4]; hxval[5]=hxtildeval[5];// y pos,vel,acc
		hxval[6]=hxtildeval[6];hxval[7]=hxtildeval[7]; hxval[8]=hxtildeval[8];// z pos,vel,acc

		////reassign all P diagonals values
		hPval[0]=hPtildeval[0];hPval[10]=hPtildeval[10];hPval[20]=hPtildeval[20];hPval[30]=hPtildeval[30];
		hPval[40]=hPtildeval[40];hPval[50]=hPtildeval[50];hPval[60]=hPtildeval[60];hPval[70]=hPtildeval[70];hPval[80]=hPtildeval[80];

		Pxk= hxval[0]; Pyk= hxval[3]; Pzk= hxval[6];
		Pvxk= hxval[1]; Pvyk= hxval[4]; Pvzk= hxval[7];




}

void check_POScalc(int i)
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

