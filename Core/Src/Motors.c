/*
 * Motors.c
 *
 *  Created on: 8 Jan 2021
 *      Author: HENRY ODOEMELEM
 */

#include "Motors.h"
#include <string.h>

void systickDelayMS(int n);//extern by default

//F=c*n^2; thrust
int pwmMax = 4000;//2ms
int pwmMin = 2000;//
int temp1;
int temp2;
int temp3;
int temp4;

int lastpwm1 = 2074;
int lastpwm2 = 2074;
int lastpwm3 = 2074;
int lastpwm4 = 2074;

double thrustMax = 5;// for which max thrust for A2212/13T 1000kv BLDC
double thrustMin = 0.3;// at

void Motor_init(void)
{

	RCC->AHB2ENR |= (1 << 0);     // enable GPIOA clk, port A
	systickDelayMS(1);

	GPIOA->MODER = 0xABABF7AA; // PA2 and PA3 in alternate mode, PA5 output,
	//PA10 alternate mode (for usart1), PA11 alt
	//PA9 alt,PA0 and PA1 alt

	//TIM1,PA11 for tim1 ch4,PA9 for tim1 ch2
	GPIOA->AFR[1] |= 0x1010;    // enable tim1 ch4 on PA11,tim1 ch2 on PA9
	RCC->APB2ENR |= (1 << 11); // enable TIM1 clk
	TIM1->PSC = 36 - 1;         // set 2MHz clk
	TIM1->ARR = 40000 - 1;    // 50 Hz pwm ie 20 ms total period
	TIM1->CCMR1 = 0x6800; // Enable Pwm mode 1,preload enable (ch2 uses CCMR1)
	// PA9 for tim1 ch2
	TIM1->CCMR2 = 0x6800; // Enable Pwm mode 1,preload enable (ch4 uses CCMR2)
	//PA11 for tim1 ch4
	TIM1->CR1 |= 1<<7;  // Auto-reload preload enable
	TIM1->EGR = 1<<0; // Re-initialize the counter and generates an update of the registers
	TIM1->CCER = (1<<12)|(1<<4);  // enable ch4 and ch2
	TIM1->BDTR = 1<<15;  // main output enable
	TIM1->CCR4 = 2000 - 1; // about 0% rpm  on ch4
	TIM1->CCR2 = 2000 - 1; // about 0% rpm on ch2



	//TIM2, PA0 for tim2 ch1,PA1 for tim2 ch2
	// TIM2 does not have BDTR reg
	GPIOA->AFR[0] |= 0x0011;    // enable tim2 ch1 on PA0,tim2 ch2 on PA1
	RCC->APB1ENR1 |= (1 << 0); // enable TIM2 clk
	TIM2->PSC = 36 - 1;         // set 2MHz clk
	TIM2->ARR = 40000 - 1;    // 50 Hz pwm ie 20 ms total period
	TIM2->CCMR1 = 0x6868; // Enable Pwm mode 1,preload enable (ch2 and ch1)
	TIM2->CR1 |= 1<<7;  // Auto-reload preload enable
	TIM2->EGR = 1<<0; // Re-initialize the counter and generates an update of the registers
	TIM2->CCER = (1<<4)|(1<<0);  // enable ch2 and ch1
	TIM2->CCR1 = 2000 - 1; // about 0% rpm  on ch1
	TIM2->CCR2 = 2000 - 1; // about 0% rpm  on ch2


	//Enabel timers
	TIM2->CR1 |= 1<<0; // Counter enable
	TIM1->CR1 |= 1<<0; // Counter enable


}

void motorMaxAllowed(void)
{
	//lastpwm can not be greater than pwm for 15 deg


	pwm1 = lastpwm1 ;//M1
	pwm2 = lastpwm2 ;//M2
	pwm3 = lastpwm3 ;//M3
	pwm4 = lastpwm4 ;//M4

	TIM2->CCR2 = pwm1;//PA1
	TIM2->CCR1 = pwm2;//PA0
	TIM1->CCR4 = pwm3;//PA11
	TIM1->CCR2 = pwm4;//PA9

	//	pwm1 = pwmMin -1;//M1
	//	pwm2 = pwmMin - 1;//M2
	//	pwm3 = pwmMin - 1;//M3
	//	pwm4 = pwmMin - 1;//M4
	//
	//	TIM2->CCR2 = pwm1;//PA1
	//	TIM2->CCR1 = pwm2;//PA0
	//	TIM1->CCR4 = pwm3;//PA11
	//	TIM1->CCR2 = pwm4;//PA9
}

void set_Motors(double thrust1,double thrust2,double thrust3,double thrust4)
{

	//radio reading is in microseconds
	//6.25E-7 sec is time of I count of pwm clk
	//PA1(TIM2->CCR2) -> M1
	//PA0(TIM2->CCR1) -> M2
	//PA11(TIM1->CCR4) -> M3
	//PA9(TIM1->CCR2) -> M4

	//
	//	//Actual control signal
	//	temp1 = (int)((pwmMin + ((thrust1 - thrustMin)*(pwmMax - pwmMin))/(thrustMax - thrustMin))  );//M1
	//	temp2 = (int)((pwmMin + ((thrust2 - thrustMin)*(pwmMax - pwmMin))/(thrustMax - thrustMin))  );//M2
	//	temp3 = (int)((pwmMin + ((thrust3 - thrustMin)*(pwmMax - pwmMin))/(thrustMax - thrustMin))  );//M3
	//	temp4 = (int)((pwmMin + ((thrust4 - thrustMin)*(pwmMax - pwmMin))/(thrustMax - thrustMin))  );//M4
	//
	//
	//	{// do not go above this speed
	//		//pwm on period to timer counts(range: 2000(1ms) to 4000(2ms))
	//
	//		pwm1 = temp1;//M1
	//		pwm2 = temp2;//M2
	//		pwm3 = temp3;//M3
	//		pwm4 = temp4;//M4
	//
	//		//		lastpwm1 = pwm1;
	//		//		lastpwm2 = pwm2;
	//		//		lastpwm3 = pwm3;
	//		//		lastpwm4 = pwm4;
	//
	//
	//		TIM2->CCR2 = pwm1 - 1;//PA1
	//		TIM2->CCR1 = pwm2 - 1 ;//PA0
	//		TIM1->CCR4 = pwm3 - 1 ;//PA11
	//		TIM1->CCR2 = pwm4 - 1 ;//PA9
	//	}
	//
	//	//	///end actual test




	//testing hover
	pwmMin = 2000;
	pwmMax = 4000;
	thrustMin = 1000;
	thrustMax = 2000;
	temp1 = (int)((pwmMin + ((thrust1 - thrustMin)*(pwmMax - pwmMin))/(thrustMax - thrustMin))  );//M1
	temp2 = (int)((pwmMin + ((thrust2 - thrustMin)*(pwmMax - pwmMin))/(thrustMax - thrustMin))  );//M2
	temp3 = (int)((pwmMin + ((thrust3 - thrustMin)*(pwmMax - pwmMin))/(thrustMax - thrustMin))  );//M3
	temp4 = (int)((pwmMin + ((thrust4 - thrustMin)*(pwmMax - pwmMin))/(thrustMax - thrustMin)) );//M4


	//if(temp1<2900 && temp2<2900 && temp3<2900 && temp4<2900)
	{// do not go above 50% speed
		//pwm on period to timer counts(range: 1600(1ms) to 3200(2ms))
		pwm1 = temp1;//M1
		pwm2 = temp2;//M2
		pwm3 = temp3;//M3
		pwm4 = temp4;//M4

		TIM2->CCR2 = pwm1 -1;//PA1
		TIM1->CCR4 = pwm3 -1;//PA11
		TIM1->CCR2 = pwm4 -1;//PA9
		TIM2->CCR1 = pwm2 -1;//PA0


	}
	/////end of testing hover




	//    //Convert pwm on period to timer counts(range: 1600(1ms) to 3200(2ms))
	//	TIM1->CCR2 = (int)((M4*1E-6)/6.25E-7) - 1;//PA9
	//	TIM1->CCR4 = (int)((M3*1E-6)/6.25E-7) - 1;//PA11
	//	TIM2->CCR1 = (int)((M2*1E-6)/6.25E-7) - 1;//PA0
	//	TIM2->CCR2 = (int)((M1*1E-6)/6.25E-7) - 1;//PA1

	//	//Testing throttle on all 4 motors
	//	  //Convert pwm on period to timer counts(range: 1600(1ms) to 3200(2ms))
	//		TIM1->CCR2 = (int)((M3*1E-6)/6.25E-7) - 1;//PA9
	//		TIM1->CCR4 = (int)((M3*1E-6)/6.25E-7) - 1;//PA11
	//		TIM2->CCR1 = (int)((M3*1E-6)/6.25E-7) - 1;//PA0
	//		TIM2->CCR2 = (int)((M3*1E-6)/6.25E-7) - 1;//PA1

}

void turnMotorsOff(void)
{

	//radio reading is in microseconds
	//6.25E-7 sec is time of I count of pwm clk
	//PA1(TIM2->CCR2) -> M1
	//PA0(TIM2->CCR1) -> M2
	//PA11(TIM1->CCR4) -> M3
	//PA9(TIM1->CCR2) -> M4

	pwm1 = 2000;//M1
	pwm2 = 2000;//M2
	pwm3 = 2000;//M3
	pwm4 = 2000;//M4

	TIM2->CCR2 = pwm1 - 1;//PA1
	TIM2->CCR1 = pwm2 - 1;//PA0
	TIM1->CCR4 = pwm3 - 1 ;//PA11
	TIM1->CCR2 = pwm4 - 1;//PA9


}

void idleStart(){

	//radio reading is in microseconds
	//6.25E-7 sec is time of I count of pwm clk
	//PA1(TIM2->CCR2) -> M1
	//PA0(TIM2->CCR1) -> M2
	//PA11(TIM1->CCR4) -> M3
	//PA9(TIM1->CCR2) -> M4

	pwm1 = 2000;//M1
	pwm2 = 2000;//M2
	pwm3 = 2000;//M3
	pwm4 = 2000;//M4

	TIM2->CCR2 = pwm1 - 1;//PA1
	TIM2->CCR1 = pwm2 - 1;//PA0
	TIM1->CCR4 = pwm3 - 1 ;//PA11
	TIM1->CCR2 = pwm4 - 1;//PA9

}

void hover(void){

	//radio reading is in microseconds
	//6.25E-7 sec is time of I count of pwm clk
	//PA1(TIM2->CCR2) -> M1
	//PA0(TIM2->CCR1) -> M2
	//PA11(TIM1->CCR4) -> M3
	//PA9(TIM1->CCR2) -> M4



}
