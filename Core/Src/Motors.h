/*
 * Motors.h
 *
 *  Created on: 8 Jan 2021
 *      Author: HENRY ODOEMELEM
 */

#ifndef SRC_MOTORS_H_
#define SRC_MOTORS_H_



#endif /* SRC_MOTORS_H_ */


#include "stm32l4xx.h"                  // Device header
#include <stdlib.h>

extern int pwm1;
extern int pwm2;
extern int pwm3;
extern int pwm4;


void Motor_init(void);
void set_Motors(double thrust1,double thrust2,double thrust3,double thrust4);
void turnMotorsOff(void);
void motorMaxAllowed(void);
void idleStart(void);
void hover(void);
