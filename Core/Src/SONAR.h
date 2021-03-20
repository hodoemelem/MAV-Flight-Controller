/*
 * SONAR.h
 *
 *  Created on: 28 Dec 2020
 *      Author: HENRY ODOEMELEM
 */

#ifndef SRC_SONAR_H_
#define SRC_SONAR_H_



#endif /* SRC_SONAR_H_ */

#include "stm32l4xx.h"                  // Device header
#include <stdlib.h>



extern void i2c2_Config(int saddr,int maddr,int wdata);
extern void i2c2_readRequest(int saddr,int maddr,int nRegs);
extern void i2c2_stop(void);
extern uint8_t i2c2_readByte(void);
void startSonarRanging(int Rangecmd);
int processSonar(int saddr);
