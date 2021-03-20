/*
 * Radio.h
 *
 *  Created on: 28 Dec 2020
 *      Author: HENRY ODOEMELEM
 */

#ifndef SRC_RADIO_H_
#define SRC_RADIO_H_



#endif /* SRC_RADIO_H_ */



#include "stm32l4xx.h"                  // Device header
#include <stdlib.h>




// 32 Max iBus packet size (2 byte header, 14 channels x 2 bytes, 2 byte checksum)
//  TX only has 10 channels
extern uint8_t ibus[32];
extern uint8_t rawChData[32];
extern uint16_t channels[10];
extern uint16_t chHeader;
extern uint8_t IBUS_MAXCHANNELS; // My TX only has 10 channels, no point in polling the rest
extern uint8_t IBUS_BUFFSIZE;

void Process_ibus(void);
