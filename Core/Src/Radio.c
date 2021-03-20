/*
 * Radio.c
 *
 *  Created on: 28 Dec 2020
 *      Author: HENRY ODOEMELEM
 */



#include "Radio.h"
#include <string.h>


uint8_t ibusIndex = 0;
uint8_t emptyBuffer[32] = {0};
int checksum = 65535;



void Process_ibus(void)
{
	chHeader = ibus[0]|ibus[1]<<8;
	if ((ibus[0]==32) && (ibus[1] == 64))//chck the header
	{
		memcpy(rawChData,ibus,32);


		checksum = 65535;
		for(int i=0;i<=29;i++)
		{
			checksum = checksum - rawChData[i];
		}

		if(checksum == (rawChData[30]|rawChData[31]<<8) )
		{
			//process iBus buffer into channels
			for(int i=2,ch=0;ch< IBUS_MAXCHANNELS;i=i+2,ch++)
				{
					channels[ch] = rawChData[i]|rawChData[i+1]<<8;

				}



		}
		//reset buffer values
		memcpy(ibus,emptyBuffer,32);
		memcpy(rawChData,emptyBuffer,32);


	}
}
