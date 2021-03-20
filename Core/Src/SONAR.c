/*
 * SONAR.c
 *
 *  Created on: 28 Dec 2020
 *      Author: HENRY ODOEMELEM
 */
#include "SONAR.h"
int usrange = 0;
int swv = 0;
int Lite = 0;

void startSonarRanging(int Rangecmd)
{
	i2c2_Config(0x00,0x00,Rangecmd);//Do all sonar ranging
}

int processSonar(int saddr)
{

	/*
	 * You can take advantage of the fact that the SRF08 will not respond to any I2C
	 * activity whilst ranging. Therefore, if you try to read from the SRF08
	 * (we use the software revision number a location 0) then you will get 255 (0xFF)
	 * whilst ranging
	 */
	// Get software revision number check -> communication check


	i2c2_readRequest(saddr, 0x00,1);
	swv = i2c2_readByte();
	i2c2_stop();

	if(swv == 0xFF) //sensor busy, returns is 255
	{
		//do nothing
	}else
	{
		i2c2_readRequest(saddr, 0x02,2);
		//big-endian, MSB is received first
		usrange = (i2c2_readByte()<<8)|i2c2_readByte();
		i2c2_stop();
		return usrange; // shift two bytes to build 16bits int result
	}


	//Do LPF

	return -1; // not ready

}
