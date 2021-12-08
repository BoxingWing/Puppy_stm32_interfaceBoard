/*
 * CH100_Interpreter.c
 *
 *  Created on: Dec 3, 2021
 *      Author: MSI-NB
 */

#include "CH100_Interpreter.h"

void crc16_update(uint16_t *currect_crc, const uint8_t *src, uint32_t len)
{
    uint32_t crc = *currect_crc;
    uint32_t j;
    for (j=0; j < len; ++j)
    {
        uint32_t i;
        uint32_t byte = src[j];
        crc ^= byte << 8;
        for (i = 0; i < 8; ++i)
        {
            uint32_t temp = crc << 1;
            if (crc & 0x8000)
            {
                temp ^= 0x1021;
            }
            crc = temp;
        }
    }
    *currect_crc = crc;
}


int CH100_FBdecoder(uint8_t *recData, uint8_t *accData, uint8_t *omegaData, uint8_t *quatData, uint8_t *IMU_data)
{
	int i;
	int pickFlag=0;
	uint16_t payload_len;
	uint16_t crc=0;
	uint8_t *dataPack;
	uint8_t *realData;

	for (i=0;i<83;i++)
	{
		if (recData[i]==0x5A && recData[i+1]==0xA5)
			{pickFlag=1;realData=recData+i;break;}
	}

	if (pickFlag==0)
		return 1;


	dataPack=realData+6;
//	for (i=0;i<76;i++)
//		dataPack[i]=recData[i+6];

	payload_len=realData[2]+(realData[3]<<8);
	crc16_update(&crc,realData,4);
	crc16_update(&crc,realData+6,payload_len);

	if (crc!=(realData[5]<<8 | realData[4]))
		return 1;

	for (i=0;i<12;i++)
		accData[i]=dataPack[12+i];
	for (i=0;i<12;i++)
		omegaData[i]=dataPack[24+i];
	for (i=0;i<16;i++)
		quatData[i]=dataPack[60+i];

	for (i=0;i<12;i++)
		IMU_data[i]=accData[i];
	for (i=0;i<12;i++)
		IMU_data[i+12]=omegaData[i];
	for (i=0;i<16;i++)
		IMU_data[i+24]=quatData[i];

	return 0;
}
