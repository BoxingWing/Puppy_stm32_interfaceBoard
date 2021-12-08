/*
 * JY901_Interpreter.c
 *
 *  Created on: Apr 9, 2021
 *      Author: kboxi
 */

#include "JY901_Interpreter.h"

void JY901_genAskCmd(uint8_t *cmd, int fre)
{
	cmd[0]=0xFF;
	cmd[1]=0xAA;
	cmd[2]=0x03;
	cmd[3]=0x0c;
	cmd[4]=0x00;
	if (fre==200)
		cmd[3]=0x0b;
}





int JY901_FBdecoder(uint8_t *recData, uint8_t *accData, uint8_t *omegaData, uint8_t *rpyData, uint8_t *IMU_data)
{
	uint16_t tmp=0;
	int i;
	if (recData[0]!=0x55 || recData[1]!=0x51)
		return 1;
	for (i=0;i<10;i++)
		tmp+=recData[i];
	if (tmp%256!=recData[10])
		return 1;
	accData[0]=recData[2];
	accData[1]=recData[3];
	accData[2]=recData[4];
	accData[3]=recData[5];
	accData[4]=recData[6];
	accData[5]=recData[7];
	IMU_data[0]=accData[0];
	IMU_data[1]=accData[1];
	IMU_data[2]=accData[2];
	IMU_data[3]=accData[3];
	IMU_data[4]=accData[4];
	IMU_data[5]=accData[5];

	tmp=0;
	if (recData[11]!=0x55 || recData[12]!=0x52)
		return 1;
	for (i=0;i<10;i++)
		tmp+=recData[11+i];
	if (tmp%256!=recData[21])
		return 1;
	omegaData[0]=recData[13];
	omegaData[1]=recData[14];
	omegaData[2]=recData[15];
	omegaData[3]=recData[16];
	omegaData[4]=recData[17];
	omegaData[5]=recData[18];
	IMU_data[6]=omegaData[0];
	IMU_data[7]=omegaData[1];
	IMU_data[8]=omegaData[2];
	IMU_data[9]=omegaData[3];
	IMU_data[10]=omegaData[4];
	IMU_data[11]=omegaData[5];

	tmp=0;
	if (recData[22]!=0x55 || recData[23]!=0x53)
		return 1;
	for (i=0;i<10;i++)
		tmp+=recData[22+i];
	if (tmp%256!=recData[32])
		return 1;
	rpyData[0]=recData[24];
	rpyData[1]=recData[25];
	rpyData[2]=recData[26];
	rpyData[3]=recData[27];
	rpyData[4]=recData[28];
	rpyData[5]=recData[29];
	IMU_data[12]=rpyData[0];
	IMU_data[13]=rpyData[1];
	IMU_data[14]=rpyData[2];
	IMU_data[15]=rpyData[3];
	IMU_data[16]=rpyData[4];
	IMU_data[17]=rpyData[5];

	return 0;
}






