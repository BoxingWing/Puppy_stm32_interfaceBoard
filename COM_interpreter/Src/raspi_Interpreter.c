/*
 * raspi_Interpreter.c
 *
 *  Created on: Feb 7, 2021
 *      Author: kboxi
 */

#include "raspi_Interpreter.h"

int raspiCMDdecoder(uint8_t *cmd, uint8_t *readFlag, uint8_t *setFlag, uint8_t *idArray, uint8_t *servoNum, int16_t *angleArray, uint8_t *lineIdx)
// size(cmd)=60
{
	uint8_t check=0x66; // check header
	uint16_t tmp=0;
	uint8_t checksum;
	int i;
	int flag=0;

	for (i=0;i<59;i++)
		tmp+=cmd[i];
	checksum=tmp % 256;
	if (checksum!=cmd[59])
		flag=1;
	if (check!=cmd[0])
		flag=1;
	if (flag==1)
		return flag;

	*servoNum=cmd[1]; // servo numbers on each active line

	// line choose and set and read flag choose
	switch(cmd[2]){
	case 10: *readFlag=1; *setFlag=0; *lineIdx=1; break; // enable line 1
	case 11: *readFlag=0; *setFlag=1; *lineIdx=1; break;
	case 12: *readFlag=1; *setFlag=1; *lineIdx=1; break;
	case 20: *readFlag=1; *setFlag=0; *lineIdx=2; break; // enable line 2
	case 21: *readFlag=0; *setFlag=1; *lineIdx=2; break;
	case 22: *readFlag=1; *setFlag=1; *lineIdx=2; break;
	case 30: *readFlag=1; *setFlag=0; *lineIdx=3; break; // enable line 3
	case 31: *readFlag=0; *setFlag=1; *lineIdx=3; break;
	case 32: *readFlag=1; *setFlag=1; *lineIdx=3; break;
	case 40: *readFlag=1; *setFlag=0; *lineIdx=4; break; // enable line 4
	case 41: *readFlag=0; *setFlag=1; *lineIdx=4; break;
	case 42: *readFlag=1; *setFlag=1; *lineIdx=4; break;
	case 50: *readFlag=1; *setFlag=0; *lineIdx=5; break; // enable line 1-4
	case 51: *readFlag=0; *setFlag=1; *lineIdx=5; break;
	case 52: *readFlag=1; *setFlag=1; *lineIdx=5; break;
	default: *readFlag=0; *setFlag=0; *lineIdx=1;
	}
	/*
	if (cmd[2]==0)
	{*readFlag=1; *setFlag=0;}
	else if (cmd[2]==1)
	{*readFlag=0; *setFlag=1;}
	else if (cmd[2]==2)
	{*readFlag=1; *setFlag=1;}
	else
	{*readFlag=0; *setFlag=0;}*/

	for (i=0;i<12;i++)
	{
		idArray[i]=cmd[3+i];
		angleArray[i]=(cmd[15+i*2+1]<<8) | cmd[15+i*2];
	}

	return flag;
}

int raspiCMDdecoder_CH100(uint8_t *cmd, uint8_t *readFlag, uint8_t *setFlag, uint8_t *idArray, uint8_t *servoNum, int16_t *angleArray, uint8_t *lineIdx)
//size(cmd)=79
{
	uint8_t check=0x66; // check header
	uint16_t tmp=0;
	uint8_t checksum;
	int i;
	int flag=0;

	for (i=0;i<78;i++)
		tmp+=cmd[i];
	checksum=tmp % 256;
	if (checksum!=cmd[78])
		flag=1;
	if (check!=cmd[0])
		flag=1;
	if (flag==1)
		return flag;

	*servoNum=cmd[1]; // servo numbers on each active line

	// line choose and set and read flag choose
	switch(cmd[2]){
	case 10: *readFlag=1; *setFlag=0; *lineIdx=1; break; // enable line 1
	case 11: *readFlag=0; *setFlag=1; *lineIdx=1; break;
	case 12: *readFlag=1; *setFlag=1; *lineIdx=1; break;
	case 20: *readFlag=1; *setFlag=0; *lineIdx=2; break; // enable line 2
	case 21: *readFlag=0; *setFlag=1; *lineIdx=2; break;
	case 22: *readFlag=1; *setFlag=1; *lineIdx=2; break;
	case 30: *readFlag=1; *setFlag=0; *lineIdx=3; break; // enable line 3
	case 31: *readFlag=0; *setFlag=1; *lineIdx=3; break;
	case 32: *readFlag=1; *setFlag=1; *lineIdx=3; break;
	case 40: *readFlag=1; *setFlag=0; *lineIdx=4; break; // enable line 4
	case 41: *readFlag=0; *setFlag=1; *lineIdx=4; break;
	case 42: *readFlag=1; *setFlag=1; *lineIdx=4; break;
	case 50: *readFlag=1; *setFlag=0; *lineIdx=5; break; // enable line 1-4
	case 51: *readFlag=0; *setFlag=1; *lineIdx=5; break;
	case 52: *readFlag=1; *setFlag=1; *lineIdx=5; break;
	default: *readFlag=0; *setFlag=0; *lineIdx=1;
	}
	/*
	if (cmd[2]==0)
	{*readFlag=1; *setFlag=0;}
	else if (cmd[2]==1)
	{*readFlag=0; *setFlag=1;}
	else if (cmd[2]==2)
	{*readFlag=1; *setFlag=1;}
	else
	{*readFlag=0; *setFlag=0;}*/

	for (i=0;i<12;i++)
	{
		idArray[i]=cmd[3+i];
		angleArray[i]=(cmd[15+i*2+1]<<8) | cmd[15+i*2];
	}

	return flag;
}


void genAngleBackCmd(uint8_t *idArray, uint8_t servoNum, int16_t *readAngle, uint8_t *cmd, uint8_t *IMU_data)
// size(cmd)=60
{
	cmd[0]=0x66;
	cmd[1]=servoNum;
	int i;
	uint8_t low8, high8;
	uint16_t tmp=0;
	for (i=0;i<12;i++)
	{
		cmd[2+i]=idArray[i];
		low8=readAngle[i] & 0xff;
		high8=(readAngle[i]>>8) & 0xff;
		cmd[14+i*2]=low8;
		cmd[14+i*2+1]=high8;
	}
	for (i=0;i<18;i++)
	{
		cmd[38+i]=IMU_data[i];
	}
	for (i=0;i<59;i++)
		tmp+=cmd[i];
	cmd[59]=tmp % 256;
}

void genAngleBackCmd_CH100(uint8_t *idArray, uint8_t servoNum, int16_t *readAngle, uint8_t *cmd, uint8_t *IMU_data_CH100)
// size(cmd)=79
{
	cmd[0]=0x66;
	cmd[1]=servoNum;
	int i;
	uint8_t low8, high8;
	uint16_t tmp=0;
	for (i=0;i<12;i++)
	{
		cmd[2+i]=idArray[i];
		low8=readAngle[i] & 0xff;
		high8=(readAngle[i]>>8) & 0xff;
		cmd[14+i*2]=low8;
		cmd[14+i*2+1]=high8;
	}
	for (i=0;i<40;i++)
	{
		cmd[38+i]=IMU_data_CH100[i];
	}

	for (i=0;i<78;i++)
		tmp+=cmd[i];
	cmd[78]=tmp % 256;
}

