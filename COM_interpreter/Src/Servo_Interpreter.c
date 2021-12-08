/*
 * Servo_Interpreter.c
 *
 *  Created on: Feb 7, 2021
 *      Author: kboxi
 */

#include "Servo_Interpreter.h"

void genAngleAskCmd(uint8_t id, uint8_t *cmd)
{
	int tmp=0;
	int i=0;
	cmd[0]=0x12;
	cmd[1]=0x4c;
	cmd[2]=10;     // command ID, 10 for angle read command, referred as READ_ANGLE in the mannual
	cmd[3]=1;    //  length of the data portion
	cmd[4]=id;
	for (i=0;i<5;i++)
		tmp+=(int)cmd[i];
	cmd[5]=tmp%256;
}

double recAngleCmdDecode(uint8_t id, uint8_t *dataOri, double angleOld)
{
	uint8_t flag=0;
	uint8_t check[5]={0x05,0x1c,0x0a,0x03,id};
	uint8_t checkSum,tmp;
	int16_t angleOri;
	int i;
	double angleNew;
	for (i=0;i<5;i++)
		if (check[i]!=dataOri[i])
			flag=1;
	tmp=0;
	for (i=0;i<7;i++)
		tmp+=(int)dataOri[i];
	checkSum=tmp%256;
	if (checkSum!=dataOri[7])
		flag=1;
	if (flag==0)
	{
		angleOri= (dataOri[6]<<8) | dataOri[5];
		angleNew=angleOri/10.0;
		return angleNew;
	}
	else
		return angleOld;
}
void genSetAngleCmd(uint8_t id, int16_t angle, uint16_t tD, uint16_t powD, uint8_t *cmd)
{
	uint8_t angleLow, angleHigh;
	uint8_t tDLow, tDHigh;
	uint8_t powDLow, powDHigh;
	uint16_t tmp=0;
	int i;
	cmd[0]=0x12;
	cmd[1]=0x4c;
	cmd[2]=8; //command ID, 8 for angle control, referred as ROTATE in the manual
	cmd[3]=7; //length of the data portion
	cmd[4]=id;
	angleLow=angle & 0xff;
	angleHigh=(angle>>8) & 0xff;
	cmd[5]=angleLow;
	cmd[6]=angleHigh;
	tDLow=tD & 0xff;
	tDHigh= (tD>>8) & 0xff;
	cmd[7]=tDLow;
	cmd[8]=tDHigh;
	powDLow=powD & 0xff;
	powDHigh=(powD>>8) & 0xff;
	cmd[9]=powDLow;
	cmd[10]=powDHigh;
	for (i=0;i<11;i++)
		tmp+=(uint16_t)cmd[i];
	cmd[11]= tmp % 256;
}
