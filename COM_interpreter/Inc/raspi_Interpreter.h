/*
 * raspi_Interpreter.h
 *
 *  Created on: Feb 7, 2021
 *      Author: kboxi
 */

#ifndef INC_RASPI_INTERPRETER_H_
#define INC_RASPI_INTERPRETER_H_

#include <stdint.h>

int raspiCMDdecoder(uint8_t *cmd, uint8_t *readFlag, uint8_t *setFlag, uint8_t *idArray, uint8_t *servoNum, int16_t *angleArray, uint8_t *lineIdx);
int raspiCMDdecoder_CH100(uint8_t *cmd, uint8_t *readFlag, uint8_t *setFlag, uint8_t *idArray, uint8_t *servoNum, int16_t *angleArray, uint8_t *lineIdx);
void genAngleBackCmd(uint8_t *idArray, uint8_t servoNum, int16_t *readAngle, uint8_t *cmd, uint8_t *IMU_data);
void genAngleBackCmd_CH100(uint8_t *idArray, uint8_t servoNum, int16_t *readAngle, uint8_t *cmd, uint8_t *IMU_data_CH100);

#endif /* INC_RASPI_INTERPRETER_H_ */
