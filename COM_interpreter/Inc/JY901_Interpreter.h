/*
 * JY901_Interpreter.h
 *
 *  Created on: Apr 9, 2021
 *      Author: kboxi
 */

#ifndef INC_JY901_INTERPRETER_H_
#define INC_JY901_INTERPRETER_H_

#include <stdint.h>

void JY901_genAskCmd(uint8_t *cmd, int fre);
int JY901_FBdecoder(uint8_t *recData, uint8_t *accData, uint8_t *omegaData, uint8_t *rpyData, uint8_t *IMU_data);

#endif /* INC_JY901_INTERPRETER_H_ */
