/*
 * Servo_Interpreter.h
 *
 *  Created on: Feb 7, 2021
 *      Author: kboxi
 */

#ifndef INC_SERVO_INTERPRETER_H_
#define INC_SERVO_INTERPRETER_H_

#include <stdint.h>

void genAngleAskCmd(uint8_t id, uint8_t *cmd);
double recAngleCmdDecode(uint8_t id, uint8_t *dataOri, double angleOld);
void genSetAngleCmd(uint8_t id, int16_t angle, uint16_t tD, uint16_t powD, uint8_t *cmd);

#endif /* INC_SERVO_INTERPRETER_H_ */
