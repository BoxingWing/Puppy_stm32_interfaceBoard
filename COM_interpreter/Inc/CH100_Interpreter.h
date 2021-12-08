/*
 * CH100_Interpreter.h
 *
 *  Created on: Dec 3, 2021
 *      Author: MSI-NB
 */

#ifndef INC_CH100_INTERPRETER_H_
#define INC_CH100_INTERPRETER_H_

#include <stdint.h>

void crc16_update(uint16_t *currect_crc, const uint8_t *src, uint32_t len);
int CH100_FBdecoder(uint8_t *recData, uint8_t *accData, uint8_t *omegaData, uint8_t *rpyData, uint8_t *IMU_data);



#endif /* INC_CH100_INTERPRETER_H_ */
