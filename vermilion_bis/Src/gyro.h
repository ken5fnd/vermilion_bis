/*
 * gyro.h
 *
 *  Created on: 2018/08/03
 *      Author: Œ’Œå
 */

#ifndef GYRO_H_
#define GYRO_H_

#include "main.h"
#include "stm32f7xx_hal.h"
#include "misc.h"
#include "const.h"
#include "global.h"
#include <math.h>

void gyro_rw(uint8_t *_Data_Input, uint8_t *_Data_Output);
void gyro_setup();
int16_t get_ang_vel(uint8_t _axis);
int16_t gyro_get_reference();
#endif /* GYRO_H_ */
