/*
 * motor.h
 *
 *  Created on: 2018/08/12
 *      Author: Œ’Œå
 */

#ifndef MOTOR_H_
#define MOTOR_H_

void enable_motor();
void disable_motor();
void set_motor_duty(int16_t _duty_R, int16_t _duty_L);
void drive_motor();
void setup_suction();
void enable_suction(uint8_t _duty);
void disable_suction();
#endif /* MOTOR_H_ */
