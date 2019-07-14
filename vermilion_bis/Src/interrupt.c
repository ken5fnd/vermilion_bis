/*
 * interrupt.c
 *
 *  Created on: 2018/08/12
 *      Author: Œ’Œå
 */

#include "main.h"
#include "stm32f7xx_hal.h"
#include "misc.h"
#include "const.h"
#include "global.h"
#include "encorder.h"
#include "gyro.h"
#include "control.h"
#include "motor.h"
#include "wallsensor.h"
#include <math.h>

void interrupt_TIM6() {
	uint8_t Data_Input[] = { 0x00, 0x00 };
	uint8_t Data_Output[2];
	int16_t gyro_data;
	float ang_vel_gyro;
	static float ex_ang_vel[3];
	if (Flag_modeSelectEn) {
		mode_gyro_X = get_ang_vel(0);
		mode_gyro_Y = get_ang_vel(1);
		mode_gyro_Z = get_ang_vel(2);
	}
	if (Flag_countTimeEn) {
		countTime += CONTROL_CYCLE;
	}
	if (Flag_encEn) {
		get_velocity(&realState);
		realState.dis += realState.vel * CONTROL_CYCLE;
		idealState.vel += idealState.acc * CONTROL_CYCLE;
		idealState.dis += idealState.vel * CONTROL_CYCLE;
	}
	if (Flag_gyroEn) {
		ang_vel_gyro = 	(float) (get_ang_vel(2) - gyro_ref);
		if (ang_vel_gyro > 0.0) {
			ang_vel_gyro /= 16.384 * 1.0;
		} else {
			ang_vel_gyro /= 16.384 * 1.0;
		}
//		if (fabs(ang_vel_gyro - ex_ang_vel[0]) > 300.0) {
//			ang_vel_gyro = ex_ang_vel[0];
//		}
		realState.ang_vel = 1.0 * ang_vel_gyro + 0.0 * ex_ang_vel[0]
				+ 0.0 * ex_ang_vel[1] + 0.0 * ex_ang_vel[2];

		for (uint8_t i = 0; i < 2; i++) {
			ex_ang_vel[i + 1] = ex_ang_vel[1];
		}
		ex_ang_vel[0] = realState.ang_vel;

		idealState.ang_vel += idealState.ang_acc * 0.0005;
		realState.deg += realState.ang_vel * 0.0005;
		idealState.deg += idealState.ang_vel * 0.0005;
	} else {
		ang_vel_gyro = 0.0;
		ex_ang_vel[0] = 0.0;

	}
	if (fabs(error_vel.I) > 2.0 || fabs(error_ang_vel.I) > 80.0) {
		Flag_failSafeEn = true;
	}
	if (Flag_MotorEn) {
		calc_error();
		drive_motor();
	}
	if (Flag_IRLEDEn) {
		get_wallsensor_data2();
	} else {
		HAL_GPIO_WritePin(IRLED_RSLF_GPIO_Port, IRLED_RSLF_Pin, 0); //LED_RSLF
		HAL_GPIO_WritePin(IRLED_RFLS_GPIO_Port, IRLED_RFLS_Pin, 0); //LED_RFLS
	}
	save_log();
}

void interrupt_TIM7() {
	static int count;
	static uint8_t Flag_FanStateEn_ex;
	if (Flag_FanStateEn_ex != Flag_FanStateEn) {
		count = 0;
	}
	if (count == suction_ONtime || count == suction_ONtime) {
		HAL_GPIO_TogglePin(FAN_MOT_GPIO_Port, FAN_MOT_Pin);
		count = 0;
	} else {
		count++;
	}
	Flag_FanStateEn_ex = Flag_FanStateEn;

}

