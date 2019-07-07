/*
 * wallsensor.c
 *
 *  Created on: 2018/08/25
 *      Author: 健悟
 */
#include "global.h"
#include "const.h"
#include <math.h>

void wall_exist() {
	if (IR_Sen.RS < IR_Det.RS) {      //右壁の有無
		wallExist.Right = true;
	} else {
		wallExist.Right = false;
	}
	if (IR_Sen.LS < IR_Det.LS) {      //左壁の有無
		wallExist.Left = true;
	} else {
		wallExist.Left = false;
	}
	if (IR_Sen.RF < IR_Det.RF /*|| IR_Sen.LF > IR_Det.LF*/) {      //前壁の有無
		wallExist.Forward = true;
	} else {
		wallExist.Forward = false;
	}
}

void get_wallsensor_data() {
	static __IRSensors IR_Sen_buffer;
	for (uint8_t i = 0; i < EX_IR_SEN_NUM - 1; i++) {
		ex_IR_Sen[i + 1] = ex_IR_Sen[i];
		ex_IR_Sen_dif_1[i + 1] = ex_IR_Sen_dif_1[i];
	}
	ex_IR_Sen[0].LF = IR_Sen.LF;
	ex_IR_Sen[0].LS = IR_Sen.LS;
	ex_IR_Sen[0].RS = IR_Sen.RS;
	ex_IR_Sen[0].RF = IR_Sen.RF;

	ex_IR_Sen_dif_1[0].RS = IR_Sen_dif_1.RS;
	ex_IR_Sen_dif_1[0].LS = IR_Sen_dif_1.LS;

	HAL_GPIO_WritePin(IRLED_RSLF_GPIO_Port, IRLED_RSLF_Pin, 1); //LED_RSLF
	HAL_GPIO_WritePin(IRLED_RFLS_GPIO_Port, IRLED_RFLS_Pin, 0); //LED_RFLS
	for (uint16_t i; i < 1200; i++)
		;
	IR_Sen_buffer.RS = IR_Sen_raw.RSw;
	IR_Sen_buffer.LF = IR_Sen_raw.LSw;
	HAL_GPIO_WritePin(IRLED_RSLF_GPIO_Port, IRLED_RSLF_Pin, 0); //LED_RSLF
	HAL_GPIO_WritePin(IRLED_RFLS_GPIO_Port, IRLED_RFLS_Pin, 1); //LED_RFLS
	for (uint16_t i; i < 1200; i++)
		;
	IR_Sen_buffer.RF = IR_Sen_raw.RSw;
	IR_Sen_buffer.LS = IR_Sen_raw.LSw;
	HAL_GPIO_WritePin(IRLED_RSLF_GPIO_Port, IRLED_RSLF_Pin, 0); //LED_RSLF
	HAL_GPIO_WritePin(IRLED_RFLS_GPIO_Port, IRLED_RFLS_Pin, 0); //LED_RFLS
	for (uint16_t i; i < 1200; i++)
		;
	IR_Sen.RS = IR_Sen_buffer.RS - IR_Sen_raw.RSw;
	IR_Sen.LF = IR_Sen_buffer.LF - IR_Sen_raw.LSw;
	IR_Sen.RF = IR_Sen_buffer.RF - IR_Sen_raw.RSw;
	IR_Sen.LS = IR_Sen_buffer.LS - IR_Sen_raw.LSw;

	IR_Sen_dif_1.RS = (((float) IR_Sen.RS
			- (float) ex_IR_Sen[EX_IR_SEN_NUM - 1].RS) * 0.1
			+ ex_IR_Sen_dif_1[0].RS * 0.9);
	IR_Sen_dif_1.LS = (((float) IR_Sen.LS
			- (float) ex_IR_Sen[EX_IR_SEN_NUM - 1].LS) * 0.1
			+ ex_IR_Sen_dif_1[0].LS * 0.9);
}
void get_wallsensor_data2() {
	static uint8_t IRLED_phase;
	static __IRSensors IR_Sen_buffer;
	for (uint8_t i = 0; i < EX_IR_SEN_NUM - 1; i++) {
		ex_IR_Sen[i + 1] = ex_IR_Sen[i];
		ex_IR_Sen_dif_1[i + 1] = ex_IR_Sen_dif_1[i];
	}
	ex_IR_Sen[0].LF = IR_Sen.LF;
	ex_IR_Sen[0].LS = IR_Sen.LS;
	ex_IR_Sen[0].RS = IR_Sen.RS;
	ex_IR_Sen[0].RF = IR_Sen.RF;

	ex_IR_Sen_dif_1[0].RS = IR_Sen_dif_1.RS;
	ex_IR_Sen_dif_1[0].LS = IR_Sen_dif_1.LS;

	switch (IRLED_phase) {
	case 0: //全部off
		IR_Sen_buffer.RS = IR_Sen_raw.RSw;
		IR_Sen_buffer.LF = IR_Sen_raw.LSw;
		IR_Sen_buffer.RF = IR_Sen_raw.RSw;
		IR_Sen_buffer.LS = IR_Sen_raw.LSw;
		HAL_GPIO_WritePin(IRLED_RSLF_GPIO_Port, IRLED_RSLF_Pin, 1); //LED_RSLF
		HAL_GPIO_WritePin(IRLED_RFLS_GPIO_Port, IRLED_RFLS_Pin, 0); //LED_RFLS
		IRLED_phase = 1;
		break;
	case 1: //RSLF
		if (Flag_Check_IR_Sen_Mode == true) {
			IR_Sen_buffer.RS < IR_Sen_raw.RSw ?
					IR_Sen.RS = IR_Sen_raw.RSw - IR_Sen_buffer.RS : 0;
			IR_Sen_buffer.LF < IR_Sen_raw.LSw ?
					IR_Sen.LF = IR_Sen_raw.LSw - IR_Sen_buffer.LF : 0;
		} else {
			IR_Sen_buffer.RS < IR_Sen_raw.RSw ?
					IR_Sen.RS = -IR_Coefficient.RS
							* logf(IR_Sen_raw.RSw - IR_Sen_buffer.RS)
							+ IR_Segment.RS :
					0;
			IR_Sen_buffer.LF < IR_Sen_raw.LSw ?
					IR_Sen.LF = -IR_Coefficient.LF
							* logf(IR_Sen_raw.LSw - IR_Sen_buffer.LF)
							+ IR_Segment.LF :
					0;
		}
		HAL_GPIO_WritePin(IRLED_RSLF_GPIO_Port, IRLED_RSLF_Pin, 0); //LED_RSLF
		HAL_GPIO_WritePin(IRLED_RFLS_GPIO_Port, IRLED_RFLS_Pin, 1); //LED_RFLS
		IRLED_phase = 2;
		break;
	case 2: //LSRF
		if (Flag_Check_IR_Sen_Mode == true) {
			IR_Sen_buffer.RF < IR_Sen_raw.RSw ?
					IR_Sen.RF = IR_Sen_raw.RSw - IR_Sen_buffer.RF : 0;
			IR_Sen_buffer.LS < IR_Sen_raw.LSw ?
					IR_Sen.LS = IR_Sen_raw.LSw - IR_Sen_buffer.LS : 0;
		} else {
			IR_Sen_buffer.RF < IR_Sen_raw.RSw ?
					IR_Sen.RF = -IR_Coefficient.RF
							* logf(IR_Sen_raw.RSw - IR_Sen_buffer.RF)
							+ IR_Segment.RF :
					0;
			IR_Sen_buffer.LS < IR_Sen_raw.LSw ?
					IR_Sen.LS = -IR_Coefficient.LS
							* logf(IR_Sen_raw.LSw - IR_Sen_buffer.LS)
							+ IR_Segment.LS :
					0;
		}
		HAL_GPIO_WritePin(IRLED_RSLF_GPIO_Port, IRLED_RSLF_Pin, 0); //LED_RSLF
		HAL_GPIO_WritePin(IRLED_RFLS_GPIO_Port, IRLED_RFLS_Pin, 0); //LED_RFLS
		IRLED_phase = 0;
	}

	IR_Sen_dif_1.RS = ((IR_Sen.RS - ex_IR_Sen[EX_IR_SEN_NUM - 1].RS) * 0.1
			+ ex_IR_Sen_dif_1[0].RS * 0.9);
	IR_Sen_dif_1.LS = ((IR_Sen.LS - ex_IR_Sen[EX_IR_SEN_NUM - 1].LS) * 0.1
			+ ex_IR_Sen_dif_1[0].LS * 0.9);
}

void edge_break(int8_t _dir, float _target_vel, float _acc) {
	float Edge_Thr_UP_R;
	float Edge_Thr_DOWN_R;
	float Edge_Thr_UP_L;
	float Edge_Thr_DOWN_L;
	float Edge_Thr_Exist_R;
	float Edge_Thr_Exist_L;
	if (Flag_diagMode == false) {
		Edge_Thr_Exist_R = 6.0;
		Edge_Thr_Exist_L = 6.0;
		Edge_Thr_UP_R = 0.1;
		Edge_Thr_DOWN_R = -0.05;
		Edge_Thr_UP_L = 0.1;
		Edge_Thr_DOWN_L = -0.05;
	} else {
		Edge_Thr_Exist_R = 0.0;
		Edge_Thr_Exist_L = 0.0;
		Edge_Thr_UP_R = 0.1;
		Edge_Thr_DOWN_R = -0.04;
		Edge_Thr_UP_L = 0.1;
		Edge_Thr_DOWN_L = -0.04;
	}
	if (idealState.vel < _target_vel) {
		idealState.acc = _acc;
	} else {
		idealState.acc = -_acc;
	}

	if (_dir == Right) { //右周りの場合
		if (IR_Sen.RS < IR_Det.RS) {
			while (!Flag_failSafeEn) {
				if (idealState.acc > 0.0 ?
						idealState.vel > _target_vel :
						idealState.vel < _target_vel) {
					idealState.vel = _target_vel;
					idealState.acc = 0.0;
				}
				if ((IR_Sen_dif_1.RS > Edge_Thr_UP_R
				&& IR_Sen.RS > Edge_Thr_Exist_R
				) || (Flag_diagMode == false && IR_Sen.RF < 11.0)) {
					break;
				}
			}
		} else { //if (Flag_diagMode == false) {
			while (!Flag_failSafeEn) {
				if (idealState.acc > 0.0 ?
						idealState.vel > _target_vel :
						idealState.vel < _target_vel) {
					idealState.vel = _target_vel;
					idealState.acc = 0.0;
				}
				if ((IR_Sen_dif_1.RS < Edge_Thr_DOWN_R) //&& IR_Sen.RS < 8.5)
				|| (Flag_diagMode == false && IR_Sen.RF < 11.0)) {
					break;
				}
			}
			while (!Flag_failSafeEn) {
				if (idealState.acc > 0.0 ?
						idealState.vel > _target_vel :
						idealState.vel < _target_vel) {
					idealState.vel = _target_vel;
					idealState.acc = 0.0;
				}
				if ((IR_Sen_dif_1.RS > Edge_Thr_UP_R
				&& IR_Sen.RS > Edge_Thr_Exist_R - 1.0
				) || (Flag_diagMode == false && IR_Sen.RF < 11.0)) {
					break;
				}
			}
		}
	} else { //左回りの場合
		if (IR_Sen.LS < IR_Det.LS) {
			while (!Flag_failSafeEn) {
				if (idealState.acc > 0.0 ?
						idealState.vel > _target_vel :
						idealState.vel < _target_vel) {
					idealState.vel = _target_vel;
					idealState.acc = 0.0;
				}
				if ((IR_Sen_dif_1.LS > Edge_Thr_UP_L
				&& IR_Sen.LS > Edge_Thr_Exist_L
				) || (Flag_diagMode == false && IR_Sen.RF < 11.0)) {
					break;
				}

			}
		} else { // if (Flag_diagMode == false) {
			while (!Flag_failSafeEn) {
				if (idealState.acc > 0.0 ?
						idealState.vel > _target_vel :
						idealState.vel < _target_vel) {
					idealState.vel = _target_vel;
					idealState.acc = 0.0;
				}
				if ((IR_Sen_dif_1.LS < Edge_Thr_DOWN_L) //&& IR_Sen.LS < 8.5)
				|| (Flag_diagMode == false && IR_Sen.RF < 11.0)) {
					break;
				}
			}
			while (!Flag_failSafeEn) {
				if (idealState.acc > 0.0 ?
						idealState.vel > _target_vel :
						idealState.vel < _target_vel) {
					idealState.vel = _target_vel;
					idealState.acc = 0.0;
				}
				if ((IR_Sen_dif_1.LS > Edge_Thr_UP_L
				&& IR_Sen.LS > Edge_Thr_Exist_L - 1.0
				) || (Flag_diagMode == false && IR_Sen.RF < 11.0)) {
					break;
				}
			}
		}
		idealState.dis = 0.0;
		realState.dis = 0.0;
	}
}

