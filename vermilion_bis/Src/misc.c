/*
 * misc.c
 *
 *  Created on: 2018/06/08
 *      Author: 健悟
 */

#include "main.h"
#include "stm32f7xx_hal.h"
#include "global.h"
#include "define.h"
#include "motor.h"
#include "const.h"
#include "maze.h"
#include "drive.h"
#include <math.h>
#include <stdlib.h>

void LED(unsigned short _LED) {
	HAL_GPIO_WritePin(LED_L1_GPIO_Port, LED_L1_Pin, (_LED & 0x80) >> 7); //LED_L1
	HAL_GPIO_WritePin(LED_L2_GPIO_Port, LED_L2_Pin, (_LED & 0x40) >> 6); //LED_L2
	HAL_GPIO_WritePin(LED_L3_GPIO_Port, LED_L3_Pin, (_LED & 0x20) >> 5); //LED_L3
	HAL_GPIO_WritePin(LED_L4_GPIO_Port, LED_L4_Pin, (_LED & 0x10) >> 4); //LED_L4
	HAL_GPIO_WritePin(LED_R4_GPIO_Port, LED_R4_Pin, (_LED & 0x08) >> 3); //LED_R4
	HAL_GPIO_WritePin(LED_R3_GPIO_Port, LED_R3_Pin, (_LED & 0x04) >> 2); //LED_R3
	HAL_GPIO_WritePin(LED_R2_GPIO_Port, LED_R2_Pin, (_LED & 0x02) >> 1); //LED_R2
	HAL_GPIO_WritePin(LED_R1_GPIO_Port, LED_R1_Pin, (_LED & 0x01)); //LED_R1
}

void Speaker_ON(TIM_HandleTypeDef *_htim) {
	if (HAL_TIM_PWM_Start(_htim, TIM_CHANNEL_4) != HAL_OK) {
		Error_Handler();
	}
}

void Speaker_Herz(TIM_HandleTypeDef *_htim, unsigned short _Hz) {
	__HAL_TIM_PRESCALER(_htim,1000000/_Hz);
	__HAL_TIM_SetCompare(_htim,TIM_CHANNEL_4,54);
}

void Speaker_OFF(TIM_HandleTypeDef *_htim) {
	if (HAL_TIM_PWM_Stop(_htim, TIM_CHANNEL_4) != HAL_OK) {
		Error_Handler();
	}
}

void Speaker_Scale(uint16_t _Hz, uint8_t _time) {
	Speaker_ON(&htim2);
	Speaker_Herz(&htim2, _Hz);
	HAL_Delay(_time);
	Speaker_OFF(&htim2);
}
void reset_status() {
	idealState = RESETState;
	realState.deg = 0.0;
	realState.dis = 0.0;
}

void reset_error() {
	error_vel = RESET_ERROR; //速度偏差
	error_dis = RESET_ERROR; //位置偏差
	error_ang_vel = RESET_ERROR; //角速度偏差
	error_deg = RESET_ERROR; //角度偏差
	error_wall_S = RESET_ERROR; //壁制御偏差
	error_wall_S_D = RESET_ERROR; //壁制御偏差
	error_wall_F_R = RESET_ERROR; //壁制御偏差
	error_wall_F_L = RESET_ERROR; //壁制御偏差
}

void save_log() {
	static int log_poz, log_Hz;
	if (log_ideal_vel[0] == 0.0 && log_real_vel[0] == 0.0) {
		log_poz = 0;
	}
	if (Flag_logEn && log_poz < LOG_NUM) {
		if (log_Hz == LOG_HZ) {
			log_Hz = 0;
			log_ideal_vel[log_poz] = idealState.vel;
			log_real_vel[log_poz] = realState.vel;
			log_ideal_dis[log_poz] = idealState.dis;
			log_real_dis[log_poz] = realState.dis;
			log_ideal_avel[log_poz] = idealState.ang_vel;
			log_real_avel[log_poz] = realState.ang_vel;
			log_ideal_deg[log_poz] = idealState.deg;
			log_real_deg[log_poz] = realState.deg;
			log_Sen_LF[log_poz] = IR_Sen.LF;
			log_Sen_LS[log_poz] = IR_Sen.LS;
			log_Sen_RS[log_poz] = IR_Sen.RS;
			log_Sen_RF[log_poz] = IR_Sen.RF;
			log_Sen_dLF[log_poz] = IR_Sen_dif_1.LF;
			log_Sen_dLS[log_poz] = IR_Sen_dif_1.LS;
			log_Sen_dRS[log_poz] = IR_Sen_dif_1.RS;
			log_Sen_dRF[log_poz] = IR_Sen_dif_1.RF;
			log_duty_R[log_poz] = (float) (g_Duty_R / 10);
			log_duty_L[log_poz] = (float) (g_Duty_L / 10);
			log_VBatt[log_poz] = V_Batt;
			log_real_vel_R[log_poz] = realState.vel_R;
			log_real_vel_L[log_poz] = realState.vel_L;
			log_poz++;
		} else {
			log_Hz++;
		}
	} else if (Flag_logEn == false) {
		log_poz = 0;
	}
}

void print_log() {
	printf(
			"Num,ideal_vel,real_vel,ideal_dis,real_dis,ideal_ang_vel,real_ang_vel,ideal_deg,real_deg,Duty_R,Duty_L,Sen_LF,Sen_LS,Sen_RS,Sen_RF,Sen_Dif_LF,Sen_Dif_LS,Sen_Dif_RS,Sen_Dif_RF,VBatt,real_vel_R,real_vel_L\n");
	for (int i = 0; i < LOG_NUM; i++) {
		if (log_VBatt[i] == 0.0) {
			break;
		}
		printf("%d,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n",
				i, log_ideal_vel[i], log_real_vel[i], log_ideal_dis[i],
				log_real_dis[i], log_ideal_avel[i], log_real_avel[i],
				log_ideal_deg[i], log_real_deg[i], log_duty_R[i], log_duty_L[i],
				log_Sen_LF[i], log_Sen_LS[i], log_Sen_RS[i], log_Sen_RF[i],
				log_Sen_dLF[i], log_Sen_dLS[i], log_Sen_dRS[i], log_Sen_dRF[i],
				log_VBatt[i], log_real_vel_R[i], log_real_vel_L[i]);
		//HAL_Delay(1);
	}
}
void printMaze(uint8_t _isReadWall) { //斜め歩数マップ用
//_isReadWall : 直接読んでいない壁を埋めるか否か
	printf(" \n");
//最初にx=15を表示
	for (int y = 15; y > -1; y--) { //一番上の壁の表示
		printf("\033[31m");
		printf("+-----");
	}
	printf("+\n");
	for (int y = 15; y > -1; y--) { //y方向にずらす
		printf("\033[31m");
		printf("|");
		for (int x = 0; x < 15; x++) { //縦壁描画
			if ((wall_ver[y] & 1 << x)
					|| (_isReadWall && !(wall_read_ver[y] & 1 << x))) {
				printf("\033[31m");
				printf("     |");
			} else {
				if (Flag_cellfound[x][y]) {
					printf("\033[32m");
				} else {
					printf("\033[37m");
				}
				printf(" %5d", DiagStepMap_ver[x][y]);
			}
		}
		printf("\033[31m");
		printf("     |\n");
		if (y == 0) {
			for (int x = 0; x < 16; x++) { //横壁描画
				printf("+-----");
			}
		} else {
			for (int x = 0; x < 16; x++) { //横壁描画
				if (((wall_hor[x] & 1 << (y - 1))
						|| (_isReadWall && !(wall_read_hor[x] & 1 << (y - 1))))
						&& (y != 0)) {
					printf("\033[31m");
					printf("+-----");
				} else {
					printf("\033[31m");
					printf("+");
					if (Flag_cellfound[x][y]) {
						printf("\033[32m");
					} else {
						printf("\033[37m");
					}
					printf("%5d", DiagStepMap_hor[x][y - 1]);
				}
			}
		}
		printf("\033[31m");
		printf("+\n");
		printf("\033[37m");
	}
}

void printPass() {
	int j;
	for (int i = 0; pass[i] != End; i++) {
		switch (pass[i]) {
		case End:
			printf("end");
			break;
		case Straight:
			j = 1;
			while (1) {
				if (pass[i + j] != Straight) {
					break;
				}
				j++;
			}
			printf("straight x %d", j);
			i += j - 1;
			break;
		case Straight_D:
			j = 1;
			while (1) {
				if (pass[i + j] != Straight_D) {
					break;
				}
				j++;
			}
			printf("diag_straight x %d", j);
			i += j - 1;
			break;
		case TurnR_90_D:
			printf("90d_R");
			break;
		case TurnL_90_D:
			printf("90d_L");
			break;
		case TurnR_90:
			printf("90_R");
			break;
		case TurnL_90:
			printf("90_L");
			break;
		case TurnR_180:
			printf("180_R");
			break;
		case TurnL_180:
			printf("180_L");
			break;
		case TurnR_135_I:
			printf("135_IN_R");
			break;
		case TurnL_135_I:
			printf("135_IN_L");
			break;
		case TurnR_135_O:
			printf("135_OUT_R");
			break;
		case TurnL_135_O:
			printf("135_OUT_L");
			break;
		case TurnR_45_I:
			printf("45_IN_R");
			break;
		case TurnL_45_I:
			printf("45_IN_L");
			break;
		case TurnR_45_O:
			printf("45_OUT_R");
			break;
		case TurnL_45_O:
			printf("45_OUT_L");
			break;
		}
		printf("\n");
	}
	printf("end \n");
}

void TestTurn(enum turn_type_test _turn, enum turndir_slalome _dir,
		const struct TurnParameter * const _parameter, uint8_t _isSuction) {
	if (_isSuction > 0) {
		enable_suction(_isSuction);
	}
	Flag_diagMode = false;
	/*状態量リセット*/
	idealState = RESETState;

	/*制御量リセット*/
	reset_error();
	Flag_logEn = true;
	switch (_turn) {
	case Turn90s:
		trapezoid_acccel(_parameter->Turn_vel, _parameter->Turn_vel, 0.09 * 1.0,
				7.0, true, false);
//		for (int i = 0; i < 10; i++) {
		turn90_s(_dir);
		//	}
		Flag_degConEn = true;
		trapezoid_acccel(_parameter->Turn_vel, 0.0, 0.09 * 1.0, 7.0,
		false, false);
		break;
	case Turn90:
		trapezoid_acccel(_parameter->Turn_vel, _parameter->Turn_vel, 0.09 * 4.0,
				7.0,
				true, false);
		sprint_turn(_parameter->Turn_deg, _dir, _parameter, 7.0, true, false);
		Flag_degConEn = true;
		trapezoid_acccel(_parameter->Turn_vel, 0.0, 0.09 * 4.0, 7.0,
		false, false);
		break;
	case Turn180:
		trapezoid_acccel(_parameter->Turn_vel, _parameter->Turn_vel, 0.09 * 4.0,
				10,
				true, false);
		sprint_turn(_parameter->Turn_deg, _dir, _parameter, 7.0, true, false);
		Flag_degConEn = true;
		trapezoid_acccel(_parameter->Turn_vel, 0.0, 0.09 * 4.0, 10,
		false, false);
		break;
	case Turn45_I:
		trapezoid_acccel(_parameter->Turn_vel, _parameter->Turn_vel, 0.09 * 3.0,
				7.0,
				true, false);
		sprint_turn(_parameter->Turn_deg, _dir, _parameter, 7.0, true, true);
		Flag_degConEn = true;
		trapezoid_acccel(_parameter->Turn_vel, 0.0, 0.127 * 2.0, 7.0,
		false, false);
		break;
	case Turn45_O:
		Flag_diagMode = true;
		Flag_degConEn = true;
		trapezoid_acccel(_parameter->Turn_vel, _parameter->Turn_vel,
				0.127 * 2.0, 7.0, false,
				false);
		Flag_degConEn = false;
		sprint_turn(_parameter->Turn_deg, _dir, _parameter, 7.0, true, true);
		Flag_diagMode = false;
		Flag_degConEn = true;
		trapezoid_acccel(_parameter->Turn_vel, 0.0, 0.09 * 2.0, 10,
		false, false);
		break;
	case Turn135_I:
		trapezoid_acccel(_parameter->Turn_vel, _parameter->Turn_vel, 0.09 * 4.0,
				10,
				true, false);
		sprint_turn(_parameter->Turn_deg, _dir, _parameter, 7.0, true, true);
		Flag_degConEn = true;
		trapezoid_acccel(_parameter->Turn_vel, 0.0, 0.127 * 2.0, 10,
		false, false);
		break;
	case Turn135_O:
		Flag_diagMode = true;
		Flag_degConEn = true;
		trapezoid_acccel(_parameter->Turn_vel, _parameter->Turn_vel,
				0.127 * 2.0, 10, false,
				false);
		Flag_degConEn = false;
		sprint_turn(_parameter->Turn_deg, _dir, _parameter, 10, true, false);
		Flag_diagMode = false;
		Flag_degConEn = true;
		trapezoid_acccel(_parameter->Turn_vel, 0.0, 0.09 * 2.0, 10,
		false, false);
		break;
	case Turn90_D:
		Flag_diagMode = true;
		trapezoid_acccel(_parameter->Turn_vel, _parameter->Turn_vel,
				0.127 * 2.0, 10, false,
				false);
		sprint_turn(_parameter->Turn_deg, _dir, _parameter, 10, true, true);
		Flag_diagMode = false;
		Flag_degConEn = true;
		trapezoid_acccel(_parameter->Turn_vel, 0.0, 0.127 * 2.0, 10,
		false, false);
		break;
	case Turn_U:
		trapezoid_acccel(_parameter->Turn_vel, _parameter->Turn_vel, 0.09 * 2.0,
				5.0,
				true, false);
		turnU();
		idealState.dis = 0.0;
		realState.dis = 0.0;
		trapezoid_acccel(_parameter->Turn_vel, 0.0, 0.09 * 2.0, 5.0,
		true, false);
		break;
	}
	Flag_degConEn = false;
	Flag_diagMode = false;
	idealState.acc = 0;
	idealState.vel = 0;
	HAL_Delay(100);
	if (_isSuction) {
		disable_suction();
	}
	Flag_logEn = false;
	disable_motor();
}

