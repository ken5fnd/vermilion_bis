/*
 * menu.c
 *
 *  Created on: 2018/09/03
 *      Author: 健悟
 */
#include <stdint.h>
#include "global.h"
#include "const.h"
#include "stm32f7xx_hal.h"
#include "main.h"
#include "misc.h"
#include <math.h>
#include "gyro.h"
#include "encorder.h"
#include "maze.h"
#include "define.h"
#include "motor.h"
#include "drive.h"
#include "flash.h"
#include "wallsensor.h"

uint8_t ModeSelect() {
	uint8_t mode = 0;
	uint8_t LED_L_mode;
	reset: LED_L_mode = 0x10;
	LED(0x10);
	Flag_modeSelectEn = true;
	while (fabs((float) mode_gyro_Y) < 15000.0) {
		if (fabs((float) mode_gyro_X) < 20000.0) {
			HAL_Delay(1);
		} else {
			if (mode_gyro_X < 0) {
				mode++;
				if (mode == 4) {
					mode = 0;
				}
				Speaker_ON(&htim2);
				Speaker_Herz(&htim2, Sc_C);
				HAL_Delay(70);
				Speaker_OFF(&htim2);
			} else if (mode_gyro_X > 0) {

				if (mode == 0) {
					mode = 3;
				} else {
					mode--;
				}
				Speaker_ON(&htim2);
				Speaker_Herz(&htim2, Sc_B);
				HAL_Delay(70);
				Speaker_OFF(&htim2);
			}
			switch (mode) {
			case 0:
				LED(0x10);
				LED_L_mode = 0x10;
				break;
			case 1:
				LED(0x30);
				LED_L_mode = 0x30;
				break;
			case 2:
				LED(0x70);
				LED_L_mode = 0x70;
				break;
			case 3:
				LED(0xF0);
				LED_L_mode = 0xF0;
				break;
			}
			HAL_Delay(10);
		}
	}
	Speaker_Herz(&htim2, Sc_Ch);
	Speaker_ON(&htim2);
	HAL_Delay(100);
	Speaker_OFF(&htim2);
	HAL_Delay(100);

	Speaker_ON(&htim2);
	HAL_Delay(100);
	Speaker_OFF(&htim2);
	HAL_Delay(100);

	LED(0x08 | LED_L_mode);
	while (fabs((float) mode_gyro_Y) < 15000.0) {
		if (fabs((float) mode_gyro_Z) > 15000.0) {
			LED(0x00);
			Speaker_Herz(&htim2, Sc_Cl);
			Speaker_ON(&htim2);
			HAL_Delay(70);
			Speaker_OFF(&htim2);
			HAL_Delay(70);

			Speaker_ON(&htim2);
			HAL_Delay(70);
			Speaker_OFF(&htim2);
			HAL_Delay(70);

			Speaker_ON(&htim2);
			HAL_Delay(70);
			Speaker_OFF(&htim2);
			HAL_Delay(70);
			mode = 0;

			goto reset;
		}
		if (fabs((float) mode_gyro_X) < 20000.0) {
			HAL_Delay(1);
		} else {
			if (mode_gyro_X < 0) {
				if (mode >> 4 == 3) {
					mode = (mode | 0xF0) ^ 0xF0;
				} else {
					mode += 16;
				}
				Speaker_ON(&htim2);
				Speaker_Herz(&htim2, Sc_C);
				HAL_Delay(70);
				Speaker_OFF(&htim2);
			} else if (mode_gyro_X > 0) {
				if (mode >> 4 == 0) {
					mode = mode | 0x30;
				} else {
					mode -= 16;
				}
				Speaker_ON(&htim2);
				Speaker_Herz(&htim2, Sc_B);
				HAL_Delay(70);
				Speaker_OFF(&htim2);
			}

			switch (mode >> 4) {
			case 0:
				LED(0x08 | LED_L_mode);
				break;
			case 1:
				LED(0x0C | LED_L_mode);
				break;
			case 2:
				LED(0x0E | LED_L_mode);
				break;
			case 3:
				LED(0x0F | LED_L_mode);
				break;
			}
			HAL_Delay(10);
		}
	}
	Speaker_Herz(&htim2, Sc_Ch);
	Speaker_ON(&htim2);
	HAL_Delay(70);
	Speaker_OFF(&htim2);
	HAL_Delay(70);

	Speaker_ON(&htim2);
	HAL_Delay(70);
	Speaker_OFF(&htim2);
	HAL_Delay(70);

	Speaker_ON(&htim2);
	HAL_Delay(70);
	Speaker_OFF(&htim2);
	HAL_Delay(70);
	LED(0x00);
	HAL_Delay(200);
	Flag_modeSelectEn = false;
	return mode;
}
void selectmenu() {
	uint8_t mode = ModeSelect();
	switch (mode & 0x0F) {
	case 0: //探索
//		gain_ang_vel.P = 1.2; //角速度ゲイン
//		gain_ang_vel.I =  10.0; //角速度ゲイン
//		gain_deg.P = 0.02; //角度ゲイン
		Flag_IRLEDEn = true;
		HAL_Delay(100);
		while (IR_Sen.RF > 6.0)
			;
		Speaker_Scale(Sc_Dl, 70);
		Speaker_Scale(Sc_Ch, 70);
		/*ログ消去*/
		for (int i = 0; i < LOG_NUM; i++) {
			log_ideal_vel[i] = 0.0;
			log_real_vel[i] = 0.0;
		}
		gyro_ref = gyro_get_reference(); //オフセット補正
		Flag_encEn = true;
		Flag_gyroEn = true;
		HAL_Delay(10);
		enable_motor();
		reset_status();
		reset_error();
		Flag_logEn = true;
		trapezoid_acccel(explore_turn_R.Turn_vel, explore_turn_R.Turn_vel,
				0.09 * 1.0, 7.0, true, false);
		mapRenew(0);
		wall_exist();
		AdachiMethod(Goal_X, Goal_Y, true);
		idealState.dis = 0.0;
		realState.dis = 0.0;
		trapezoid_acccel(explore_turn_R.Turn_vel, 0.0, 0.09, 7.0, true, false);
		writeWallData(current_coord_x, current_coord_y, direction);
		HAL_Delay(500);
		disable_motor();
		BackUpWallData(Write);
		if (!Flag_failSafeEn) {
			HAL_Delay(500);
			Speaker_Scale(Sc_Dl, 70);
			HAL_Delay(70);
			Speaker_Scale(Sc_Ch, 70);
			Speaker_Scale(Sc_Bl, 70);
			HAL_Delay(100);
			if ((mode & 0xF0) >> 4 != 3) { //帰り探索する場合
				for (int i = 0; i < 16; i++) {
					half_wall_ver[i] = wall_ver[i];
					half_wall_hor[i] = wall_hor[i];

					half_wall_read_ver[i] = wall_read_ver[i];
					half_wall_read_hor[i] = wall_read_hor[i];
				}
				reset_status();
				enable_motor();
				if (wallExist.Forward == true) {
					HAL_Delay(100);
					if (fabs(IR_Sen.RF - IR_Ref_single.RF) > 0.3
							|| fabs(IR_Sen.LF - IR_Ref_single.LF) > 0.3) {
						FrontWallControl();
					}
					if (wallExist.Right == true
							&& fabs(IR_Sen.RS - IR_Sen.LS) > 0.5) {
						trapezoid_slalome(900.0, -90.0, 10000.0);
						FrontWallControl();
						trapezoid_slalome(900.0, -90.0, 10000.0);
					} else if (wallExist.Left == true
							&& fabs(IR_Sen.RS - IR_Sen.LS) > 0.5) {
						trapezoid_slalome(900.0, 90.0, 10000.0);
						FrontWallControl();
						trapezoid_slalome(900.0, 90.0, 10000.0);
					} else {
						trapezoid_slalome(900.0, 180.0, 10000.0);
					}

					HAL_Delay(100);
					trapezoid_acccel(explore_turn_R.Turn_vel,
							explore_turn_R.Turn_vel, 0.09, 7.0,
							true, false);
					mapRenew(180);
				} else {
					reset_status();
					reset_error();
					trapezoid_acccel(explore_turn_R.Turn_vel,
							explore_turn_R.Turn_vel, 0.09, 7.0, true,
							false);
					mapRenew(0);
				}
				wall_exist();
				SearchAlgorithm();
				if (Flag_failSafeEn == true) {
					for (int i = 0; i < 16; i++) {
						wall_ver[i] = half_wall_ver[i];
						wall_hor[i] = half_wall_hor[i];

						wall_read_ver[i] = half_wall_read_ver[i];
						wall_read_hor[i] = half_wall_read_hor[i];
					}

				}
				if (!Flag_failSafeEn) {
					trapezoid_acccel(explore_turn_R.Turn_vel, 0.0, 0.09, 5.0,
					true, false);
					trapezoid_slalome(700.0, 180.0, 10000.0);
					reset_status();
					reset_error();
					HAL_Delay(300);
					mapRenew(180);
					BackUpWallData(Write);
				}
				stepMapRenew(Goal_X, Goal_Y, true, true);
			}
			disable_motor();
		}
		current_coord_x = 0;
		current_coord_y = 0;
		break;
	case 1: //最短
//		gain_ang_vel.P = 1.0; //角速度ゲイン
//		gain_ang_vel.I =  7.0; //角速度ゲイン
//		gain_deg.P = 0.02; //角度ゲイン
		wall_read_hor[0] = wall_read_hor[0] | 1;
		stepMapRenew_DiagMap(Goal_X, Goal_Y, true, true);
		if (DiagStepMap_hor[0][0] == 0xFFFF) { //パスが詰まった場合は壁を消す
			Flag_failSafeEn = true;
			break;
		}
		generatePass(true);
		Flag_IRLEDEn = true;
		HAL_Delay(100);
		while (IR_Sen.RF > 6.0)
			;
		Speaker_ON(&htim2);
		Speaker_Herz(&htim2, 587);
		HAL_Delay(70);
		Speaker_Herz(&htim2, 2093);
		HAL_Delay(70);
		Speaker_OFF(&htim2);
		gyro_ref = gyro_get_reference();
		Flag_encEn = true;
		Flag_gyroEn = true;
		reset_status();
		reset_error();
		HAL_Delay(100);
		enable_motor();
		switch ((mode & 0xF0) >> 4) {
		case 0:
			SprintRun(&parameter1, true, 100);
			break;
		case 1:
			SprintRun(&parameter2, true, 100);
			break;
		case 2:
			SprintRun(&parameter3, true, 100);
			break;
		case 3:
			SprintRun(&parameter4, true, 100);
			break;
		}
		if (!Flag_failSafeEn) {
			disable_suction();
			disable_motor();
			HAL_Delay(200);
			LED(0x00);
		}
		break;
	case 2: //マップ表示
		switch ((mode & 0xF0) >> 4) {
		case 0: //ログ表示
			stepMapRenew(Goal_X, Goal_Y, true, true);
			stepMapRenew_DiagMap(Goal_X, Goal_Y, true, true);
			printMaze(true);
			generatePass(true);
			printPass();
			break;
		case 1:
			BackUpWallData(Read);
			wall_read_hor[0] = wall_read_hor[0] | 1;
			Speaker_ON(&htim2);
			Speaker_Herz(&htim2, 587);
			HAL_Delay(70);
			Speaker_Herz(&htim2, 2093);
			HAL_Delay(70);
			Speaker_OFF(&htim2);
			break;
		case 2: //センサー値表示
			Flag_IRLEDEn = true;
//			HAL_Delay(100);
//			while (IR_Sen.RF > 6.0)
//				;
//			Speaker_ON(&htim2);
//			Speaker_Herz(&htim2, 587);
//			HAL_Delay(70);
//			Speaker_Herz(&htim2, 2093);
//			HAL_Delay(70);
//			Speaker_OFF(&htim2);
			gyro_ref = gyro_get_reference();
////			printf("ref=%f\n", (float)gyro_ref);
			Flag_gyroEn = true;
			while (1) {
//				printf("\rLF=%6.3f,LS=%6.3f,RS=%6.3f,RF=%6.3f", IR_Sen.LF,
//						IR_Sen.LS, IR_Sen.RS, IR_Sen.RF);
				printf("%f\n", realState.ang_vel);
				HAL_Delay(100);
			}
			printf("\n");
			break;
		case 3: //ログ表示
			print_log();
			break;
		}
		break;
	case 3: //デバッグ、その他
		switch ((mode & 0xF0) >> 4) {
		case 0: //走行テスト
			Flag_IRLEDEn = true;
			HAL_Delay(100);
			while (IR_Sen.RF > 6.0)
				;
			Speaker_ON(&htim2);
			Speaker_Herz(&htim2, 587);
			HAL_Delay(70);
			Speaker_Herz(&htim2, 2093);
			HAL_Delay(70);
			Speaker_OFF(&htim2);
			/*ログ消去*/
			for (int i = 0; i < LOG_NUM; i++) {
				log_ideal_vel[i] = 0.0;
				log_real_vel[i] = 0.0;
			}
			gyro_ref = gyro_get_reference();
			printf("%d\n", gyro_ref);
			Flag_encEn = true;
			Flag_gyroEn = true;
			//HAL_Delay(10);
			reset_status();
			reset_error();
			HAL_Delay(100);
			enable_motor();
			HAL_Delay(100);
//			enable_suction(100);
//			HAL_Delay(500);
//			Flag_diagMode = true;
//			Flag_degConEn = true;
			Flag_logEn = true;
			TEST_TURN;
			reset_status();
			reset_error();
			HAL_Delay(500);
			disable_suction();
			HAL_Delay(500);
			disable_motor();
			realState = RESETState;
			Flag_logEn = false;
			Flag_gyroEn = false;
			break;
		case 1:
			Flag_IRLEDEn = true;
			HAL_Delay(100);
			while (IR_Sen.RF > 6.0)
				;
			Speaker_ON(&htim2);
			Speaker_Herz(&htim2, 587);
			HAL_Delay(70);
			Speaker_Herz(&htim2, 2093);
			HAL_Delay(70);
			Speaker_OFF(&htim2);
			/*ログ消去*/
			for (int i = 0; i < LOG_NUM; i++) {
				log_ideal_vel[i] = 0.0;
				log_real_vel[i] = 0.0;
			}
			gyro_ref = gyro_get_reference();
			Flag_encEn = true;
			Flag_gyroEn = true;
			enable_motor();
			Flag_logEn = true;
			HAL_Delay(100);
			reset_status();
			reset_error();

//			HAL_GPIO_WritePin(MOT_R_STB_GPIO_Port, MOT_R_STB_Pin, 1); //
//			HAL_GPIO_WritePin(MOT_L_STB_GPIO_Port, MOT_L_STB_Pin, 1); //
//			set_motor_duty(300, 300);

			trapezoid_acccel(1.0, 1.0, 0.09 * 1.0, 7.0, true, false);
			edge_break(Left, 1.0, 10.0);
			idealState.dis = 0.0;
			realState.dis = 0.0;
			trapezoid_acccel(1.0, 0.0, 0.09 * 1.0, 7.0, true, false);
			HAL_Delay(3000);
			disable_motor();
			Flag_logEn = false;
			break;
		case 2:
			Flag_IRLEDEn = true;
			Flag_Check_IR_Sen_Mode = true;
			HAL_Delay(100);
			for (uint8_t i = 1; i <= 30; i++) {
				while (IR_Sen.RF < 600.0)
					;
				Speaker_ON(&htim2);
				Speaker_Herz(&htim2, 587);
				HAL_Delay(70);
				Speaker_Herz(&htim2, 2093);
				HAL_Delay(70);
				Speaker_OFF(&htim2);
				printf("%4.1f, %f\n", IR_Sen.LS, (float) i / 2.0);
				HAL_Delay(100);
			}
			break;
		case 3:
			Flag_IRLEDEn = true;
			HAL_Delay(100);
			while (IR_Sen.RF > 6.0)
				;
			Speaker_ON(&htim2);
			Speaker_Herz(&htim2, 587);
			HAL_Delay(70);
			Speaker_Herz(&htim2, 2093);
			HAL_Delay(70);
			Speaker_OFF(&htim2);
			enable_suction(100);
			LED(0xFF);
			while (1) {
				printf("%f\n", V_Batt);
				HAL_Delay(10);
			}
			break;
		}
		break;
	}
}

