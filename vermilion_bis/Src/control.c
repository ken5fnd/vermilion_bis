/*
 * control.c
 *
 *  Created on: 2018/06/23
 *      Author: 健悟
 */

#include "main.h"
#include "stm32f7xx_hal.h"
#include "misc.h"
#include "const.h"
#include "global.h"
#include "encorder.h"
#include "gyro.h"
#include <math.h>
#include <stdlib.h>

void calc_error() { ///制御量算
	float gap_R = 0.06; //右壁変化量
	float gap_L = 0.06; //左壁変化量
	/*壁制御の制御量*/
	if (Flag_wallConEn_S == true) { //横壁制御
		if (Flag_diagMode == false) { //普通の壁制御
			error_wall_S_D.P = 0.0;
			error_wall_S_D.D = 0.0;
			error_wall_S_D.ex_P = 0.0;
			if (IR_Sen.RF > THR_WALLCON_RF) { //横壁制御
				if ((IR_Sen.RS < IR_Con.RS && IR_Sen_dif_1.RS > -0.3
						&& IR_Sen_dif_1.RS < gap_R)
						&& (IR_Sen.LS < IR_Con.LS && IR_Sen_dif_1.LS > -0.3
								&& IR_Sen_dif_1.LS < gap_L)) { //両壁
					error_wall_S.P = (IR_Ref_double.RS - IR_Sen.RS)
							- (IR_Ref_double.LS - IR_Sen.LS);
					error_wall_S.D = (error_wall_S.P - error_wall_S.ex_P)
							/ CONTROL_CYCLE;
					error_wall_S.ex_P = error_wall_S.P;

				} else if (IR_Sen.RS < IR_Con.RS && IR_Sen_dif_1.RS > -0.3
						&& IR_Sen_dif_1.RS < gap_R) { //右壁
					error_wall_S.P = (IR_Ref_single.RS - IR_Sen.RS) * 2.0;
					error_wall_S.D = (error_wall_S.P - error_wall_S.ex_P)
							/ CONTROL_CYCLE;
					error_wall_S.ex_P = error_wall_S.P;

				} else if (IR_Sen.LS < IR_Con.LS && IR_Sen_dif_1.LS > -0.3
						&& IR_Sen_dif_1.LS < gap_L) { //左壁
					error_wall_S.P = (IR_Sen.LS - IR_Ref_single.LS) * 2.0;
					error_wall_S.D = (error_wall_S.P - error_wall_S.ex_P)
							/ CONTROL_CYCLE;
					error_wall_S.ex_P = error_wall_S.P;

				} else { //壁なし
					error_wall_S.P = 0.0;
					error_wall_S.D = 0.0;
					error_wall_S.ex_P = 0.0;
				}
				error_ang_vel.I = 0.0;
			} else { //前距離センサが近すぎる場合
				error_wall_S.P = 0.0;
				error_wall_S.D = 0.0;
				error_wall_S.ex_P = 0.0;
			}
		} else if (Flag_diagMode == true) { //斜め制御
			error_wall_S.P = 0.0;
			error_wall_S.D = 0.0;
			error_wall_S.ex_P = 0.0;
//			if (IR_Sen.RS < 0.0) {
//				error_wall_S_D.P = (0.0 - IR_Sen.RS) * 0.5;
//			} else if (IR_Sen.LS < 0.0) {
//				error_wall_S_D.P = -(0.0 - IR_Sen.LS) * 0.5;
//			} else
			Flag_degConEn = false;
			if (IR_Sen.RF < 11.0) {
				error_wall_S_D.P = (11.0 - IR_Sen.RF);
				error_ang_vel.I = 0.0;
			} else if (IR_Sen.LF < 8.0) {
				error_wall_S_D.P = -(8.0 - IR_Sen.LF);
				error_ang_vel.I = 0.0;
			} else {
				Flag_degConEn = true;
				error_wall_S_D.P = 0.0;
				error_wall_S_D.ex_P = 0.0;
			}
			error_wall_S_D.D = (error_wall_S_D.P - error_wall_S_D.ex_P)
					/ CONTROL_CYCLE;
			error_wall_S_D.ex_P = error_wall_S_D.P;
		}
	} else if (Flag_wallConEn_F == true) { //前壁制御
		if (IR_Sen.RF < IR_Con.RF) {
			error_wall_F_R.P = IR_Sen.RF - IR_Ref_single.RF;
			error_wall_F_R.D = (error_wall_F_R.P - error_wall_F_R.ex_P)
					/ CONTROL_CYCLE;
			error_wall_F_R.ex_P = error_wall_F_R.P;
			/*角度の制御*/
			error_wall_F_L.P = IR_Sen.LF - IR_Ref_single.LF;
			error_wall_F_L.D = (error_wall_F_L.P - error_wall_F_L.ex_P)
					/ CONTROL_CYCLE;
			error_wall_F_L.ex_P = error_wall_F_L.P;
		} else {
			error_wall_F_R.P = 0;
			error_wall_F_R.D = 0;
			error_wall_F_R.ex_P = 0;

			error_wall_F_L.P = 0;
			error_wall_F_L.D = 0;
			error_wall_F_L.ex_P = 0;
		}
	} else {
		error_wall_S.P = 0.0;
		error_wall_S.D = 0.0;
		error_wall_S.ex_P = 0.0;
	}

	if (Flag_wallConEn_F == false) {
		error_wall_F_R.P = 0;
		error_wall_F_R.D = 0;
		error_wall_F_R.ex_P = 0;

		error_wall_F_L.P = 0;
		error_wall_F_L.D = 0;
		error_wall_F_L.ex_P = 0;
	}

	/*角度の制御量*/
	if (Flag_degConEn) {
		error_deg.P = idealState.deg - realState.deg;
		error_deg.I += error_deg.P * CONTROL_CYCLE;
		error_deg.D = (error_deg.P - error_deg.ex_P) / CONTROL_CYCLE;
		error_deg.ex_P = error_deg.P;
	} else {
		error_deg.P = 0.0;
		error_deg.I = 0.0;
		error_deg.D = 0.0;
	}

	/*角速度の制御量*/
	if (Flag_FanAccel == false) {
		error_ang_vel.P =
				idealState.ang_vel
						- realState.ang_vel+ (error_wall_S.P * gain_wall_S.P + error_wall_S.D * gain_wall_S.D)
						* (idealState.acc != 0.0 ? 0.8 : 1.0)
						+ (error_wall_S_D.P * gain_wall_S_D.P
								+ error_wall_S_D.D * gain_wall_S_D.D)
						* (idealState.acc != 0.0 ? 0.7 : 1.0)
						+ (error_deg.P * gain_deg.P + error_deg.I * gain_deg.I
								+ error_deg.D * gain_deg.D) / CONTROL_CYCLE;
		error_ang_vel.I += error_ang_vel.P * CONTROL_CYCLE;
		error_ang_vel.D =
				(error_ang_vel.ex_P - error_ang_vel.P) / CONTROL_CYCLE;
		error_ang_vel.ex_P = error_ang_vel.P;
	} else {
		error_ang_vel.P = 0.0;
		error_ang_vel.I = 0.0;
		error_ang_vel.D = 0.0;
	}
	/*位置の制御量*/
	error_dis.P = idealState.dis - realState.dis;
	error_dis.I += error_dis.P * CONTROL_CYCLE;
	error_dis.D = (error_dis.P - error_dis.ex_P) / CONTROL_CYCLE;
	error_dis.ex_P = error_dis.P;

	/*速度の制御量*/
	error_vel.P = idealState.vel - realState.vel
			+ (error_dis.P * gain_dis.P + error_dis.I * gain_dis.I
					+ error_dis.D * gain_dis.D);

	error_vel.I += error_vel.P * CONTROL_CYCLE;
	error_vel.D = (error_vel.P - error_vel.ex_P) / CONTROL_CYCLE;
	error_vel.ex_P = error_vel.P;
}

