/*
 * motor.c
 *
 *  Created on: 2018/07/02
 *      Author: 健悟
 */
#include "global.h"
#include "const.h"
#include <math.h>
#include <stdlib.h>

void set_motor_duty(int16_t _duty_R, int16_t _duty_L) {
	/*右モーター*/
	if (_duty_R == 0) {
		HAL_GPIO_WritePin(MOT_R_PH_1_GPIO_Port, MOT_R_PH_1_Pin, 1); //
		HAL_GPIO_WritePin(MOT_R_PH_2_GPIO_Port, MOT_R_PH_2_Pin, 1); //
	} else if (_duty_R > 0) {
		HAL_GPIO_WritePin(MOT_R_PH_1_GPIO_Port, MOT_R_PH_1_Pin, 1); //
		HAL_GPIO_WritePin(MOT_R_PH_2_GPIO_Port, MOT_R_PH_2_Pin, 0); //
		__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,abs(_duty_R));
	} else {
		HAL_GPIO_WritePin(MOT_R_PH_1_GPIO_Port, MOT_R_PH_1_Pin, 0); //
		HAL_GPIO_WritePin(MOT_R_PH_2_GPIO_Port, MOT_R_PH_2_Pin, 1); //
		__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,abs(_duty_R));
	}

	/*左モーター*/
	if (_duty_L == 0) {
		HAL_GPIO_WritePin(MOT_L_PH_1_GPIO_Port, MOT_L_PH_1_Pin, 1); //
		HAL_GPIO_WritePin(MOT_L_PH_2_GPIO_Port, MOT_L_PH_2_Pin, 1); //
	} else if (_duty_L > 0) {
		HAL_GPIO_WritePin(MOT_L_PH_1_GPIO_Port, MOT_L_PH_1_Pin, 1); //
		HAL_GPIO_WritePin(MOT_L_PH_2_GPIO_Port, MOT_L_PH_2_Pin, 0); //
		__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_2,abs(_duty_L));
	} else {
		HAL_GPIO_WritePin(MOT_L_PH_1_GPIO_Port, MOT_L_PH_1_Pin, 0); //
		HAL_GPIO_WritePin(MOT_L_PH_2_GPIO_Port, MOT_L_PH_2_Pin, 1); //
		__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_2,abs(_duty_L));
	}
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
}

void enable_motor() {
	Flag_MotorEn = true;
	HAL_GPIO_WritePin(MOT_R_STB_GPIO_Port, MOT_R_STB_Pin, 1); //
	HAL_GPIO_WritePin(MOT_L_STB_GPIO_Port, MOT_L_STB_Pin, 1); //
	set_motor_duty(0, 0);
}

void disable_motor() {
	Flag_MotorEn = false;
	HAL_GPIO_WritePin(MOT_R_STB_GPIO_Port, MOT_R_STB_Pin, 0); //
	HAL_GPIO_WritePin(MOT_L_STB_GPIO_Port, MOT_L_STB_Pin, 0); //
	HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_2);
	HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_2);
}

void setup_suction() {
	suction_ONtime = 10;
	suction_OFFtime = 190;
}

void enable_suction(uint8_t _duty) {
	Flag_FanStateEn = true;
	//Flag_FanAccel = true;
	for (uint8_t i = 1; i <= _duty / 10; i++) {
		suction_ONtime = 10 + i;
		suction_OFFtime = 190 - i;
		HAL_Delay(100);
	}
	HAL_Delay(500);
	reset_error();
	reset_status();
	Flag_FanAccel = false;

}

void disable_suction() {
	Flag_FanStateEn = false;
	disable_motor();
//	idealState.deg = 0.0;
//	Flag_degConEn = true;
//	for (uint8_t i = 1; suction_ONtime > 10; i++) {
//		suction_ONtime -= i;
//		suction_OFFtime += i;
//		HAL_Delay(100);
//	}
	suction_ONtime = 10;
	suction_OFFtime = 190;
	HAL_Delay(500);
	//Flag_degConEn = false;

}

void drive_motor() {
	int16_t duty_R, duty_L;
	float rpm_R, rpm_L,	 	 //タイヤ回転数 (/min)
			Volt_R, Volt_L,	 //出力モーター電圧(V)
			Torque_R, Torque_L; //要求出力トルク(mNm)

	/*右モーター*/
	rpm_R = realState.vel_R / WHEEL_DIA_R / M_PI * 4.0 * 60.0; //回転数(/min) = タイヤ速(m/s)/タイヤ半径(m)*2/π*ギヤ比*60
	Volt_R = 0.0;//rpm_R / 8950.0; //逆起電圧(V) = 回転数*逆起電圧定数(rpm/V)
	Torque_R = (idealState.acc * MASS
			+ idealState.ang_acc / 180.0 * M_PI * TREAD / 2.0 * MOMENT_R)
			* WHEEL_DIA_R / 4.0 * 1000.0; //要求トルク(mNm) = 重量(kg)*加速度(m/s/s)*タイヤ半径(m)/ギヤ比/モーター個数*1000

	Volt_R += Torque_R / 1.07 * 0.355; //要求電圧(V) = 要求トルク(mNm)/トルク定数(mNm/A)*端子間抵抗(Ω)

	duty_R = (int16_t) (((Volt_R) / V_Batt * 1000.0
			+ ((error_vel.P * gain_vel.P + error_vel.I * gain_vel.I
					+ error_vel.D * gain_vel.D)
					+ (error_ang_vel.P * gain_ang_vel.P
							+ error_ang_vel.I * gain_ang_vel.I
							+ error_ang_vel.D * gain_ang_vel.D))
					* (Flag_wallConEn_F == true ? 0.0 : 1.0)
			+ (error_wall_F_R.P * gain_wall_F_R.P
					+ error_wall_F_R.D * gain_wall_F_R.D)
					* (Flag_wallConEn_F == true ? 1.0 : 0.0))); //出力デューティー(%) = 要求電圧(V)/電源電圧(V)

//	if (idealState.acc > 0.0) {
//		duty_R += (int16_t) (7 * fabs(idealState.acc));
//	} else if (idealState.acc < 0.0) {
//		duty_R += (int16_t) (7 * fabs(idealState.acc));;
//	} else {
//		duty_R += 17;
//	}

	if (duty_R > 999) {
		duty_R = 999;
	} else if (duty_R < -999) {
		duty_R = -999;
	}

	/*左モーター*/
	rpm_L = realState.vel_L / WHEEL_DIA_L / M_PI * 4.0 * 60.0; //回転数(/min) = タイヤ速(m/s)/タイヤ半径(m)*2/π*ギヤ比*60
	Volt_L = 0.0; //rpm_L / 8950.0; 	//逆起電圧(V) = 回転数*逆起電圧定数(rpm/V)
	Torque_L = (idealState.acc * MASS
			- idealState.ang_acc / 180.0 * M_PI * TREAD / 2.0 * MOMENT_L)
			* WHEEL_DIA_L / 4.0 * 1000.0; //要求トルク(mNm) = 重量(kg)*加速度(m/s/s)*タイヤ半径(m)/ギヤ比/モーター個数*1000

	Volt_L += Torque_L / 1.07 * 0.355; //要求電圧(V) = 要求トルク(mNm)/トルク定数(mNm/A)*端子間抵抗(Ω)

	duty_L = (int16_t) (((Volt_L) / V_Batt * 1000.0
			+ ((error_vel.P * gain_vel.P + error_vel.I * gain_vel.I
					+ error_vel.D * gain_vel.D)
					- (error_ang_vel.P * gain_ang_vel.P
							+ error_ang_vel.I * gain_ang_vel.I
							+ error_ang_vel.D * gain_ang_vel.D))
					* (Flag_wallConEn_F == true ? 0.0 : 1.0)
			+ (error_wall_F_L.P * gain_wall_F_L.P
					+ error_wall_F_L.D * gain_wall_F_L.D)
					* (Flag_wallConEn_F == true ? 1.0 : 0.0))); //出力デューティー(%) = 要求電圧(V)/電源電圧(V)*500

//	if (idealState.acc > 0.0) {
//		duty_L -= (int16_t) (7 * fabs(idealState.acc));;
//	} else if (idealState.acc < 0.0) {
//		duty_L -= (int16_t) (7 * fabs(idealState.acc));;
//	} else {
//		duty_L -= 17;
//	}

	if (duty_L > 999) {
		duty_L = 999;
	} else if (duty_L < -999) {
		duty_L = -999;
	}

	set_motor_duty(duty_R, duty_L);
	g_Duty_R = duty_R;
	g_Duty_L = duty_L;

}
