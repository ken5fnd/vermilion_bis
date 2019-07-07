/*
 * encorder.c
 *
 *  Created on: 2018/06/23
 *      Author: 健悟
 */

#include "main.h"
#include "stm32f7xx_hal.h"
#include "misc.h"
#include "const.h"
#include "global.h"
#include <math.h>

void get_velocity(__Mouse_State *_realState) { //エンコーダーの値を得る
	uint8_t Data_Input[] = { 0x00, 0x00 };
	uint8_t Data_Output[2];
	float d_deg;
	static float ex_enc_deg_R, ex_enc_deg_L;
	static float ex_vel_R, ex_vel_L;

	hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH; //CPOL = 1

	/*右エンコーダー*/
	HAL_GPIO_WritePin(CS_ENC_R_GPIO_Port, CS_ENC_R_Pin, 0);
	HAL_SPI_TransmitReceive(&hspi1, (uint8_t*) Data_Input,
			(uint8_t*) Data_Output, 2, 10);
	HAL_GPIO_WritePin(CS_ENC_R_GPIO_Port, CS_ENC_R_Pin, 1);

	_realState->enc_deg_R = (float) ((Data_Output[0] << 8)
			+ (uint16_t) Data_Output[1]) / 32767.0 * 360.0;

	d_deg = _realState->enc_deg_R - ex_enc_deg_R;

	if (d_deg > 180.0) {
		d_deg = (_realState->enc_deg_R - 360.0) - ex_enc_deg_R;
	} else if (d_deg < -180.0) {
		d_deg = _realState->enc_deg_R + (360.0 - ex_enc_deg_R);
	}

	_realState->vel_R = (d_deg / 360.0 * M_PI * WHEEL_DIA_R / CONTROL_CYCLE)
			* 0.1 + ex_vel_R * 0.9;
	ex_vel_R = _realState->vel_R;
	ex_enc_deg_R = _realState->enc_deg_R;

	/*左エンコーダー*/
	HAL_GPIO_WritePin(CS_ENC_L_GPIO_Port, CS_ENC_L_Pin, 0);
	HAL_SPI_TransmitReceive(&hspi1, (uint8_t*) Data_Input,
			(uint8_t*) Data_Output, 2, 10);
	HAL_GPIO_WritePin(CS_ENC_L_GPIO_Port, CS_ENC_L_Pin, 1);

	_realState->enc_deg_L = (float) ((Data_Output[0] << 8)
			+ (uint16_t) Data_Output[1]) / 32767.0 * 360.0;

	d_deg = _realState->enc_deg_L - ex_enc_deg_L;

	if (d_deg > 180.0) {
		d_deg = (_realState->enc_deg_L - 360.0) - ex_enc_deg_L;
	} else if (d_deg < -180.0) {
		d_deg = _realState->enc_deg_L + (360.0 - ex_enc_deg_L);
	}

	_realState->vel_L = (-d_deg / 360.0 * M_PI * WHEEL_DIA_L / CONTROL_CYCLE)
			* 0.1 + ex_vel_L * 0.9;
	ex_vel_L = _realState->vel_L;
	ex_enc_deg_L = _realState->enc_deg_L;

	_realState->vel = (_realState->vel_R + _realState->vel_L) / 2.0;

}
