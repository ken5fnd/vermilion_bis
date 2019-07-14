/*
 * gyro.c
 *
 *  Created on: 2018/08/02
 *      Author: Œ’Œå
 */

#include "main.h"
#include "stm32f7xx_hal.h"
#include "misc.h"
#include "const.h"
#include "global.h"
#include <math.h>

void gyro_rw(uint8_t *_Data_Input, uint8_t *_Data_Output) {
	volatile uint8_t dummy;
	hspi1.Init.CLKPolarity = SPI_POLARITY_LOW; //CPOL = 0
	HAL_GPIO_WritePin(CS_GYRO_GPIO_Port, CS_GYRO_Pin, 0);
	for (uint8_t i = 0; i < 5; i++) {
		dummy++;
	}
	HAL_SPI_TransmitReceive(&hspi1, _Data_Input, _Data_Output, 2, 10);
	HAL_GPIO_WritePin(CS_GYRO_GPIO_Port, CS_GYRO_Pin, 1);
}

void gyro_setup() {
	uint8_t Data_Input[] = { 0x00, 0x00 };
	uint8_t Data_Output[2];

	Data_Input[0] = 0xF5;
	Data_Input[1] = 0x00; //Disable Sleep
	gyro_rw(Data_Input, Data_Output);
	printf("%x\n", Data_Output[1]);
	HAL_Delay(15);

	Data_Input[0] = 0x6B;
	Data_Input[1] = 0x03; //Disable Sleep
	gyro_rw(Data_Input, Data_Output);
	HAL_Delay(100);

	Data_Input[0] = 0x68;
	Data_Input[1] = 0x07; //USER BANK 0
	gyro_rw(Data_Input, Data_Output);
	HAL_Delay(100);

	Data_Input[0] = 0x6A;
	Data_Input[1] = 0x10; //Reset
	gyro_rw(Data_Input, Data_Output);
	HAL_Delay(100);

	Data_Input[0] = 0x1B;
	Data_Input[1] = 0x18; //disable sleep
	gyro_rw(Data_Input, Data_Output);
	HAL_Delay(15);

}

int16_t get_ang_vel(uint8_t _axis) {
	uint8_t Data_Input[] = { 0x00, 0x00 };
	uint8_t Data_Output[2];
	int16_t gyro_data;

	switch (_axis) {
	case 0: //X
		Data_Input[0] = 0xC3;
		break;
	case 1: //Y
		Data_Input[0] = 0xC5;
		break;
	case 2: //Z
		Data_Input[0] = 0xC7;
		break;
	}
	gyro_rw(Data_Input, Data_Output);
	gyro_data = Data_Output[1];
	gyro_data = gyro_data << 8;
	switch (_axis) {
	case 0: //X
		Data_Input[0] = 0xC4;
		break;
	case 1: //Y
		Data_Input[0] = 0xC6;
		break;
	case 2: //Z
		Data_Input[0] = 0xC8;
		break;
	}

	gyro_rw(Data_Input, Data_Output);
	gyro_data |= (int16_t)Data_Output[1];

	return gyro_data;
}
int16_t gyro_get_reference() {
	int32_t sum_data = 0;
	Flag_gyroEn = false;
	HAL_Delay(100);
	for (uint16_t i = 0; i < 1000; i++) {
		sum_data += (int32_t) get_ang_vel(2);
		HAL_Delay(1);
	}
	return (int16_t) (sum_data / 1000);
}
