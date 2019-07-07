/*
 * global.h
 *
 *  Created on: 2018/06/13
 *      Author: ����
 */

#ifndef GLOBAL_H_
#define GLOBAL_H_

#include "const.h"
#include "stm32f7xx_hal.h"
#include "define.h"

typedef struct { //�Z���T�[�l
	volatile float LF, LS, RS, RF;
	volatile int16_t RSw, LSw;
} __IRSensors;

typedef struct { //�Z���T�[�l����
	volatile float LF, LS, RS, RF;
} __IRSensors_dif;

typedef struct { //�ԑ̏��
	volatile float dis; //����
	volatile float vel_R; //�E�^�C�����x
	volatile float vel_L; //���^�C�����x
	volatile float vel; //�d�S���x

	volatile float acc;
	volatile float jerk;
	volatile float ang_jerk;

	volatile float deg; //�p�x
	volatile float ang_vel; //�p���x
	volatile float ang_acc; //�p�����x

	volatile float enc_deg_R; //�G���R�[�_�[�p�x
	volatile float enc_deg_L;

} __Mouse_State;

typedef struct { //�Q�C��
	float P;
	float I;
	float D;
} __Gain;

typedef struct { //�΍�
	float P;
	float I;
	float D;
	float ex_P; //�O��̒l
} __Error;

typedef struct {
	uint8_t Forward;
	uint8_t Left;
	uint8_t Right;
} __WallExist;

extern ADC_HandleTypeDef hadc1;
extern DMA_HandleTypeDef hdma_adc1;

extern SPI_HandleTypeDef hspi1;

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim8;

extern UART_HandleTypeDef huart1;

extern __IRSensors IR_Sen_raw; //�Z���T�[���l
extern __IRSensors IR_Sen;
extern __IRSensors ex_IR_Sen[EX_IR_SEN_NUM]; //�ߋ�5ms�̃Z���T�[�l
extern __IRSensors_dif IR_Sen_dif_1;
extern __IRSensors_dif ex_IR_Sen_dif_1[EX_IR_SEN_NUM]; //�ߋ�5ms�̃Z���T�[�l
extern __IRSensors IR_Ref_single;
extern __IRSensors IR_Ref_double;
extern __IRSensors IR_Det;
extern __IRSensors IR_Con;
extern __IRSensors IR_Coefficient; //�Z���T�[�W��
extern __IRSensors IR_Segment; //�Z���T�[�ؕ�
extern __Mouse_State idealState; //���z���
extern __Mouse_State realState; //�������
extern const __Mouse_State RESETState; //���Z�b�g�p

extern __Gain gain_vel; //���������Q�C��
extern __Gain gain_dis; //�ʒu�Q�C��
extern __Gain gain_ang_vel; //��]�����Q�C��
extern __Gain gain_deg; //��]�����Q�C��
extern __Gain gain_wall_S; //�ǐ���Q�C��
extern __Gain gain_wall_S_D; //�ǐ���Q�C��
extern __Gain gain_wall_F_R; //�ǐ���Q�C��
extern __Gain gain_wall_F_L; //�ǐ���Q�C��

extern __Error error_vel; //���x�΍�
extern __Error error_dis; //�ʒu�΍�
extern __Error error_ang_vel; //�p���x�΍�
extern __Error error_deg; //�p�x�΍�
extern __Error error_wall_S; //�ǐ���΍�
extern __Error error_wall_S_D; //�ǐ���΍�
extern __Error error_wall_F_R; //�ǐ���΍�
extern __Error error_wall_F_L; //�ǐ���΍�
extern const __Error RESET_ERROR;

extern __WallExist wallExist;

extern volatile float V_Batt;

extern volatile uint16_t ADC_Buffer[ADC_BUFFER_LENGTH]; //AD�ϊ��o�b�t�@

extern volatile uint8_t timeON, timeOFF;

extern volatile uint16_t waitTime;

extern volatile uint8_t current_coord_x, current_coord_y;

extern volatile uint8_t Goal_X, Goal_Y, Goal_L;

extern volatile uint8_t Flag_FanStateEn;

extern volatile uint8_t Flag_failSafeEn;

extern volatile uint8_t Flag_wallConEn_S;

extern volatile uint8_t Flag_wallConEn_F;

extern volatile uint8_t Flag_degConEn;

extern volatile uint8_t Flag_modeSelectEn;

extern volatile uint8_t Flag_IR_SensorEn;

extern volatile float countTime;

extern volatile uint8_t Flag_countTimeEn;

extern volatile uint8_t Flag_exploreDone;

extern volatile uint8_t Flag_logEn;

extern volatile float log_ideal_vel[LOG_NUM], log_real_vel[LOG_NUM],
		log_ideal_dis[LOG_NUM],
		log_real_dis[LOG_NUM], //
		log_ideal_avel[LOG_NUM],
		log_real_avel[LOG_NUM], //
		log_ideal_deg[LOG_NUM],
		log_real_deg[LOG_NUM], //
		log_duty_R[LOG_NUM],
		log_duty_L[LOG_NUM], //
		log_Sen_LF[LOG_NUM], log_Sen_LS[LOG_NUM], log_Sen_RS[LOG_NUM],
		log_Sen_RF[LOG_NUM], log_Sen_dLF[LOG_NUM], log_Sen_dLS[LOG_NUM],
		log_Sen_dRS[LOG_NUM], log_Sen_dRF[LOG_NUM], log_VBatt[LOG_NUM],
		log_real_vel_R[LOG_NUM], log_real_vel_L[LOG_NUM];

extern volatile uint8_t suction_ONtime, suction_OFFtime;

extern volatile uint8_t Flag_diagMode;

extern volatile int16_t gyro_ref;

extern volatile uint8_t Flag_gyroEn;

extern volatile uint8_t Flag_MotorEn; //���[�^�[�L��or����

extern volatile uint8_t Flag_encEn; //�G���R�[�_�[�L��or����

extern volatile uint8_t Flag_IRLEDEn;

extern volatile uint8_t Flag_FanAccel;

extern volatile uint8_t Flag_Check_IR_Sen_Mode;

extern volatile int16_t mode_gyro_X, mode_gyro_Y, mode_gyro_Z;

extern volatile int16_t g_Duty_R, g_Duty_L;

#endif /* GLOBAL_H_ */
