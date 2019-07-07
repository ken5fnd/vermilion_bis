/*
 * global.h
 *
 *  Created on: 2018/06/13
 *      Author: 健悟
 */

#ifndef GLOBAL_H_
#define GLOBAL_H_

#include "const.h"
#include "stm32f7xx_hal.h"
#include "define.h"

typedef struct { //センサー値
	volatile float LF, LS, RS, RF;
	volatile int16_t RSw, LSw;
} __IRSensors;

typedef struct { //センサー値差分
	volatile float LF, LS, RS, RF;
} __IRSensors_dif;

typedef struct { //車体状態
	volatile float dis; //距離
	volatile float vel_R; //右タイヤ速度
	volatile float vel_L; //左タイヤ速度
	volatile float vel; //重心速度

	volatile float acc;
	volatile float jerk;
	volatile float ang_jerk;

	volatile float deg; //角度
	volatile float ang_vel; //角速度
	volatile float ang_acc; //角加速度

	volatile float enc_deg_R; //エンコーダー角度
	volatile float enc_deg_L;

} __Mouse_State;

typedef struct { //ゲイン
	float P;
	float I;
	float D;
} __Gain;

typedef struct { //偏差
	float P;
	float I;
	float D;
	float ex_P; //前回の値
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

extern __IRSensors IR_Sen_raw; //センサー生値
extern __IRSensors IR_Sen;
extern __IRSensors ex_IR_Sen[EX_IR_SEN_NUM]; //過去5msのセンサー値
extern __IRSensors_dif IR_Sen_dif_1;
extern __IRSensors_dif ex_IR_Sen_dif_1[EX_IR_SEN_NUM]; //過去5msのセンサー値
extern __IRSensors IR_Ref_single;
extern __IRSensors IR_Ref_double;
extern __IRSensors IR_Det;
extern __IRSensors IR_Con;
extern __IRSensors IR_Coefficient; //センサー係数
extern __IRSensors IR_Segment; //センサー切片
extern __Mouse_State idealState; //理想状態
extern __Mouse_State realState; //現実状態
extern const __Mouse_State RESETState; //リセット用

extern __Gain gain_vel; //直線方向ゲイン
extern __Gain gain_dis; //位置ゲイン
extern __Gain gain_ang_vel; //回転方向ゲイン
extern __Gain gain_deg; //回転方向ゲイン
extern __Gain gain_wall_S; //壁制御ゲイン
extern __Gain gain_wall_S_D; //壁制御ゲイン
extern __Gain gain_wall_F_R; //壁制御ゲイン
extern __Gain gain_wall_F_L; //壁制御ゲイン

extern __Error error_vel; //速度偏差
extern __Error error_dis; //位置偏差
extern __Error error_ang_vel; //角速度偏差
extern __Error error_deg; //角度偏差
extern __Error error_wall_S; //壁制御偏差
extern __Error error_wall_S_D; //壁制御偏差
extern __Error error_wall_F_R; //壁制御偏差
extern __Error error_wall_F_L; //壁制御偏差
extern const __Error RESET_ERROR;

extern __WallExist wallExist;

extern volatile float V_Batt;

extern volatile uint16_t ADC_Buffer[ADC_BUFFER_LENGTH]; //AD変換バッファ

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

extern volatile uint8_t Flag_MotorEn; //モーター有効or無効

extern volatile uint8_t Flag_encEn; //エンコーダー有効or無効

extern volatile uint8_t Flag_IRLEDEn;

extern volatile uint8_t Flag_FanAccel;

extern volatile uint8_t Flag_Check_IR_Sen_Mode;

extern volatile int16_t mode_gyro_X, mode_gyro_Y, mode_gyro_Z;

extern volatile int16_t g_Duty_R, g_Duty_L;

#endif /* GLOBAL_H_ */
