#include "global.h"
#include "const.h"
#include "stm32f7xx_hal.h"

__IRSensors IR_Sen_raw; //センサー生値
__IRSensors IR_Sen; //センサー値
__IRSensors ex_IR_Sen[EX_IR_SEN_NUM]; //過去5msのセンサー値
__IRSensors_dif IR_Sen_dif_1; //センサー値差分
__IRSensors_dif ex_IR_Sen_dif_1[EX_IR_SEN_NUM];
__IRSensors IR_Ref_single = { 2.0, 6.0, 6.0, 2.0 }; //センサー値リファレンス(片壁)
__IRSensors IR_Ref_double = { 2.0, 6.0, 6.0, 2.0 }; //センサー値リファレンス(両壁)
__IRSensors IR_Det = { 12.5, 8.7, 8.7, 15.5 }; //センサー閾値(壁検出)
__IRSensors IR_Con = { 8.0, 7.5, 7.5, 8.0 }; //センサー閾値(壁制御)

__IRSensors IR_Coefficient = { 3.682, 4.275, 3.147, 4.15 }; //センサー係数
__IRSensors IR_Segment = { 30.542, 32.782, 24.88, 35.162 }; //センサー切片

__Mouse_State idealState; //理想状態
__Mouse_State realState; //現実状態
const __Mouse_State RESETState = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
		0.0 }; //リセット用

__Gain gain_vel = { 3000.0, 10000.0, 0.0 }; //速度ゲイン
__Gain gain_dis = { 0.0, 0.0, 0.0 }; //位置ゲイン
__Gain gain_ang_vel = { 1.0, 7.0, 0.0 }; //角速度ゲイン
__Gain gain_deg = { 0.02, 0.0, 0.0 }; //角度ゲイン
__Gain gain_wall_S = { 60.0, 0.0, 0.0 }; //壁制御ゲイン(横壁)
__Gain gain_wall_S_D = { 20.0, 0.0, 0.0 }; //壁制御ゲイン(横壁,斜め)
__Gain gain_wall_F_R = { 50.0, 0.0, 1.0 }; //壁制御ゲイン(前壁,位置)
__Gain gain_wall_F_L = { 50.0, 0.0, 1.0 }; //壁制御ゲイン(前壁,角度)

__Error error_vel; //速度偏差
__Error error_dis; //位置偏差
__Error error_ang_vel; //角速度偏差
__Error error_deg; //角度偏差
__Error error_wall_S; //壁制御偏差
__Error error_wall_S_D; //壁制御偏差
__Error error_wall_F_R; //壁制御偏差
__Error error_wall_F_L; //壁制御偏差
const __Error RESET_ERROR = { 0, 0, 0, 0 }; //制御量リセット

__WallExist wallExist; //壁の存在の有無

volatile float V_Batt; //電源電圧

volatile float countTime; //時間計測

int MazeDir; //迷路上での車体の向き

volatile float log_ideal_vel[LOG_NUM], log_real_vel[LOG_NUM],
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

volatile uint16_t ADC_Buffer[ADC_BUFFER_LENGTH]; //AD変換バッファ

volatile uint16_t waitTime; //wait関数用時間カウント変数

volatile uint8_t current_coord_x, current_coord_y; //現在の迷路上での座標

volatile uint8_t Goal_X, Goal_Y, Goal_L; //ゴール座標と区間大きさ

volatile int16_t gyro_ref; //ジャイロリファレンス値

volatile int16_t g_Duty_R, g_Duty_L;

volatile uint8_t suction_ONtime, suction_OFFtime;

/*フラグ系*/

volatile uint8_t Flag_diagMode; //斜め制御状態か否か

volatile uint8_t Flag_FanStateEn; //吸引ファンの有効or無効

volatile uint8_t Flag_failSafeEn; //フェイルセーフ有効or無効

volatile uint8_t Flag_wallConEn_S; //壁制御有効or無効

volatile uint8_t Flag_wallConEn_F; //前壁制御有効or無効

volatile uint8_t Flag_logEn; //ログ取り有効or無効

volatile uint8_t Flag_MotorEn; //モーター有効or無効

volatile uint8_t Flag_encEn; //エンコーダー有効or無効

volatile uint8_t Flag_gyroEn; //ジャイロ有効or無効

volatile uint8_t Flag_modeSelectEn; //モードセレクト有効or無効

volatile uint8_t Flag_exploreDone; //探索完了フラグ

volatile uint8_t Flag_countTimeEn; //時間計測有効or無効

volatile uint8_t Flag_degConEn; //角度制御有効

volatile uint8_t Flag_IRLEDEn; //壁センサーLED有効or無効

volatile uint8_t Flag_FanAccel;

volatile uint8_t Flag_Check_IR_Sen_Mode;

volatile int16_t mode_gyro_X, mode_gyro_Y, mode_gyro_Z;

