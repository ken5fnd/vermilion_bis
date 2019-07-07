#include "global.h"
#include "const.h"
#include "stm32f7xx_hal.h"

__IRSensors IR_Sen_raw; //�Z���T�[���l
__IRSensors IR_Sen; //�Z���T�[�l
__IRSensors ex_IR_Sen[EX_IR_SEN_NUM]; //�ߋ�5ms�̃Z���T�[�l
__IRSensors_dif IR_Sen_dif_1; //�Z���T�[�l����
__IRSensors_dif ex_IR_Sen_dif_1[EX_IR_SEN_NUM];
__IRSensors IR_Ref_single = { 2.0, 6.0, 6.0, 2.0 }; //�Z���T�[�l���t�@�����X(�Е�)
__IRSensors IR_Ref_double = { 2.0, 6.0, 6.0, 2.0 }; //�Z���T�[�l���t�@�����X(����)
__IRSensors IR_Det = { 12.5, 8.7, 8.7, 15.5 }; //�Z���T�[臒l(�ǌ��o)
__IRSensors IR_Con = { 8.0, 7.5, 7.5, 8.0 }; //�Z���T�[臒l(�ǐ���)

__IRSensors IR_Coefficient = { 3.682, 4.275, 3.147, 4.15 }; //�Z���T�[�W��
__IRSensors IR_Segment = { 30.542, 32.782, 24.88, 35.162 }; //�Z���T�[�ؕ�

__Mouse_State idealState; //���z���
__Mouse_State realState; //�������
const __Mouse_State RESETState = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
		0.0 }; //���Z�b�g�p

__Gain gain_vel = { 3000.0, 10000.0, 0.0 }; //���x�Q�C��
__Gain gain_dis = { 0.0, 0.0, 0.0 }; //�ʒu�Q�C��
__Gain gain_ang_vel = { 1.0, 7.0, 0.0 }; //�p���x�Q�C��
__Gain gain_deg = { 0.02, 0.0, 0.0 }; //�p�x�Q�C��
__Gain gain_wall_S = { 60.0, 0.0, 0.0 }; //�ǐ���Q�C��(����)
__Gain gain_wall_S_D = { 20.0, 0.0, 0.0 }; //�ǐ���Q�C��(����,�΂�)
__Gain gain_wall_F_R = { 50.0, 0.0, 1.0 }; //�ǐ���Q�C��(�O��,�ʒu)
__Gain gain_wall_F_L = { 50.0, 0.0, 1.0 }; //�ǐ���Q�C��(�O��,�p�x)

__Error error_vel; //���x�΍�
__Error error_dis; //�ʒu�΍�
__Error error_ang_vel; //�p���x�΍�
__Error error_deg; //�p�x�΍�
__Error error_wall_S; //�ǐ���΍�
__Error error_wall_S_D; //�ǐ���΍�
__Error error_wall_F_R; //�ǐ���΍�
__Error error_wall_F_L; //�ǐ���΍�
const __Error RESET_ERROR = { 0, 0, 0, 0 }; //����ʃ��Z�b�g

__WallExist wallExist; //�ǂ̑��݂̗L��

volatile float V_Batt; //�d���d��

volatile float countTime; //���Ԍv��

int MazeDir; //���H��ł̎ԑ̂̌���

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

volatile uint16_t ADC_Buffer[ADC_BUFFER_LENGTH]; //AD�ϊ��o�b�t�@

volatile uint16_t waitTime; //wait�֐��p���ԃJ�E���g�ϐ�

volatile uint8_t current_coord_x, current_coord_y; //���݂̖��H��ł̍��W

volatile uint8_t Goal_X, Goal_Y, Goal_L; //�S�[�����W�Ƌ�ԑ傫��

volatile int16_t gyro_ref; //�W���C�����t�@�����X�l

volatile int16_t g_Duty_R, g_Duty_L;

volatile uint8_t suction_ONtime, suction_OFFtime;

/*�t���O�n*/

volatile uint8_t Flag_diagMode; //�΂ߐ����Ԃ��ۂ�

volatile uint8_t Flag_FanStateEn; //�z���t�@���̗L��or����

volatile uint8_t Flag_failSafeEn; //�t�F�C���Z�[�t�L��or����

volatile uint8_t Flag_wallConEn_S; //�ǐ���L��or����

volatile uint8_t Flag_wallConEn_F; //�O�ǐ���L��or����

volatile uint8_t Flag_logEn; //���O���L��or����

volatile uint8_t Flag_MotorEn; //���[�^�[�L��or����

volatile uint8_t Flag_encEn; //�G���R�[�_�[�L��or����

volatile uint8_t Flag_gyroEn; //�W���C���L��or����

volatile uint8_t Flag_modeSelectEn; //���[�h�Z���N�g�L��or����

volatile uint8_t Flag_exploreDone; //�T�������t���O

volatile uint8_t Flag_countTimeEn; //���Ԍv���L��or����

volatile uint8_t Flag_degConEn; //�p�x����L��

volatile uint8_t Flag_IRLEDEn; //�ǃZ���T�[LED�L��or����

volatile uint8_t Flag_FanAccel;

volatile uint8_t Flag_Check_IR_Sen_Mode;

volatile int16_t mode_gyro_X, mode_gyro_Y, mode_gyro_Z;

