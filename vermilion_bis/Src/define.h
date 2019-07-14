/*
 * define.h
 *
 *  Created on: 2018/08/18
 *      Author: ����
 */

#ifndef DEFINE_H_
#define DEFINE_H_

#include "maze.h"
#include "misc.h"

#define LOG_TIME (5.0)
#define LOG_DTIME (0.005) //0.0005

#define LOG_NUM ((uint16_t)(LOG_TIME/LOG_DTIME)) //���O�̌�
#define LOG_HZ ((uint16_t)(LOG_DTIME / 0.0005) - 1)  //���O�̃T���v�����O����

//#define LOG_ACC //�����x���O
//#define LOG_VEL //���x���O
//#define LOG_VEL_ACC//�����x�Z���T�[���O
//#define LOG_DIS //�������O
//#define LOG_ANGACC //�p�����x���O
//#define LOG_ANGVEL //�p���x���O
//#define LOG_ANGVEL_ENC
#define LOG_DEG //�p�x���O

//#define LOG_SEN //�Z���T�[�l���O
//#define LOG_SEN_EDGE //�Z���T�[�l���O(�ω���)

//#define LOG_SEN_RS //�Z���T�[�l���O
//#define LOG_SEN_LS //�Z���T�[�l���O

//#define LOG_DUTY //���[�^�[Duty
//#define LOG_ERROR_VEL
//#define LOG_ERROR_ANG
//#define LOG_ERROR_WALL_S //�ǐ���G���[
//#define LOG_ACC_SEN

//#define LOG_ENC_DEG

//#define LOG_V_BATT

#define TEST_TURN	trapezoid_acccel(0.6, 0.0, 0.09 * 7.0, 17.0, false, false)
//#define TEST_TURN	trapezoid_acccel(1.0, 0.0, 0.127 * 8.0, 10.0, true, true)
//#define TEST_TURN trapezoid_slalome(700.0, -360.0, 10000.0)

//#define TEST_TURN TestTurn(Turn90s, Right, &v07d90s_R, false)
//#define TEST_TURN TestTurn(Turn90s, Left, &v07d90s_L, false)
//#define TEST_TURN TestTurn(Turn90s, Right, &explore_turn_R, false)
//#define TEST_TURN TestTurn(Turn90s, Left, &explore_turn_L, false)

//#define TEST_TURN TestTurn(Turn_U, Left, &v05d90s_R, false)

//*�p�����[�^�[�������Ă��邩�m�F���܂��傤*//

//#define TURN_TEST_MODE
//
//#define TEST_TURN TestTurn(Turn90, Right, &v15d90_R, 100)
//#define TEST_TURN TestTurn(Turn180, Right, &v13d180_R, 100)
//#define TEST_TURN TestTurn(Turn45_I, Right, &v13d45_I_R, 100)
//#define TEST_TURN TestTurn(Turn135_I, Right, &v13d135_I_R, 100)
//#define TEST_TURN TestTurn(Turn45_O, Right, &v15d45_O_R, 100)
//#define TEST_TURN TestTurn(Turn135_O, Right, &v13d135_O_R, 100)
//#define TEST_TURN TestTurn(Turn90_D, Right, &v12d90_D_R, 100)

//#define TEST_TURN TestTurn(Turn90, Right, &v17d90_R, 100)
//#define TEST_TURN TestTurn(Turn180, Right, &v15d180_R, 100)
//#define TEST_TURN TestTurn(Turn45_I, Right, &v15d45_I_R, 100)
//#define TEST_TURN TestTurn(Turn135_I, Right, &v15d135_I_R, 100)
//#define TEST_TURN TestTurn(Turn45_O, Right, &v17d45_O_R, 100)
//#define TEST_TURN TestTurn(Turn135_O, Right, &v15d135_O_R, 100)
//#define TEST_TURN TestTurn(Turn90_D, Right, &v14d90_D_R, 100)

//#define TEST_TURN TestTurn(Turn90, Right, &v19d90_R, 100)
//#define TEST_TURN TestTurn(Turn180, Right, &v17d180_R, 100)
//#define TEST_TURN TestTurn(Turn45_I, Right, &v16d45_I_R, 100)
//#define TEST_TURN TestTurn(Turn135_I, Right, &v16d135_I_R, 100)
//#define TEST_TURN TestTurn(Turn45_O, Right, &v18d45_O_R, 100)
//#define TEST_TURN TestTurn(Turn135_O, Right, &v16d135_O_R, 100)
//#define TEST_TURN TestTurn(Turn90_D, Right, &v15d90_D_R, 100)

//#define TEST_TURN TestTurn(Turn90, Right, &v20d90_R, 100)
//#define TEST_TURN TestTurn(Turn180, Right, &v18d180_R, 100)
//#define TEST_TURN TestTurn(Turn45_I, Right, &v17d45_I_R, 100)
//#define TEST_TURN TestTurn(Turn135_I, Right, &v17d135_I_R, 100)
//#define TEST_TURN TestTurn(Turn45_O, Right, &v19d45_O_R, 100)
//#define TEST_TURN TestTurn(Turn135_O, Right, &v17d135_O_R, 100)
//#define TEST_TURN TestTurn(Turn90_D, Right, &v16d90_D_R, 100)


#endif /* DEFINE_H_ */
