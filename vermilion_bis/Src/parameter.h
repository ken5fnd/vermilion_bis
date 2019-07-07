/*
 * parameter.h
 *
 *  Created on: 2018/08/25
 *      Author: ����
 */

#ifndef PARAMETER_H_
#define PARAMETER_H_

/*
 * turnparameter.hpp
 *
 *  Created on: 2016/09/20
 *      Author: ����
 */

#ifndef TURNPARAMETER_HPP_
#define TURNPARAMETER_HPP_

struct TurnParameter { //�^�[���̃p�����[�^�[
	volatile float Turn_deg, //�^�[���p�x(deg)
			Turn_vel, //�^�[�����x(m/s)
			Turn_radius, //�^�[�����a(m)
			Turn_ang_acc, //�^�[�������x(deg/s^2)
			Turn_offset_F, //�I�t�Z�b�g�O(m)
			Turn_offset_B; //�I�t�Z�b�g��(m)
};

struct parameter { //�p�����[�^�[
/*�����x*/
	float straight_acc;

	/*���i�ō���*/
	float straight_vel; //���i�ō���(m/s)

	/*�΂ߒ��i�ō���*/
	float straight_vel_D; //���i�ō���(m/s)

	/*90�x�^�[��*/
	struct TurnParameter turn90_R, turn90_L;

	/*180�x�^�[��*/
	struct TurnParameter turn180_R, turn180_L;

	/*45�x�^�[����*/
	struct TurnParameter turn45_I_R, turn45_I_L;

	/*135�x�^�[����*/
	struct TurnParameter turn135_I_R, turn135_I_L;

	/*45�x�^�[���o*/
	struct TurnParameter turn45_O_R, turn45_O_L;

	/*135�x�^�[���o*/
	struct TurnParameter turn135_O_R, turn135_O_L;

	/*�΂�90�x�^�[��*/
	struct TurnParameter turn90_D_R, turn90_D_L;
};

extern struct parameter parameter1;
extern struct parameter parameter2;
extern struct parameter parameter3;
extern struct parameter parameter4;

extern struct TurnParameter explore_turn_R;
extern struct TurnParameter explore_turn_L;

extern struct TurnParameter v05d90s_R;
extern struct TurnParameter v05d90s_L;

extern struct TurnParameter v06d90s_R;
extern struct TurnParameter v06d90s_L;

extern struct TurnParameter v07d90s_R;
extern struct TurnParameter v07d90s_L;

extern struct TurnParameter v08d180_R;
extern struct TurnParameter v08d45_I_R;
extern struct TurnParameter v08135_I_R;
extern struct TurnParameter v08d135_O_R;
extern struct TurnParameter v08d180_L;
extern struct TurnParameter v08d45_I_L;
extern struct TurnParameter v08d135_I_L;
extern struct TurnParameter v08d135_O_L;
extern struct TurnParameter v08d90_D_R;
extern struct TurnParameter v08d90_D_L;

extern struct TurnParameter v10d90_R;
extern struct TurnParameter v10d90_L;
extern struct TurnParameter v10d180_R;
extern struct TurnParameter v10d45_I_R;
extern struct TurnParameter v10d135_I_R;
extern struct TurnParameter v10d135_O_R;
extern struct TurnParameter v10d180_L;
extern struct TurnParameter v10d45_I_L;
extern struct TurnParameter v10d135_I_L;
extern struct TurnParameter v10d135_O_L;

extern struct TurnParameter v11d90_R;
extern struct TurnParameter v11d90_L;
extern struct TurnParameter v11d45_O_R;
extern struct TurnParameter v11d45_O_L;
extern struct TurnParameter v11d90_D_R;
extern struct TurnParameter v11d90_D_L;

extern struct TurnParameter v12d90_R;
extern struct TurnParameter v12d90_L;
extern struct TurnParameter v12d45_O_R;
extern struct TurnParameter v12d45_O_L;
extern struct TurnParameter v12d135_O_R;
extern struct TurnParameter v12d135_O_L;
extern struct TurnParameter v12d90_D_R;
extern struct TurnParameter v12d90_D_L;

extern struct TurnParameter v13d90_R;
extern struct TurnParameter v13d180_R;
extern struct TurnParameter v13d45_I_R;
extern struct TurnParameter v13d135_I_R;
extern struct TurnParameter v13d45_O_R;
extern struct TurnParameter v13d135_O_R;
extern struct TurnParameter v13d90_D_R;
extern struct TurnParameter v13d90_L;
extern struct TurnParameter v13d180_L;
extern struct TurnParameter v13d45_I_L;
extern struct TurnParameter v13d135_I_L;
extern struct TurnParameter v13d45_O_L;
extern struct TurnParameter v13d135_O_L;
extern struct TurnParameter v13d90_D_L;

extern struct TurnParameter v14d90_D_R;
extern struct TurnParameter v14d90_D_L;

extern struct TurnParameter v15d90_R;
extern struct TurnParameter v15d90_L;
extern struct TurnParameter v15d180_R;
extern struct TurnParameter v15d180_L;
extern struct TurnParameter v15d45_I_R;
extern struct TurnParameter v15d45_I_L;
extern struct TurnParameter v15d135_I_R;
extern struct TurnParameter v15d135_I_L;
extern struct TurnParameter v15d45_O_R;
extern struct TurnParameter v15d45_O_L;
extern struct TurnParameter v15d135_O_R;
extern struct TurnParameter v15d135_O_L;
extern struct TurnParameter v15d90_D_R;
extern struct TurnParameter v15d90_D_L;

extern struct TurnParameter v16d90_R;
extern struct TurnParameter v16d90_L;
extern struct TurnParameter v16d180_R;
extern struct TurnParameter v16d180_L;
extern struct TurnParameter v16d45_I_R;
extern struct TurnParameter v16d45_I_L;
extern struct TurnParameter v16d135_I_R;
extern struct TurnParameter v16d135_I_L;
extern struct TurnParameter v16d45_O_R;
extern struct TurnParameter v16d45_O_L;
extern struct TurnParameter v16d135_O_R;
extern struct TurnParameter v16d135_O_L;
extern struct TurnParameter v16d90_D_R;
extern struct TurnParameter v16d90_D_L;

extern struct TurnParameter v17d90_R;
extern struct TurnParameter v17d90_L;
extern struct TurnParameter v17d180_R;
extern struct TurnParameter v17d180_L;
extern struct TurnParameter v17d45_I_R;
extern struct TurnParameter v17d45_I_L;
extern struct TurnParameter v17d135_I_R;
extern struct TurnParameter v17d135_I_L;
extern struct TurnParameter v17d45_O_R;
extern struct TurnParameter v17d45_O_L;
extern struct TurnParameter v17d135_O_R;
extern struct TurnParameter v17d135_O_L;
extern struct TurnParameter v17d90_D_R;
extern struct TurnParameter v17d90_D_L;

extern struct TurnParameter v18d90_R;
extern struct TurnParameter v18d90_L;
extern struct TurnParameter v18d180_R;
extern struct TurnParameter v18d180_L;
extern struct TurnParameter v18d45_I_R;
extern struct TurnParameter v18d45_I_L;
extern struct TurnParameter v18d135_I_R;
extern struct TurnParameter v18d135_I_L;
extern struct TurnParameter v18d45_O_R;
extern struct TurnParameter v18d45_O_L;
extern struct TurnParameter v18d135_O_R;
extern struct TurnParameter v18d135_O_L;
extern struct TurnParameter v18d90_D_R;
extern struct TurnParameter v18d90_D_L;

extern struct TurnParameter v19d90_R;
extern struct TurnParameter v19d90_L;
extern struct TurnParameter v19d180_R;
extern struct TurnParameter v19d180_L;
extern struct TurnParameter v19d45_I_R;
extern struct TurnParameter v19d45_I_L;
extern struct TurnParameter v19d135_I_R;
extern struct TurnParameter v19d135_I_L;
extern struct TurnParameter v19d45_O_R;
extern struct TurnParameter v19d45_O_L;
extern struct TurnParameter v19d135_O_R;
extern struct TurnParameter v19d135_O_L;
extern struct TurnParameter v19d90_D_R;
extern struct TurnParameter v19d90_D_L;

extern struct TurnParameter v195d90_D_R;
extern struct TurnParameter v195d90_D_L;

extern struct TurnParameter v20d90_R;
extern struct TurnParameter v20d90_L;
extern struct TurnParameter v20d180_R;
extern struct TurnParameter v20d180_L;
extern struct TurnParameter v20d45_I_R;
extern struct TurnParameter v20d45_I_L;
extern struct TurnParameter v20d135_I_R;
extern struct TurnParameter v20d135_I_L;
extern struct TurnParameter v20d45_O_R;
extern struct TurnParameter v20d45_O_L;
extern struct TurnParameter v20d135_O_R;
extern struct TurnParameter v20d135_O_L;
extern struct TurnParameter v20d90_D_R;
extern struct TurnParameter v20d90_D_L;

extern struct TurnParameter v21d90_R;
extern struct TurnParameter v21d90_L;

extern struct TurnParameter v22d90_R;
extern struct TurnParameter v22d90_L;
extern struct TurnParameter v22d45_I_R;
extern struct TurnParameter v22d45_I_L;
extern struct TurnParameter v22d45_O_R;
extern struct TurnParameter v22d45_O_L;

void set_parameter();

#endif /* TURNPARAMETER_HPP_ */

#endif /* PARAMETER_H_ */
