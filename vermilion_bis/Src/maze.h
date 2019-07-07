/*
 * maze.h
 *
 *  Created on: 2018/08/25
 *      Author: ����
 */

#ifndef MAZE_H_
#define MAZE_H_

/*
 * maze.hpp
 *
 *  Created on: 2017/08/18
 *      Author: ����
 */

#ifndef MAZE_HPP_
#define MAZE_HPP_

#define EXPANSION 0
#define ERASE 1

#define WRITE 0

#define VERTICAL 0
#define HORIZONTAL 4
#include "parameter.h"
#include <stdint.h>

enum direction_maze {
	Void = -1, North = 0,    //0
	NorthEast,    //1
	East,    //2
	SouthEast,    //3
	South,    //4
	SouthWest,    //5
	West,    //6
	NorthWest    //7
};

enum turndir_slalome {
	Forward, //0
	Right = -1,
	Left = 1,
	Back //2
};
enum turn_type_sprint {
	Straight = -4,
	Straight_D,
	TurnR_90_D,
	TurnL_90_D,
	End = 0,
	TurnR_90,
	TurnL_90,
	TurnR_180,
	TurnL_180,
	TurnR_45_I = 10,
	TurnL_45_I,
	TurnR_135_I,
	TurnL_135_I,
	TurnR_45_O,
	TurnL_45_O,
	TurnR_135_O,
	TurnL_135_O
};

enum turndir_maze {
	Stop = 0, F, R, L
};

enum DataFlash {
	Read, Write
};

enum VerOrHor {
	ver, hor
};

struct Nodes {
	uint8_t isNodeFixed;	//�m�[�h���m�肩�ۂ�
	uint8_t isNodeVaild;	//�m�[�h���L�����ۂ�
	unsigned short NodeCost;	//�m�[�h�̃R�X�g
	enum direction_maze PrevNode;	//�O�̃m�[�h�̌���
};

extern volatile int direction;
extern volatile unsigned short wall_ver[16], //�c�@�E�[��MSB�@���[��LSB ���񂩂珇
		wall_hor[16]; //���@������MSB�@�㋴��LEB�@���񂩂珇

/*�Г����̕Ǐ��*/
extern volatile unsigned short half_wall_ver[16], //�c�@�E�[��MSB�@���[��LSB ���񂩂珇
		half_wall_hor[16]; //���@������MSB�@�㋴��LEB�@���񂩂珇

//�ǂ𒼐ړǂ񂾂��ǂ���
extern volatile unsigned short wall_read_ver[16], //�c�@�E�[��MSB�@���[��LSB ���񂩂珇
		wall_read_hor[16]; //���@������MSB�@�㋴��LEB�@���񂩂珇

/*�Г����̕Ǐ��*/
extern volatile unsigned short half_wall_read_ver[16], //�c�@�E�[��MSB�@���[��LSB ���񂩂珇
		half_wall_read_hor[16]; //���@������MSB�@�㋴��LEB�@���񂩂珇

extern unsigned short StepMap[16][16];
extern unsigned short DiagStepMap_ver[15][16];
extern unsigned short DiagStepMap_hor[16][15];

extern uint8_t Flag_cellfound[16][16];

extern enum turn_type_sprint pass[256];

extern int ex_CurrentX[5], ex_CurrentY[5];
extern int ex_dir[5][3];

extern enum turndir_maze routeDir[256];

extern enum turndir_maze routeDir[256];

void mapRenew(int _turnDir);
uint8_t checkMapWall(unsigned char _x, unsigned char _y, int _dir,
		uint8_t _state);
void writeMapWall(unsigned char _X, unsigned char _Y, int _dir,
		uint8_t _isErase);
void writeWallData(unsigned char _X, unsigned char _Y, int _dir);
void eraseWallData(unsigned char _X, unsigned char _Y, int _dir);
void stepMapRenew(unsigned char _Goal_X, unsigned char _Goal_Y, uint8_t _state,
		uint8_t _is4cell);
void stepMapRenew_DiagMap(unsigned char _Goal_X, unsigned char _Goal_Y,
		uint8_t _state, uint8_t _is4cell);
void AdachiMethod(unsigned char _Goal_X, unsigned char _Goal_Y,
		uint8_t _isAccel);
void SearchAlgorithm();
void SprintRun(const struct parameter * _pointer, uint8_t _isDiag,
		uint8_t _suctionduty);
void generateRoute_Simple();
void generateRoute_DiagMap();
void generatePass(uint8_t _isDiag);

void straight18();
void turn90_s(int _dir);
void turnU();
void turnU_2();
void FrontWallControl();
void sprint_turn(float _deg, int8_t _dir, const struct TurnParameter * const _parameter,
		float _acc, uint8_t _isWallGap, uint8_t _diagTurn);

//volatile extern unsigned char wall[16][16]; //�Ǐ��
//
//extern volatile unsigned short StepMap_ver[16 - 1][16]; //�����}�b�v(�c)
//extern volatile unsigned short StepMap_hor[16][16 - 1]; //�����}�b�v(��)
//
//void maze_Stepmap( ///�����}�b�v���������C�W�J����
//		unsigned char _mode, //���[�h�I��(������ or �W�J)
//		unsigned char _goal_x, //�S�[�����Wx
//		unsigned char _goal_y, //�S�[�����Wy
//		uint8_t _isUnreadWall //�ǂ�ł��Ȃ��ǂ��܂߂邩
//		);

#endif /* MAZE_HPP_ */

#endif /* MAZE_H_ */
