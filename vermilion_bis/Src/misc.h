/*
 * misc.h
 *
 *  Created on: 2018/06/08
 *      Author: Œ’Œå
 */

#ifndef MISC_H_
#define MISC_H_

#include "maze.h"
#include "const.h"

enum turn_type_test {
	Turn90s,
	Turn90,
	Turn90_N,
	Turn180,
	Turn180_N,
	Turn45_I,
	Turn45_I_N,
	Turn45_O,
	Turn135_I,
	Turn135_I_N,
	Turn135_O,
	Turn90_D,
	Turn_U
};

void LED(unsigned short _LED);
void Speaker_ON(TIM_HandleTypeDef *_htim);
void Speaker_Herz(TIM_HandleTypeDef *_htim, unsigned short _Hz);
void Speaker_OFF(TIM_HandleTypeDef *_htim);
void Speaker_Scale(uint16_t _Hz, uint8_t _time);
void reset_status();
void reset_error();
void save_log();
void print_log();
void printMaze(uint8_t _isReadWall);
void printPass() ;
void TestTurn(enum turn_type_test _turn, enum turndir_slalome _dir,
		const struct TurnParameter * const _parameter, uint8_t _isSuction);
#endif /* MISC_H_ */
