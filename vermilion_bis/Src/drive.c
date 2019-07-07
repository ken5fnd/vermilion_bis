/*
 * drive.c
 *
 *  Created on: 2018/08/18
 *      Author: ����
 */

#include "global.h"
#include <math.h>
#include "misc.h"
#include <stdio.h>

void trapezoid_acccel(float _max_vel, float _min_vel, float _dis, float _acc,
		uint8_t _wallConEn, uint8_t _diagEn) {
	volatile float d1, d2, d3, t1, t3;
	if (Flag_failSafeEn) {
		return;
	}
	t1 = (_max_vel - idealState.vel) / _acc;
	t3 = (_max_vel - _min_vel) / _acc;

	d1 = (_max_vel + idealState.vel) * t1 / 2.0;
	d3 = (_max_vel + _min_vel) * t3 / 2.0;

	if (d1 + d3 > fabs(_dis)) { //�O�p�����̏ꍇ
		d1 = _dis / 2.0
				+ (powf(_min_vel, 2.0) - powf(idealState.vel, 2.0)) / 4.0
						/ _acc;
		d2 = 0.0;
		d3 = _dis / 2.0
				+ (powf(idealState.vel, 2.0) - powf(_min_vel, 2.0)) / 4.0
						/ _acc;
	} else {
		d2 = fabs(_dis) - (d1 + d3);
	}

	if (_dis < 0) {
		_max_vel *= -1;
		_min_vel *= -1;
		_acc *= -1;
	}
	Flag_wallConEn_S = _wallConEn; //�ǐ���L��
	//Flag_degConEn = true;
	/*������������*/
	idealState.acc = _acc;
	while (Flag_failSafeEn == false && fabs(idealState.dis) < fabs(d1))
		;
	/*���������܂�*/

	/*������������*/
	idealState.acc = 0;
	if (d2 != 0.0) {
		while (Flag_failSafeEn == false && fabs(idealState.dis) < fabs(d1 + d2))
			;
	}
	/*���������܂�*/

	/*������������*/
	idealState.acc = -_acc;
	if (_diagEn) { //�΂ߐ��䒆�͑O�ǃZ���T�[��؂�(�O�ǂɊ����邽��)
		Flag_wallConEn_S = false;
		idealState.ang_vel = 0.0;
	}
	while (Flag_failSafeEn == false && fabs(idealState.dis) < fabs(_dis)) {
		if (_dis > 0.0 && (idealState.vel < _min_vel)) {
			idealState.acc = 0.0;
			idealState.vel = _min_vel + 0.1;
		} else if (_dis < 0.0 && (idealState.vel > _min_vel)) {
			idealState.acc = 0.0;
			idealState.vel = _min_vel - 0.1;
		}
	}
	/*���������܂�*/
	idealState.vel = _min_vel;
	idealState.acc = 0.0;
	idealState.dis = 0.0;
	realState.dis = 0.0;
}

void trapezoid_slalome(float _max_ang_vel, float _deg, float _acc) { //��`����
	volatile float theta1, theta2, theta3, t1, t3;
	if (Flag_failSafeEn) {
		return;
	}
	t1 = (_max_ang_vel) / _acc;
	t3 = t1;

	theta1 = (_max_ang_vel) * t1 / 2.0;
	theta3 = (_max_ang_vel) * t3 / 2.0;

	if (fabs(theta1 + theta3) > fabs(_deg)) { //�O�p�����̏ꍇ
		theta1 = _deg / 2.0;
		theta2 = 0.0;
		theta3 = _deg / 2.0;
	} else {
		theta2 = fabs(_deg) - (theta1 + theta3);
	}

	if (_deg < 0) {
		_max_ang_vel *= -1;
		_acc *= -1;
	}
	/*������������*/
	Flag_wallConEn_S = false;
	idealState.ang_acc = _acc;
	idealState.deg = 0.0;
	realState.deg = 0.0;
	while (Flag_failSafeEn == false && fabs(idealState.deg) < fabs(theta1))
		;
	/*���������܂�*/

	/*������������*/

	idealState.ang_acc = 0.0;
	if (theta2 != 0.0) {
		while (Flag_failSafeEn == false
				&& fabs(idealState.deg) < fabs(theta1 + theta2))
			;
	}
	/*���������܂�*/

	/*������������*/
	idealState.ang_acc = -_acc;
	Flag_degConEn = true;
	while (Flag_failSafeEn == false && fabs(idealState.deg) < fabs(_deg)) {
		if ((_deg > 0.0) && (idealState.ang_vel < 50.0)) {
			idealState.ang_vel = 50.0;
			idealState.ang_acc = 0.0;
		} else if ((_deg < 0.0) && (idealState.ang_vel > -50.0)) {
			idealState.ang_vel = -50.0;
			idealState.ang_acc = 0.0;
		}
//		if(fabs(realState.deg) > fabs(_deg)){
//			break;
//		}
	}
	/*���������܂�*/
	Flag_degConEn = false;
	idealState.ang_vel = 0.0;
	idealState.ang_acc = 0.0;
	idealState.deg = 0.0;
	realState.deg = 0.0;
	idealState.dis = 0.0;
	realState.dis = 0.0;
}
void napier_slalome(float _max_ang_vel, float _deg, float _acc_time) { //�l�C�s�A����
	volatile float const_time;
	if (Flag_failSafeEn) {
		return;
	}

	Flag_wallConEn_S = false;
	idealState.deg = 0.0;
	realState.deg = 0.0;
	if (_deg < 0.0) {
		_max_ang_vel *= -1.0;
	}
	countTime = 0.0;
	Flag_countTimeEn = true;
	Flag_degConEn = true;
	while (!Flag_failSafeEn && countTime < _acc_time - CONTROL_CYCLE) {
		idealState.ang_vel = _max_ang_vel / 0.367879
				* expf(-1.0 / (1.0 - powf(1.0 - countTime / _acc_time, 2.0)));
	}
	const_time = fabs((_deg - _max_ang_vel * _acc_time) / _max_ang_vel);

	Flag_countTimeEn = false;
	countTime = 0.0;
	Flag_countTimeEn = true;
	while (!Flag_failSafeEn && countTime < const_time)
		;
	Flag_countTimeEn = false;
	countTime = 0.0;
	Flag_countTimeEn = true;
	while (!Flag_failSafeEn && countTime < _acc_time - CONTROL_CYCLE) {
//		idealState.ang_vel =
//				max_ang_vel / 0.367879 * expf(-1 / (1- powf(1 - (_acc_time- countTime) / _acc_time, 2.0)));
		idealState.ang_vel =
				_max_ang_vel
						- _max_ang_vel / 0.367879
								* expf(
										-1.0
												/ (1.0
														- powf(
																1.0
																		- countTime
																				/ _acc_time,
																2.0)));
	}
	Flag_countTimeEn = false;
	idealState.ang_vel = 0.0;
	idealState.dis = 0.0;
	idealState.deg = 0.0;
	realState.dis = 0.0;
	realState.deg = 0.0;
	Flag_degConEn = false;
}

void accel(float _vel, float _acc) {
	if (Flag_failSafeEn) {
		return;
	}
	idealState.acc = _acc;

	Flag_wallConEn_S = true; //�ǐ���L��

	/*������������*/
	idealState.acc = _acc;
	while (Flag_failSafeEn == false && idealState.vel < _vel)
		;
	idealState.acc = 0.0;
	idealState.vel = _vel;
	/*���������܂�*/
}

void stop(float _dis, float _acc) {
	/*�����͌��݂̋������l�����Ē����̃}�X�Ɏ~�܂�悤�ɂ��Ȃ���΂����Ȃ�*/
	if (Flag_failSafeEn) {
		return;
	}
	volatile float d = _dis - (idealState.vel * idealState.vel / _acc / 2.0);

	/*������������*/
	idealState.acc = 0.0;
	while (Flag_failSafeEn == false && idealState.dis < d && realState.vel > 0.0)
		;

	/*���������܂�*/

	/*������������*/
	idealState.acc = -_acc;
	while (Flag_failSafeEn == false && idealState.dis < _dis
			&& realState.vel > 0.0)
		;
	idealState.acc = 0.0;
	idealState.vel = 0.0;
	Flag_wallConEn_S = false; //�ǐ��䖳��
	/*���������܂�*/
	realState.dis = 0.0;
	idealState.dis = 0.0;

}
