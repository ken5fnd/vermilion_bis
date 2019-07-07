/*
 * drive.h
 *
 *  Created on: 2018/08/18
 *      Author: Œ’Œå
 */

#ifndef DRIVE_H_
#define DRIVE_H_

void trapezoid_acccel(float _max_vel, float _min_vel, float _dis, float _acc,
		uint8_t _wallConEn, uint8_t _diagEn);
void trapezoid_slalome(float _max_ang_vel, float _deg, float _acc_time);
void napier_slalome(float _max_ang_vel, float _deg, float _acc_time);
void accel(float _vel, float _acc);
void stop(float _dis, float _acc);

#endif /* DRIVE_H_ */
