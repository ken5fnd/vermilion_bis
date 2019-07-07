/*
 * wallsensor.h
 *
 *  Created on: 2018/08/25
 *      Author: Œ’Œå
 */

#ifndef WALLSENSOR_H_
#define WALLSENSOR_H_

void wall_exist();
void get_wallsensor_data();
void get_wallsensor_data2();
void edge_break(int8_t _dir, float _target_vel, float _acc);
#endif /* WALLSENSOR_H_ */
