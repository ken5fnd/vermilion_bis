/*
 * parameter.c
 *
 *  Created on: 2018/08/25
 *      Author: 健悟
 */

/*
 * turnparameter.cpp
 *
 *  Created on: 2016/09/20
 *      Author: 健悟
 */
#include "parameter.h"

/*								{ 角度(deg),速度(m/s),半径(m),角加速度(deg/s/s),前距離(m),後距離(m)} */
struct TurnParameter explore_turn_R;
struct TurnParameter explore_turn_L;

struct TurnParameter v05d90s_R = { 90.0, 0.5, 0.060, 8000.0, 0.0137, 0.0152 };
struct TurnParameter v05d90s_L = { 90.0, 0.5, 0.060, 8000.0, 0.0087, 0.0252 };

struct TurnParameter v06d90s_R = { 90.0, 0.6, 0.060, 15000.0, 0.016, 0.021 };
struct TurnParameter v06d90s_L = { 90.0, 0.6, 0.060, 15000.0, 0.017, 0.024 };

struct TurnParameter v07d90s_R = { 90.0, 0.7, 0.060, 15000.0, 0.004, 0.02 };
struct TurnParameter v07d90s_L = { 90.0, 0.7, 0.060, 15000.0, 0.009, 0.021 };

struct TurnParameter v08d180_R = { 180.0, 0.8, 0.090, 10000.0, 0.0500, 0.0 };
struct TurnParameter v08d180_L = { 180.0, 0.8, 0.090, 10000.0, 0.0500, 0.0 };
struct TurnParameter v08d135_I_R = { 135.0, 0.8, 0.090, 10000.0, 0.0409, 0.0 };
struct TurnParameter v08d135_I_L = { 135.0, 0.8, 0.090, 10000.0, 0.0409, 0.0 };
struct TurnParameter v08d135_O_R = { 135.0, 0.8, 0.090, 10000.0, 0.0285, 0.0 };
struct TurnParameter v08d135_O_L = { 135.0, 0.8, 0.090, 10000.0, 0.0285, 0.0 };
struct TurnParameter v08d90_D_R = { 90.0, 0.8, 0.090, 10000.0, 0.0294, 0.0 };
struct TurnParameter v08d90_D_L = { 90.0, 0.8, 0.090, 10000.0, 0.0294, 0.0 };

struct TurnParameter v09d45_O_R = { 45.0, 0.9, 0.070, 10000.0, 0.0772, 0.0 };
struct TurnParameter v09d45_O_L = { 45.0, 0.9, 0.070, 10000.0, 0.0772, 0.0 };

struct TurnParameter v10d90_R = { 90.0, 1.0, 0.120, 10000.0, 0.0353, 0.0 };
struct TurnParameter v10d90_L = { 90.0, 1.0, 0.120, 10000.0, 0.0353, 0.0 };
struct TurnParameter v10d180_R = { 180.0, 1.0, 0.090, 10000.0, 0.0500, 0.0 };
struct TurnParameter v10d180_L = { 180.0, 1.0, 0.090, 10000.0, 0.0500, 0.0 };
struct TurnParameter v10d45_I_R = { 45.0, 1.0, 0.160, 10000.0, 0.0119, 0.0 };
struct TurnParameter v10d45_I_L = { 45.0, 1.0, 0.160, 10000.0, 0.0119, 0.0 };
struct TurnParameter v10d135_I_R = { 135.0, 1.0, 0.070, 15000.0, 0.0625, 0.0 };
struct TurnParameter v10d135_I_L = { 135.0, 1.0, 0.070, 15000.0, 0.0625, 0.0 };
struct TurnParameter v10d45_O_R = { 45.0, 1.0, 0.090, 10000.0, 0.0696, 0.0 };
struct TurnParameter v10d45_O_L = { 45.0, 1.0, 0.090, 10000.0, 0.0696, 0.0 };
struct TurnParameter v10d135_O_R = { 135.0, 1.0, 0.070, 10000.0, 0.0477, 0.0 };
struct TurnParameter v10d135_O_L = { 135.0, 1.0, 0.070, 10000.0, 0.0477, 0.0 };

struct TurnParameter v11d90_R = { 90.0, 1.1, 0.160, 10000.0, 0.0063, 0.0 };
struct TurnParameter v11d90_L = { 90.0, 1.1, 0.160, 10000.0, 0.0063, 0.0 };
struct TurnParameter v11d45_O_R = { 45.0, 1.1, 0.170, 10000.0, 0.0478, 0.0 };
struct TurnParameter v11d45_O_L = { 45.0, 1.1, 0.170, 10000.0, 0.0478, 0.0 };
struct TurnParameter v11d90_D_R = { 90.0, 1.1, 0.080, 13000.0, 0.0234, 0.0 };
struct TurnParameter v11d90_D_L = { 90.0, 1.1, 0.080, 13000.0, 0.0234, 0.0 };

struct TurnParameter v12d90_R = { 90.0, 1.2, 0.120, 30000.0, 0.0561, 0.0 };
struct TurnParameter v12d90_L = { 90.0, 1.2, 0.120, 30000.0, 0.0561, 0.0 };
struct TurnParameter v12d45_I_R = { 45.0, 1.2, 0.090, 10000.0, 0.0142, 0.0 };
struct TurnParameter v12d45_I_L = { 45.0, 1.2, 0.090, 10000.0, 0.0142, 0.0 };
struct TurnParameter v12d135_I_R = { 135.0, 1.2, 0.080, 13000.0, 0.0326, 0.0 };
struct TurnParameter v12d135_I_L = { 135.0, 1.2, 0.080, 13000.0, 0.0326, 0.0 };
struct TurnParameter v12d45_O_R = { 45.0, 1.2, 0.090, 10000.0, 0.0545, 0.0 };
struct TurnParameter v12d45_O_L = { 45.0, 1.2, 0.090, 10000.0, 0.0545, 0.0 };
struct TurnParameter v12d135_O_R = { 135.0, 1.2, 0.080, 13000.0, 0.0251, 0.0 };
struct TurnParameter v12d135_O_L = { 135.0, 1.2, 0.080, 13000.0, 0.0251, 0.0 };
struct TurnParameter v12d90_D_R = { 90.0, 1.2, 0.080, 17000.0, 0.015, 0.0146 };
struct TurnParameter v12d90_D_L = { 90.0, 1.2, 0.080, 17000.0, 0.015, 0.0146 };

struct TurnParameter v13d90_R = { 90.0, 1.3, 0.09, 13000.0, 0.0528, 0.0 };
struct TurnParameter v13d90_L = { 90.0, 1.3, 0.09, 13000.0, 0.0528, 0.0 };
struct TurnParameter v13d180_R = { 180.0, 1.3, 0.088, 15000.0, 0.020, 0.0145 };
struct TurnParameter v13d180_L = { 180.0, 1.3, 0.088, 15000.0, 0.020, 0.0145 };
struct TurnParameter v13d45_I_R = { 45.0, 1.3, 0.090, 15000.0, 0.012, 0.0580 };
struct TurnParameter v13d45_I_L = { 45.0, 1.3, 0.090, 15000.0, 0.012, 0.0580 };
struct TurnParameter v13d135_I_R = { 135.0, 1.3, 0.080, 15000.0, 0.023, 0.0178 };
struct TurnParameter v13d135_I_L = { 135.0, 1.3, 0.080, 15000.0, 0.023, 0.0178 };
struct TurnParameter v13d45_O_R = { 45.0, 1.3, 0.110, 13000.0, 0.0403, 0.0 };
struct TurnParameter v13d45_O_L = { 45.0, 1.3, 0.110, 13000.0, 0.0403, 0.0 };
struct TurnParameter v13d135_O_R = { 135.0, 1.3, 0.080, 17000.0, 0.018, 0.0333 };
struct TurnParameter v13d135_O_L = { 135.0, 1.3, 0.080, 17000.0, 0.018, 0.0333 };
struct TurnParameter v13d90_D_R = { 90.0, 1.3, 0.080, 17000.0, 0.016, 0.0 };
struct TurnParameter v13d90_D_L = { 90.0, 1.3, 0.080, 17000.0, 0.016, 0.0 };

struct TurnParameter v14d90_D_R = { 90.0, 1.4, 0.080, 20000.0, 0.0078, 0.0166 };
struct TurnParameter v14d90_D_L = { 90.0, 1.4, 0.080, 20000.0, 0.0078, 0.0166 };

struct TurnParameter v15d90_R = { 90.0, 1.5, 0.120, 15000.0, 0.013, 0.0286 };
struct TurnParameter v15d90_L = { 90.0, 1.5, 0.120, 15000.0, 0.013, 0.0286 };
struct TurnParameter v15d180_R = { 180.0, 1.5, 0.087, 15000.0, 0.005, 0.0 };
struct TurnParameter v15d180_L = { 180.0, 1.5, 0.087, 15000.0, 0.005, 0.0 };
struct TurnParameter v15d45_I_R = { 45.0, 1.5, 0.09, 19000.0, 0.013, 0.0 };
struct TurnParameter v15d45_I_L = { 45.0, 1.5, 0.09, 19000.0, 0.013, 0.0 };
struct TurnParameter v15d135_I_R = { 135.0, 1.5, 0.080, 20000.0, 0.029, 0.0141 };
struct TurnParameter v15d135_I_L = { 135.0, 1.5, 0.080, 20000.0, 0.029, 0.0141 };
struct TurnParameter v15d45_O_R = { 45.0, 1.5, 0.120, 18000.0, 0.045, 0.0101 };
struct TurnParameter v15d45_O_L = { 45.0, 1.5, 0.120, 18000.0, 0.045, 0.0101 };
struct TurnParameter v15d135_O_R = { 135.0, 1.5, 0.080, 18000.0, 0.004, 0.0248 };
struct TurnParameter v15d135_O_L = { 135.0, 1.5, 0.080, 18000.0, 0.004, 0.0248 };
struct TurnParameter v15d90_D_R = { 90.0, 1.5, 0.080, 22000.0, 0.0050, 0.0157 };
struct TurnParameter v15d90_D_L = { 90.0, 1.5, 0.080, 22000.0, 0.0050, 0.0157 };

struct TurnParameter v16d180_R = { 180.0, 1.6, 0.090, 20000.0, 0.0100, 0.0 };
struct TurnParameter v16d180_L = { 180.0, 1.6, 0.090, 20000.0, 0.0100, 0.0 };
struct TurnParameter v16d45_I_R = { 45.0, 1.6, 0.090, 22000.0, 0.005, 0.0470 };
struct TurnParameter v16d45_I_L = { 45.0, 1.6, 0.090, 22000.0, 0.005, 0.0470 };
struct TurnParameter v16d135_I_R = { 135.0, 1.6, 0.080, 22000.0, 0.023, 0.0062 };
struct TurnParameter v16d135_I_L = { 135.0, 1.6, 0.080, 22000.0, 0.023, 0.0062 };
struct TurnParameter v16d45_O_R = { 45.0, 1.6, 0.130, 20000.0, 0.0146, 0.0146 };
struct TurnParameter v16d45_O_L = { 45.0, 1.6, 0.130, 20000.0, 0.0146, 0.0146 };
struct TurnParameter v16d135_O_R = { 135.0, 1.6, 0.080, 23000.0, 0.01, 0.0216 };
struct TurnParameter v16d135_O_L = { 135.0, 1.6, 0.080, 23000.0, 0.01, 0.0216 };
struct TurnParameter v16d90_D_R = { 90.0, 1.6, 0.080, 23000.0, 0.0, 0.0131 };
struct TurnParameter v16d90_D_L = { 90.0, 1.6, 0.080, 23000.0, 0.0, 0.0131 };

struct TurnParameter v17d90_R = { 90.0, 1.7, 0.120, 22000.0, 0.017, 0.0319 };
struct TurnParameter v17d90_L = { 90.0, 1.7, 0.120, 22000.0, 0.017, 0.0319 };
struct TurnParameter v17d180_R = { 180.0, 1.7, 0.088, 20000.0, 0.0100, 0.0113 };
struct TurnParameter v17d180_L = { 180.0, 1.7, 0.088, 20000.0, 0.0100, 0.0113 };
struct TurnParameter v17d45_I_R = { 45.0, 1.7, 0.090, 25000.0, 0.005, 0.0430 };
struct TurnParameter v17d45_I_L = { 45.0, 1.7, 0.090, 25000.0, 0.005, 0.0430 };
struct TurnParameter v17d135_I_R = { 135.0, 1.7, 0.080, 24000.0, 0.016, 0.0041 };
struct TurnParameter v17d135_I_L = { 135.0, 1.7, 0.080, 24000.0, 0.016, 0.0041 };
struct TurnParameter v17d45_O_R = { 45.0, 1.7, 0.120, 20000.0, 0.04, 0.0058 };
struct TurnParameter v17d45_O_L = { 45.0, 1.7, 0.120, 20000.0, 0.04, 0.0058 };
struct TurnParameter v17d135_O_R = { 135.0, 1.7, 0.080, 25000.0, 0.006, 0.0133 };
struct TurnParameter v17d135_O_L = { 135.0, 1.7, 0.080, 25000.0, 0.006, 0.0133 };
struct TurnParameter v17d90_D_R = { 90.0, 1.7, 0.08, 25000.0, 0.00, 0.0033 };
struct TurnParameter v17d90_D_L = { 90.0, 1.7, 0.08, 25000.0, 0.00, 0.0033 };

struct TurnParameter v18d90_R = { 90.0, 1.8, 0.120, 20000.0, 0.0076, 0.0304 };
struct TurnParameter v18d90_L = { 90.0, 1.8, 0.120, 20000.0, 0.0076, 0.0304 };
struct TurnParameter v18d180_R = { 180.0, 1.8, 0.088, 20000.0, 0.0050, 0.0 };
struct TurnParameter v18d180_L = { 180.0, 1.8, 0.088, 20000.0, 0.0050, 0.0 };
struct TurnParameter v18d45_I_R = { 45.0, 1.8, 0.120, 25000.0, 0.0011, 0.0468 };
struct TurnParameter v18d45_I_L = { 45.0, 1.8, 0.120, 25000.0, 0.0011, 0.0468 };
struct TurnParameter v18d135_I_R = { 135.0, 1.8, 0.080, 24000.0, 0.0152, 0.0030 };
struct TurnParameter v18d135_I_L = { 135.0, 1.8, 0.080, 24000.0, 0.0152, 0.0030 };
struct TurnParameter v18d45_O_R = { 45.0, 1.8, 0.120, 22000.0, 0.035, 0.0015 };
struct TurnParameter v18d45_O_L = { 45.0, 1.8, 0.120, 22000.0, 0.035, 0.0015 };
struct TurnParameter v18d135_O_R = { 135.0, 1.8, 0.080, 23000.0, 0.00, 0.0118 };
struct TurnParameter v18d135_O_L = { 135.0, 1.8, 0.080, 23000.0, 0.00, 0.0118 };
struct TurnParameter v18d90_D_R = { 90.0, 1.8, 0.080, 28000.0, 0.0141, 0.0034 };
struct TurnParameter v18d90_D_L = { 90.0, 1.8, 0.080, 28000.0, 0.0141, 0.0034 };

struct TurnParameter v19d90_R = { 90.0, 1.9, 0.120, 22000.0, 0.006, 0.0153 };
struct TurnParameter v19d90_L = { 90.0, 1.9, 0.120, 22000.0, 0.006, 0.0153 };
struct TurnParameter v19d45_I_R = { 45.0, 1.9, 0.120, 30000.0, 0.0010, 0.0 };
struct TurnParameter v19d45_I_L = { 45.0, 1.9, 0.120, 30000.0, 0.0010, 0.0 };
struct TurnParameter v19d135_I_R = { 135.0, 1.9, 0.090, 50000.0, 0.0161, 0.0 };
struct TurnParameter v19d135_I_L = { 135.0, 1.9, 0.090, 50000.0, 0.0161, 0.0 };
struct TurnParameter v19d45_O_R = { 45.0, 1.9, 0.120, 25000.0, 0.035, 0.0152 };
struct TurnParameter v19d45_O_L = { 45.0, 1.9, 0.120, 25000.0, 0.035, 0.0152 };
struct TurnParameter v19d135_O_R = { 135.0, 1.9, 0.090, 50000.0, 0.0127, 0.0 };
struct TurnParameter v19d135_O_L = { 135.0, 1.9, 0.090, 50000.0, 0.0127, 0.0 };
struct TurnParameter v19d90_D_R = { 90.0, 1.9, 0.095, 50000.0, 0.0113, 0.0 };
struct TurnParameter v19d90_D_L = { 90.0, 1.9, 0.095, 50000.0, 0.0113, 0.0 };

struct TurnParameter v20d90_R = { 90.0, 2.0, 0.12, 25000.0, 0.005, 0.0103 };
struct TurnParameter v20d90_L = { 90.0, 2.0, 0.12, 25000.0, 0.005, 0.0103 };
struct TurnParameter v20d45_O_R = { 45.0, 2.0, 0.120, 30000.0, 0.0460, 0.0 };
struct TurnParameter v20d45_O_L = { 45.0, 2.0, 0.120, 30000.0, 0.0460, 0.0 };


/*									   { 加速度(m/s/s),直進最高速度(m/s),斜め最高速度(m/s)}*/
struct parameter parameter1;

struct parameter parameter2;

struct parameter parameter3;

struct parameter parameter4;

void set_parameter() {
	parameter1.straight_acc = 14.0;
	parameter1.straight_vel = 3.5;
	parameter1.straight_vel_D = 3.5;
	parameter1.turn90_R = v15d90_R;
	parameter1.turn90_L = v15d90_L;
	parameter1.turn180_R = v13d180_R;
	parameter1.turn180_L = v13d180_L;
	parameter1.turn45_I_R = v13d45_I_R;
	parameter1.turn45_I_L = v13d45_I_L;
	parameter1.turn135_I_R = v13d135_I_R;
	parameter1.turn135_I_L = v13d135_I_L;
	parameter1.turn45_O_R = v15d45_O_R;
	parameter1.turn45_O_L = v15d45_O_L;
	parameter1.turn135_O_R = v13d135_O_R;
	parameter1.turn135_O_L = v13d135_O_L;
	parameter1.turn90_D_R = v12d90_D_R;
	parameter1.turn90_D_L = v12d90_D_L;

	parameter2.straight_acc = 14.0;
	parameter2.straight_vel = 4.0;
	parameter2.straight_vel_D = 4.0;
	parameter2.turn90_R = v17d90_R;
	parameter2.turn90_L = v17d90_L;
	parameter2.turn180_R = v15d180_R;
	parameter2.turn180_L = v15d180_L;
	parameter2.turn45_I_R = v15d45_I_R;
	parameter2.turn45_I_L = v15d45_I_L;
	parameter2.turn135_I_R = v15d135_I_R;
	parameter2.turn135_I_L = v15d135_I_L;
	parameter2.turn45_O_R = v17d45_O_R;
	parameter2.turn45_O_L = v17d45_O_L;
	parameter2.turn135_O_R = v15d135_O_R;
	parameter2.turn135_O_L = v15d135_O_L;
	parameter2.turn90_D_R = v14d90_D_R;
	parameter2.turn90_D_L = v14d90_D_L;

	parameter3.straight_acc = 15.0;
	parameter3.straight_vel = 4.3;
	parameter3.straight_vel_D = 4.3;
	parameter3.turn90_R = v18d90_R;
	parameter3.turn90_L = v18d90_L;
	parameter3.turn180_R = v17d180_R;
	parameter3.turn180_L = v17d180_L;
	parameter3.turn45_I_R = v16d45_I_R;
	parameter3.turn45_I_L = v16d45_I_L;
	parameter3.turn135_I_R = v16d135_I_R;
	parameter3.turn135_I_L = v16d135_I_L;
	parameter3.turn45_O_R = v18d45_O_R;
	parameter3.turn45_O_L = v18d45_O_L;
	parameter3.turn135_O_R = v16d135_O_R;
	parameter3.turn135_O_L = v16d135_O_L;
	parameter3.turn90_D_R = v15d90_D_R;
	parameter3.turn90_D_L = v15d90_D_L;

	parameter4.straight_acc = 17.0;
	parameter4.straight_vel = 4.5;
	parameter4.straight_vel_D = 4.5;
	parameter4.turn90_R = v20d90_R;
	parameter4.turn90_L = v20d90_L;
	parameter4.turn180_R = v18d180_R;
	parameter4.turn180_L = v18d180_L;
	parameter4.turn45_I_R = v17d45_I_R;
	parameter4.turn45_I_L = v17d45_I_L;
	parameter4.turn135_I_R = v17d135_I_R;
	parameter4.turn135_I_L = v17d135_I_L;
	parameter4.turn45_O_R = v19d45_O_R;
	parameter4.turn45_O_L = v19d45_O_L;
	parameter4.turn135_O_R = v17d135_O_R;
	parameter4.turn135_O_L = v17d135_O_L;
	parameter4.turn90_D_R = v16d90_D_R;
	parameter4.turn90_D_L = v16d90_D_L;
}

