/*
 * maze.c
 *
 *  Created on: 2018/08/25
 *      Author: 健悟
 */

/*迷路系の処理*/
//#define EXPANSION 0
//#define ERASE 1
//
//#define WRITE 0
//
//#define VERTICAL 0
//#define HORIZONTAL 4
//volatile unsigned char wall[16][16]; //壁情報
///*（縦壁）”壁の有無（MSB）”，”壁を直接読んだかどうか”，”壁の有無バックアップ”，”壁を直接読んだかバックアップ”*/
///*（横壁）”壁の有無”，”壁を直接読んだかどうか”，”壁の有無バックアップ”，”壁を直接読んだかバックアップ（LSB）”*/
//
//volatile unsigned short StepMap_ver[16 - 1][16]; //歩数マップ(縦)
//volatile unsigned short StepMap_hor[16][16 - 1]; //歩数マップ(横)
//
//void maze_WallWrite(unsigned char _Mode, unsigned char _x, unsigned char _y,
//		unsigned char _VorH) { ///壁情報を操作する関数
//	_Mode == WRITE ?
//			wall[_x][_y] |= 0x01 >> _VorH :
//			wall[_x][_y] &= 0xFF - (0x80 >> _VorH);
//}
//
//void maze_WallBackup() { ///迷路情報をバックアップ
//	for (unsigned char x = 0; x < 16; x++) {
//		for (unsigned char y = 0; x < 16; y++) {
//			wall[x][y] += 0x40 & (wall[x][y] >> 0x01);
//			wall[x][y] += 0x20 & (wall[x][y] >> 0x01);
//			wall[x][y] += 0x04 & (wall[x][y] >> 0x01);
//			wall[x][y] += 0x02 & (wall[x][y] >> 0x01);
//		}
//	}
//}
//
//void maze_Stepmap( ///歩数マップを初期化，展開する
//		unsigned char _mode, //モード選択(初期化 or 展開)
//		unsigned char _goal_x, //ゴール座標x
//		unsigned char _goal_y, //ゴール座標y
//		uint8_t _isUnreadWall //読んでいない壁を含めるか
//		) {
//
//	if (_mode == EXPANSION) { //歩数マップ展開
//
//	} else if (_mode == ERASE) { //歩数マップ初期化
//
//	}
//}
//
//void maze_checkWallExist() { ///壁の有無を判断する
//
//}
#include "maze.h"
#include "global.h"
#include "const.h"
#include "motor.h"
#include "drive.h"
#include "misc.h"
#include "flash.h"
#include "wallsensor.h"
#include <math.h>

#define Step_S  (20)
#define Step_D  (15)	//歩数
#define DE_Step_S  (4)
#define DE_Step_D  (4)	//直線歩数減少係数

volatile int direction = North;
/*インクリメントで右回転				 */
/*ディクリメントで左回転するようになったよ*/

volatile unsigned short wall_ver[16], //縦　右端がMSB　左端がLSB 下列から順
		wall_hor[16]; //横　下橋がMSB　上橋がLEB　左列から順

/*片道分の壁情報*/
volatile unsigned short half_wall_ver[16], //縦　右端がMSB　左端がLSB 下列から順
		half_wall_hor[16]; //横　下橋がMSB　上橋がLEB　左列から順

//壁を直接読んだかどうか
volatile unsigned short wall_read_ver[16], //縦　右端がMSB　左端がLSB 下列から順
		wall_read_hor[16]; //横　下橋がMSB　上橋がLEB　左列から順

/*片道分の壁情報*/
volatile unsigned short half_wall_read_ver[16], //縦　右端がMSB　左端がLSB 下列から順
		half_wall_read_hor[16]; //横　下橋がMSB　上橋がLEB　左列から順

unsigned short StepMap[16][16];
unsigned short DiagStepMap_ver[15][16];
unsigned short DiagStepMap_hor[16][15];

uint8_t Flag_cellfound[16][16];

enum turn_type_sprint pass[256];

int ex_CurrentX[5], ex_CurrentY[5];
int ex_dir[5][3];

enum turndir_maze routeDir[256];

uint8_t checkMapWall(unsigned char _x, unsigned char _y, int _dir,
		uint8_t _state) { //マップ上の壁取得状況を読む
	//_state:直接壁を読んだか否か
	//true=直接読んでいない場所に壁を入れる ,false=直接読んでいない場所に壁を入れない
	switch (_dir) {
	case North:
		if (wall_hor[_x] & 1 << _y) {
			return true;
		}
		break;
	case East:
		if (wall_ver[_y] & 1 << _x) {
			return true;
		}
		break;
	case South:
		if (wall_hor[_x] & 1 << (_y - 1)) {
			return true;
		}
		break;
	case West:
		if (wall_ver[_y] & 1 << (_x - 1)) {
			return true;
		}
		break;
	default:
		return false;
	}
	if (_state) {
		switch (_dir) {
		case North:
			if (!(wall_read_hor[_x] & 1 << _y)) {
				return true;
			}
			break;
		case East:
			if (!(wall_read_ver[_y] & 1 << _x)) {
				return true;
			}
			break;
		case South:
			if (!(wall_read_hor[_x] & 1 << (_y - 1))) {
				return true;
			}
			break;
		case West:
			if (!(wall_read_ver[_y] & 1 << (_x - 1))) {
				return true;
			}
			break;
		default:
			return false;
		}
	}
	return false;
}

void writeMapWall(unsigned char _X, unsigned char _Y, int _dir,
		uint8_t _isErase) { //壁を追加、削除する
	switch (_dir) {
	case North:
		if (_isErase) {
			wall_hor[_X] = (wall_hor[_X] | 1 << _Y) ^ 1 << _Y;
		} else {
			wall_hor[_X] = wall_hor[_X] | 1 << _Y;
		}
		break;
	case East:
		if (_isErase) {
			wall_ver[_Y] = (wall_ver[_Y] | 1 << _X) ^ 1 << _X;
		} else {
			wall_ver[_Y] = wall_ver[_Y] | 1 << _X;
		}

		break;
	case West:
		if (_isErase) {
			wall_ver[_Y] = (wall_ver[_Y] | 1 << (_X - 1)) ^ 1 << (_X - 1);
		} else {
			wall_ver[_Y] = wall_ver[_Y] | 1 << (_X - 1);
		}

		break;
	case South:
		if (_isErase) {
			wall_hor[_X] = (wall_hor[_X] | 1 << (_Y - 1)) ^ 1 << (_Y - 1);
		} else {
			wall_hor[_X] = wall_hor[_X] | 1 << (_Y - 1);
		}

		break;
	default:
		break;
	}
}

void writeWallData(unsigned char _X, unsigned char _Y, int _dir) {
	unsigned char poz = 0;
	uint8_t North_PA = 0, West_PA = 0, East_PA = 0, South_PA = 0;
	switch (_dir) {
	case North:
		North_PA = wallExist.Forward;
		West_PA = wallExist.Left;
		East_PA = wallExist.Right;

		wall_read_hor[_X] = wall_read_hor[_X] | 1 << _Y; //北
		wall_read_ver[_Y] = wall_read_ver[_Y] | 1 << _X; //東
		wall_read_ver[_Y] = wall_read_ver[_Y] | 1 << (_X - 1); //西

		break;
	case East:
		North_PA = wallExist.Left;
		South_PA = wallExist.Right;
		East_PA = wallExist.Forward;

		wall_read_hor[_X] = wall_read_hor[_X] | 1 << _Y; //北
		wall_read_ver[_Y] = wall_read_ver[_Y] | 1 << _X; //東
		wall_read_hor[_X] = wall_read_hor[_X] | 1 << (_Y - 1); //南

		break;
	case South:
		South_PA = wallExist.Forward;
		West_PA = wallExist.Right;
		East_PA = wallExist.Left;

		wall_read_ver[_Y] = wall_read_ver[_Y] | 1 << _X; //東
		wall_read_ver[_Y] = wall_read_ver[_Y] | 1 << (_X - 1); //西
		wall_read_hor[_X] = wall_read_hor[_X] | 1 << (_Y - 1); //南

		break;
	case West:
		North_PA = wallExist.Right;
		West_PA = wallExist.Forward;
		South_PA = wallExist.Left;

		wall_read_hor[_X] = wall_read_hor[_X] | 1 << _Y; //北
		wall_read_ver[_Y] = wall_read_ver[_Y] | 1 << (_X - 1); //西
		wall_read_hor[_X] = wall_read_hor[_X] | 1 << (_Y - 1); //南

		break;
	}
	for (int i = 0; i < 4; i++) {
		ex_CurrentX[i + 1] = ex_CurrentX[i];
		ex_CurrentY[i + 1] = ex_CurrentY[i];
		ex_dir[i + 1][0] = ex_dir[i][0];
		ex_dir[i + 1][1] = ex_dir[i][1];
		ex_dir[i + 1][2] = ex_dir[i][2];
	}
	ex_CurrentX[0] = current_coord_x;
	ex_CurrentY[0] = current_coord_y;
	ex_dir[0][0] = -1;
	ex_dir[0][1] = -1;
	ex_dir[0][2] = -1;
	if (North_PA) {
		if (!checkMapWall(_X, _Y, North, false)) { //既に壁が入っている場合は追加しない
			writeMapWall(_X, _Y, North, false);
			ex_dir[0][poz] = North;
			poz++;
		}
	}
	if (East_PA) {
		if (!checkMapWall(_X, _Y, East, false)) { //既に壁が入っている場合は追加しない
			writeMapWall(_X, _Y, East, false);
			ex_dir[0][poz] = East;
			poz++;
		}
	}
	if (West_PA) {
		if (!checkMapWall(_X, _Y, West, false)) { //既に壁が入っている場合は追加しない
			writeMapWall(_X, _Y, West, false);
			ex_dir[0][poz] = West;
			poz++;
		}
	}
	if (South_PA) {
		if (!checkMapWall(_X, _Y, South, false)) { //既に壁が入っている場合は追加しない
			writeMapWall(_X, _Y, South, false);
			ex_dir[0][poz] = South;
			poz++;
		}

	}

}

void stepMapRenew_DiagMap(unsigned char _Goal_X, unsigned char _Goal_Y,
		uint8_t _state, uint8_t _is4cell) {	//斜め歩数マップ
//_state :直接読んでいない場所に壁を入れるかどうか
//_is4cell : 4マス対応にするかどうか
	unsigned char q_x[5000], q_y[5000], x, y;
	enum VerOrHor q_vh[5000], vh;
	unsigned short head = 0, tail = 1;

	for (int x = 0; x < 15; x++) {
		for (int y = 0; y < 16; y++) {
			DiagStepMap_ver[x][y] = 0xFFFF;
		}
	}
	for (int x = 0; x < 16; x++) {
		for (int y = 0; y < 15; y++) {
			DiagStepMap_hor[x][y] = 0xFFFF;
		}
	}
	DiagStepMap_hor[_Goal_X][_Goal_Y] = 0;
	q_x[0] = _Goal_X;
	q_y[0] = _Goal_Y;
	q_vh[0] = hor;
	if (_is4cell) {
		/*ゴール区画に壁が入るのを防止*/
		DiagStepMap_ver[_Goal_X][_Goal_Y] = 0;
		q_x[1] = _Goal_X;
		q_y[1] = _Goal_Y;
		q_vh[1] = ver;
		DiagStepMap_ver[_Goal_X][_Goal_Y + 1] = 0;
		q_x[2] = _Goal_X;
		q_y[2] = _Goal_Y + 1;
		q_vh[2] = ver;
		DiagStepMap_hor[_Goal_X + 1][_Goal_Y] = 0;
		q_x[3] = _Goal_X + 1;
		q_y[3] = _Goal_Y;
		q_vh[3] = hor;
		tail = 4;
	}
	for (; head != tail; head++) {
		//t_printf("x = %d,y = %d\n", q_x[head], q_y[head]);
		if (q_vh[head] == ver) {	//縦壁
			//////*北西*//////
			if ((checkMapWall(q_x[head], q_y[head], North, _state) == false)//壁がなく
					&& (DiagStepMap_hor[q_x[head]][q_y[head]]
							> DiagStepMap_ver[q_x[head]][q_y[head]] + Step_D)//歩数が大きく
					&& q_y[head] < 15) {								//座標が迷路内
				q_x[tail] = q_x[head];
				q_y[tail] = q_y[head];
				q_vh[tail] = hor;
				tail++;
				DiagStepMap_hor[q_x[head]][q_y[head]] =
						DiagStepMap_ver[q_x[head]][q_y[head]] + Step_D;
				vh = hor;
				x = q_x[head];
				y = q_y[head];
				for (unsigned char i = 1;; i++) {
					if (vh == ver) {
						if ((checkMapWall(x, y, North, _state) == false)//壁がなく
								&& (DiagStepMap_hor[x][y]
										> DiagStepMap_ver[x][y] + Step_D
												- (DE_Step_D * i)) //歩数が大きく
								&& y < 15) {
							DiagStepMap_hor[x][y] = DiagStepMap_ver[x][y]
									+ Step_D - (DE_Step_D * i);
							if (DiagStepMap_hor[x][y]
									<= DiagStepMap_ver[x][y]) {
								DiagStepMap_hor[x][y] = DiagStepMap_ver[x][y]
										+ 1;
							}
							vh = hor;
							q_x[tail] = x;
							q_y[tail] = y;
							q_vh[tail] = hor;
							tail++;
						} else {
							break;
						}
					} else if (vh == hor) {
						if ((checkMapWall(x - 1, y + 1, East, _state) == false)	//壁がなく
								&& (DiagStepMap_ver[x - 1][y + 1]
										> DiagStepMap_hor[x][y] + Step_D
												- (DE_Step_D * i))	//歩数が大きく
								&& x > 0 && q_y[head] < 15) {
							DiagStepMap_ver[x - 1][y + 1] =
									DiagStepMap_hor[x][y] + Step_D
											- (DE_Step_D * i);
							if (DiagStepMap_ver[x - 1][y + 1]
									<= DiagStepMap_hor[x][y]) {
								DiagStepMap_ver[x - 1][y + 1] =
										DiagStepMap_hor[x][y] + 1;
							}
							x--;
							y++;
							vh = ver;
							q_x[tail] = x;
							q_y[tail] = y;
							q_vh[tail] = ver;
							tail++;
						} else {
							break;
						}
					}
				}
			}
			//////*北東*//////
			if ((checkMapWall(q_x[head] + 1, q_y[head], North, _state) == false)//壁がなく
					&& (DiagStepMap_hor[q_x[head] + 1][q_y[head]]
							> DiagStepMap_ver[q_x[head]][q_y[head]] + Step_D)//歩数が大きく
					&& q_x[head] < 15 && q_y[head] < 15) {			//座標が迷路内
				q_x[tail] = q_x[head] + 1;
				q_y[tail] = q_y[head];
				q_vh[tail] = hor;
				tail++;
				DiagStepMap_hor[q_x[head] + 1][q_y[head]] =
						DiagStepMap_ver[q_x[head]][q_y[head]] + Step_D;
				vh = hor;
				x = q_x[head] + 1;
				y = q_y[head];
				for (unsigned char i = 1;; i++) {
					if (vh == ver) {
						if ((checkMapWall(x + 1, y, North, _state) == false)//壁がなく
								&& (DiagStepMap_hor[x + 1][y]
										> DiagStepMap_ver[x][y] + Step_D
												- (DE_Step_D * i))		//歩数が大きく
								&& x < 15 && y < 15) {
							DiagStepMap_hor[x + 1][y] = DiagStepMap_ver[x][y]
									+ Step_D - (DE_Step_D * i);
							if (DiagStepMap_hor[x + 1][y]
									<= DiagStepMap_ver[x][y]) {
								DiagStepMap_hor[x + 1][y] =
										DiagStepMap_ver[x][y] + 1;
							}
							x++;
							vh = hor;
							q_x[tail] = x;
							q_y[tail] = y;
							q_vh[tail] = hor;
							tail++;
						} else {
							break;
						}
					} else if (vh == hor) {
						if ((checkMapWall(x, y + 1, East, _state) == false)	//壁がなく
								&& (DiagStepMap_ver[x][y + 1]
										> DiagStepMap_hor[x][y] + Step_D
												- (DE_Step_D * i))	//歩数が大きく
								&& x < 15 && y < 15) {
							DiagStepMap_ver[x][y + 1] = DiagStepMap_hor[x][y]
									+ Step_D - (DE_Step_D * i);
							if (DiagStepMap_ver[x][y + 1]
									<= DiagStepMap_hor[x][y]) {
								DiagStepMap_ver[x][y + 1] =
										DiagStepMap_hor[x][y] + 1;
							}
							y++;
							vh = ver;
							q_x[tail] = x;
							q_y[tail] = y;
							q_vh[tail] = ver;
							tail++;
						} else {
							break;
						}
					}
				}
			}
			//////*東*//////
			if ((checkMapWall(q_x[head] + 1, q_y[head], East, _state) == false)	//壁がなく
					&& (DiagStepMap_ver[q_x[head] + 1][q_y[head]]
							> DiagStepMap_ver[q_x[head]][q_y[head]] + Step_S)//歩数が大きく
					&& q_x[head] < 14) {			//座標が迷路内
				q_x[tail] = q_x[head] + 1;
				q_y[tail] = q_y[head];
				q_vh[tail] = ver;
				tail++;
				DiagStepMap_ver[q_x[head] + 1][q_y[head]] =
						DiagStepMap_ver[q_x[head]][q_y[head]] + Step_S;
				x = q_x[head] + 1;
				y = q_y[head];
				vh = ver;
				for (unsigned char i = 1;; i++) {
					if ((checkMapWall(x + 1, y, East, _state) == false)	//壁がなく
							&& (DiagStepMap_ver[x + 1][y]
									> DiagStepMap_ver[x][y] + Step_S
											- (DE_Step_S * i))	//歩数が大きく
							&& x < 14) {
						DiagStepMap_ver[x + 1][y] = DiagStepMap_ver[x][y]
								+ Step_S - (DE_Step_S * i);
						if (DiagStepMap_ver[x + 1][y]
								<= DiagStepMap_ver[x][y]) {
							DiagStepMap_ver[x + 1][y] = DiagStepMap_ver[x][y]
									+ 1;
						}
						x++;
						q_x[tail] = x;
						q_y[tail] = y;
						q_vh[tail] = ver;
						tail++;
					} else {
						break;
					}
				}
			}
			//////*南東*//////
			if ((checkMapWall(q_x[head] + 1, q_y[head] - 1, North, _state)
					== false)			//壁がなく
					&& (DiagStepMap_hor[q_x[head] + 1][q_y[head] - 1]
							> DiagStepMap_ver[q_x[head]][q_y[head]] + Step_D)//歩数が大きく
					&& q_x[head] < 15 && q_y[head] > 0) {			//座標が迷路内
				q_x[tail] = q_x[head] + 1;
				q_y[tail] = q_y[head] - 1;
				q_vh[tail] = hor;
				tail++;
				DiagStepMap_hor[q_x[head] + 1][q_y[head] - 1] =
						DiagStepMap_ver[q_x[head]][q_y[head]] + Step_D;
				vh = hor;
				x = q_x[head] + 1;
				y = q_y[head] - 1;
				for (unsigned char i = 1;; i++) {
					if (vh == ver) {
						if ((checkMapWall(x + 1, y - 1, North, _state) == false)//壁がなく
								&& (DiagStepMap_hor[x + 1][y - 1]
										> DiagStepMap_ver[x][y] + Step_D
												- (DE_Step_D * i))		//歩数が大きく
								&& x < 15 && y > 0) {
							DiagStepMap_hor[x + 1][y - 1] =
									DiagStepMap_ver[x][y] + Step_D
											- (DE_Step_D * i);
							if (DiagStepMap_hor[x + 1][y - 1]
									<= DiagStepMap_ver[x][y]) {
								DiagStepMap_hor[x + 1][y - 1] =
										DiagStepMap_ver[x][y] + 1;
							}
							x++;
							y--;
							vh = hor;
							q_x[tail] = x;
							q_y[tail] = y;
							q_vh[tail] = hor;
							tail++;
						} else {
							break;
						}
					} else if (vh == hor) {
						if ((checkMapWall(x, y, East, _state) == false)	//壁がなく
								&& (DiagStepMap_ver[x][y]
										> DiagStepMap_hor[x][y] + Step_D
												- (DE_Step_D * i))	//歩数が大きく
								&& x < 15) {
							DiagStepMap_ver[x][y] = DiagStepMap_hor[x][y]
									+ Step_D - (DE_Step_D * i);
							if (DiagStepMap_ver[x][y]
									<= DiagStepMap_hor[x][y]) {
								DiagStepMap_ver[x][y] = DiagStepMap_hor[x][y]
										+ 1;
							}
							vh = ver;
							q_x[tail] = x;
							q_y[tail] = y;
							q_vh[tail] = ver;
							tail++;
						} else {
							break;
						}
					}
				}
			}
			//////*南西*//////
			if ((checkMapWall(q_x[head], q_y[head] - 1, North, _state) == false)//壁がなく
					&& (DiagStepMap_hor[q_x[head]][q_y[head] - 1]
							> DiagStepMap_ver[q_x[head]][q_y[head]] + Step_D)//歩数が大きく
					&& q_y[head] > 0) {			//座標が迷路内
				q_x[tail] = q_x[head];
				q_y[tail] = q_y[head] - 1;
				q_vh[tail] = hor;
				tail++;
				DiagStepMap_hor[q_x[head]][q_y[head] - 1] =
						DiagStepMap_ver[q_x[head]][q_y[head]] + Step_D;
				vh = hor;
				x = q_x[head];
				y = q_y[head] - 1;
				for (unsigned char i = 1;; i++) {
					if (vh == ver) {
						if ((checkMapWall(x, y - 1, North, _state) == false)//壁がなく
								&& (DiagStepMap_hor[x][y - 1]
										> DiagStepMap_ver[x][y] + Step_D
												- (DE_Step_D * i))		//歩数が大きく
								&& y > 0) {
							DiagStepMap_hor[x][y - 1] = DiagStepMap_ver[x][y]
									+ Step_D - (DE_Step_D * i);
							if (DiagStepMap_hor[x][y - 1]
									<= DiagStepMap_ver[x][y]) {
								DiagStepMap_hor[x][y - 1] =
										DiagStepMap_ver[x][y] + 1;
							}
							y--;
							vh = hor;
							q_x[tail] = x;
							q_y[tail] = y;
							q_vh[tail] = hor;
							tail++;
						} else {
							break;
						}
					} else if (vh == hor) {
						if ((checkMapWall(x - 1, y, East, _state) == false)	//壁がなく
								&& (DiagStepMap_ver[x - 1][y]
										> DiagStepMap_hor[x][y] + Step_D
												- (DE_Step_D * i))	//歩数が大きく
								&& x > 0) {
							DiagStepMap_ver[x - 1][y] = DiagStepMap_hor[x][y]
									+ Step_D - (DE_Step_D * i);
							if (DiagStepMap_ver[x - 1][y]
									<= DiagStepMap_hor[x][y]) {
								DiagStepMap_ver[x - 1][y] =
										DiagStepMap_hor[x][y] + 1;
							}
							x--;
							vh = ver;
							q_x[tail] = x;
							q_y[tail] = y;
							q_vh[tail] = ver;
							tail++;
						} else {
							break;
						}
					}
				}
			}
			//////*西*//////
			if ((checkMapWall(q_x[head] - 1, q_y[head], East, _state) == false)	//壁がなく
					&& (DiagStepMap_ver[q_x[head] - 1][q_y[head]]
							> DiagStepMap_ver[q_x[head]][q_y[head]] + Step_S)//歩数が大きく
					&& q_x[head] > 0) {			//座標が迷路内
				q_x[tail] = q_x[head] - 1;
				q_y[tail] = q_y[head];
				q_vh[tail] = ver;
				tail++;
				DiagStepMap_ver[q_x[head] - 1][q_y[head]] =
						DiagStepMap_ver[q_x[head]][q_y[head]] + Step_S;
				x = q_x[head] - 1;
				y = q_y[head];
				vh = ver;
				for (unsigned char i = 1;; i++) {
					if ((checkMapWall(x - 1, y, East, _state) == false)	//壁がなく
							&& (DiagStepMap_ver[x - 1][y]
									> DiagStepMap_ver[x][y] + Step_S
											- (DE_Step_S * i))	//歩数が大きく
							&& x > 0) {
						DiagStepMap_ver[x - 1][y] = DiagStepMap_ver[x][y]
								+ Step_S - (DE_Step_S * i);
						if (DiagStepMap_ver[x - 1][y]
								<= DiagStepMap_ver[x][y]) {
							DiagStepMap_ver[x - 1][y] = DiagStepMap_ver[x][y]
									+ 1;
						}
						x--;
						q_x[tail] = x;
						q_y[tail] = y;
						q_vh[tail] = ver;
						tail++;
					} else {
						break;
					}
				}
			}
		} else if (q_vh[head] == hor) {			//横壁
			//////*北*//////
			if ((checkMapWall(q_x[head], q_y[head] + 1, North, _state) == false)//壁がなく
					&& (DiagStepMap_hor[q_x[head]][q_y[head] + 1]
							> DiagStepMap_hor[q_x[head]][q_y[head]] + Step_S)//歩数が大きく
					&& q_y[head] < 14) {			//座標が迷路内
				q_x[tail] = q_x[head];
				q_y[tail] = q_y[head] + 1;
				q_vh[tail] = hor;
				tail++;
				DiagStepMap_hor[q_x[head]][q_y[head] + 1] =
						DiagStepMap_hor[q_x[head]][q_y[head]] + Step_S;
				x = q_x[head];
				y = q_y[head] + 1;
				vh = hor;
				for (unsigned char i = 1;; i++) {
					if ((checkMapWall(x, y + 1, North, _state) == false)//壁がなく
							&& (DiagStepMap_hor[x][y + 1]
									> DiagStepMap_hor[x][y] + Step_S
											- (DE_Step_S * i))			//歩数が大きく
							&& y < 14) {
						DiagStepMap_hor[x][y + 1] = DiagStepMap_hor[x][y]
								+ Step_S - (DE_Step_S * i);
						if (DiagStepMap_hor[x][y + 1]
								<= DiagStepMap_hor[x][y]) {
							DiagStepMap_hor[x][y + 1] = DiagStepMap_hor[x][y]
									+ 1;
						}
						y++;
						q_x[tail] = x;
						q_y[tail] = y;
						q_vh[tail] = hor;
						tail++;
					} else {
						break;
					}
				}
			}
			//////*北東*//////
			if ((checkMapWall(q_x[head], q_y[head] + 1, East, _state) == false)	//壁がなく
					&& (DiagStepMap_ver[q_x[head]][q_y[head] + 1]
							> DiagStepMap_hor[q_x[head]][q_y[head]] + Step_D)//歩数が大きく
					&& q_x[head] < 15 && q_y[head] < 15) {			//座標が迷路内
				q_x[tail] = q_x[head];
				q_y[tail] = q_y[head] + 1;
				q_vh[tail] = ver;
				tail++;
				DiagStepMap_ver[q_x[head]][q_y[head] + 1] =
						DiagStepMap_hor[q_x[head]][q_y[head]] + Step_D;
				vh = ver;
				x = q_x[head];
				y = q_y[head] + 1;
				for (unsigned char i = 1;; i++) {
					if (vh == ver) {
						if ((checkMapWall(x + 1, y, North, _state) == false)//壁がなく
								&& (DiagStepMap_hor[x + 1][y]
										> DiagStepMap_ver[x][y] + Step_D
												- (DE_Step_D * i))		//歩数が大きく
								&& x < 15 && y < 15) {
							DiagStepMap_hor[x + 1][y] = DiagStepMap_ver[x][y]
									+ Step_D - (DE_Step_D * i);
							if (DiagStepMap_hor[x + 1][y]
									<= DiagStepMap_ver[x][y]) {
								DiagStepMap_hor[x + 1][y] =
										DiagStepMap_ver[x][y] + 1;
							}
							x++;
							vh = hor;
							q_x[tail] = x;
							q_y[tail] = y;
							q_vh[tail] = hor;
							tail++;
						} else {
							break;
						}
					} else if (vh == hor) {
						if ((checkMapWall(x, y + 1, East, _state) == false)	//壁がなく
								&& (DiagStepMap_ver[x][y + 1]
										> DiagStepMap_hor[x][y] + Step_D
												- (DE_Step_D * i))	//歩数が大きく
								&& x < 15 && y < 15) {
							DiagStepMap_ver[x][y + 1] = DiagStepMap_hor[x][y]
									+ Step_D - (DE_Step_D * i);
							if (DiagStepMap_ver[x][y + 1]
									<= DiagStepMap_hor[x][y]) {
								DiagStepMap_ver[x][y + 1] =
										DiagStepMap_hor[x][y] + 1;
							}
							y++;
							vh = ver;
							q_x[tail] = x;
							q_y[tail] = y;
							q_vh[tail] = ver;
							tail++;
						} else {
							break;
						}
					}
				}
			}
			//////*南東*//////
			if ((checkMapWall(q_x[head], q_y[head], East, _state) == false)	//壁がなく
					&& (DiagStepMap_ver[q_x[head]][q_y[head]]
							> DiagStepMap_hor[q_x[head]][q_y[head]] + Step_D)//歩数が大きく
					&& q_x[head] < 15) {			//座標が迷路内
				q_x[tail] = q_x[head];
				q_y[tail] = q_y[head];
				q_vh[tail] = ver;
				tail++;
				DiagStepMap_ver[q_x[head]][q_y[head]] =
						DiagStepMap_hor[q_x[head]][q_y[head]] + Step_D;
				vh = ver;
				x = q_x[head];
				y = q_y[head];
				for (unsigned char i = 1;; i++) {
					if (vh == ver) {
						if ((checkMapWall(x + 1, y - 1, North, _state) == false)//壁がなく
								&& (DiagStepMap_hor[x + 1][y - 1]
										> DiagStepMap_ver[x][y] + Step_D
												- (DE_Step_D * i))		//歩数が大きく
								&& (DE_Step_D * i < Step_D) && x < 15
								&& y > 0) {
							DiagStepMap_hor[x + 1][y - 1] =
									DiagStepMap_ver[x][y] + Step_D
											- (DE_Step_D * i);
							if (DiagStepMap_hor[x + 1][y - 1]
									<= DiagStepMap_ver[x][y]) {
								DiagStepMap_hor[x + 1][y - 1] =
										DiagStepMap_ver[x][y] + 1;
							}
							x++;
							y--;
							vh = hor;
							q_x[tail] = x;
							q_y[tail] = y;
							q_vh[tail] = hor;
							tail++;
						} else {
							break;
						}
					} else if (vh == hor) {
						if ((checkMapWall(x, y, East, _state) == false)	//壁がなく
								&& (DiagStepMap_ver[x][y]
										> DiagStepMap_hor[x][y] + Step_D
												- (DE_Step_D * i))	//歩数が大きく
								&& x < 15) {
							DiagStepMap_ver[x][y] = DiagStepMap_hor[x][y]
									+ Step_D - (DE_Step_D * i);
							if (DiagStepMap_ver[x][y]
									<= DiagStepMap_hor[x][y]) {
								DiagStepMap_ver[x][y] = DiagStepMap_hor[x][y]
										+ 1;
							}
							vh = ver;
							q_x[tail] = x;
							q_y[tail] = y;
							q_vh[tail] = ver;
							tail++;
						} else {
							break;
						}
					}
				}
			}
			//////*南*//////
			if ((checkMapWall(q_x[head], q_y[head] - 1, North, _state) == false)//壁がなく
					&& (DiagStepMap_hor[q_x[head]][q_y[head] - 1]
							> DiagStepMap_hor[q_x[head]][q_y[head]] + Step_S)//歩数が大きく
					&& q_y[head] > 0) {			//座標が迷路内
				q_x[tail] = q_x[head];
				q_y[tail] = q_y[head] - 1;
				q_vh[tail] = hor;
				tail++;
				DiagStepMap_hor[q_x[head]][q_y[head] - 1] =
						DiagStepMap_hor[q_x[head]][q_y[head]] + Step_S;
				x = q_x[head];
				y = q_y[head] - 1;
				vh = hor;
				for (unsigned char i = 1;; i++) {
					if ((checkMapWall(x, y - 1, North, _state) == false)//壁がなく
							&& (DiagStepMap_hor[x][y - 1]
									> DiagStepMap_hor[x][y] + Step_S
											- (DE_Step_S * i))			//歩数が大きく
							&& y > 0) {
						DiagStepMap_hor[x][y - 1] = DiagStepMap_hor[x][y]
								+ Step_S - (DE_Step_S * i);
						if (DiagStepMap_hor[x][y - 1]
								<= DiagStepMap_hor[x][y]) {
							DiagStepMap_hor[x][y - 1] = DiagStepMap_hor[x][y]
									+ 1;
						}
						y--;
						q_x[tail] = x;
						q_y[tail] = y;
						q_vh[tail] = hor;
						tail++;
					} else {
						break;
					}
				}
			}
			//////*南西*//////
			if ((checkMapWall(q_x[head] - 1, q_y[head], East, _state) == false)	//壁がなく
					&& (DiagStepMap_ver[q_x[head] - 1][q_y[head]]
							> DiagStepMap_hor[q_x[head]][q_y[head]] + Step_D)//歩数が大きく
					&& q_x[head] > 0) {			//座標が迷路内
				q_x[tail] = q_x[head] - 1;
				q_y[tail] = q_y[head];
				q_vh[tail] = ver;
				tail++;
				DiagStepMap_ver[q_x[head] - 1][q_y[head]] =
						DiagStepMap_hor[q_x[head]][q_y[head]] + Step_D;
				vh = ver;
				x = q_x[head] - 1;
				y = q_y[head];
				for (unsigned char i = 1;; i++) {
					if (vh == ver) {
						if ((checkMapWall(x, y - 1, North, _state) == false)//壁がなく
								&& (DiagStepMap_hor[x][y - 1]
										> DiagStepMap_ver[x][y] + Step_D
												- (DE_Step_D * i))		//歩数が大きく
								&& y > 0) {
							DiagStepMap_hor[x][y - 1] = DiagStepMap_ver[x][y]
									+ Step_D - (DE_Step_D * i);
							if (DiagStepMap_hor[x][y - 1]
									<= DiagStepMap_ver[x][y]) {
								DiagStepMap_hor[x][y - 1] =
										DiagStepMap_ver[x][y] + 1;
							}
							y--;
							vh = hor;
							q_x[tail] = x;
							q_y[tail] = y;
							q_vh[tail] = hor;
							tail++;
						} else {
							break;
						}
					} else if (vh == hor) {
						if ((checkMapWall(x - 1, y, East, _state) == false)	//壁がなく
								&& (DiagStepMap_ver[x - 1][y]
										> DiagStepMap_hor[x][y] + Step_D
												- (DE_Step_D * i))	//歩数が大きく
								&& x > 0) {
							DiagStepMap_ver[x - 1][y] = DiagStepMap_hor[x][y]
									+ Step_D - (DE_Step_D * i);
							if (DiagStepMap_ver[x - 1][y]
									<= DiagStepMap_hor[x][y]) {
								DiagStepMap_ver[x - 1][y] =
										DiagStepMap_hor[x][y] + 1;
							}
							x--;
							vh = ver;
							q_x[tail] = x;
							q_y[tail] = y;
							q_vh[tail] = ver;
							tail++;
						} else {
							break;
						}
					}
				}
			}
			//////*北西*//////
			if ((checkMapWall(q_x[head] - 1, q_y[head] + 1, East, _state)
					== false)			//壁がなく
					&& (DiagStepMap_ver[q_x[head] - 1][q_y[head] + 1]
							> DiagStepMap_hor[q_x[head]][q_y[head]] + Step_D)//歩数が大きく
					&& q_x[head] > 0 && q_y[head] < 15) {			//座標が迷路内
				q_x[tail] = q_x[head] - 1;
				q_y[tail] = q_y[head] + 1;
				q_vh[tail] = ver;
				tail++;
				DiagStepMap_ver[q_x[head] - 1][q_y[head] + 1] =
						DiagStepMap_hor[q_x[head]][q_y[head]] + Step_D;
				vh = ver;
				x = q_x[head] - 1;
				y = q_y[head] + 1;
				for (unsigned char i = 1;; i++) {
					if (vh == ver) {
						if ((checkMapWall(x, y, North, _state) == false)//壁がなく
								&& (DiagStepMap_hor[x][y]
										> DiagStepMap_ver[x][y] + Step_D
												- (DE_Step_D * i))		//歩数が大きく
								&& y < 15) {
							DiagStepMap_hor[x][y] = DiagStepMap_ver[x][y]
									+ Step_D - (DE_Step_D * i);
							if (DiagStepMap_hor[x][y]
									<= DiagStepMap_ver[x][y]) {
								DiagStepMap_hor[x][y] = DiagStepMap_ver[x][y]
										+ 1;
							}
							vh = hor;
							q_x[tail] = x;
							q_y[tail] = y;
							q_vh[tail] = hor;
							tail++;
						} else {
							break;
						}
					} else if (vh == hor) {
						if ((checkMapWall(x - 1, y + 1, East, _state) == false)	//壁がなく
								&& (DiagStepMap_ver[x - 1][y + 1]
										> DiagStepMap_hor[x][y] + Step_D
												- (DE_Step_D * i))	//歩数が大きく
								&& x > 0 && q_y[head] < 15) {
							DiagStepMap_ver[x - 1][y + 1] =
									DiagStepMap_hor[x][y] + Step_D
											- (DE_Step_D * i);
							if (DiagStepMap_ver[x - 1][y + 1]
									<= DiagStepMap_hor[x][y]) {
								DiagStepMap_ver[x - 1][y + 1] =
										DiagStepMap_hor[x][y] + 1;
							}
							x--;
							y++;
							vh = ver;
							q_x[tail] = x;
							q_y[tail] = y;
							q_vh[tail] = ver;
							tail++;
						} else {
							break;
						}
					}
				}
			}
		}
	}
}

void AdachiMethod(unsigned char _Goal_X, unsigned char _Goal_Y,
		uint8_t _isAccel) {	//足立法
	//uint8_t _isAccel:既知区間加速するか否か
	int _dir;
	volatile uint16_t countStraight = 0;	//既知区間中に直線の数を数える
	uint8_t North_exist = 0, West_exist = 0, East_exist = 0, South_exist = 0;
	//unsigned char stack_CurrentX[3], stack_CurrentY[3];
	while (Flag_failSafeEn == false
			&& !(current_coord_x == _Goal_X && current_coord_y == _Goal_Y)) {
		North_exist = 0;
		West_exist = 0;
		East_exist = 0;
		South_exist = 0;
		_dir = 3;
		if (StepMap[current_coord_x][current_coord_y] == 255) {
			stop(0.09, 7.0);
			HAL_Delay(500);
			Flag_failSafeEn = true;
			break;
		}
		switch (direction) {
		case North:
			North_exist = wallExist.Forward;
			West_exist = wallExist.Left;
			East_exist = wallExist.Right;
			break;
		case East:
			North_exist = wallExist.Left;
			South_exist = wallExist.Right;
			East_exist = wallExist.Forward;
			break;
		case South:
			South_exist = wallExist.Forward;
			West_exist = wallExist.Right;
			East_exist = wallExist.Left;
			break;
		case West:
			North_exist = wallExist.Right;
			West_exist = wallExist.Forward;
			South_exist = wallExist.Left;
			break;
		}
		writeWallData(current_coord_x, current_coord_y, direction);
		stepMapRenew(_Goal_X, _Goal_Y, false, false);
		if ((StepMap[current_coord_x][current_coord_y + 1]
				== StepMap[current_coord_x][current_coord_y] - 1)
				&& (current_coord_y != 15) && !North_exist) { //北
			switch (direction) {
			case North:
				_dir = Forward;
				break;
			case East:
				_dir = Left;
				break;
			case South:
				_dir = Back;
				break;
			case West:
				_dir = Right;
				break;
			}
		} else if ((StepMap[current_coord_x + 1][current_coord_y]
				== StepMap[current_coord_x][current_coord_y] - 1)
				&& (current_coord_x != 15) && !East_exist) { //東
			switch (direction) {
			case North:
				_dir = Right;
				break;
			case East:
				_dir = Forward;
				break;
			case South:
				_dir = Left;
				break;
			case West:
				_dir = Back;
				break;
			}
		} else if ((StepMap[current_coord_x][current_coord_y - 1]
				== StepMap[current_coord_x][current_coord_y] - 1)
				&& (current_coord_y != 0) && !South_exist) { //南
			switch (direction) {
			case North:
				_dir = Back;
				break;
			case East:
				_dir = Right;
				break;
			case South:
				_dir = Forward;
				break;
			case West:
				_dir = Left;
				break;
			}
		} else if ((StepMap[current_coord_x - 1][current_coord_y]
				== StepMap[current_coord_x][current_coord_y] - 1)
				&& (current_coord_x != 0) && !West_exist) { //西
			switch (direction) {
			case North:
				_dir = Left;
				break;
			case East:
				_dir = Back;
				break;
			case South:
				_dir = Right;
				break;
			case West:
				_dir = Forward;
				break;
			}
		}
		switch (_dir) {
		case Forward:
			/*直進*/
			if (_isAccel) { //既知区間加速
				switch (direction) {
				case North:
					for (countStraight = 0;
							StepMap[current_coord_x][current_coord_y
									+ countStraight]
									== StepMap[current_coord_x][current_coord_y
											+ countStraight + 1] + 1 //歩数が小さく
									&& Flag_cellfound[current_coord_x][current_coord_y
											+ countStraight + 1] //既知区間であり
									&& !checkMapWall(current_coord_x,
											current_coord_y + countStraight,
											North,
											true) //壁がない
							; countStraight++)
						;
					if (countStraight < 2) {
						straight18();
					} else {
						trapezoid_acccel(1.0, explore_turn_R.Turn_vel,
								0.18 * (float) countStraight, 7.0,
								true,
								false);
						current_coord_y += countStraight;
					}
					break;
				case East:
					for (countStraight = 0;
							StepMap[current_coord_x + countStraight][current_coord_y]
									== StepMap[current_coord_x + countStraight
											+ 1][current_coord_y] + 1
									&& Flag_cellfound[current_coord_x
											+ countStraight + 1][current_coord_y]
									&& !checkMapWall(
											current_coord_x + countStraight,
											current_coord_y, East, true);
							countStraight++)
						;
					if (countStraight < 2) {
						straight18();
					} else {
						trapezoid_acccel(1.0, explore_turn_R.Turn_vel,
								0.18 * (float) countStraight, 7.0, true, false);
						current_coord_x += countStraight;
					}
					break;
				case South:
					for (countStraight = 0;
							StepMap[current_coord_x][current_coord_y
									- countStraight]
									== StepMap[current_coord_x][current_coord_y
											- countStraight - 1] + 1
									&& Flag_cellfound[current_coord_x][current_coord_y
											- countStraight - 1]
									&& !checkMapWall(current_coord_x,
											current_coord_y - countStraight,
											South,
											true); countStraight++)
						;
					if (countStraight < 2) {
						straight18();
					} else {
						trapezoid_acccel(1.0, explore_turn_R.Turn_vel,
								0.18 * (float) countStraight, 7.0, true, false);
						current_coord_y -= countStraight;
					}
					break;
				case West:
					for (countStraight = 0;
							StepMap[current_coord_x - countStraight][current_coord_y]
									== StepMap[current_coord_x - countStraight
											- 1][current_coord_y] + 1
									&& Flag_cellfound[current_coord_x
											- countStraight - 1][current_coord_y]
									&& !checkMapWall(
											current_coord_x - countStraight,
											current_coord_y, West, true);
							countStraight++)
						;
					if (countStraight < 2) {
						straight18();
					} else {
						trapezoid_acccel(1.0, explore_turn_R.Turn_vel,
								0.18 * (float) countStraight, 7.0, true, false);
						current_coord_x -= countStraight;
					}
					break;
				}
			} else {
				straight18();
			}
			LED(0x00);
			break;
		case Left:
			/*左折*/
			turn90_s(Left);
			LED(0xF0);
			break;
		case Right:
			/*右折*/
			turn90_s(Right);
			LED(0x0F);
			break;
		case Back:
			/*Uターン*/
			turnU_2();
			break;
		}
		wall_exist();
		idealState.dis = 0.0;
		realState.dis = 0.0;
	}
}

void mapRenew(int _turnDir) {
	if (_turnDir == Right) {
		direction += 2;
		if (direction > 7) {
			direction = 0;
		}
	} else if (_turnDir == Left) {
		direction -= 2;
		if (direction < 0) {
			direction = 6;
		}

	} else if (_turnDir == 180) {
		direction += 4;
		if (direction == 8) {
			direction = 0;
		} else if (direction == 10) {
			direction = 2;
		}
	}
	switch (direction) {
	case North:
		current_coord_y++;
		break;
	case East:
		current_coord_x++;
		break;
	case South:
		current_coord_y--;
		break;
	case West:
		current_coord_x--;
		break;
	}
	Flag_cellfound[current_coord_x][current_coord_y] = true;
}

void SearchAlgorithm() { //経路生成探索アルゴリズム
	unsigned char x, y;
	unsigned short MinStep;
	int8_t vh = hor;
	int8_t MinDir = 0;
	int8_t ExDir = 0;
	uint8_t Flag_RouteDone = false; //経路ができたフラグ
	while (Flag_failSafeEn == false) {
		if (Flag_RouteDone) { //最短経路ができたら帰ってくる
			Flag_exploreDone = true;
			trapezoid_acccel(explore_turn_R.Turn_vel, 0.0, 0.09, 7.0, true,
			false);
			disable_motor();
			Speaker_ON(&htim2);
			Speaker_Herz(&htim2, Sc_Ah);
			HAL_Delay(70);
			Speaker_Herz(&htim2, Sc_Ch);
			HAL_Delay(70);
			Speaker_OFF(&htim2);
			BackUpWallData(Write);
			HAL_Delay(300);
			enable_motor();
			if (wallExist.Forward) {
				HAL_Delay(100);
				if (fabs(IR_Sen.RF - IR_Ref_single.RF) > 0.5
						|| fabs(IR_Sen.LF - IR_Ref_single.LF) > 0.5) {
					FrontWallControl();
				}
				if (wallExist.Right == true
						&& (fabs(IR_Sen.RF - IR_Ref_single.RF) > 0.5
								|| fabs(IR_Sen.LF - IR_Ref_single.LF) > 0.5)) {
					trapezoid_slalome(900.0, -90.0, 10000.0);
					FrontWallControl();
					trapezoid_slalome(900.0, -90.0, 10000.0);
				} else if (wallExist.Left == true
						&& (fabs(IR_Sen.RF - IR_Ref_single.RF) > 0.5
								|| fabs(IR_Sen.LF - IR_Ref_single.LF) > 0.5)) {
					trapezoid_slalome(900.0, 90.0, 10000.0);
					FrontWallControl();
					trapezoid_slalome(900.0, 90.0, 10000.0);
				} else {
					trapezoid_slalome(900.0, 180.0, 10000.0);
				}

				HAL_Delay(100);
				trapezoid_acccel(explore_turn_R.Turn_vel,
						explore_turn_R.Turn_vel, 0.09, 7.0,
						true, false);
				mapRenew(180);
			} else {
				trapezoid_acccel(explore_turn_R.Turn_vel,
						explore_turn_R.Turn_vel, 0.09 * 1.0, 7.0, true, false);
				mapRenew(0);
			}
			idealState.dis = 0.0;
			realState.dis = 0.0;
			wall_exist();
			AdachiMethod(0, 0, true);
			break;
		}
		x = 0;
		y = 0;
		vh = hor;
		stepMapRenew_DiagMap(Goal_X, Goal_Y, false, true); //壁を入れない状態で歩数マップ展開
		while (Flag_failSafeEn == false) {
			if (DiagStepMap_ver[x][y] == 0 || DiagStepMap_hor[x][y] == 0) {
				Flag_RouteDone = true;
				break;
			}
			if (vh == ver) {
				MinStep = DiagStepMap_ver[x][y];
				//////*北西*//////
				if (MinStep > DiagStepMap_hor[x][y] && y < 15) {
					MinStep = DiagStepMap_hor[x][y];
					MinDir = NorthWest;
					vh = hor;
				}
				//////*北東*//////
				if (MinStep > DiagStepMap_hor[x + 1][y] && x < 15 && y < 15) {
					MinStep = DiagStepMap_hor[x + 1][y];
					MinDir = NorthEast;
					vh = hor;
				}
				//////*東*//////
				if (MinStep > DiagStepMap_ver[x + 1][y] && x < 14) {
					MinStep = DiagStepMap_ver[x + 1][y];
					MinDir = East;
					vh = ver;
				}
				//////*南東*//////
				if (MinStep > DiagStepMap_hor[x + 1][y - 1] && x < 15
						&& y > 0) {
					MinStep = DiagStepMap_hor[x + 1][y - 1];
					MinDir = SouthEast;
					vh = hor;
				}
				//////*南西*//////
				if (MinStep > DiagStepMap_hor[x][y - 1] && y > 0) {
					MinStep = DiagStepMap_hor[x][y - 1];
					MinDir = SouthWest;
					vh = hor;
				}
				//////*西*//////
				if (MinStep > DiagStepMap_ver[x - 1][y] && x > 0) {
					MinStep = DiagStepMap_ver[x - 1][y];
					MinDir = West;
					vh = ver;
				}
				switch (ExDir) {
				case NorthWest:
					//////*北西*//////
					if (MinStep > DiagStepMap_hor[x][y] && y < 15) {
						MinStep = DiagStepMap_hor[x][y];
						MinDir = NorthWest;
						vh = hor;
					}
					break;
				case NorthEast:
					//////*北東*//////
					if (MinStep > DiagStepMap_hor[x + 1][y] && x < 15
							&& y < 15) {
						MinStep = DiagStepMap_hor[x + 1][y];
						MinDir = NorthEast;
						vh = hor;
					}
					break;
				case East:
					//////*東*//////
					if (MinStep > DiagStepMap_ver[x + 1][y] && x < 14) {
						MinStep = DiagStepMap_ver[x + 1][y];
						MinDir = East;
						vh = ver;
					}
					break;
				case SouthEast:
					//////*南東*//////
					if (MinStep > DiagStepMap_hor[x + 1][y - 1] && x < 15
							&& y > 0) {
						MinStep = DiagStepMap_hor[x + 1][y - 1];
						MinDir = SouthEast;
						vh = hor;
					}
					break;
				case SouthWest:
					//////*南西*//////
					if (MinStep > DiagStepMap_hor[x][y - 1] && y > 0) {
						MinStep = DiagStepMap_hor[x][y - 1];
						MinDir = SouthWest;
						vh = hor;
					}
					break;
				case West:
					//////*西*//////
					if (MinStep > DiagStepMap_ver[x - 1][y] && x > 0) {
						MinStep = DiagStepMap_ver[x - 1][y];
						MinDir = West;
						vh = ver;
					}
					break;
				}
				switch (MinDir) {
				case NorthWest:
					break;
				case NorthEast:
					x++;
					break;
				case East:
					x++;
					break;
				case SouthEast:
					x++;
					y--;
					break;
				case SouthWest:
					y--;
					break;
				case West:
					x--;
					break;
				}
			} else if (vh == hor) {
				MinStep = DiagStepMap_hor[x][y];
				//////*北*//////
				if (MinStep > DiagStepMap_hor[x][y + 1] && y < 14) {
					MinStep = DiagStepMap_hor[x][y + 1];
					MinDir = North;
					vh = hor;
				}
				//////*北東*//////
				if (MinStep > DiagStepMap_ver[x][y + 1] && x < 15 && y < 15) {
					MinStep = DiagStepMap_ver[x][y + 1];
					MinDir = NorthEast;
					vh = ver;
				}
				//////*南東*//////
				if (MinStep > DiagStepMap_ver[x][y] && y < 15) {
					MinStep = DiagStepMap_ver[x][y];
					MinDir = SouthEast;
					vh = ver;
				}
				//////*南*//////
				if (MinStep > DiagStepMap_hor[x][y - 1] && y > 0) {
					MinStep = DiagStepMap_hor[x][y - 1];
					MinDir = South;
					vh = hor;
				}
				//////*南西*//////
				if (MinStep > DiagStepMap_ver[x - 1][y] && x > 0) {
					MinStep = DiagStepMap_ver[x - 1][y];
					MinDir = SouthWest;
					vh = ver;
				}
				//////*北西*//////
				if (MinStep > DiagStepMap_ver[x - 1][y + 1] && x > 0
						&& y < 15) {
					MinStep = DiagStepMap_ver[x - 1][y + 1];
					MinDir = NorthWest;
					vh = ver;
				}
				switch (ExDir) {
				case North:
					//////*北*//////
					if (MinStep > DiagStepMap_hor[x][y + 1] && y < 14) {
						MinStep = DiagStepMap_hor[x][y + 1];
						MinDir = North;
						vh = hor;
					}
					break;
				case NorthEast:
					//////*北東*//////
					if (MinStep > DiagStepMap_ver[x][y + 1] && x < 15
							&& y < 15) {
						MinStep = DiagStepMap_ver[x][y + 1];
						MinDir = NorthEast;
						vh = ver;
					}
					break;
				case SouthEast:
					//////*南東*//////
					if (MinStep > DiagStepMap_ver[x][y] && y < 15) {
						MinStep = DiagStepMap_ver[x][y];
						MinDir = SouthEast;
						vh = ver;
					}
					break;
				case South:
					//////*南*//////
					if (MinStep > DiagStepMap_hor[x][y - 1] && y > 0) {
						MinStep = DiagStepMap_hor[x][y - 1];
						MinDir = South;
						vh = hor;
					}
					break;
				case SouthWest:
					//////*南西*//////
					if (MinStep > DiagStepMap_ver[x - 1][y] && x > 0) {
						MinStep = DiagStepMap_ver[x - 1][y];
						MinDir = SouthWest;
						vh = ver;
					}
					break;
				case NorthWest:
					//////*北西*//////
					if (MinStep > DiagStepMap_ver[x - 1][y + 1] && x > 0
							&& y < 15) {
						MinStep = DiagStepMap_ver[x - 1][y + 1];
						MinDir = NorthWest;
						vh = ver;
					}
					break;
				}
				switch (MinDir) {
				case North:
					y++;
					break;
				case NorthEast:
					y++;
					break;
				case SouthEast:
					break;
				case South:
					y--;
					break;
				case SouthWest:
					x--;
					break;
				case NorthWest:
					x--;
					y++;
					break;
				}
			}
			ExDir = MinDir;
			if (Flag_cellfound[x][y] == false) { //未知区間が現れた場合
				wall_exist();
				AdachiMethod(x, y, true);
				writeWallData(current_coord_x, current_coord_y, direction);
				Flag_cellfound[current_coord_x][current_coord_y] = true;
				break;
			}
		}
	}
}

void stepMapRenew(unsigned char _Goal_X, unsigned char _Goal_Y, uint8_t _state,
		uint8_t _is4cell) { //歩数マップ展開
	//_state :直接読んでいない場所に壁を入れるかどうか
	//_is4cell : 4マス対応にするかどうか
	unsigned char q[500];
	unsigned char head = 0, tail = 1, x, y;
	for (int x = 0; x < 16; x++) {
		for (int y = 0; y < 16; y++) {
			StepMap[x][y] = 255;
		}
	}
	StepMap[_Goal_X][_Goal_Y] = 0;
	q[0] = _Goal_X * 16 + _Goal_Y;
	if (_is4cell) {
		/*ゴール区画に壁が入るのを防止*/
		writeMapWall(_Goal_X, _Goal_Y, North, true);
		writeMapWall(_Goal_X, _Goal_Y, East, true);
		writeMapWall(_Goal_X + 1, _Goal_Y + 1, South, true);
		writeMapWall(_Goal_X + 1, _Goal_Y + 1, West, true);
		StepMap[_Goal_X + 1][_Goal_Y] = 0;
		StepMap[_Goal_X][_Goal_Y + 1] = 0;
		StepMap[_Goal_X + 1][_Goal_Y + 1] = 0;
		q[1] = (_Goal_X + 1) * 16 + _Goal_Y;
		q[2] = _Goal_X * 16 + (_Goal_Y + 1);
		q[3] = (_Goal_X + 1) * 16 + (_Goal_Y + 1);
		tail = 4;
	}

	while (head != tail) {
		x = q[head] >> 4;
		y = q[head] & 0x0f;
		head++;
		if ((checkMapWall(x, y, North, _state) == false)
				&& (StepMap[x][y + 1] == 255) && y != 15) {	//北壁が無く未更新の場合
			q[tail] = x * 16 + y + 1;
			tail++;
			StepMap[x][y + 1] = StepMap[x][y] + 1;
		}
		if ((checkMapWall(x, y, East, _state) == false)
				&& (StepMap[x + 1][y] == 255) && x != 15) {	//東壁が無く未更新の場合
			q[tail] = (x + 1) * 16 + y;
			tail++;
			StepMap[x + 1][y] = StepMap[x][y] + 1;
		}
		if ((checkMapWall(x, y, South, _state) == false)
				&& (StepMap[x][y - 1] == 255) && y != 0) {	//南壁が無く未更新の場合
			q[tail] = x * 16 + y - 1;
			tail++;
			StepMap[x][y - 1] = StepMap[x][y] + 1;
		}
		if ((checkMapWall(x, y, West, _state) == false)
				&& (StepMap[x - 1][y] == 255) && x != 0) {	//北壁が無く未更新の場合
			q[tail] = (x - 1) * 16 + y;
			tail++;
			StepMap[x - 1][y] = StepMap[x][y] + 1;
		}
	}
}

void generateRoute_DiagMap() {		//斜め歩数マップ用ルート生成
	uint8_t x, y;
	volatile uint16_t MinStep;
	volatile uint16_t i;
	int8_t vh = hor;
	int8_t MinDir = 0;
	int8_t ExDir = 0;
	x = 0;
	y = 0;
	routeDir[0] = F;
	for (i = 1;; i++) {
		if (vh == ver) {
			MinStep = DiagStepMap_ver[x][y];
			//////*東*//////
			if (MinStep > DiagStepMap_ver[x + 1][y] && x < 14) {
				MinStep = DiagStepMap_ver[x + 1][y];
				MinDir = East;
				vh = ver;
			}
			//////*西*//////
			if (MinStep > DiagStepMap_ver[x - 1][y] && x > 0) {
				MinStep = DiagStepMap_ver[x - 1][y];
				MinDir = West;
				vh = ver;
			}
			//////*北西*//////
			if (MinStep > DiagStepMap_hor[x][y] && y < 15) {
				MinStep = DiagStepMap_hor[x][y];
				MinDir = NorthWest;
				vh = hor;
			}
			//////*北東*//////
			if (MinStep > DiagStepMap_hor[x + 1][y] && x < 15 && y < 15) {
				MinStep = DiagStepMap_hor[x + 1][y];
				MinDir = NorthEast;
				vh = hor;
			}
			//////*南東*//////
			if (MinStep > DiagStepMap_hor[x + 1][y - 1] && x < 15 && y > 0) {
				MinStep = DiagStepMap_hor[x + 1][y - 1];
				MinDir = SouthEast;
				vh = hor;
			}
			//////*南西*//////
			if (MinStep > DiagStepMap_hor[x][y - 1] && y > 0) {
				MinStep = DiagStepMap_hor[x][y - 1];
				MinDir = SouthWest;
				vh = hor;
			}
			if (MinStep == 0) {
				break;
			}
			switch (ExDir) {
			case NorthWest:
				//////*北西*//////
				if (MinStep > DiagStepMap_hor[x][y] && y < 15) {
					MinStep = DiagStepMap_hor[x][y];
					MinDir = NorthWest;
					vh = hor;
				}
				break;
			case NorthEast:
				//////*北東*//////
				if (MinStep > DiagStepMap_hor[x + 1][y] && x < 15 && y < 15) {
					MinStep = DiagStepMap_hor[x + 1][y];
					MinDir = NorthEast;
					vh = hor;
				}
				break;
			case East:
				//////*東*//////
				if (MinStep > DiagStepMap_ver[x + 1][y] && x < 14) {
					MinStep = DiagStepMap_ver[x + 1][y];
					MinDir = East;
					vh = ver;
				}
				break;
			case SouthEast:
				//////*南東*//////
				if (MinStep > DiagStepMap_hor[x + 1][y - 1] && x < 15
						&& y > 0) {
					MinStep = DiagStepMap_hor[x + 1][y - 1];
					MinDir = SouthEast;
					vh = hor;
				}
				break;
			case SouthWest:
				//////*南西*//////
				if (MinStep > DiagStepMap_hor[x][y - 1] && y > 0) {
					MinStep = DiagStepMap_hor[x][y - 1];
					MinDir = SouthWest;
					vh = hor;
				}
				break;
			case West:
				//////*西*//////
				if (MinStep > DiagStepMap_ver[x - 1][y] && x > 0) {
					MinStep = DiagStepMap_ver[x - 1][y];
					MinDir = West;
					vh = ver;
				}
				break;
			}
			switch (MinDir) {
			case NorthWest:
				routeDir[i] = R;
				break;
			case NorthEast:
				routeDir[i] = L;
				x++;
				break;
			case East:
				routeDir[i] = F;
				x++;
				break;
			case SouthEast:
				routeDir[i] = R;
				x++;
				y--;
				break;
			case SouthWest:
				routeDir[i] = L;
				y--;
				break;
			case West:
				routeDir[i] = F;
				x--;
				break;
			}
		} else if (vh == hor) {
			MinStep = DiagStepMap_hor[x][y];
			//////*北*//////
			if (MinStep > DiagStepMap_hor[x][y + 1] && y < 14) {
				MinStep = DiagStepMap_hor[x][y + 1];
				MinDir = North;
				vh = hor;
			}
			//////*南*//////
			if (MinStep > DiagStepMap_hor[x][y - 1] && y > 0) {
				MinStep = DiagStepMap_hor[x][y - 1];
				MinDir = South;
				vh = hor;
			}
			//////*北東*//////
			if (MinStep > DiagStepMap_ver[x][y + 1] && x < 15 && y < 15) {
				MinStep = DiagStepMap_ver[x][y + 1];
				MinDir = NorthEast;
				vh = ver;
			}
			//////*南東*//////
			if (MinStep > DiagStepMap_ver[x][y] && x < 15 && y < 16) {
				MinStep = DiagStepMap_ver[x][y];
				MinDir = SouthEast;
				vh = ver;
			}
			//////*南西*//////
			if (MinStep > DiagStepMap_ver[x - 1][y] && x > 0) {
				MinStep = DiagStepMap_ver[x - 1][y];
				MinDir = SouthWest;
				vh = ver;
			}
			//////*北西*//////
			if (MinStep > DiagStepMap_ver[x - 1][y + 1] && x > 0 && y < 15) {
				MinStep = DiagStepMap_ver[x - 1][y + 1];
				MinDir = NorthWest;
				vh = ver;
			}
			if (MinStep == 0) {
				break;
			}
			switch (ExDir) {
			case North:
				//////*北*//////
				if (MinStep > DiagStepMap_hor[x][y + 1] && y < 14) {
					MinStep = DiagStepMap_hor[x][y + 1];
					MinDir = North;
					vh = hor;
				}
				break;
			case NorthEast:
				//////*北東*//////
				if (MinStep > DiagStepMap_ver[x][y + 1] && x < 15 && y < 15) {
					MinStep = DiagStepMap_ver[x][y + 1];
					MinDir = NorthEast;
					vh = ver;
				}
				break;
			case SouthEast:
				//////*南東*//////
				if (MinStep > DiagStepMap_ver[x][y] && x < 15 && y < 16) {
					MinStep = DiagStepMap_ver[x][y];
					MinDir = SouthEast;
					vh = ver;
				}
				break;
			case South:
				//////*南*//////
				if (MinStep > DiagStepMap_hor[x][y - 1] && y > 0) {
					MinStep = DiagStepMap_hor[x][y - 1];
					MinDir = South;
					vh = hor;
				}
				break;
			case SouthWest:
				//////*南西*//////
				if (MinStep > DiagStepMap_ver[x - 1][y] && x > 0) {
					MinStep = DiagStepMap_ver[x - 1][y];
					MinDir = SouthWest;
					vh = ver;
				}
				break;
			case NorthWest:
				//////*北西*//////
				if (MinStep > DiagStepMap_ver[x - 1][y + 1] && x > 0
						&& y < 15) {
					MinStep = DiagStepMap_ver[x - 1][y + 1];
					MinDir = NorthWest;
					vh = ver;
				}
				break;
			}
			switch (MinDir) {
			case North:
				routeDir[i] = F;
				y++;
				break;
			case NorthEast:
				routeDir[i] = R;
				y++;
				break;
			case SouthEast:
				routeDir[i] = L;
				break;
			case South:
				routeDir[i] = F;
				y--;
				break;
			case SouthWest:
				routeDir[i] = R;
				x--;
				break;
			case NorthWest:
				routeDir[i] = L;
				x--;
				y++;
				break;
			}
		}
		ExDir = MinDir;
		printf("x=%d,y=%d,%d,", x, y, MinStep);
		if (routeDir[i] == F) {
			printf("%d,F\n", i);
		} else if (routeDir[i] == R) {
			printf("%d,R\n", i);
		} else if (routeDir[i] == L) {
			printf("%d,L\n", i);
		} else {
			printf("%d,S\n", i);
		}
	}
	if (routeDir[i + 1] == F) {
		printf("%d,F\n", i);
	} else if (routeDir[i + 1] == R) {
		printf("%d,R\n", i);
	} else if (routeDir[i + 1] == L) {
		printf("%d,L\n", i);
	} else {
		printf("%d,S\n", i);
	}
}

void generatePass(uint8_t _isDiag) {
	int i, j = 0;
//移動方向を決める
	writeMapWall(0, 0, North, true);		//何故かスタートの壁が入る場合があるので一応削除
//generateRoute_Simple();						   //経路導出
	generateRoute_DiagMap();	//経路導出
//以下パス生成
	uint8_t _Flag_diagMode = false;

	for (i = 0; routeDir[j] != Stop; i++) {
		if (_isDiag) {	//斜めありパス
			if (_Flag_diagMode == false) {	//通常モード
				if (routeDir[j] == F && routeDir[j + 1] == R
						&& routeDir[j + 2] <= F) {	//右大周り90度
					if (pass[i - 1] > 0 || i == 0) {
						pass[i] = TurnR_90;
					} else {
						pass[i] = Straight;
						pass[i + 1] = TurnR_90;
						i++;
					}
					j++;
				} else if (routeDir[j] == F && routeDir[j + 1] == L
						&& routeDir[j + 2] <= F) {	//左大周り90度
					if (pass[i - 1] > 0 && i != 0) {
						pass[i] = TurnL_90;
					} else {
						pass[i] = Straight;
						pass[i + 1] = TurnL_90;
						i++;
					}
					j++;
				} else if (routeDir[j] == F && routeDir[j + 1] == R
						&& routeDir[j + 2] == R && routeDir[j + 3] <= F) {//右大周り180度
					if (pass[i - 1] > 0 && i != 0) {
						pass[i] = TurnR_180;
					} else {
						pass[i] = Straight;
						pass[i + 1] = TurnR_180;
						i++;
					}
					j += 2;
				} else if (routeDir[j] == F && routeDir[j + 1] == L
						&& routeDir[j + 2] == L && routeDir[j + 3] <= F) {//左大周り180度
					if (pass[i - 1] > 0 && i != 0) {
						pass[i] = TurnL_180;
					} else {
						pass[i] = Straight;
						pass[i + 1] = TurnL_180;
						i++;
					}
					j += 2;
				} else if (routeDir[j] == F && routeDir[j + 1] == R
						&& routeDir[j + 2] == R && routeDir[j + 3] > F) {//右大周り135度
					if (pass[i - 1] > 0 || i == 0) {
						pass[i] = TurnR_135_I;
					} else {
						pass[i] = Straight;
						pass[i + 1] = TurnR_135_I;
						i++;
					}
					j += 2;
					_Flag_diagMode = true;
				} else if (routeDir[j] == F && routeDir[j + 1] == L
						&& routeDir[j + 2] == L && routeDir[j + 3] > F) {//左大周り135度
					if (pass[i - 1] > 0 && i != 0) {
						pass[i] = TurnL_135_I;
					} else {
						pass[i] = Straight;
						pass[i + 1] = TurnL_135_I;
						i++;
					}
					j += 2;
					_Flag_diagMode = true;
				} else if (routeDir[j] == F && routeDir[j + 1] == R
						&& routeDir[j + 2] == L) {	//右大周り45度
					if (pass[i - 1] > 0 || i == 0) {
						pass[i] = TurnR_45_I;
					} else {
						pass[i] = Straight;
						pass[i + 1] = TurnR_45_I;
						i++;
					}
					j++;
					_Flag_diagMode = true;
				} else if (routeDir[j] == F && routeDir[j + 1] == L
						&& routeDir[j + 2] == R) {	//左大周り45度
					if (pass[i - 1] > 0 && i != 0) {
						pass[i] = TurnL_45_I;
					} else {
						pass[i] = Straight;
						pass[i + 1] = TurnL_45_I;
						i++;
					}
					j++;
					_Flag_diagMode = true;
				} else if (routeDir[j] == F && routeDir[j + 1] == Stop) {//終端直線
					pass[i] = Straight;
					pass[i + 1] = Straight;
					pass[i + 2] = Straight;
					pass[i + 3] = Straight;
					pass[i + 4] = Straight;
					i += 3;
				} else {	//直線
					if (pass[i - 1] > 0 || i == 0) {
						pass[i] = Straight;
					} else {
						pass[i] = Straight;
						pass[i + 1] = Straight;
						i++;
					}
				}
			} else {	//斜めモード
				if (routeDir[j] == R && routeDir[j + 1] <= F) {	//右大周り45度
					pass[i] = TurnR_45_O;
					_Flag_diagMode = false;
				} else if (routeDir[j] == L && routeDir[j + 1] <= F) {//左大周り45度
					pass[i] = TurnL_45_O;
					_Flag_diagMode = false;
				} else if (routeDir[j] == R && routeDir[j + 1] == R
						&& routeDir[j + 2] == F) {	//右大周り135度
					pass[i] = TurnR_135_O;
					_Flag_diagMode = false;
					j++;
				} else if (routeDir[j] == L && routeDir[j + 1] == L
						&& routeDir[j + 2] == F) {	//左大周り135度
					pass[i] = TurnL_135_O;
					_Flag_diagMode = false;
					j++;
				} else if (routeDir[j] == R && routeDir[j + 1] == R
						&& routeDir[j + 2] != F) {	//右大周り90度
					pass[i] = TurnR_90_D;
					j++;
				} else if (routeDir[j] == L && routeDir[j + 1] == L
						&& routeDir[j + 2] != F) {	//左大周り90度
					pass[i] = TurnL_90_D;
					j++;
				} else if (routeDir[j] > F && routeDir[j + 1] == Stop) {//終端斜め直線
					pass[i] = Straight_D;
					pass[i + 1] = Straight_D;
					i++;
				} else {
					pass[i] = Straight_D;
				}
			}
		} else {	//斜め無しパス
			if (routeDir[j] == R) {
				pass[i] = TurnR_90;
			} else if (routeDir[j] == L) {
				pass[i] = TurnL_90;
			} else if (routeDir[j] == F) {
				if (i == 0) {
					pass[i] = Straight;
				} else {
					pass[i] = Straight;
					pass[i + 1] = Straight;
					i++;
				}
			}
		}
		j++;
	}
	switch (routeDir[j - 1]) {	//ゴール処理
	case R:
		switch (routeDir[j - 2]) {
		case R:
			if (routeDir[j - 3] == F) {	//180度
				pass[i - 1] = TurnR_180;
			} else {
				pass[i - 1] = TurnR_135_O;
			}
			break;
		case L:
			pass[i - 1] = TurnR_45_O;
			break;
		case F:
			pass[i - 1] = TurnR_90;
			break;
		default:
			break;
		}
		break;
	case L:
		switch (routeDir[j - 2]) {
		case L:
			if (routeDir[j - 3] == F) {	//180度
				pass[i - 1] = TurnL_180;
			} else {
				pass[i - 1] = TurnL_135_O;
			}
			break;
		case R:
			pass[i - 1] = TurnL_45_O;
			break;
		case F:
			pass[i - 1] = TurnL_90;
			break;
		default:
			break;
		}
		break;
	case F:
		pass[i - 1] = Straight;
		pass[i] = Straight;
		break;
	default:
		break;
	}
//pass[i] = Straight;
}

void SprintRun(const struct parameter * _pointer, uint8_t _isDiag,
		uint8_t _suctionduty) {
	int j = 0;
	generatePass(_isDiag);
	Flag_diagMode = false;
	Flag_wallConEn_S = false;
	enable_motor();
	LED(0xFF);
	if (_isDiag) {
		enable_suction(_suctionduty);
	}
//Flag_wallConEn = true;
	/*状態量リセット*/
	idealState = RESETState;
	realState = RESETState;

	/*制御量リセット*/
	error_vel = RESET_ERROR;
	error_dis = RESET_ERROR;
	error_ang_vel = RESET_ERROR;
	error_deg = RESET_ERROR;
	error_wall_S = RESET_ERROR;
	error_wall_F_R = RESET_ERROR;
	error_wall_F_L = RESET_ERROR;
	Flag_logEn = true;
	for (int i = 0; pass[i] != End; i++) {
		realState.dis = 0;
		float min_vel = 0.0;
		if (pass[i] == Straight) {
			for (j = 1; pass[i + j] == Straight; j++)
				;
			/*ターンごとに終端速度を変えたい(願望)*/
			if (_isDiag == true) {
				switch (pass[i + j]) {
				case TurnR_90:
					min_vel = _pointer->turn90_R.Turn_vel;
					if (j > 2) {
						min_vel = _pointer->turn90_R.Turn_vel;
					}
					break;
				case TurnL_90:
					min_vel = _pointer->turn90_R.Turn_vel;
					if (j > 2) {
						min_vel = _pointer->turn90_R.Turn_vel;
					}
					break;
				case TurnR_180:
					min_vel = _pointer->turn180_R.Turn_vel;
					if (j > 2) {
						min_vel = _pointer->turn180_R.Turn_vel;
					}
					break;
				case TurnL_180:
					min_vel = _pointer->turn180_R.Turn_vel;
					if (j > 2) {
						min_vel = _pointer->turn180_R.Turn_vel;
					}
					break;
				case TurnR_45_I:
					min_vel = _pointer->turn45_I_R.Turn_vel;
					if (j > 2) {
						min_vel = _pointer->turn45_I_R.Turn_vel;
					}
					break;
				case TurnL_45_I:
					min_vel = _pointer->turn45_I_R.Turn_vel;
					if (j > 2) {
						min_vel = _pointer->turn45_I_R.Turn_vel;
					}
					break;
				case TurnR_135_I:
					min_vel = _pointer->turn135_I_R.Turn_vel;
					if (j > 2) {
						min_vel = _pointer->turn135_I_R.Turn_vel;
					}
					break;
				case TurnL_135_I:
					min_vel = _pointer->turn135_I_R.Turn_vel;
					if (j > 2) {
						min_vel = _pointer->turn135_I_R.Turn_vel;
					}
					break;
				default:
					break;
				}
			} else if (_isDiag == false) {	//小回りパス時
				min_vel = 0.5;
			}
			trapezoid_acccel(_pointer->straight_vel, min_vel,
					0.09 * (float) j - 0.02, _pointer->straight_acc, true,
					false);
			i += j - 1;
		} else if (pass[i] == Straight_D) {
			for (j = 1; pass[i + j] == Straight_D; j++)
				;
			switch (pass[i + j]) {
			case TurnR_90_D:
				min_vel = _pointer->turn90_D_R.Turn_vel;
				break;
			case TurnL_90_D:
				min_vel = _pointer->turn90_D_R.Turn_vel;
				break;
			case TurnR_45_O:
				min_vel = _pointer->turn45_O_R.Turn_vel;
				break;
			case TurnL_45_O:
				min_vel = _pointer->turn45_O_R.Turn_vel;
				break;
			case TurnR_135_O:
				min_vel = _pointer->turn135_O_R.Turn_vel;
				break;
			case TurnL_135_O:
				min_vel = _pointer->turn135_O_R.Turn_vel;
				break;
			default:
				break;
			}
			trapezoid_acccel(_pointer->straight_vel, min_vel,
					0.127 * (float) j - 0.02, _pointer->straight_acc,
					true, true);
			i += j - 1;
		} else if (pass[i] == TurnL_90) {
			if (_isDiag) {
				idealState.acc = 0;
				sprint_turn(_pointer->turn90_L.Turn_deg, Left,
						&_pointer->turn90_L, _pointer->straight_acc, true,
						false);
			} else {
				turn90_s(Left);
			}
		} else if (pass[i] == TurnR_90) {
			if (_isDiag) {
				idealState.acc = 0;
				if (i == 0) {
					if (_pointer->turn90_R.Turn_vel > 1.0) {	//開幕ターン速は1.3以下
						accel(v13d90_R.Turn_vel, 20);
//						while (Flag_failSafeEn == false && idealState.dis < 0.02)
//							;
						sprint_turn(_pointer->turn90_R.Turn_deg, Right,
								&v13d90_R, _pointer->straight_acc, false,
								false);
					} else {
						accel(_pointer->turn90_R.Turn_vel, 15);
//						while (Flag_failSafeEn == false && idealState.dis < 0.02)
//							;
						sprint_turn(_pointer->turn90_R.Turn_deg, Right,
								&_pointer->turn90_R, _pointer->straight_acc,
								false, false);
					}
				} else {
					sprint_turn(_pointer->turn90_R.Turn_deg, Right,
							&_pointer->turn90_R, _pointer->straight_acc,
							true, false);
				}
			} else {
				turn90_s(Right);
			}
		} else if (pass[i] == TurnL_180) {
			idealState.acc = 0;
			sprint_turn(_pointer->turn180_L.Turn_deg, Left,
					&_pointer->turn180_L, _pointer->straight_acc, true, false);
		} else if (pass[i] == TurnR_180) {
			idealState.acc = 0;
			sprint_turn(_pointer->turn180_R.Turn_deg, Right,
					&_pointer->turn180_R, _pointer->straight_acc, true, false);
		} else if (pass[i] == TurnL_90_D) {
			sprint_turn(_pointer->turn90_D_L.Turn_deg, Left,
					&_pointer->turn90_D_L, _pointer->straight_acc, true, true);
		} else if (pass[i] == TurnR_90_D) {
			sprint_turn(_pointer->turn90_D_R.Turn_deg, Right,
					&_pointer->turn90_D_R, _pointer->straight_acc, true, true);
		} else if (pass[i] == TurnL_135_I) {
			idealState.acc = 0;
			sprint_turn(_pointer->turn135_I_L.Turn_deg, Left,
					&_pointer->turn135_I_L, _pointer->straight_acc, true,
					true);
			Flag_diagMode = true;
		} else if (pass[i] == TurnR_135_I) {
			idealState.acc = 0;
			if (i == 0) {
				if (_pointer->turn135_I_R.Turn_vel > 1.3) {	//開幕ターン速は1.3以下
					accel(v13d135_I_R.Turn_vel, 20);
					sprint_turn(v13d135_I_R.Turn_deg, Right, &v13d135_I_R,
							_pointer->straight_acc, false, true);
				} else {
					accel(_pointer->turn135_I_R.Turn_vel, 15);
					sprint_turn(_pointer->turn135_I_R.Turn_deg, Right,
							&_pointer->turn135_I_R, _pointer->straight_acc,
							false, true);
				}
			} else {
				sprint_turn(_pointer->turn135_I_R.Turn_deg, Right,
						&_pointer->turn135_I_R, _pointer->straight_acc,
						true, true);
			}
			Flag_diagMode = true;
		} else if (pass[i] == TurnL_45_I) {
			idealState.acc = 0;
			sprint_turn(_pointer->turn45_I_L.Turn_deg, Left,
					&_pointer->turn45_I_L, _pointer->straight_acc, true, true);
			Flag_diagMode = true;
		} else if (pass[i] == TurnR_45_I) {
			idealState.acc = 0;
			if (i == 0) {
				if (_pointer->turn45_I_R.Turn_vel > 1.3) {	//開幕ターン速は1.3以下
					accel(v13d45_I_R.Turn_vel, 20);
//					while (Flag_failSafeEn == false && idealState.dis < 0.02)
//						;
					sprint_turn(v13d45_I_R.Turn_deg, Right, &v13d45_I_R,
							_pointer->straight_acc, false, true);
				} else {
					accel(_pointer->turn45_I_R.Turn_vel, 15);
//					while (Flag_failSafeEn == false && idealState.dis < 0.02)
//						;
					sprint_turn(_pointer->turn45_I_R.Turn_deg, Right,
							&_pointer->turn45_I_R, _pointer->straight_acc,
							false, true);
				}
			} else {
				sprint_turn(_pointer->turn45_I_R.Turn_deg, Right,
						&_pointer->turn45_I_R, _pointer->straight_acc, true,
						true);
			}
			Flag_diagMode = true;
		} else if (pass[i] == TurnL_135_O) {
			sprint_turn(_pointer->turn135_O_L.Turn_deg, Left,
					&_pointer->turn135_O_L, _pointer->straight_acc, true,
					false);
			Flag_diagMode = false;
		} else if (pass[i] == TurnR_135_O) {
			sprint_turn(_pointer->turn135_O_R.Turn_deg, Right,
					&_pointer->turn135_O_R, _pointer->straight_acc, true,
					false);
			Flag_diagMode = false;
		} else if (pass[i] == TurnL_45_O) {
			sprint_turn(_pointer->turn45_O_L.Turn_deg, Left,
					&_pointer->turn45_O_L, _pointer->straight_acc, true, false);
			Flag_diagMode = false;
		} else if (pass[i] == TurnR_45_O) {
			sprint_turn(_pointer->turn45_O_R.Turn_deg, Right,
					&_pointer->turn45_O_R, _pointer->straight_acc, true, false);
			Flag_diagMode = false;
		}
	}
	Flag_degConEn = true;
	if (Flag_diagMode) {
		stop(0.127, _pointer->straight_acc);
	} else {
		stop(0.18, _pointer->straight_acc);
	}
	Flag_logEn = false;
	LED(0x00);
	Flag_degConEn = false;
	reset_status();
	reset_error();
	HAL_Delay(500);
	disable_suction();
	disable_motor();

}

void straight18() {
	mapRenew(0);
	Flag_wallConEn_S = true;
	while (Flag_failSafeEn == false) {
		if (idealState.dis >= 0.18) {
			break;
		}
	}
	idealState.dis = 0.0;
	realState.dis = 0.0;
}

void turn90_s(int _dir) {
	float max_ang_vel;
	mapRenew(_dir);
	if (wallExist.Forward == true
			&& (fabs(IR_Sen.RF - (IR_Ref_single.RF + 9.0)) > 3.0
					|| fabs(IR_Sen.LF - (IR_Ref_single.LF + 9.0)) > 3.0)) {
		trapezoid_acccel(explore_turn_R.Turn_vel, 0.0, 0.09, 7.0,
		false, false);
		reset_error();
		reset_status();
		FrontWallControl();
		trapezoid_slalome(700.0, 90.0 * (float) _dir, 10000.0);
		HAL_Delay(100);
		trapezoid_acccel(explore_turn_R.Turn_vel, explore_turn_R.Turn_vel, 0.09,
				7.0,
				true, false);
	} else {
		if (_dir == Left) {
			max_ang_vel = (explore_turn_L.Turn_vel) / explore_turn_L.Turn_radius
					* 180.0 / M_PI;
			//turn(explore_turn_L.Turn_deg, _dir, &explore_turn_L, 10, false, false);
			trapezoid_acccel(explore_turn_L.Turn_vel, explore_turn_L.Turn_vel,
					explore_turn_L.Turn_offset_F, 5.0, true, false);
			trapezoid_slalome(max_ang_vel, explore_turn_L.Turn_deg,
					explore_turn_L.Turn_ang_acc);
			trapezoid_acccel(explore_turn_L.Turn_vel, explore_turn_L.Turn_vel,
					explore_turn_L.Turn_offset_B, 5.0, true, false);
		} else if (_dir == Right) {
			max_ang_vel = (explore_turn_R.Turn_vel) / explore_turn_R.Turn_radius
					* 180.0 / M_PI;
			//turn(explore_turn_R.Turn_deg, _dir, &explore_turn_R, 10, false, false);
			trapezoid_acccel(explore_turn_R.Turn_vel, explore_turn_R.Turn_vel,
					explore_turn_R.Turn_offset_F, 5.0, true, false);
			trapezoid_slalome(max_ang_vel, explore_turn_R.Turn_deg * -1.0,
					explore_turn_R.Turn_ang_acc);
			trapezoid_acccel(explore_turn_R.Turn_vel, explore_turn_R.Turn_vel,
					explore_turn_R.Turn_offset_B, 5.0, true, false);
		}
	}
}

void sprint_turn(float _deg, int8_t _dir,
		const struct TurnParameter * const _parameter, float _acc,
		uint8_t _isWallGap, uint8_t _diagTurn) {

	float max_ang_vel = (_parameter->Turn_vel) / _parameter->Turn_radius
			* 180.0/ M_PI;
	if (_isWallGap) {
		Flag_wallConEn_S = true;
		edge_break(_dir, _parameter->Turn_vel, _acc);
	} else {
		idealState.vel = _parameter->Turn_vel;
		idealState.acc = 0.0;
	}
	idealState.dis = 0.0;
	realState.dis = 0.0;
	trapezoid_acccel(idealState.vel, idealState.vel, _parameter->Turn_offset_F,
			30.0, true, Flag_diagMode);
	trapezoid_slalome(max_ang_vel, _deg * _dir, _parameter->Turn_ang_acc);
	Flag_diagMode = _diagTurn;
//	Flag_degConEn=true;
//	trapezoid_acccel(idealState.vel, idealState.vel, _parameter->Turn_offset_B,
//			30.0, true, false);
//	Flag_degConEn=false;
}

void turnU() {
	stop(0.09, 7.0);
	realState.deg = 0;
	idealState.deg = 0;
	HAL_Delay(100);
	trapezoid_slalome(700.0, 180.0, 10000.0);
	reset_status();
	HAL_Delay(100);
	if (Flag_exploreDone == false) {
		BackUpWallData(Write);
	}
	trapezoid_acccel(0.3, 0.0, -0.03, 7.0, false, false);
	accel(explore_turn_R.Turn_vel, 7.0);
	if (wallExist.Right && wallExist.Left) { //両壁がある場合
		while (!Flag_failSafeEn && idealState.dis < 0.12) {
			if (fabs(IR_Sen_dif_1.RS) > 0.1) {
				Speaker_Scale(Sc_Bh, 100);
				idealState.dis = 0.0;
				realState.dis = 0.0;
				trapezoid_acccel(explore_turn_R.Turn_vel,
						explore_turn_R.Turn_vel, 0.052, 7.0, true, false);
				break;
			}
			if (fabs(IR_Sen_dif_1.LS) > 0.1) {
				Speaker_Scale(Sc_Ch, 100);
				idealState.dis = 0.0;
				realState.dis = 0.0;
				trapezoid_acccel(explore_turn_R.Turn_vel,
						explore_turn_R.Turn_vel, 0.049, 7.0, true, false);
				break;
			}
		}
	} else if (wallExist.Right) { //右壁がある場合
		while (!Flag_failSafeEn && idealState.dis < 0.12) {
			if (fabs(IR_Sen_dif_1.RS) > 0.1) {
				Speaker_Scale(Sc_Bh, 100);
				idealState.dis = 0.0;
				realState.dis = 0.0;
				trapezoid_acccel(explore_turn_R.Turn_vel,
						explore_turn_R.Turn_vel, 0.052, 7.0, true, false);
				break;
			}
		}
	} else if (wallExist.Left) { //左壁がある場合
		while (!Flag_failSafeEn && idealState.dis < 0.12) {
			if (fabs(IR_Sen_dif_1.LS) > 0.1) {
				Speaker_Scale(Sc_Ch, 100);
				idealState.dis = 0.0;
				realState.dis = 0.0;
				trapezoid_acccel(explore_turn_R.Turn_vel,
						explore_turn_R.Turn_vel, 0.049, 7.0, true, false);
				break;
			}
		}
	} else { //壁が見えない場合
		trapezoid_acccel(explore_turn_R.Turn_vel, explore_turn_R.Turn_vel, 0.12,
				7.0,
				true, false);
	}
	mapRenew(180);
}

void FrontWallControl() {
	Flag_wallConEn_F = true;
	HAL_Delay(400);
	Flag_wallConEn_F = false;
	realState.dis = 0;
	idealState.dis = 0;
	realState.deg = 0;
	idealState.deg = 0;
	reset_error();
}

void turnU_2() {
	trapezoid_acccel(explore_turn_R.Turn_vel, 0.0, 0.09, 7.0,
	false, false);
	HAL_Delay(50);
	disable_motor();
	if (Flag_exploreDone == false) {
		BackUpWallData(Write);
	}
	reset_error();
	reset_status();
	enable_motor();
	if (wallExist.Forward == true
			&& (fabs(IR_Sen.RF - IR_Ref_single.RF) > 0.3
					|| fabs(IR_Sen.LF - IR_Ref_single.LF) > 0.3)) {
		FrontWallControl();
	}
	if (wallExist.Right == true && fabs(IR_Sen.RS - IR_Sen.LS) > 0.5) {
		trapezoid_slalome(900.0, -90.0, 10000.0);
		FrontWallControl();
		trapezoid_slalome(900.0, -90.0, 10000.0);
	} else if (wallExist.Left == true && fabs(IR_Sen.RS - IR_Sen.LS) > 0.5) {
		trapezoid_slalome(900.0, 90.0, 10000.0);
		FrontWallControl();
		trapezoid_slalome(900.0, 90.0, 10000.0);
	} else {
		trapezoid_slalome(900.0, 180.0, 10000.0);
	}
	reset_error();
	reset_status();
	HAL_Delay(100);
	trapezoid_acccel(explore_turn_R.Turn_vel, explore_turn_R.Turn_vel, 0.09,
			7.0,
			true, false);
	mapRenew(180);
}

