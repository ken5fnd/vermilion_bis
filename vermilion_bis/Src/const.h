/*
 * const.h
 *
 *  Created on: 2018/06/13
 *      Author: ����
 */

#ifndef CONST_H_
#define CONST_H_

#define CONTROL_CYCLE (0.0005) //�������(sec)

#define WHEEL_DIA_R (0.0231) //�E�^�C�����a(m)
#define WHEEL_DIA_L (WHEEL_DIA_R * 1.0) //���^�C�����a(m)
#define TREAD (0.063) //�^�C���ԋ���(m)

#define MASS (0.09) //�d��(kg)
#define MOMENT_R (0.1) //���񃂁[�����g
#define MOMENT_L (0.1) //���񃂁[�����g

#define SLIP_K (0.0) //�X���b�v�p�萔

#define EX_IR_SEN_NUM (15) //

#define ADC_BUFFER_LENGTH (3)

#define THR_WALLCON_RF (12.0) //�E�O�Z���T�[(RF)�̕ǐ���L��臒l

#define true (1)
#define false (0)

#define Sc_Al 440
#define Sc_Bl 494
#define Sc_Cl 523
#define Sc_Dl 587
#define Sc_El 659
#define Sc_Fl 698
#define Sc_Gl 784

#define Sc_A 880
#define Sc_B 988
#define Sc_C 1047
#define Sc_D 1175
#define Sc_E 1319
#define Sc_F 1397
#define Sc_G 1568

#define Sc_Ah 1760
#define Sc_Bh 1976
#define Sc_Ch 2093
#define Sc_Dh 2349
#define Sc_Eh 2637
#define Sc_Fh 2794
#define Sc_Gh 3136

#endif /* CONST_H_ */
