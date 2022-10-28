#ifndef __ENCODER_H
#define __ENCODER_H
#include "sys.h"
#include "stm32f10x.h"

extern int16_t Encoder_Count;

#define ENCODER_TIM_PERIOD (u16)(65535)   //103�Ķ�ʱ����16λ 2��16�η������65536
void Encoder_Init_right(void);
void Encoder_Init_left(void);
int Read_Encoder(u8 TIMX);
#endif
