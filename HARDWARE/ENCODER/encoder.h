#ifndef __ENCODER_H
#define __ENCODER_H
#include "sys.h"
#include "stm32f10x.h"

extern int16_t Encoder_Count;

#define ENCODER_TIM_PERIOD (u16)(65535)   //103的定时器是16位 2的16次方最大是65536
void Encoder_Init_right(void);
void Encoder_Init_left(void);
int Read_Encoder(u8 TIMX);
#endif
