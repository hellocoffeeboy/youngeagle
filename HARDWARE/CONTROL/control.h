#ifndef __CONTROL_H
#define __CONTROL_H

#include "stm32f10x.h"
#include "sys.h"


#define PI 3.14159265
void EXTI9_5_IRQHandler(void);
int balance_UP(float Angle , float Mechanical_balance, float Gyro);
int velocity(int encoder_left, int encoder_right, int target);
int turn(int encoder_left,int encoder_right,float gyro,int target);

extern float balance_UP_KP; 	 // 小车直立环PD参数
extern float balance_UP_KI;
extern float balance_UP_KD;

extern float velocity_KP;        // 小车速度环PI参数
extern float velocity_KI;
extern float velocity_KD;

extern float turnKP;
extern float turnKI;
extern float turnKD;

void Timer_Init(void);
void TIM2_IRQHandler(void);

#endif
