#ifndef __MOTOR_H
#define __MOTOR_H
#include <sys.h>	



#define PWMA1   TIM3->CCR1  //PC6
#define PWMA2   TIM3->CCR2  //PC7
#define PWMB1   TIM3->CCR3  //PC8
#define PWMB2   TIM3->CCR4  //PC9


void Motor_Init(void);
void Set_Pwm(int moto1,int moto2);
int myabs(int a);
void Xianfu_Pwm(void);
void Turn_Off(float angle, float voltage);
#endif
