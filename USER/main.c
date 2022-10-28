#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "led.h"
#include "key.h"
#include "usart2.h"
#include "adc.h"
#include "oled.h"
#include "motor.h"
#include "encoder.h"
#include "math.h"
#include "exti.h"
#include "pwm.h"
#include "mpu6050.h"
#include "mpuiic.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "control.h"

float Voltage; 
int VelocityTarget=0,Turn=0;
int Moto1=0,Moto2=0;
int Encoder_Left=0,Encoder_Right=0;
float t = 0;
float pitch,roll,yaw,pitch1; 								  			 //欧拉角(姿态角)
short aacx,aacy,aacz;											 //加速度传感器原始数据
short gyrox,gyroy,gyroz;
int Uart_Receive=0;
int encoder_right= 0;
int encoder_left= 0;

 int main(void)
 {	  
	LED_Init();		  	        //初始化与LED连接的硬件接口
	KEY_Init();                 //初始化与按键连接的IO
	delay_init();	            //延时函数初始化	
	uart1_init(115200);	        //串口1初始化
	uart3_init(9600);
	delay_ms(100);
	NVIC_Configuration();		//=====中断优先级分组,其中包含了所有的中断优先级的配置,方便管理和一次性修改。
	Encoder_Init_right();        //=====初始化编码器2
	Encoder_Init_left();
	OLED_Init();                //=====OLED初始化
	OLED_Clear();				//=====OLED清屏
	MPU_Init();					//=====初始化MPU6050
	mpu_dmp_init();				//=====初始化MPU6050的DMP模式
    Timer_Init();	 
	TIM3_PWM_Init(7199,10);   	//=====初始化PWM 100KHZ,用于驱动电机。
    delay_ms(1000);				//=====延时1s解决小车上电轮子乱转的问题	 
	 
	while(1)
	{
//		Set_Pwm(3000,3000);
	}
}

