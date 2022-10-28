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
float pitch,roll,yaw,pitch1; 								  			 //ŷ����(��̬��)
short aacx,aacy,aacz;											 //���ٶȴ�����ԭʼ����
short gyrox,gyroy,gyroz;
int Uart_Receive=0;
int encoder_right= 0;
int encoder_left= 0;

 int main(void)
 {	  
	LED_Init();		  	        //��ʼ����LED���ӵ�Ӳ���ӿ�
	KEY_Init();                 //��ʼ���밴�����ӵ�IO
	delay_init();	            //��ʱ������ʼ��	
	uart1_init(115200);	        //����1��ʼ��
	uart3_init(9600);
	delay_ms(100);
	NVIC_Configuration();		//=====�ж����ȼ�����,���а��������е��ж����ȼ�������,��������һ�����޸ġ�
	Encoder_Init_right();        //=====��ʼ��������2
	Encoder_Init_left();
	OLED_Init();                //=====OLED��ʼ��
	OLED_Clear();				//=====OLED����
	MPU_Init();					//=====��ʼ��MPU6050
	mpu_dmp_init();				//=====��ʼ��MPU6050��DMPģʽ
    Timer_Init();	 
	TIM3_PWM_Init(7199,10);   	//=====��ʼ��PWM 100KHZ,�������������
    delay_ms(1000);				//=====��ʱ1s���С���ϵ�������ת������	 
	 
	while(1)
	{
//		Set_Pwm(3000,3000);
	}
}

