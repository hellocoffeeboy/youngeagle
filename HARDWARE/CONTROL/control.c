#include "control.h"
#include "mpu6050.h"
#include "inv_mpu.h"
#include "encoder.h"
#include "motor.h"
#include "usart.h"
#include "filter.h"
#include "sys.h"
#include "led.h"
#include "oled.h"

//float balance_UP_KP=220; 	 // С��ֱ����PD����,���ǵ��õ�
//float balance_UP_KI=0;
//float balance_UP_KD=2.4;

//float velocity_KP=-80;        // С���ٶȻ�PI����
//float velocity_KI=-0.425;
//float velocity_KD=0;

//float turnKP=15;
//float turnKI=0;
//float turnKD=0;

int Balance_Pwm,Velocity_Pwm,Turn_Pwm;

float Mechanical_angle=0; 

float balance_UP_KP=0; 	 // С��ֱ����PD����
float balance_UP_KI=0;
float balance_UP_KD=0;

float velocity_KP=0;        // С���ٶȻ�PI����
float velocity_KI=0;
float velocity_KD=0;

float turnKP=0;
float turnKI=0;  
float turnKD=0;


void Timer_Init(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	
	TIM_InternalClockConfig(TIM2);
	
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInitStructure.TIM_Period = 7200 - 1;
	TIM_TimeBaseInitStructure.TIM_Prescaler = 50 - 1;
	TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseInitStructure);
	
	TIM_ClearFlag(TIM2, TIM_FLAG_Update);
	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_Init(&NVIC_InitStructure);
	
	TIM_Cmd(TIM2, ENABLE);
}


/**************************************************************************
�������ܣ����еĿ��ƴ��붼��������
         5ms��ʱ�ж���MPU6050��INT���Ŵ���
         �ϸ�֤���������ݴ����ʱ��ͬ��	
		 ��MPU6050�Ĳ���Ƶ�������У����ó�100HZ�����ɱ�֤6050��������10ms����һ�Ρ�
**************************************************************************/
int kk = 23443;

void oled_show_data(void)
{
	OLED_ShowString(1,1,(u8*)"left:",16);
	OLED_ShowString(1,3,(u8*)"right:",16);
	OLED_ShowNumber(64,1,Encoder_Right,5,16);
	OLED_ShowNumber(64,3,Encoder_Left,5,16);
	if (pitch > 0)
	{
		OLED_ShowString(1,5,(u8*)"pitch:",16);
		OLED_ShowChar(75,5,'+',16);
		OLED_ShowNumber(80,5,pitch,4,16);
	}
	else 
	{
		OLED_ShowString(1,5,(u8*)"pitch:",16);
		OLED_ShowChar(75,5,'-',16);
		OLED_ShowNumber(80,5,-pitch,4,16);
	}
}


void TIM2_IRQHandler(void)
{
	if (TIM_GetITStatus(TIM2, TIM_IT_Update) == SET)
	{
		mpu_dmp_get_data(&pitch,&roll,&yaw);
		MPU_Get_Gyroscope(&gyrox,&gyroy,&gyroz);
//		pitch1 = MPU_Data_Get_Filter();
//		printf("%f,%f,%f\n", Accel_Angle,pitch1,roll);
		Encoder_Left = Read_Encoder(3);
		Encoder_Right = Read_Encoder(4);
		
		oled_show_data();
		
		Balance_Pwm = balance_UP(pitch,Mechanical_angle,gyroy);
		Velocity_Pwm = velocity(Encoder_Left,Encoder_Right,VelocityTarget);

        Turn_Pwm =turn(Encoder_Left,Encoder_Right,gyroz,Turn);        //===ת��PID����	
		
		Moto1=Balance_Pwm+Velocity_Pwm-Turn_Pwm;                 //===�������ֵ������PWM
		Moto2=Balance_Pwm+Velocity_Pwm+Turn_Pwm;                 //===�������ֵ������PWM
		
		Xianfu_Pwm();  											 //===PWM�޷�
		Turn_Off(pitch,12);										 //===���Ƕ��Լ���ѹ�Ƿ�����
		Set_Pwm(Moto1,Moto2);                                    //===��ֵ��PWM�Ĵ���  
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
	}
}

/**************************************************************************
�������ܣ�ֱ��PD����
��ڲ������Ƕȡ���еƽ��Ƕȣ���е��ֵ�������ٶ�
����  ֵ��ֱ������PWM
**************************************************************************/

int balance_UP(float Angle , float Mechanical_balance, float Gyro)
{
	float Bias;
	int balance;
	Bias = Angle - Mechanical_balance;
	balance = balance_UP_KP * Bias + balance_UP_KD * Gyro;
	return balance;
}

int PwmMax = 3000;

float Pid(float current , float target, float kp, float ki, float kd)
{
	static float bias = 0,baseLast = 0, baseLastLast = 0;
	static int PwmOut = 0;
	static int iError = 0, dError = 0;
	
	bias = target - current;
	
	iError += bias;
	
	dError = baseLast - baseLastLast;
	
	baseLastLast = baseLast;
	baseLast = bias;
	
	iError = iError > PwmMax ? PwmMax : (iError < -PwmMax ? -PwmMax : iError);
	
	PwmOut = kp * bias + ki * iError + kd * dError;
	
	return PwmOut;
}


/**************************************************************************
�������ܣ��ٶ�PI����
��ڲ����������������ֵ
����  ֵ���ٶȿ���PWM
**************************************************************************/

int velocity(int encoder_left, int encoder_right, int target)
{
	static float Velocity, ErrorLast, Error, ErrorLowOut;
	static float ErrorIntegral;
	float a = 0.8;
	//=============�ٶ�PI������================//
	Error = (encoder_left + encoder_right) - target;
	
	ErrorLowOut = (1-a)*Error + a*ErrorLast; // ʹ�ò��θ���ƽ�����˳���Ƶ���ţ������ٶ�ͻ��
    ErrorLast = ErrorLowOut;   // ��ֹ�ٶȹ���Ӱ��ֱ��������������

	ErrorIntegral += ErrorLowOut;
	
	ErrorIntegral=ErrorIntegral>10000?10000:(ErrorIntegral<(-10000)?(-10000):ErrorIntegral);
	
	Velocity = ErrorLowOut * velocity_KP + ErrorIntegral * velocity_KI;
	
	if (pitch > 40 || pitch < -40)   ErrorIntegral = 0;
	
	return Velocity;
}

/**************************************************************************
�������ܣ�ת��PD����
��ڲ����������������ֵ��Z����ٶ�
����  ֵ��ת�����PWM
**************************************************************************/
int turn(int encoder_left,int encoder_right,float gyro,int target)//ת�����
{
	 static float Turn_Target,Encoder_temp,Turn_Convert=0.9,Turn_Count;
	  float Turn_Amplitude=66;     
	  //=============ң��������ת����=======================//
	  //��һ������Ҫ�Ǹ�����תǰ���ٶȵ����ٶȵ���ʼ�ٶȣ�����С������Ӧ��
  	if(target!=0)
		{
			if(++Turn_Count==1)
			Encoder_temp=myabs(encoder_left+encoder_right);      
			Turn_Convert=55/Encoder_temp;
			if(Turn_Convert<0.6)Turn_Convert=0.6;
			if(Turn_Convert>3)Turn_Convert=3;
		}	
	  else
		{
			Turn_Convert=0.9;
			Turn_Count=0;
			Encoder_temp=0;
		}			
		if(target>0)	         Turn_Target+=Turn_Convert;
		else if(target<0)	     Turn_Target-=Turn_Convert; 
		else Turn_Target=0;
    if(Turn_Target>Turn_Amplitude)  Turn_Target=Turn_Amplitude;    //===ת��	�ٶ��޷�
	  if(Turn_Target<-Turn_Amplitude) Turn_Target=-Turn_Amplitude;
		if(target!=0)  turnKD=0;        
		else turnKD=0.5;   //ת���ʱ��ȡ�������ǵľ��� �е�ģ��PID��˼��
  	//=============ת��PD������=======================//
		return -Turn_Target*turnKP-gyro*turnKD;                 //===���Z�������ǽ���PD����
}
