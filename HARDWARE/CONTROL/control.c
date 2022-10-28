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

//float balance_UP_KP=220; 	 // 小车直立环PD参数,这是调好的
//float balance_UP_KI=0;
//float balance_UP_KD=2.4;

//float velocity_KP=-80;        // 小车速度环PI参数
//float velocity_KI=-0.425;
//float velocity_KD=0;

//float turnKP=15;
//float turnKI=0;
//float turnKD=0;

int Balance_Pwm,Velocity_Pwm,Turn_Pwm;

float Mechanical_angle=0; 

float balance_UP_KP=0; 	 // 小车直立环PD参数
float balance_UP_KI=0;
float balance_UP_KD=0;

float velocity_KP=0;        // 小车速度环PI参数
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
函数功能：所有的控制代码都在这里面
         5ms定时中断由MPU6050的INT引脚触发
         严格保证采样和数据处理的时间同步	
		 在MPU6050的采样频率设置中，设置成100HZ，即可保证6050的数据是10ms更新一次。
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

        Turn_Pwm =turn(Encoder_Left,Encoder_Right,gyroz,Turn);        //===转向环PID控制	
		
		Moto1=Balance_Pwm+Velocity_Pwm-Turn_Pwm;                 //===计算左轮电机最终PWM
		Moto2=Balance_Pwm+Velocity_Pwm+Turn_Pwm;                 //===计算右轮电机最终PWM
		
		Xianfu_Pwm();  											 //===PWM限幅
		Turn_Off(pitch,12);										 //===检查角度以及电压是否正常
		Set_Pwm(Moto1,Moto2);                                    //===赋值给PWM寄存器  
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
	}
}

/**************************************************************************
函数功能：直立PD控制
入口参数：角度、机械平衡角度（机械中值）、角速度
返回  值：直立控制PWM
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
函数功能：速度PI控制
入口参数：电机编码器的值
返回  值：速度控制PWM
**************************************************************************/

int velocity(int encoder_left, int encoder_right, int target)
{
	static float Velocity, ErrorLast, Error, ErrorLowOut;
	static float ErrorIntegral;
	float a = 0.8;
	//=============速度PI控制器================//
	Error = (encoder_left + encoder_right) - target;
	
	ErrorLowOut = (1-a)*Error + a*ErrorLast; // 使得波形更加平滑，滤除高频干扰，放置速度突变
    ErrorLast = ErrorLowOut;   // 防止速度过大影响直立环的正常工作

	ErrorIntegral += ErrorLowOut;
	
	ErrorIntegral=ErrorIntegral>10000?10000:(ErrorIntegral<(-10000)?(-10000):ErrorIntegral);
	
	Velocity = ErrorLowOut * velocity_KP + ErrorIntegral * velocity_KI;
	
	if (pitch > 40 || pitch < -40)   ErrorIntegral = 0;
	
	return Velocity;
}

/**************************************************************************
函数功能：转向PD控制
入口参数：电机编码器的值、Z轴角速度
返回  值：转向控制PWM
**************************************************************************/
int turn(int encoder_left,int encoder_right,float gyro,int target)//转向控制
{
	 static float Turn_Target,Encoder_temp,Turn_Convert=0.9,Turn_Count;
	  float Turn_Amplitude=66;     
	  //=============遥控左右旋转部分=======================//
	  //这一部分主要是根据旋转前的速度调整速度的起始速度，增加小车的适应性
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
    if(Turn_Target>Turn_Amplitude)  Turn_Target=Turn_Amplitude;    //===转向	速度限幅
	  if(Turn_Target<-Turn_Amplitude) Turn_Target=-Turn_Amplitude;
		if(target!=0)  turnKD=0;        
		else turnKD=0.5;   //转向的时候取消陀螺仪的纠正 有点模糊PID的思想
  	//=============转向PD控制器=======================//
		return -Turn_Target*turnKP-gyro*turnKD;                 //===结合Z轴陀螺仪进行PD控制
}
