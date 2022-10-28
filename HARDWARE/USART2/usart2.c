#include "usart2.h"
#include "stdio.h"
#include "string.h"
#include "control.h"


u8 Mode = 98;
u8 USART3_state = USART3_Waiting;  //接收状态标记
u8 USART3_Rx_index = 0;            //串口2接收数据位数的数组下标
u8 USART3_Tx_index = 0;            //串口2发送数据位数的数组下标

int controlFlag = 0;               //设置pid和控制小车的标志位
int rxLength = 17;                 //数据包总长度
          
unsigned char Usart3Rx_Info[USART_Rx_LEN];   //存放接收数据的数组
unsigned char Usart3Tx_Info[USART_Tx_LEN];   //存放发送数据的数组
 


/**************************************************************************
函数功能：串口2初始化
入口参数：bound:波特率
返回  值：无
**************************************************************************/
void uart3_init(u32 bound)
{  	 
	//GPIO端口设置
    GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);	 //使能UGPIOB时钟
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE); 	 //使能USART3时钟
	//USAR3_TX  
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;                //PB10
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	         //复用推挽输出
    GPIO_Init(GPIOB, &GPIO_InitStructure);
   
    //USART3_RX	  
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;                //PB11
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;    //浮空输入
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    //USART 初始化设置
	USART_InitStructure.USART_BaudRate = bound;                                       //串口波特率
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;						  //字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;                            //一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;                               //无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;   //无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	                  //收发模式
    USART_Init(USART3, &USART_InitStructure);                                         //初始化串口2
    USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);                                    //开启串口接受中断
    USART_Cmd(USART3, ENABLE);                                                        //使能串口2 
}



/**************************************************************************
函数功能：串口3接收中断
入口参数：无
返回  值：无
**************************************************************************/


void USART3_IRQHandler(void)                	           //串口2中断服务程序
{
	if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)  //接收中断
	{
		u8 Res;
	    Res = USART_ReceiveData(USART3);     	           //读取接收到的数据
		switch (Res)
		{
			case 'c':controlFlag = 0; break;
			case 'z':controlFlag = 1; break; //直立环
			case 's':controlFlag = 2; break; //速度环
			case 'x':controlFlag = 3; break; //转向环
			default:  break;
		}
		if (controlFlag == 0)  carControl();               //小车控制模式
		else  pidControl();          //调pid模式
	}
} 


/**************************************************************************
函数功能：进行数据包的校验
入口参数：无
返回  值：无
**************************************************************************/

void pidControl() 
{
	    u8 Res;
	    Res =USART_ReceiveData(USART3);     	                   //读取接收到的数据
	    printf("in pid\r\n");
		if ((USART3_state == USART3_Waiting) && (Res == 'A'))      //判断帧头
		{
			USART3_state = USART3_Receiving;                       //帧头正确就改变串口2的状态
			USART3_Rx_index = 0;                                   //接收数组的计数器
			Usart3Rx_Info[USART3_Rx_index] = Res;                  //进行pid数据的存储
            USART3_Rx_index++;                       
		}
		else if (USART3_state == USART3_Receiving)
		{
			if (USART3_Rx_index < rxLength)
			{
				Usart3Rx_Info[USART3_Rx_index]=Res;
				USART3_Rx_index++;                                                          //没有接收完就一直在这里循环
				if (USART3_Rx_index == rxLength && Usart3Rx_Info[USART3_Rx_index-1] == 'B') //接收完成，判断帧尾，如果帧尾正确就修改pid
                {   
					switch (controlFlag)
						{
							case 1:changePID(&balance_UP_KP,&balance_UP_KI,&balance_UP_KD); break; //直立环
							case 2:changePID(&velocity_KP,&velocity_KI,&velocity_KD); break; //速度环
							case 3:changePID(&turnKP,&turnKI,&turnKD); break; //转向环
							default: break;
						}

                   USART3_Rx_index = 0;                                                     //数据长度置0
                   USART3_state = USART3_Waiting;                                           //串口2的状态回到等待状态
                }
				if (USART3_Rx_index == rxLength && Usart3Rx_Info[USART3_Rx_index-1] != 'B') //接收完成，判断帧尾，如果错误就重新接收
                {      
				   printf("PLEASE SET AGAIN");          
                   USART3_Rx_index = 0;                                                     //数据长度置0
                   USART3_state = USART3_Waiting;                                           //串口2的状态回到等待状态
                }
			}
		  
		} 
		USART_ClearFlag(USART3,USART_IT_RXNE);                                              //清除标志位
}

/**************************************************************************
函数功能：控制小车运动
入口参数：无
返回  值：无
**************************************************************************/

void carControl()
{
	u8 Res;
	Res =USART_ReceiveData(USART3);     	                   //读取接收到的数据
	printf("in car\r\n");
	switch(Res)
	{
		case '1': VelocityTarget = -30; Turn =   0;   break;   //前进
		case '2': VelocityTarget =   0; Turn = -40;   break;   //往左
		case '3': VelocityTarget =   0; Turn =   0;   break;   //停止
		case '4': VelocityTarget =   0; Turn =  40;   break;   //往右
		case '5': VelocityTarget =  30; Turn =   0;   break;   //后退
		case '6': VelocityTarget +=  5; Turn =   0;   break;   //加速
		case '7': VelocityTarget -=  5; Turn =   0;   break;   //减速
		default : break;
	}
}	

/**************************************************************************
函数功能：修改pid参数
入口参数：*kp:要修改的kp值的指针,*kp:要修改的kp值的指针,*kp:要修改的kp值的指针
返回  值：无
**************************************************************************/


void changePID(float *kp,float *ki,float *kd)
{
	int i = 1;
	printf("in pid\r\n");                                          //下面4行为将输入的数据转换为4位整数和1位小数
	*kp=0,*ki=0,*kd=0;
	for (;i<=5;)  if (i == 5)  {*kp = *kp + (Usart3Rx_Info[i++] - 48) * 1.0 / 10;} else {*kp = *kp * 10 + (Usart3Rx_Info[i++] - 48);}
	for (;i<=10;) if (i == 10) {*ki = *ki + (Usart3Rx_Info[i++] - 48) * 1.0 / 10;} else {*ki = *ki * 10 + (Usart3Rx_Info[i++] - 48);}
	for (;i<=15;) if (i == 15) {*kd = *kd + (Usart3Rx_Info[i++] - 48) * 1.0 / 10;} else {*kd = *kd * 10 + (Usart3Rx_Info[i++] - 48);}
	velocity_KP=-velocity_KP;                                  //这里根据自己pid参数的极性来设置
	velocity_KI=-velocity_KI;                                  //这里我的pid参数全是负的，所以都带了一个负号
	velocity_KD=-velocity_KD;
	printf("%f,%f,%f",*kp,*ki,*kd);  //用于查看是不是协议正确
}

/**************************************************************************
函数功能：串口3发送数据
入口参数：byte：字节
返回  值：无
**************************************************************************/

void Uart3SendByte(char byte)                                    //串口发送一个字节
{
		USART_SendData(USART3, byte);                            //通过库函数  发送数据
		while( USART_GetFlagStatus(USART3,USART_FLAG_TC)!= SET);  
		//等待发送完成。   检测 USART_FLAG_TC 是否置1；          //见库函数 P359 介绍
}

/**************************************************************************
函数功能：串口3发送数据
入口参数：*buf：字符串的地址，len：字符串的长度
返回  值：无
**************************************************************************/

void Uart3SendBuf(char *buf, u16 len)
{
	u16 i;
	for(i=0; i<len; i++)Uart3SendByte(*buf++);
}

/**************************************************************************
函数功能：串口2发送数据
入口参数：byte：字符串
返回  值：无
**************************************************************************/

void Uart3SendStr(char *str)
{
	u16 i,len;
	len = strlen(str);
	for(i=0; i<len; i++)Uart3SendByte(*str++);
}
