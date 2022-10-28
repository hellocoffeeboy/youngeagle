#include "usart2.h"
#include "stdio.h"
#include "string.h"
#include "control.h"


u8 Mode = 98;
u8 USART3_state = USART3_Waiting;  //����״̬���
u8 USART3_Rx_index = 0;            //����2��������λ���������±�
u8 USART3_Tx_index = 0;            //����2��������λ���������±�

int controlFlag = 0;               //����pid�Ϳ���С���ı�־λ
int rxLength = 17;                 //���ݰ��ܳ���
          
unsigned char Usart3Rx_Info[USART_Rx_LEN];   //��Ž������ݵ�����
unsigned char Usart3Tx_Info[USART_Tx_LEN];   //��ŷ������ݵ�����
 


/**************************************************************************
�������ܣ�����2��ʼ��
��ڲ�����bound:������
����  ֵ����
**************************************************************************/
void uart3_init(u32 bound)
{  	 
	//GPIO�˿�����
    GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);	 //ʹ��UGPIOBʱ��
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE); 	 //ʹ��USART3ʱ��
	//USAR3_TX  
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;                //PB10
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	         //�����������
    GPIO_Init(GPIOB, &GPIO_InitStructure);
   
    //USART3_RX	  
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;                //PB11
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;    //��������
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    //USART ��ʼ������
	USART_InitStructure.USART_BaudRate = bound;                                       //���ڲ�����
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;						  //�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;                            //һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;                               //����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;   //��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	                  //�շ�ģʽ
    USART_Init(USART3, &USART_InitStructure);                                         //��ʼ������2
    USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);                                    //�������ڽ����ж�
    USART_Cmd(USART3, ENABLE);                                                        //ʹ�ܴ���2 
}



/**************************************************************************
�������ܣ�����3�����ж�
��ڲ�������
����  ֵ����
**************************************************************************/


void USART3_IRQHandler(void)                	           //����2�жϷ������
{
	if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)  //�����ж�
	{
		u8 Res;
	    Res = USART_ReceiveData(USART3);     	           //��ȡ���յ�������
		switch (Res)
		{
			case 'c':controlFlag = 0; break;
			case 'z':controlFlag = 1; break; //ֱ����
			case 's':controlFlag = 2; break; //�ٶȻ�
			case 'x':controlFlag = 3; break; //ת��
			default:  break;
		}
		if (controlFlag == 0)  carControl();               //С������ģʽ
		else  pidControl();          //��pidģʽ
	}
} 


/**************************************************************************
�������ܣ��������ݰ���У��
��ڲ�������
����  ֵ����
**************************************************************************/

void pidControl() 
{
	    u8 Res;
	    Res =USART_ReceiveData(USART3);     	                   //��ȡ���յ�������
	    printf("in pid\r\n");
		if ((USART3_state == USART3_Waiting) && (Res == 'A'))      //�ж�֡ͷ
		{
			USART3_state = USART3_Receiving;                       //֡ͷ��ȷ�͸ı䴮��2��״̬
			USART3_Rx_index = 0;                                   //��������ļ�����
			Usart3Rx_Info[USART3_Rx_index] = Res;                  //����pid���ݵĴ洢
            USART3_Rx_index++;                       
		}
		else if (USART3_state == USART3_Receiving)
		{
			if (USART3_Rx_index < rxLength)
			{
				Usart3Rx_Info[USART3_Rx_index]=Res;
				USART3_Rx_index++;                                                          //û�н������һֱ������ѭ��
				if (USART3_Rx_index == rxLength && Usart3Rx_Info[USART3_Rx_index-1] == 'B') //������ɣ��ж�֡β�����֡β��ȷ���޸�pid
                {   
					switch (controlFlag)
						{
							case 1:changePID(&balance_UP_KP,&balance_UP_KI,&balance_UP_KD); break; //ֱ����
							case 2:changePID(&velocity_KP,&velocity_KI,&velocity_KD); break; //�ٶȻ�
							case 3:changePID(&turnKP,&turnKI,&turnKD); break; //ת��
							default: break;
						}

                   USART3_Rx_index = 0;                                                     //���ݳ�����0
                   USART3_state = USART3_Waiting;                                           //����2��״̬�ص��ȴ�״̬
                }
				if (USART3_Rx_index == rxLength && Usart3Rx_Info[USART3_Rx_index-1] != 'B') //������ɣ��ж�֡β�������������½���
                {      
				   printf("PLEASE SET AGAIN");          
                   USART3_Rx_index = 0;                                                     //���ݳ�����0
                   USART3_state = USART3_Waiting;                                           //����2��״̬�ص��ȴ�״̬
                }
			}
		  
		} 
		USART_ClearFlag(USART3,USART_IT_RXNE);                                              //�����־λ
}

/**************************************************************************
�������ܣ�����С���˶�
��ڲ�������
����  ֵ����
**************************************************************************/

void carControl()
{
	u8 Res;
	Res =USART_ReceiveData(USART3);     	                   //��ȡ���յ�������
	printf("in car\r\n");
	switch(Res)
	{
		case '1': VelocityTarget = -30; Turn =   0;   break;   //ǰ��
		case '2': VelocityTarget =   0; Turn = -40;   break;   //����
		case '3': VelocityTarget =   0; Turn =   0;   break;   //ֹͣ
		case '4': VelocityTarget =   0; Turn =  40;   break;   //����
		case '5': VelocityTarget =  30; Turn =   0;   break;   //����
		case '6': VelocityTarget +=  5; Turn =   0;   break;   //����
		case '7': VelocityTarget -=  5; Turn =   0;   break;   //����
		default : break;
	}
}	

/**************************************************************************
�������ܣ��޸�pid����
��ڲ�����*kp:Ҫ�޸ĵ�kpֵ��ָ��,*kp:Ҫ�޸ĵ�kpֵ��ָ��,*kp:Ҫ�޸ĵ�kpֵ��ָ��
����  ֵ����
**************************************************************************/


void changePID(float *kp,float *ki,float *kd)
{
	int i = 1;
	printf("in pid\r\n");                                          //����4��Ϊ�����������ת��Ϊ4λ������1λС��
	*kp=0,*ki=0,*kd=0;
	for (;i<=5;)  if (i == 5)  {*kp = *kp + (Usart3Rx_Info[i++] - 48) * 1.0 / 10;} else {*kp = *kp * 10 + (Usart3Rx_Info[i++] - 48);}
	for (;i<=10;) if (i == 10) {*ki = *ki + (Usart3Rx_Info[i++] - 48) * 1.0 / 10;} else {*ki = *ki * 10 + (Usart3Rx_Info[i++] - 48);}
	for (;i<=15;) if (i == 15) {*kd = *kd + (Usart3Rx_Info[i++] - 48) * 1.0 / 10;} else {*kd = *kd * 10 + (Usart3Rx_Info[i++] - 48);}
	velocity_KP=-velocity_KP;                                  //��������Լ�pid�����ļ���������
	velocity_KI=-velocity_KI;                                  //�����ҵ�pid����ȫ�Ǹ��ģ����Զ�����һ������
	velocity_KD=-velocity_KD;
	printf("%f,%f,%f",*kp,*ki,*kd);  //���ڲ鿴�ǲ���Э����ȷ
}

/**************************************************************************
�������ܣ�����3��������
��ڲ�����byte���ֽ�
����  ֵ����
**************************************************************************/

void Uart3SendByte(char byte)                                    //���ڷ���һ���ֽ�
{
		USART_SendData(USART3, byte);                            //ͨ���⺯��  ��������
		while( USART_GetFlagStatus(USART3,USART_FLAG_TC)!= SET);  
		//�ȴ�������ɡ�   ��� USART_FLAG_TC �Ƿ���1��          //���⺯�� P359 ����
}

/**************************************************************************
�������ܣ�����3��������
��ڲ�����*buf���ַ����ĵ�ַ��len���ַ����ĳ���
����  ֵ����
**************************************************************************/

void Uart3SendBuf(char *buf, u16 len)
{
	u16 i;
	for(i=0; i<len; i++)Uart3SendByte(*buf++);
}

/**************************************************************************
�������ܣ�����2��������
��ڲ�����byte���ַ���
����  ֵ����
**************************************************************************/

void Uart3SendStr(char *str)
{
	u16 i,len;
	len = strlen(str);
	for(i=0; i<len; i++)Uart3SendByte(*str++);
}
