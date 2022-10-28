#ifndef __USRAT2_H
#define __USRAT2_H 
#include "sys.h"	  	

#define USART_Rx_LEN  	 200  	           //�����������ֽ��� 200
#define USART_Tx_LEN     200
#define EN_USART1_RX     1		           //ʹ�ܣ�1��/��ֹ��0������1����
#define USART3_Waiting   0
#define USART3_Receiving 1
#define USART3_Success   2 
#define USART3_Failed    3

void changePID (float *kp,float *ki,float * kd);
void carControl(void);
void pidControl(void);

void uart3_init       (u32 bound);
void USART3_IRQHandler(void);
void Uart3SendByte    (char byte);         //���ڷ���һ���ֽ�
void Uart3SendBuf     (char *buf, u16 len);
void Uart3SendStr     (char *str);
void BluetoothCMD     (int Uart_Receive);

#endif

