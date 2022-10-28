#ifndef __USRAT2_H
#define __USRAT2_H 
#include "sys.h"	  	

#define USART_Rx_LEN  	 200  	           //定义最大接收字节数 200
#define USART_Tx_LEN     200
#define EN_USART1_RX     1		           //使能（1）/禁止（0）串口1接收
#define USART3_Waiting   0
#define USART3_Receiving 1
#define USART3_Success   2 
#define USART3_Failed    3

void changePID (float *kp,float *ki,float * kd);
void carControl(void);
void pidControl(void);

void uart3_init       (u32 bound);
void USART3_IRQHandler(void);
void Uart3SendByte    (char byte);         //串口发送一个字节
void Uart3SendBuf     (char *buf, u16 len);
void Uart3SendStr     (char *str);
void BluetoothCMD     (int Uart_Receive);

#endif

