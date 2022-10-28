#include "key.h"
#include "delay.h"

/**************************************************************************
�������ܣ�������ʼ��
��ڲ�������
����  ֵ���� 
**************************************************************************/
void KEY_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE); //ʹ��PA�˿�ʱ��
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;	           //�˿�����
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;         //��������
  GPIO_Init(GPIOC, &GPIO_InitStructure);					      //�����趨������ʼ��GPIOA 
} 
/**************************************************************************
�������ܣ�����ɨ��
��ڲ�����mode:0,��֧��������;1,֧��������;
����  ֵ������״̬ 0���޶��� 1������ 
**************************************************************************/
int KEY_Scan(u8 mode)
{
	static u8 key_up = 1; //�����ɿ���־
	if (mode) key_up = 1; //֧������
	if (key_up && GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_5) == 0)
	{
		delay_ms(10);
		key_up = 0;
		return 1;
	} else if(GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_5) == 1){
		key_up = 1;
	}
	return 0;
}
