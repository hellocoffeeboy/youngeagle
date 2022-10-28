#include "key.h"
#include "delay.h"

/**************************************************************************
函数功能：按键初始化
入口参数：无
返回  值：无 
**************************************************************************/
void KEY_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE); //使能PA端口时钟
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;	           //端口配置
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;         //上拉输入
  GPIO_Init(GPIOC, &GPIO_InitStructure);					      //根据设定参数初始化GPIOA 
} 
/**************************************************************************
函数功能：按键扫描
入口参数：mode:0,不支持连续按;1,支持连续按;
返回  值：按键状态 0：无动作 1：单击 
**************************************************************************/
int KEY_Scan(u8 mode)
{
	static u8 key_up = 1; //按键松开标志
	if (mode) key_up = 1; //支持连按
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
