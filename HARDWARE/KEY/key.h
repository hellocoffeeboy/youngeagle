#ifndef __KEY_H
#define __KEY_H	 
#include "sys.h"

#define KEY PCin(5)

void KEY_Init(void);          //按键初始化
int KEY_Scan(u8 mode);        //按键扫描

#endif  
