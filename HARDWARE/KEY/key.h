#ifndef __KEY_H
#define __KEY_H	 
#include "sys.h"

#define KEY PCin(5)

void KEY_Init(void);          //������ʼ��
int KEY_Scan(u8 mode);        //����ɨ��

#endif  
