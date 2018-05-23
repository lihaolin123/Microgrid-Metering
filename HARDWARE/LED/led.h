#ifndef __LED_H
#define __LED_H
#include "sys.h"

//LED端口定义
#define LED0 PAout(6)	// D2
#define LED1 PAout(7)	// D3	 
void LED_Init(void);

//固态继电器端口定义
extern void SSR_Init(void);
#define SSR_ON()	GPIO_SetBits(GPIOD,GPIO_Pin_0)//输出高
#define SSR_OFF()	GPIO_ResetBits(GPIOD,GPIO_Pin_0)//输出低
#endif
