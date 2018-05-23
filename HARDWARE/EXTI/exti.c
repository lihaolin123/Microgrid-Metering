#include "exti.h"
/**
*描述:配置掉电检测
*参数:
*返回:
**/
void PWR_PVD_Init(void) 
{ 
	EXTI_InitTypeDef EXTI_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);//使能PWR时钟
	
	EXTI_StructInit(&EXTI_InitStructure);
	EXTI_InitStructure.EXTI_Line = EXTI_Line16;             //PVD连接到中断线16上
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;     //使用中断模式
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;  //电压低于阀值时产生中断
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;               //使能中断线
	EXTI_Init(&EXTI_InitStructure);                         //初始

	PWR_PVDLevelConfig(PWR_PVDLevel_7);//设定监控阀值 
	PWR_PVDCmd(ENABLE);//使能PVD     
}


