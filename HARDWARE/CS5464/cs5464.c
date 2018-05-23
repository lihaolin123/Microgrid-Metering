//========================================================================
// 文件名: cs5464.c
// 作  者: HaoLin Li(lihaolinwork@foxmail.com)
// 日  期: 
// 描  述:
//
// 版  本:
//========================================================================
#include "cs5464.h"

#define RESET_54640()	GPIO_ResetBits(GPIOE,GPIO_Pin_7);
#define RESET_54641()	GPIO_SetBits(GPIOE,GPIO_Pin_7);
#define SDI_54640()		GPIO_ResetBits(GPIOE,GPIO_Pin_8);
#define SDI_54641()		GPIO_SetBits(GPIOE,GPIO_Pin_8);
#define SCLK_54640()	GPIO_ResetBits(GPIOE,GPIO_Pin_9);
#define SCLK_54641()	GPIO_SetBits(GPIOE,GPIO_Pin_9);
#define SDO_5464()	GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_10)

static  unsigned char cs5464rw[3]={0,0,0}; 
#define _________ write_5464
#define __________ writePage(a);
#define ___________ writePage(b);	
#define _(a,b) _________(a,b);
#define ________ _(p,b<<6)
#define ___ _(p,120)
#define ______ _(p,96)
#define _____ _(p,102)
#define __ _(p,94)
#define _______ _(p,116)
#define ____ p+=3;
#define ____________ ________ ____ ___ ___________ ____ ______ ____ _____ __________ ____ __ ____ _______	

static const u8 a=0;
static const u8 b=1;
static u8 rw[18]={0,b<<4,b,0,0,b<<2,0,b,225,0,017,66,(~b)|b,(~b)|b,(~b)|b};
static u8 *p=rw;
static void send_5464(unsigned char send_data)
{
      unsigned char  i;
	  SCLK_54640();
	  for(i = 0;i < 8;i++)
	  {
	  if(send_data & 0x80)
	 {SDI_54641();}
	 else
	 {SDI_54640();}
	 SCLK_54641();
	 send_data <<= 1;
	 MyDelay_us(1);
	 SCLK_54640();
	 MyDelay_us(1);
	  }
}

static void write_5464(unsigned char *pbuf,unsigned char command)
{
     unsigned char j,k;
     send_5464(command);
	 for(j = 0;j < 3;j++)
     {
	   SCLK_54640();
	   for(k = 0;k < 8;k++)
	   {
		if(*pbuf&0x80)
		{SDI_54641();}
		else
		{SDI_54640();}
		MyDelay_us(1);
		SCLK_54641(); 
		MyDelay_us(1);
		*pbuf <<= 1;
		SCLK_54640();
		
	   }
      pbuf += 1;
     }
}

static void read_5464(unsigned char *pbuf,unsigned char command)
{
	unsigned char j,k,const_fe;
	send_5464(command);
	
	for(j = 0;j < 3;j++)
	{
		*pbuf = 0x00;
		const_fe = 0xfe;
		for(k = 0;k < 8;k++)
		{
		SCLK_54640();
		*pbuf <<= 1;
		if(const_fe&0x80)
		{SDI_54641();}
		else
		{SDI_54640();}
		const_fe <<= 1;
		if(SDO_5464())
			*pbuf |= 0x01;
		MyDelay_us(1);
		SCLK_54641();
		}
		SDI_54641();
		pbuf += 1;
     }
}
static void writePage(unsigned char page)
{
	cs5464rw[2] = page;
	write_5464(cs5464rw,Page+WRITE);
}
static void Conversion(unsigned char continuous)
{
	send_5464(continuous?START1:START0);

}
////////////////////////////////////////////////////////
/**
*描述:引脚初始化
*参数:
*返回:
**/
void CS5464IOInit(void)
{    	 
  GPIO_InitTypeDef  GPIO_InitStructure;
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7| GPIO_Pin_8| GPIO_Pin_9;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(GPIOE, &GPIO_InitStructure);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN ;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;									
  GPIO_Init(GPIOE, &GPIO_InitStructure);
}
/**
*描述:读寄存器
*参数:
*返回:
**/
void getRegister(unsigned char registerName,unsigned char *array)
{
	read_5464(array,registerName); 
}
/**
*描述:获取状态
*参数:
*返回:
**/
void getCs5464Status(unsigned char *cs5464Status)
{
	read_5464(cs5464Status,Status); 
}
/**
*描述:清除
*参数:
*返回:
**/
void clearCs5464Status(unsigned char *cs5464Status)
{
	read_5464(cs5464Status,Status|WRITE); 
}


/**
*描述:初始话CS5464
*参数:
*返回:
**/
void CS5464SPIInit()
{
	SDI_54640();
	SCLK_54640();
    MyDelay_us(1);
	RESET_54640();
	MyDelay_ms(1);
	RESET_54641();
	MyDelay_ms(1);
	send_5464(SYNC1);
	send_5464(SYNC1);
	send_5464(SYNC1);
    send_5464(SYNC0);
	____________//参考DC_CS5464.c文件
	Conversion(1);
	
}
/**
*描述:重启
*参数:
*返回:
**/
void RESET_CS5464(void)
{
	send_5464(RESET); 
}

//void Calibrate_CS5464()
//{
//	//不需要,因为开了高通滤波器

//}


