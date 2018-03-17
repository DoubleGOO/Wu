/**
  ******************************************************************************
  * @file    usart.c
  * @author  佟源洋
  * @version V1.0
  * @date    1-April-2014
  * @brief   This file includes the driver for STM32F407VGT6's usart
	* @attention:          
						 1、使用stm32f4discovery时，不能使用USART1，USART1的PA9脚已经作为VBUS_FS使用，强行使用串口会出现乱码。
							  如若使用串口，请使用其他串口。参见stm32f4 dicovery原理图。
						 2、stm32f4固件库默认HSE_VALUE为25MHZ，而板载晶振为8MHZ,需修改stm32f4xx_conf.h，重定义HSE_VALUE为8MHZ。否则串口也会出现乱码。
								#if defined  (HSE_VALUE) 
								#undef HSE_VALUE 
								#define HSE_VALUE    ((uint32_t)8000000) 
								#endif
xaing：佟源祥代码中usart2用于和PC通信，usart3用于和摄像头连接
  ******************************************************************s************
*/
/**
  ******************************************************************************
  * @file    usart.c
  * @author  巫子晨
  * @version V1.1
  * @date    17-April-2017
  * @brief   与副控板通信的相关函数以及变量都在该函数中
	* @attention:          
						 
  ******************************************************************s************
*/
#include "UART4.h"
#include "delay.h"
#include "camera.h"
#include <stdio.h>
#include <stdlib.h>
#include "UART1.h"
unsigned char Report_Buf[UART4_RECEIVE_BUFSIZE] = "";
float distance_front = 0,distance_back = 0,distance_left = 0,distance_right = 0;
int UART4_CH[6]={0};
int get_new = 0;//数据更新时置1
int USART_REC_LEN = 0,USART_REC_LEN_LAST = 0;;
//unsigned char UART4_buffer[RX_BUFFER_SIZE]={'\r','\n','1','\r','\n'};
unsigned char UART4_buffer[RX_BUFFER_SIZE]={'\r','\n','1','\r','\n'};

void Initial_UART4(u32 bound){
    GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE); //Ê¹ÄÜGPIOAÊ±ÖÓ
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4,ENABLE);//Ê¹ÄÜUSART1Ê±ÖÓ

	GPIO_PinAFConfig(GPIOC, GPIO_PinSource10, GPIO_AF_UART4);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource11, GPIO_AF_UART4);
	

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;  // ¸´ÓÃÍÆÍìÊä³ö
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	USART_InitStructure.USART_BaudRate = bound;//²¨ÌØÂÊÉèÖÃ
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//×Ö³¤Îª8Î»Êý¾Ý¸ñÊ½
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//Ò»¸öÍ£Ö¹Î»
	USART_InitStructure.USART_Parity = USART_Parity_No;//ÎÞÆæÅ¼Ð£ÑéÎ»
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//ÎÞÓ²¼þÊý¾ÝÁ÷¿ØÖÆ
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//ÊÕ·¢Ä£Ê½
    USART_Init(UART4, &USART_InitStructure); //³õÊ¼»¯´®¿Ú
	
    USART_Cmd(UART4, ENABLE);  //Ê¹ÄÜ´®¿Ú
	
	USART_ClearFlag(UART4, USART_FLAG_RXNE);

#if EN_USART4_RX	
	USART_ITConfig(UART4, USART_IT_RXNE, ENABLE);//¿ªÆôÏà¹ØÖÐ¶Ï
	
	NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn;		//´®¿Ú ÖÐ¶ÏÍ¨µÀ
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0; //ÇÀÕ¼ÓÅÏÈ¼¶
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =14;		//×ÓÓÅÏÈ¼¶
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQÍ¨µÀÊ¹ÄÜ
	NVIC_Init(&NVIC_InitStructure);	//¸ù¾ÝÖ¸¶¨µÄ²ÎÊý³õÊ¼»¯VIC¼Ä´æÆ÷¡¢

#endif
	
}

void uart4_putchar(unsigned char ch) //发送一个字符
{  
    USART_ClearFlag(UART4,USART_FLAG_TC);
    USART_SendData(UART4,ch);
    while( USART_GetFlagStatus(UART4,USART_FLAG_TC) == RESET );

}

void UART4Write(unsigned char *data,uint32_t len)	 //发送n个字符
{
    uint16_t i;
    if( len != 0)
    { 
        for( i = 0; i < len; i++ )
        {
            uart4_putchar(data[i]);
        }
    }
}

u16 UART4_Receiver_buf(u8 *data,u32 len,u32 time) //等待接收数据：接收到的数据   接收到的个数   等待接收的时间 
{      
    u16 i = 0;
    while(1)
    { 
        if( USART_GetFlagStatus(UART4,USART_FLAG_RXNE) != RESET )
        {	
            data[i] = USART_ReceiveData(UART4);
            i++;
            if( i == len && (len != 0) )
            {   
                break; 
            }
        }
				if((time--)==0)
					break;
    } 
    return i;
}

void Get_Remote()
{
	char str[10] = {0};
	int len = 0;
	int i = USART_REC_LEN_LAST;//记录上一次读到的位置
	int m = 0;
	int n = 0;
	int flag = 0;
	int time = 0;//计时用，防止卡死
	for(m = 0;m<RX_BUFFER_SIZE;m++)
  {
		if(i<RX_BUFFER_SIZE-3&&UART4_buffer[i]=='+'&&UART4_buffer[i+1]=='I'&&UART4_buffer[i+2]=='P'&&UART4_buffer[i+3]=='D')
		{
			i = i+7;
			flag = 1;
			break;			
		}
		if(++i>=RX_BUFFER_SIZE)
			i=i-RX_BUFFER_SIZE;
	}
	if(flag == 0)
	{
		USART_REC_LEN_LAST=0;
		return;
	}
	if(i>=RX_BUFFER_SIZE)
			i=i-RX_BUFFER_SIZE;
	m=0;
	while(UART4_buffer[i]!=':')
	{
		str[m] = UART4_buffer[i];
		if(++i>=RX_BUFFER_SIZE)
			i=i-RX_BUFFER_SIZE;
		m++;
		if(++time>=50)
			return;
	}
	len = atof(str);
	
	
	if(++i>=RX_BUFFER_SIZE)
			i=i-RX_BUFFER_SIZE;
	if(len==18||len==36||len==54)
	{
	  for(m=0;m<6;m++)
	  {
		  for(n = 0;n<3;n++)
			{
				str[n] = UART4_buffer[i];
				if(atof(str)<=255&&atof(str)>=0)
					UART4_CH[m] = atof(str);
				if(++i>=RX_BUFFER_SIZE)
			    i=i-RX_BUFFER_SIZE;
			}
	  }
		UART4_CH[0]=(UART4_CH[0]-128);//CH1
		UART4_CH[1]=(UART4_CH[1]-128);//CH2
		UART4_CH[2]=UART4_CH[2];//CH3
		UART4_CH[3]=(UART4_CH[3]-128)*2;//CH4
		UART4_CH[4]=UART4_CH[4];//CH5 
		UART4_CH[5]=UART4_CH[5];//CH6
  }
	
	USART_REC_LEN_LAST = i;
	
}

void Get_Distance()
{
	unsigned char m;
	unsigned int i=3;
	char str[10] = {0};
	
	static uint32_t last_call_us = 0;
    uint32_t now_time = micros();
   if(now_time - last_call_us < 20000 )return ; //更新速率为50hz
		last_call_us = now_time ;
	
	UART4_Receiver_buf(UART4_buffer,24,200);
	for (m = 0; m < 6;m++)
	{	
		str[m]='\0';
	}
	for (m = 0; m < 5;m++)
	{	
		str[m]=UART4_buffer[i++];
	}
	distance_front = atof(str)/10000;
	
	for (m = 0; m < 6;m++)
	{	
		str[m]='\0';
	}
	for (m = 0; m < 5;m++)
	{	
		str[m]=UART4_buffer[i++];
	}
	distance_right = atof(str)/10000;
	
	for (m = 0; m < 6;m++)
	{	
		str[m]='\0';
	}
	for (m = 0; m < 5;m++)
	{	
		str[m]=UART4_buffer[i++];
	}
	distance_left = atof(str)/10000;
	
	for (m = 0; m < 6;m++)
	{	
		str[m]='\0';
	}
	for (m = 0; m < 5;m++)
	{	
		str[m]=UART4_buffer[i++];
	}
	distance_back = atof(str)/10000;
	
}

void UART4_Get_Data(void)
{
	if(get_new)
	{
//  Get_Distance();
	  Get_Remote();
	}
	get_new = 0;
}

//u16 USART_RX_STA=0; 
void UART4_IRQHandler(void)
{
  unsigned char data;

	if(USART_GetITStatus(UART4, USART_IT_RXNE) != RESET)
  {
		
		data=USART_ReceiveData(UART4);
		if(data=='\r'||data=='\n')
			return;
		UART4_buffer[USART_REC_LEN++]=data;
		if(USART_REC_LEN>=RX_BUFFER_SIZE)
			USART_REC_LEN=0;
		get_new = 1;
//		if((USART_RX_STA&0x8000)==0)
//		{
//			if(USART_RX_STA&0x4000)
//			{
//				if(data!=0x0a)USART_RX_STA=0;
//				else USART_RX_STA|=0x8000;	
//			}
//			else 
//			{	
//				if(data==0x0d)USART_RX_STA|=0x4000;
//				else
//				{
//					UART4_buffer[USART_RX_STA&0X3FFF]=data ;
//					USART_RX_STA++;
//					if(USART_RX_STA>(RX_BUFFER_SIZE-1))USART_RX_STA=0; 
//				}		 
//			}
//		}
  }

}

