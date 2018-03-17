// /* BoardB.c file
// 编写者：xiang
// 编译环境：MDK  Version: 5.14

// 占用STM32 资源：
// 1. 使用UART4发送和接收主板B数据

// 功能：和主板B通信，发送高度，发送启动信息，接收命令
// ------------------------------------
//  */

// #include "BoardB.h"
// #include "GCS_Protocol.h"

// extern uint8_t Command;

// /*
// 串口设置的一般步骤可以总结为如下几个步骤：
// 1) 串口时钟使能， GPIO 时钟使能。
// 2) 设置引脚复用器映射：调用 GPIO_PinAFConfig 函数。
// 3) GPIO 初始化设置：要设置模式为复用功能。
// 4) 串口参数初始化：设置波特率，字长，奇偶校验等参数。
// 5) 开启中断并且初始化 NVIC，使能中断（如果需要开启中断才需要这个步骤）。
// 6) 使能串口。
// 7) 编写中断处理函数：函数名格式为 USARTxIRQHandler(x 对应串口号)。
// */


// //串口4初始化
// void Initial_UART4(u32 baudrate)
// {
// 	//uart4的引脚：PC10 TX；PC11 RX
//  	GPIO_InitTypeDef GPIO_InitStructure;
// 	USART_InitTypeDef USART_InitStructure;
// 	NVIC_InitTypeDef NVIC_InitStructure; 

// 	/* 使能 UART4 模块的时钟  使能 UART4对应的引脚端口PC的时钟*/
// 	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4,ENABLE);
// 	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC , ENABLE);
//   	 /* 配置UART4 的发送引脚
// 	 配置PC10 为复用输出  刷新频率50MHz
// 	  */
//   	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
// 		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//     GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
//     GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;  // 复用推挽输出
//     GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
//   	GPIO_Init(GPIOC, &GPIO_InitStructure);    
//   	/* 
// 	  配置UART4 的接收引脚
// 	  配置PC11为浮地输入 
// 	 */
//   	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
//   	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
// 		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
//   	GPIO_Init(GPIOC, &GPIO_InitStructure);

// 		GPIO_PinAFConfig(GPIOC,GPIO_PinSource10,GPIO_AF_UART4);
//     GPIO_PinAFConfig(GPIOC,GPIO_PinSource11,GPIO_AF_UART4);
// 	/* 
// 	  UART4的配置:
// 	  1.波特率为调用程序指定的输入 baudrate;
// 	  2. 8位数据			  USART_WordLength_8b;
// 	  3.一个停止位			  USART_StopBits_1;
// 	  4. 无奇偶效验			  USART_Parity_No ;
// 	  5.不使用硬件流控制	  USART_HardwareFlowControl_None;
// 	  6.使能发送和接收功能	  USART_Mode_Rx | USART_Mode_Tx;
// 	 */
// 	USART_InitStructure.USART_BaudRate = baudrate;
// 	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
// 	USART_InitStructure.USART_StopBits = USART_StopBits_1;
// 	USART_InitStructure.USART_Parity = USART_Parity_No ;
// 	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
// 	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	 //开启接收功能	和发送功能
// 	//应用配置到UART4
// 	USART_Init(UART4, &USART_InitStructure); 
	
// 	USART_ITConfig(UART4, USART_IT_RXNE, ENABLE);	//使能接收中断	
//   USART_Cmd(UART4, ENABLE);//启动UART4
// 	//配置中断优先级
// 	/* Enable the UART4 Interrupt */
// 	NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn;
// 	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
// 	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 6;
// 	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
// 	NVIC_Init(&NVIC_InitStructure);
		
// }

// //uart4给主板发送一个字节
// void BoardB_SendChar(uint8_t data)
// {
// 		USART_SendData(UART4, data);
// 	//等待发送完成.
//   	while (!(UART4->SR & USART_FLAG_TXE));	
// }

// void BoardB_Sendfloat(float data)
// {

// }

// //uart4中断程序
// void UART4_IRQHandler(void)
// {
// 	uint8_t uart4_indata = Command_Stay;
// 	if(USART_GetITStatus(UART4, USART_IT_RXNE) != RESET)
// 	{
// 		uart4_indata = USART_ReceiveData(UART4);	//取接收到的字节
// 		if(uart4_indata>0x00 && uart4_indata < CommandNum)//判断接收到的数据是否有效
// 		{
// 			Command = uart4_indata;
// 		}
// 		else
// 			Command = Command_Stay;
// 		/* Clear the UART4 RX interrupt */
//   	USART_ClearITPendingBit(UART4, USART_IT_RXNE);
// 	}	
// }
