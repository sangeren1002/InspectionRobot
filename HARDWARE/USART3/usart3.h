#ifndef __USART3_H
#define __USART3_H 
#include "sys.h"
#include "stdio.h"	  


#define USART3_MAX_RECV_LEN		200					//接收缓存区大小，最大为200个字节
#define USART3_MAX_SEND_LEN		400					//发送缓存区大小，最大为400个字节
#define USART3_RX_EN 			1					//0,串口接收失能;1,串口接收使能

extern u8  USART3_RX_BUF[USART3_MAX_RECV_LEN]; 		//串口接收缓存区，大小为USART3_MAX_RECV_LEN字节
extern u8  USART3_TX_BUF[USART3_MAX_SEND_LEN]; 		//串口发送缓存区，大小为USART3_MAX_SEND_LEN字节
extern vu16 USART3_RX_STA;   								//串口3接收状态标志

void usart3_init(u32 bound);     //串口3初始化
void u3_printf(char* fmt,...);   //串口3打印
#endif	   
















