#include "sys.h"		    
#include "rs485.h"	 
#include "delay.h"
#include "string.h"


#if EN_USART2_RX   		//如果使能了接收   	  

u8 RS485_RX_BUF[64];  	//接收缓冲,最大64个字节.
u8 RS485_RX_CNT=0;      //接收到的数据长度
u8 buffer[6]={0x7f,0x10,0x02,0x06,0x11,0x58};//深圳优贝特电池信息获取命令
u8 rs485buf[35]={0}; 	//电池信息数据缓存区

BattaryMSG battarymsg;

void UART4_IRQHandler(void)
{
	u8 res;	    
	if(USART_GetITStatus(UART4, USART_IT_RXNE) != RESET)//接收到数据
	{	 	
	  res =USART_ReceiveData(UART4);//;读取接收到的数据USART2->DR
		if(RS485_RX_CNT<64)
		{
			RS485_RX_BUF[RS485_RX_CNT]=res;		//记录接收到的值
			RS485_RX_CNT++;						//接收数据增加1 
		} 
	}  											 
} 
#endif	

//初始化IO 串口2
//bound:波特率	  
void MYRS485_Init(u32 bound)
{  	 
	
  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC|RCC_AHB1Periph_GPIOD,ENABLE); //使能GPIOA时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4,ENABLE);//使能USART2时钟
	
  //串口2引脚复用映射
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource10,GPIO_AF_UART4); //GPIOA2复用为USART2
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource11,GPIO_AF_UART4); //GPIOA3复用为USART2
	
	//USART2    
  	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11; //GPIOA2与GPIOA3
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//速度100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //上拉
	GPIO_Init(GPIOC,&GPIO_InitStructure); //初始化PA2，PA3
	
	//PG8推挽输出，485模式控制  
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12; //GPIOG8
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//速度100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //上拉
	GPIO_Init(GPIOC,&GPIO_InitStructure); //初始化PG8
		//PG8推挽输出，485模式控制  
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0; //GPIOG8
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//速度100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //上拉
	GPIO_Init(GPIOD,&GPIO_InitStructure); //初始化PG8

   //USART2 初始化设置
	USART_InitStructure.USART_BaudRate = bound;//波特率设置
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
  USART_Init(UART4, &USART_InitStructure); //初始化串口2
	
    USART_Cmd(UART4, ENABLE);  //使能串口 2
	
	USART_ClearFlag(UART4, USART_FLAG_TC);
	
#if EN_USART2_RX	
	USART_ITConfig(UART4, USART_IT_RXNE, ENABLE);//开启接受中断

	//Usart2 NVIC 配置
  NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3;//抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =3;		//子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器、

#endif	
	
	MYRS485_RX_EN;				//默认为接收模式	
}

//初始化IO 串口2
//bound:波特率	  
void RS485_Init(u32 bound)
{  	 
	
  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE); //使能GPIOA时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);//使能USART2时钟
	
  //串口2引脚复用映射
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource2,GPIO_AF_USART2); //GPIOA2复用为USART2
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource3,GPIO_AF_USART2); //GPIOA3复用为USART2
	
	//USART2    
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3; //GPIOA2与GPIOA3
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//速度100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //上拉
	GPIO_Init(GPIOA,&GPIO_InitStructure); //初始化PA2，PA3
	
	//PG8推挽输出，485模式控制  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8; //GPIOG8
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//速度100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //上拉
	GPIO_Init(GPIOG,&GPIO_InitStructure); //初始化PG8
	

   //USART2 初始化设置
	USART_InitStructure.USART_BaudRate = bound;//波特率设置
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
  USART_Init(USART2, &USART_InitStructure); //初始化串口2
	
  USART_Cmd(USART2, ENABLE);  //使能串口 2
	
	USART_ClearFlag(USART2, USART_FLAG_TC);
	
#if EN_USART2_RX	
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);//开启接受中断

	//Usart2 NVIC 配置
  NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3;//抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =3;		//子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器、

#endif	
	
	RS485_TX_EN=0;				//默认为接收模式	
}

//RS485发送len个字节.
//buf:发送区首地址
//len:发送的字节数(为了和本代码的接收匹配,这里建议不要超过64个字节)
void RS485_Send_Data(u8 *buf,u8 len)
{
	u8 t;
	MYRS485_TX_EN;			//设置为发送模式
    delay_ms(5);
  	for(t=0;t<len;t++)		//循环发送数据
	{
        while(USART_GetFlagStatus(UART4,USART_FLAG_TC)==RESET); //等待发送结束		
        USART_SendData(UART4,buf[t]); //发送数据
	}	 
	while(USART_GetFlagStatus(UART4,USART_FLAG_TC)==RESET); //等待发送结束		
	RS485_RX_CNT=0;	  
	MYRS485_RX_EN;				//设置为接收模式
    delay_ms(5);	
}
//RS485查询接收到的数据
//buf:接收缓存首地址
//len:读到的数据长度
void RS485_Receive_Data(u8 *buf,u8 *len)
{
	u8 rxlen=RS485_RX_CNT;
	u8 i=0;
	*len=0;				//默认为0
	delay_ms(10);		//等待10ms,连续超过10ms没有接收到一个数据,则认为接收结束
	if(rxlen==RS485_RX_CNT&&rxlen)//接收到了数据,且接收完成了
	{
		for(i=0;i<rxlen;i++)
		{
			buf[i]=RS485_RX_BUF[i];	
		}		
		*len=RS485_RX_CNT;	//记录本次数据长度
		RS485_RX_CNT=0;		//清零
	}
}

//void BattaryMSGAnalysis(void)
//{
//     battarymsg.chargingstatus  = rs485buf[5];
//     battarymsg.current         = rs485buf[9];
//     battarymsg.voltage         = rs485buf[15] + rs485buf[16]*256;
//     battarymsg.charging_times  = rs485buf[19] + rs485buf[20]*256;
//     battarymsg.remaining_total_capacity  = rs485buf[21] + (rs485buf[22]*256);
//     battarymsg.total_capacity  = rs485buf[23] + (rs485buf[24]*256);
//     battarymsg.remaining_battery_percentage         = (float)(battarymsg.remaining_total_capacity*100/battarymsg.total_capacity);
//}
//void Get_BattaryMsg(void)
//{
//	u8 datalen;
//	RS485_Send_Data(buffer,strlen((const char*)buffer));	//发送电池信息获取命令
//    delay_ms(100);
//    RS485_Receive_Data(rs485buf,&datalen);
//    BattaryMSGAnalysis(); 

//}




