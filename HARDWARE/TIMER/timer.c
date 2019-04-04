#include "timer.h"
#include "led.h"
#include "usart3.h"
#include "usart.h"

 
extern u8 ov_frame;
extern volatile u16 jpeg_data_len;
extern void usbapp_pulling(void); 

vu8 framecnt;		//统一的帧计数器
vu8 framecntout;	//统一的帧计数器输出变量
u16 timecnt=0;

extern u16 UART3_RX_STA;
extern u16 USART_RX_STA;
//定时器7中断服务程序    
void TIM7_IRQHandler(void)
{ 	
   		    
	if(TIM_GetITStatus(TIM7,TIM_IT_Update)==SET)//是更新中断
	{	 			   
		USART_RX_STA|=1<<15;	//标记接收完成
		TIM_ClearITPendingBit(TIM7,TIM_IT_Update);  //清楚中断标志位    
   
		TIM_Cmd(TIM7,DISABLE); 	//关闭定时器7 
	}	     											 
} 
//通用定时器7,中断初始化
//这里时钟选择为APB1的2倍，APB1为42M
//arr：自动重装值
//psc时钟预分频数
//定时器溢出时间计算方法:Tout=((arr+1)*(psc+1))/Ft us.
//Ft=定时器中作频率，单位为:Mhz 
void TIM7_Int_Init(u16 arr,u16 psc)
{	 
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7,ENABLE);  ///ʹ使能TIM7时钟
	
  TIM_TimeBaseInitStructure.TIM_Period = arr; 	//自动重装值
	TIM_TimeBaseInitStructure.TIM_Prescaler=psc;  //定时器分频
	TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up; //向上计数模式
	TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	
	TIM_TimeBaseInit(TIM7,&TIM_TimeBaseInitStructure);
	
	TIM_ITConfig(TIM7,TIM_IT_Update,ENABLE); //允许定时器6更新中断
	TIM_Cmd(TIM7,ENABLE); //使能定时器6
	
	NVIC_InitStructure.NVIC_IRQChannel=TIM7_IRQn;  
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0x00; //抢占优先级0
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=0x01; //子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStructure); 							 
} 
//定时器4中断服务程序    
void TIM5_IRQHandler(void)
{ 	
   		    
	if(TIM_GetITStatus(TIM5,TIM_IT_Update)==SET)//是更新中断
	{	 			   
		USART_RX_STA|=1<<15;	//标记接收完成
		TIM_ClearITPendingBit(TIM5,TIM_IT_Update);  //清楚中断标志位    
   
		TIM_Cmd(TIM5,DISABLE); 	//关闭定时器4 
	}	     											 
} 
//通用定时器4,中断初始化
//这里时钟选择为APB1的2倍，APB1为42M
//arr：自动重装值
//psc时钟预分频数
//定时器溢出时间计算方法:Tout=((arr+1)*(psc+1))/Ft us.
//Ft=定时器中作频率，单位为:Mhz 
void TIM5_Int_Init(u16 arr,u16 psc)
{	 
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5,ENABLE);  ///ʹ使能TIM6时钟
	
    TIM_TimeBaseInitStructure.TIM_Period = arr; 	//自动重装值
	TIM_TimeBaseInitStructure.TIM_Prescaler=psc;  //定时器分频
	TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up; //向上计数模式
	TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	
	TIM_TimeBaseInit(TIM5,&TIM_TimeBaseInitStructure);
	
	TIM_ITConfig(TIM5,TIM_IT_Update,ENABLE); //允许定时器4更新中断
	TIM_Cmd(TIM5,ENABLE); //使能定时器4
	
	NVIC_InitStructure.NVIC_IRQChannel=TIM5_IRQn;  
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0x00; //抢占优先级0
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=0x01; //子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStructure); 							 
}

//定时器7中断服务程序    
void TIM3_IRQHandler(void)
{ 	
   		    
	if(TIM_GetITStatus(TIM3,TIM_IT_Update)==SET)//是更新中断
	{
        timecnt++;
	
		TIM_ClearITPendingBit(TIM3,TIM_IT_Update);  //清楚中断标志位 
    }        
}

void TIM3_Int_Init(u16 arr,u16 psc)
{	 
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);  ///ʹ使能TIM6时钟
	
    TIM_TimeBaseInitStructure.TIM_Period = arr; 	//自动重装值
	TIM_TimeBaseInitStructure.TIM_Prescaler=psc;  //定时器分频
	TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up; //向上计数模式
	TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	
	TIM_TimeBaseInit(TIM3,&TIM_TimeBaseInitStructure);
	
	TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE); //允许定时器4更新中断
	TIM_Cmd(TIM3,ENABLE); //使能定时器4
	
	NVIC_InitStructure.NVIC_IRQChannel=TIM3_IRQn;  
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0x00; //抢占优先级0
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=0x01; //子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStructure); 							 
}







