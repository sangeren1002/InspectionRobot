/**
  ******************************************************************************
  * @file    can_stm32.c
  * @author  Zhenglin R&D Driver Software Team
  * @version V1.0.0
  * @date    26/04/2015
  * @brief   This file is can_stm32 file.
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "can_stm32.h"
#include "canfestival.h"
#include "SEGGER_RTT.h"
#include "GlobalVar.h"
#include "motor_control.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
  * @brief  canInit
  * @param  CANx:CAN1 or CAN2 bitrate
  * @retval 0��Success
  */
unsigned char canInit(CAN_TypeDef* CANx,unsigned int bitrate)
{
	GPIO_InitTypeDef GPIO_InitStructure; 
	CAN_InitTypeDef		 CAN_InitStructure;
	CAN_FilterInitTypeDef  CAN_FilterInitStructure;
	NVIC_InitTypeDef  NVIC_InitStructure;
	//使能相关时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);//使能PORTA时钟																 

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);//使能CAN1时钟	

	//初始化GPIO
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8| GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
	GPIO_Init(GPIOB, &GPIO_InitStructure);//初始化PA11,PA12

	//引脚复用映射配置
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource8,GPIO_AF_CAN1); //GPIOA11复用为CAN1
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource9,GPIO_AF_CAN1); //GPIOA12复用为CAN1

	//CAN单元设置
	CAN_InitStructure.CAN_TTCM=DISABLE; //非时间触发通信模式	
	CAN_InitStructure.CAN_ABOM=DISABLE; //软件自动离线管理	  
	CAN_InitStructure.CAN_AWUM=DISABLE;//睡眠模式通过软件唤醒(清除CAN->MCR的SLEEP位)
	CAN_InitStructure.CAN_NART=ENABLE;	//禁止报文自动传送 
	CAN_InitStructure.CAN_RFLM=DISABLE; //报文不锁定,新的覆盖旧的  
	CAN_InitStructure.CAN_TXFP=DISABLE; //优先级由报文标识符决定 
	CAN_InitStructure.CAN_Mode= CAN_Mode_Normal;	 //模式设置 
	CAN_InitStructure.CAN_SJW=CAN_SJW_1tq; //重新同步跳跃宽度(Tsjw)为tsjw+1个时间单位 CAN_SJW_1tq~CAN_SJW_4tq
	CAN_InitStructure.CAN_BS1=CAN_BS1_6tq; //Tbs1范围CAN_BS1_1tq ~CAN_BS1_16tq
	CAN_InitStructure.CAN_BS2=CAN_BS1_7tq;//Tbs2范围CAN_BS2_1tq ~	CAN_BS2_8tq
	CAN_InitStructure.CAN_Prescaler=bitrate;  //分频系数(Fdiv)为brp+1	
	CAN_Init(CAN1, &CAN_InitStructure);   // 初始化CAN1 

	//配置过滤器
	CAN_FilterInitStructure.CAN_FilterNumber=0;	  //过滤器0
	CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask; 
	CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit; //32位 
	CAN_FilterInitStructure.CAN_FilterIdHigh=0x0000;////32位ID
	CAN_FilterInitStructure.CAN_FilterIdLow=0x0000;
	CAN_FilterInitStructure.CAN_FilterMaskIdHigh=0x0000;//32位MASK
	CAN_FilterInitStructure.CAN_FilterMaskIdLow=0x0000;
	CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_Filter_FIFO0;//过滤器0关联到FIFO0
	CAN_FilterInitStructure.CAN_FilterActivation=ENABLE; //激活过滤器0
	CAN_FilterInit(&CAN_FilterInitStructure);//滤波器初始化


	CAN_ITConfig(CAN1,CAN_IT_FMP0,ENABLE);//FIFO0消息挂号中断允许.			
	NVIC_InitStructure.NVIC_IRQChannel = CAN1_RX0_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;	  // 主优先级为1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;			  // 次优先级为0
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
#if 0

		CAN_InitTypeDef        CAN_InitStructure;
		CAN_FilterInitTypeDef  CAN_FilterInitStructure;
		GPIO_InitTypeDef  GPIO_InitStructure;
		NVIC_InitTypeDef NVIC_InitStructure;	         
		/* CAN GPIOs configuration **************************************************/
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;               
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		if(CANx == CAN1)
		{
				/* CAN register init */
				CAN_DeInit(CANx);
				/* Enable GPIO clock */
				RCC_AHB1PeriphClockCmd(CAN1_GPIO_CLK, ENABLE);
				/* Enable CAN clock */
				RCC_APB1PeriphClockCmd(CAN1_CLK, ENABLE);
				/* Connect CAN pins to AF9 */
				GPIO_PinAFConfig(CAN1_GPIO_PORT, CAN1_RX_SOURCE, CAN1_AF_PORT);
				GPIO_PinAFConfig(CAN1_GPIO_PORT, CAN1_TX_SOURCE, CAN1_AF_PORT); 
				
				/* Configure CAN RX and TX pins */
				GPIO_InitStructure.GPIO_Pin = CAN1_RX_PIN | CAN1_TX_PIN;
				GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
				GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
				GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
				GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
				GPIO_Init(CAN1_GPIO_PORT, &GPIO_InitStructure);
				GPIO_PinAFConfig(GPIOA,GPIO_PinSource11,GPIO_AF_CAN1); //GPIOA11复用为CAN1
	  			GPIO_PinAFConfig(GPIOA,GPIO_PinSource12,GPIO_AF_CAN1); //GPIOA12复用为CAN1
				/* CAN configuration ********************************************************/  
				CAN_StructInit(&CAN_InitStructure);
			
				/* CAN cell init */
				CAN_InitStructure.CAN_TTCM = DISABLE;    
				CAN_InitStructure.CAN_ABOM = ENABLE;   
				CAN_InitStructure.CAN_AWUM = DISABLE;		
				CAN_InitStructure.CAN_NART = DISABLE;	
				CAN_InitStructure.CAN_RFLM = DISABLE;	
				CAN_InitStructure.CAN_TXFP = DISABLE;		
				CAN_InitStructure.CAN_Mode = CAN_Mode_Normal;	
				CAN_InitStructure.CAN_SJW = CAN_SJW_1tq;  
				 
				CAN_InitStructure.CAN_BS1 = CAN_BS1_6tq;
				CAN_InitStructure.CAN_BS2 = CAN_BS2_7tq;
				CAN_InitStructure.CAN_Prescaler = bitrate;   
				CAN_Init(CANx, &CAN_InitStructure);
			
				CAN_FilterInitStructure.CAN_FilterNumber = 0;	 
				CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask;   
				CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit;
				CAN_FilterInitStructure.CAN_FilterIdHigh = 0x0000;
				CAN_FilterInitStructure.CAN_FilterIdLow = 0x0000;
				CAN_FilterInitStructure.CAN_FilterMaskIdHigh = 0x0000;
				CAN_FilterInitStructure.CAN_FilterMaskIdLow = 0x0000;
				CAN_FilterInitStructure.CAN_FilterFIFOAssignment = 0;           
				CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;           
				CAN_FilterInit(&CAN_FilterInitStructure);
						
				/* Enable FIFO 0 message pending Interrupt */
				CAN_ITConfig(CANx, CAN_IT_FMP0, ENABLE);	              
				
				NVIC_InitStructure.NVIC_IRQChannel = CAN1_RX0_IRQn;
				NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;
				NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x01;
				NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
				NVIC_Init(&NVIC_InitStructure);

		}
		else
		{
				/* Enable GPIO clock */
				RCC_AHB1PeriphClockCmd(CAN2_GPIO_CLK, ENABLE);
			
				/* Connect CAN pins to AF9 */
				GPIO_PinAFConfig(CAN2_GPIO_PORT, CAN2_RX_SOURCE, CAN2_AF_PORT);
				GPIO_PinAFConfig(CAN2_GPIO_PORT, CAN2_TX_SOURCE, CAN2_AF_PORT); 
				
				/* Configure CAN RX and TX pins */
				GPIO_InitStructure.GPIO_Pin = CAN2_RX_PIN | CAN2_TX_PIN;
				GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
				GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
				GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
				GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
				GPIO_Init(CAN2_GPIO_PORT, &GPIO_InitStructure);
			
				/* CAN configuration ********************************************************/  
				/* Enable CAN clock */
				RCC_APB1PeriphClockCmd(CAN2_CLK, ENABLE);
				
				/* CAN register init */
				CAN_DeInit(CANx);
				CAN_StructInit(&CAN_InitStructure);
			
				/* CAN cell init */
				CAN_InitStructure.CAN_TTCM = DISABLE;    
				CAN_InitStructure.CAN_ABOM = ENABLE;    
				CAN_InitStructure.CAN_AWUM = DISABLE;	
				CAN_InitStructure.CAN_NART = DISABLE;	
				CAN_InitStructure.CAN_RFLM = DISABLE;	
				CAN_InitStructure.CAN_TXFP = DISABLE;		
				CAN_InitStructure.CAN_Mode = CAN_Mode_Normal;	
				CAN_InitStructure.CAN_SJW = CAN_SJW_1tq; 
					
				CAN_InitStructure.CAN_BS1 = CAN_BS1_6tq;
				CAN_InitStructure.CAN_BS2 = CAN_BS2_7tq;
				CAN_InitStructure.CAN_Prescaler = bitrate;    
				CAN_Init(CANx, &CAN_InitStructure);		

				CAN_SlaveStartBank(14);                             
				CAN_FilterInitStructure.CAN_FilterNumber = 14;	  
				CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask;   
				CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit;
				CAN_FilterInitStructure.CAN_FilterIdHigh = 0x0000;
				CAN_FilterInitStructure.CAN_FilterIdLow = 0x0000;
				CAN_FilterInitStructure.CAN_FilterMaskIdHigh = 0x0000;
				CAN_FilterInitStructure.CAN_FilterMaskIdLow = 0x0000;
				CAN_FilterInitStructure.CAN_FilterFIFOAssignment = 0;           
				CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;         

				CAN_FilterInit(&CAN_FilterInitStructure);
						
				/* Enable FIFO 0 message pending Interrupt */
				CAN_ITConfig(CANx, CAN_IT_FMP0, ENABLE);	             
				NVIC_InitStructure.NVIC_IRQChannel = CAN2_RX0_IRQn;	 
				NVIC_Init(&NVIC_InitStructure);                       
		}
#endif
		return 0;
}
/**
  * @brief  canSend
	* @param  CANx:CAN1 or CAN2   m:can message
  * @retval 0��Success
  */
unsigned char canSend(CAN_PORT CANx, Message *m)	                
{
		unsigned char ret;
		unsigned char i;
		CanTxMsg TxMessage;
		TxMessage.StdId = (uint32_t)(m->cob_id);
		TxMessage.ExtId = 0x00;
		TxMessage.RTR = m->rtr;								  
		TxMessage.IDE = CAN_ID_STD;                           
		TxMessage.DLC = m->len;                              
		for(i=0;i<m->len;i++)                                 
		{
				TxMessage.Data[i] = m->data[i];
		}
		ret = CAN_Transmit(CANx, &TxMessage);
		if(ret == CAN_TxStatus_NoMailBox)
		{
				return 1;	
		}
		else 
		{
				return 0;    
		}
}
/**
  * @brief  This function handles CAN1 RX0 request.
  * @param  None
  * @retval None
  */
CANOpen_Message CAN1_Rx_m;
Message RxMSG = Message_Initializer;
void CAN1_RX0_IRQHandler(void)
{  
    unsigned int i = 0;
				
	CAN_Receive(CAN1, CAN_FIFO0, &(CAN1_Rx_m.m));	        //��CAN1 FIFO0��������
	RxMSG.cob_id = (uint16_t)(CAN1_Rx_m.m.StdId);
	RxMSG.rtr = CAN1_Rx_m.m.RTR;
	RxMSG.len = CAN1_Rx_m.m.DLC;
	for(i=0;i<RxMSG.len;i++)
	{
		RxMSG.data[i] = CAN1_Rx_m.m.Data[i];
	}
   AnalysisMessagefromDriver();
//	SEGGER_RTT_printf(0, "can master revcive data!!!!!!!!!!!!\n");
	canDispatch(CANOpenMasterObject, &(RxMSG));
}

/**
  * @}
  */ 
/**
  * @brief  This function handles CAN2 RX0 request.
  * @param  None
  * @retval None
  */
void CAN2_RX0_IRQHandler(void)
{
  CANOpen_Message CAN2_Rx_m;                          
  CAN_Receive(CAN2, CAN_FIFO0, &(CAN2_Rx_m.m));	  
}
/******************* (C) COPYRIGHT 2015 Personal Electronics *****END OF FILE****/
