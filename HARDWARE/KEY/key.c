#include "key.h"
#include "delay.h" 

//void KEY_Init(void)
//{
//	
//	GPIO_InitTypeDef  GPIO_InitStructure;

//  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA|RCC_AHB1Periph_GPIOE, ENABLE);//ʹ��GPIOA,GPIOEʱ��
// 
//  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4; //KEY0 KEY1 KEY2��Ӧ����
//  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;//��ͨ����ģʽ
//  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100M
//  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//����
//  GPIO_Init(GPIOE, &GPIO_InitStructure);//��ʼ��GPIOE2,3,4
//	
//	 
//  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;//WK_UP��Ӧ����PA0
//  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN ;//����
//  GPIO_Init(GPIOA, &GPIO_InitStructure);//��ʼ��GPIOA0
// 
//} 

u8 KEY_Scan(u8 mode)
{	 
	static u8 key_up=1;//�������ɿ���־
	if(mode)key_up=1;  //֧������		  
	if(key_up&&(KEY0==0||KEY1==0||KEY2==0||WK_UP==1))
	{
		delay_ms(10);//ȥ���� 
		key_up=0;
		if(KEY0==0)return 1;
		else if(KEY1==0)return 2;
		else if(KEY2==0)return 3;
		else if(WK_UP==1)return 4;
	}else if(KEY0==1&&KEY1==1&&KEY2==1&&WK_UP==0)key_up=1; 	    
 	return 0;// �ް�������
}
void BSP_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
    
    /* 板载GPIO时钟使能 */
	LED_BSP_CLK_ENABLE();
    YUNTAI_CLK_ENABLE();
    CMODE_CLK_ENABLE();
    BJ_CLK_ENABLE();
    PZ_SWITCH_CLK_ENABLE();
    EMERSTOP_CLK_ENABLE();
	
    //板载LED灯管脚初始化
	GPIO_InitStructure.GPIO_Pin=LED_SUCCESS_BSP_PIN|LED_ERROR_BSP_PIN;  //PF8引脚
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_OUT; //输出模式
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_100MHz; //速度100M
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP; //推挽输出
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP;   //上拉
	GPIO_Init(LED_BSP_PORT,&GPIO_InitStructure);
    
	//板载BEEP  GPIO_Pin_8
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_8;  //PF8引脚
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_OUT; //输出模式
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_100MHz; //速度100M
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP; //推挽输出
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP;   //上拉
	GPIO_Init(GPIOF,&GPIO_InitStructure);
	
	//手动遥控转换 GPIO_Pin_4 ==0为自动模式 ==1为手动遥控模式
	GPIO_InitStructure.GPIO_Pin=CMODE_PIN;  //PG4引脚
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IN;  //输入
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_DOWN;  //上拉输入
	GPIO_Init(CMODE_PORT,&GPIO_InitStructure); 			//初始化GPIOE
    
	//云台控制信号输出  
	GPIO_InitStructure.GPIO_Pin=YUNTAI_PIN;  //PF8引脚
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_OUT; //输出模式
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_100MHz; //速度100M
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP; //推挽输出
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_DOWN;   //上拉
	GPIO_Init(YUNTAI_PORT,&GPIO_InitStructure);
    	
    //初始化报警指示灯引脚
    GPIO_InitStructure.GPIO_Pin = BJGREEN_PIN | BJRED_PIN | BJYELLOW_PIN | BJBUZZER_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//输出模式
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;//下拉
    GPIO_Init(BJ_PORT, &GPIO_InitStructure);//初始化GPIOG
    
    //碰撞开关检测引脚  检测高电平
    GPIO_InitStructure.GPIO_Pin = PZ_SWITCH_DETEC_PIN;
    GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IN;  //输入
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP;  //上拉输入
    GPIO_Init(PZ_SWITCH_DETEC_PORT, &GPIO_InitStructure);
    
    //急停开关检测引脚  检测低电平
    GPIO_InitStructure.GPIO_Pin = EMERSTOP_SWITCH_DETEC_PIN;
    GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IN;  //输入
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP;  //下拉输入
    GPIO_Init(EMERSTOP_SWITCH_DETEC_PORT, &GPIO_InitStructure);
    
	
	BJBEEP =0;//报警灯蜂鸣器
	BJRED=0;
	BJYELLOW=0;
	BJGREEN =0;
	YUNTAI=1;
}



















