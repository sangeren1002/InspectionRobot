#include "key.h"
#include "delay.h" 

//void KEY_Init(void)
//{
//	
//	GPIO_InitTypeDef  GPIO_InitStructure;

//  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA|RCC_AHB1Periph_GPIOE, ENABLE);//使能GPIOA,GPIOE时钟
// 
//  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4; //KEY0 KEY1 KEY2对应引脚
//  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;//普通输入模式
//  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100M
//  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
//  GPIO_Init(GPIOE, &GPIO_InitStructure);//初始化GPIOE2,3,4
//	
//	 
//  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;//WK_UP对应引脚PA0
//  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN ;//下拉
//  GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化GPIOA0
// 
//} 

u8 KEY_Scan(u8 mode)
{	 
	static u8 key_up=1;//按键按松开标志
	if(mode)key_up=1;  //支持连按		  
	if(key_up&&(KEY0==0||KEY1==0||KEY2==0||WK_UP==1))
	{
		delay_ms(10);//去抖动 
		key_up=0;
		if(KEY0==0)return 1;
		else if(KEY1==0)return 2;
		else if(KEY2==0)return 3;
		else if(WK_UP==1)return 4;
	}else if(KEY0==1&&KEY1==1&&KEY2==1&&WK_UP==0)key_up=1; 	    
 	return 0;// 无按键按下
}
void BSP_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
    
    /* 鏉胯浇GPIO鏃堕挓浣胯兘 */
	LED_BSP_CLK_ENABLE();
    YUNTAI_CLK_ENABLE();
    CMODE_CLK_ENABLE();
    BJ_CLK_ENABLE();
    PZ_SWITCH_CLK_ENABLE();
    EMERSTOP_CLK_ENABLE();
	
    //鏉胯浇LED鐏鑴氬垵濮嬪寲
	GPIO_InitStructure.GPIO_Pin=LED_SUCCESS_BSP_PIN|LED_ERROR_BSP_PIN;  //PF8寮曡剼
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_OUT; //杈撳嚭妯″紡
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_100MHz; //閫熷害100M
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP; //鎺ㄦ尳杈撳嚭
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP;   //涓婃媺
	GPIO_Init(LED_BSP_PORT,&GPIO_InitStructure);
    
	//鏉胯浇BEEP  GPIO_Pin_8
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_8;  //PF8寮曡剼
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_OUT; //杈撳嚭妯″紡
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_100MHz; //閫熷害100M
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP; //鎺ㄦ尳杈撳嚭
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP;   //涓婃媺
	GPIO_Init(GPIOF,&GPIO_InitStructure);
	
	//鎵嬪姩閬ユ帶杞崲 GPIO_Pin_4 ==0涓鸿嚜鍔ㄦā寮� ==1涓烘墜鍔ㄩ仴鎺фā寮�
	GPIO_InitStructure.GPIO_Pin=CMODE_PIN;  //PG4寮曡剼
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IN;  //杈撳叆
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_DOWN;  //涓婃媺杈撳叆
	GPIO_Init(CMODE_PORT,&GPIO_InitStructure); 			//鍒濆鍖朑PIOE
    
	//浜戝彴鎺у埗淇″彿杈撳嚭  
	GPIO_InitStructure.GPIO_Pin=YUNTAI_PIN;  //PF8寮曡剼
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_OUT; //杈撳嚭妯″紡
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_100MHz; //閫熷害100M
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP; //鎺ㄦ尳杈撳嚭
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_DOWN;   //涓婃媺
	GPIO_Init(YUNTAI_PORT,&GPIO_InitStructure);
    	
    //鍒濆鍖栨姤璀︽寚绀虹伅寮曡剼
    GPIO_InitStructure.GPIO_Pin = BJGREEN_PIN | BJRED_PIN | BJYELLOW_PIN | BJBUZZER_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//杈撳嚭妯″紡
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//鎺ㄦ尳杈撳嚭
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;//涓嬫媺
    GPIO_Init(BJ_PORT, &GPIO_InitStructure);//鍒濆鍖朑PIOG
    
    //纰版挒寮�鍏虫娴嬪紩鑴�  妫�娴嬮珮鐢靛钩
    GPIO_InitStructure.GPIO_Pin = PZ_SWITCH_DETEC_PIN;
    GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IN;  //杈撳叆
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP;  //涓婃媺杈撳叆
    GPIO_Init(PZ_SWITCH_DETEC_PORT, &GPIO_InitStructure);
    
    //鎬ュ仠寮�鍏虫娴嬪紩鑴�  妫�娴嬩綆鐢靛钩
    GPIO_InitStructure.GPIO_Pin = EMERSTOP_SWITCH_DETEC_PIN;
    GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IN;  //杈撳叆
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP;  //涓嬫媺杈撳叆
    GPIO_Init(EMERSTOP_SWITCH_DETEC_PORT, &GPIO_InitStructure);
    
	
	BJBEEP =0;//鎶ヨ鐏渹楦ｅ櫒
	BJRED=0;
	BJYELLOW=0;
	BJGREEN =0;
	YUNTAI=1;
}



















