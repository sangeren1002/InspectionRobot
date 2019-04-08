#ifndef __BSP_GPIO_H
#define __BSP_GPIO_H	 
#include "sys.h" 
#include "main.h"

/* 宏定义 --------------------------------------------------------------------*/
#if BOARD_SELECTION == 1//使用正点原子开发板
#define LED_BSP_CLK_ENABLE()            RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF,ENABLE) //板载LED灯时钟使能
#define LED_BSP_PORT                    GPIOF                  // 板载LED灯接口PORTD
#define LED_SUCCESS_BSP_PIN             GPIO_Pin_9            // 板载LED状态正常指示灯管脚号
#define LED_ERROR_BSP_PIN               GPIO_Pin_10            // 板载LED状态异常指示灯管脚号

#define YUNTAI_CLK_ENABLE()             RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG,ENABLE)    //云台使能时钟使能
#define YUNTAI_PORT                     GPIOG                   // 云台使能信号接口PORTD
#define YUNTAI_PIN                      GPIO_Pin_9              // 云台使能信号管脚号

#define CMODE_CLK_ENABLE()              RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG,ENABLE) //手自动模式时钟使能
#define CMODE_PORT                      GPIOG                   // 手自动模式转换接口PORTD
#define CMODE_PIN                       GPIO_Pin_4              // 手自动模式转换管脚号

#define BJ_CLK_ENABLE()                 RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG,ENABLE)    //报警灯时钟使能
#define BJ_PORT                         GPIOG                   // 报警灯接口PORTD
#define BJGREEN_PIN                     GPIO_Pin_5              // 报警灯绿色管脚号
#define BJRED_PIN                       GPIO_Pin_6              // 报警灯红色管脚号
#define BJBUZZER_PIN                    GPIO_Pin_10              // 报警灯蜂鸣器管脚号
#define BJYELLOW_PIN                    GPIO_Pin_7              // 报警灯黄色管脚号

#define PZ_SWITCH_CLK_ENABLE()          RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE,ENABLE)    //碰撞开关时钟使能
#define PZ_SWITCH_DETEC_PORT            GPIOE                   // 碰撞开关检测接口PORTD
#define PZ_SWITCH_DETEC_PIN             GPIO_Pin_7              // 碰撞开关检测管脚号

#define EMERSTOP_CLK_ENABLE()           RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE,ENABLE)    //急停按钮时钟使能
#define EMERSTOP_SWITCH_DETEC_PORT      GPIOE                   // 急停按钮检测接口PORTD
#define EMERSTOP_SWITCH_DETEC_PIN       GPIO_Pin_8              // 急停按钮检测管脚号

#define LED0 PFout(9)	// DS0
#define LED1 PFout(10)	// DS1	
#define BJBEEP PGout(10) //报警灯蜂鸣器
#define BJRED PGout(6) //报警灯红色
#define BJYELLOW PGout(7) //报警灯黄色
#define BJGREEN PGout(5) //报警灯绿色
#define YUNTAI PGout(9) //报警灯绿色
#define AUTOMODE PGin(4) //手自动模式切换
#define PZ_CHECK PEin(7) //碰撞开关检测
#define EMERSTOP PEin(8) //碰撞开关检测


#elif BOARD_SELECTION == 2 //使用LIUGC201809 第一版

#define LED_BSP_CLK_ENABLE()            RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG,ENABLE) //板载LED灯时钟使能
#define LED_BSP_PORT                    GPIOG                  // 板载LED灯接口PORTD
#define LED_SUCCESS_BSP_PIN             GPIO_Pin_13            // 板载LED状态正常指示灯管脚号
#define LED_ERROR_BSP_PIN               GPIO_Pin_14            // 板载LED状态异常指示灯管脚号

#define YUNTAI_CLK_ENABLE()             RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE,ENABLE)    //云台使能时钟使能
#define YUNTAI_PORT                     GPIOE                   // 云台使能信号接口PORTD
#define YUNTAI_PIN                      GPIO_Pin_1              // 云台使能信号管脚号

#define CMODE_CLK_ENABLE()              RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG,ENABLE) //手自动模式时钟使能
#define CMODE_PORT                      GPIOG                   // 手自动模式转换接口PORTD
#define CMODE_PIN                       GPIO_Pin_4              // 手自动模式转换管脚号

#define BJ_CLK_ENABLE()                 RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG,ENABLE)    //报警灯时钟使能
#define BJ_PORT                         GPIOG                   // 报警灯接口PORTD
#define BJGREEN_PIN                     GPIO_Pin_2              // 报警灯绿色管脚号
#define BJRED_PIN                       GPIO_Pin_3              // 报警灯红色管脚号
#define BJBUZZER_PIN                    GPIO_Pin_5              // 报警灯蜂鸣器管脚号
#define BJYELLOW_PIN                    GPIO_Pin_6              // 报警灯黄色管脚号

#define PZ_SWITCH_CLK_ENABLE()          RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE,ENABLE)    //碰撞开关时钟使能
#define PZ_SWITCH_DETEC_PORT            GPIOE                   // 碰撞开关检测接口PORTD
#define PZ_SWITCH_DETEC_PIN             GPIO_Pin_7              // 碰撞开关检测管脚号

#define EMERSTOP_CLK_ENABLE()           RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE,ENABLE)    //急停按钮时钟使能
#define EMERSTOP_SWITCH_DETEC_PORT      GPIOE                   // 急停按钮检测接口PORTD
#define EMERSTOP_SWITCH_DETEC_PIN       GPIO_Pin_8              // 急停按钮检测管脚号

#define LED0 PGout(13)	// DS0
#define LED1 PGout(14)	// DS1	

#define BJBEEP PGout(5) //报警灯蜂鸣器
#define BJRED PGout(3) //报警灯红色
#define BJYELLOW PGout(6) //报警灯黄色
#define BJGREEN PGout(2) //报警灯绿色
#define YUNTAI PEout(1) //报警灯绿色
#define AUTOMODE PGin(4) //手自动模式切换
#define PZ_CHECK PEin(7) //碰撞开关检测
#define EMERSTOP PEin(8) //碰撞开关检测



#elif BOARD_SELECTION == 3 //使用LIUGC20190331 第二版

#define LED_BSP_CLK_ENABLE()            RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE) //板载LED灯时钟使能
#define LED_BSP_PORT                    GPIOA                 	// 板载LED灯接口PORTD
#define LED_SUCCESS_BSP_PIN             GPIO_Pin_11            // 板载LED状态正常指示灯管脚号
#define LED_ERROR_BSP_PIN               GPIO_Pin_12            // 板载LED状态异常指示灯管脚号

#define YUNTAI_CLK_ENABLE()             RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF,ENABLE)    //云台使能时钟使能
#define YUNTAI_PORT                     GPIOF                   // 云台使能信号接口PORTD
#define YUNTAI_PIN                      GPIO_Pin_15              // 云台使能信号管脚号

#define CMODE_CLK_ENABLE()              RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG,ENABLE) //手自动模式时钟使能
#define CMODE_PORT                      GPIOG                   // 手自动模式转换接口PORTD
#define CMODE_PIN                       GPIO_Pin_6              // 手自动模式转换管脚号

#define BJ_CLK_ENABLE()                 RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG,ENABLE)    //报警灯时钟使能
#define BJ_PORT                         GPIOG                   // 报警灯接口PORTD
#define BJGREEN_PIN                     GPIO_Pin_5              // 报警灯绿色管脚号
#define BJRED_PIN                       GPIO_Pin_4              // 报警灯红色管脚号
#define BJBUZZER_PIN                    GPIO_Pin_3              // 报警灯蜂鸣器管脚号
#define BJYELLOW_PIN                    GPIO_Pin_2              // 报警灯黄色管脚号

#define PZ_SWITCH_CLK_ENABLE()          RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE,ENABLE)    //碰撞开关时钟使能
#define PZ_SWITCH_DETEC_PORT            GPIOE                   // 碰撞开关检测接口PORTD
#define PZ_SWITCH_DETEC_PIN             GPIO_Pin_7              // 碰撞开关检测管脚号

#define EMERSTOP_CLK_ENABLE()           RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE,ENABLE)    //急停按钮时钟使能
#define EMERSTOP_SWITCH_DETEC_PORT      GPIOE                   // 急停按钮检测接口PORTD
#define EMERSTOP_SWITCH_DETEC_PIN       GPIO_Pin_8              // 急停按钮检测管脚号

#define LED0 PAout(11)	// DS0
#define LED1 PAout(12)	// DS1	
#define BJBEEP PGout(3) //报警灯蜂鸣器
#define BJRED PGout(4) //报警灯红色
#define BJYELLOW PGout(2) //报警灯黄色
#define BJGREEN PGout(5) //报警灯绿色
#define YUNTAI PFout(15) //报警灯绿色
#define AUTOMODE PGin(6) //手自动模式切换
#define PZ_CHECK PEin(7) //碰撞开关检测
#define EMERSTOP PEin(8) //碰撞开关检测

#endif

#define KEY0_PRES 	1
#define KEY1_PRES	2
#define KEY2_PRES	3
#define WKUP_PRES   4

 
//#define AUTOMODE PGin(4) //手自动模式切换
//#define PZ_CHECK PEin(7) //碰撞开关检测
//#define EMERSTOP PEin(8) //碰撞开关检测

//#define BEEP PFout(8) //板载蜂鸣器

//#define BJBEEP PGout(5) //报警灯蜂鸣器
//#define BJRED PGout(3) //报警灯红色
//#define BJYELLOW PGout(6) //报警灯黄色
//#define BJGREEN PGout(2) //报警灯绿色
//#define YUNTAI PEout(1) //报警灯绿色



#define KEY0 		PEin(4)   	//PE4
#define KEY1 		PEin(3)		//PE3 
#define KEY2 		PEin(2)		//P32
#define WK_UP 	PAin(0)		//PA0







void BSP_GPIO_Init(void); //板载蜂鸣器及外接报警灯初始化

void KEY_Init(void);	//IO��ʼ��
u8 KEY_Scan(u8);  		//����ɨ�躯��	

#endif
