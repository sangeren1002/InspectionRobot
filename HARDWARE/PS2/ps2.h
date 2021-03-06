#ifndef __PS2_H
#define __PS2_H
#include "sys.h"

#define LIU_201903_PS2 1

#if LIU_201903_PS2
#define PS2_CLK_ENABLE()           	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC|RCC_AHB1Periph_GPIOA,ENABLE)    //急停按钮时钟使能
#define PS2_PORTA      				GPIOA                  // 接口PORTD
#define PS2_PORTC     				GPIOC                  // 接口PORTD
#define PS2_DI_PIN       			GPIO_Pin_3             // 
#define PS2_DO_PIN       			GPIO_Pin_9              // 
#define PS2_CS_PIN       			GPIO_Pin_6              // 
#define PS2_CLK_PIN       			GPIO_Pin_7              // 

#define DI   PAin(3)           //PB12  输入

#define DO_H PCout(9)=1        //命令位高
#define DO_L PCout(9)=0        //命令位低

#define CS_H PCout(6)=1       //CS拉高
#define CS_L PCout(6)=0       //CS拉低

#define CLK_H PCout(7)=1      //时钟拉高
#define CLK_L PCout(7)=0      //时钟拉低

#else
#define PS2_CLK_ENABLE()           	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE)    //急停按钮时钟使能
#define PS2_PORT      				GPIOB                  // 接口PORTD
#define PS2_DI_PIN       			GPIO_Pin_12             // 
#define PS2_DO_PIN       			GPIO_Pin_13              // 
#define PS2_CS_PIN       			GPIO_Pin_14              // 
#define PS2_CLK_PIN       			GPIO_Pin_15              // 

#define DI   PBin(12)           //PB12  输入

#define DO_H PBout(13)=1        //命令位高
#define DO_L PBout(13)=0        //命令位低

#define CS_H PBout(14)=1       //CS拉高
#define CS_L PBout(14)=0       //CS拉低

#define CLK_H PBout(15)=1      //时钟拉高
#define CLK_L PBout(15)=0      //时钟拉低
#endif

//These are our button constants
#define PSB_SELECT      1
#define PSB_L3          2
#define PSB_R3          3
#define PSB_START       4
#define PSB_PAD_UP      5
#define PSB_PAD_RIGHT   6
#define PSB_PAD_DOWN    7
#define PSB_PAD_LEFT    8
#define PSB_L2         9
#define PSB_R2          10
#define PSB_L1          11
#define PSB_R1          12
#define PSB_GREEN       13
#define PSB_RED         14
#define PSB_BLUE        15
#define PSB_PINK        16
#define PSB_TRIANGLE    13
#define PSB_CIRCLE      14
#define PSB_CROSS       15
#define PSB_SQUARE      26

//#define WHAMMY_BAR		8

//These are stick values
#define PSS_RX 5                //右摇杆X轴数据
#define PSS_RY 6								//右摇杆Y轴数据
#define PSS_LX 7								//左摇杆X轴数据
#define PSS_LY 8								//左摇杆Y轴数据



extern u8 Data[9];
extern u16 MASK[16];
extern u16 Handkey;
extern s16 swe_sped[4];

void PS2_Init(void);
u8 PS2_RedLight(void);//判断是否为红灯模式
void PS2_ReadData(void);
void PS2_Cmd(u8 CMD);		  //
u8 PS2_DataKey(void);		  //键值读取
u8 PS2_AnologData(u8 button); //得到一个摇杆的模拟量
void PS2_ClearData(void);	  //清除数据缓存区
void PS2_SetInit(void); 



	
#endif


