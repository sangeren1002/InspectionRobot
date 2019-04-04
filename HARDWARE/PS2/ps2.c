#include "ps2.h" 
#include "delay.h"


s16 swe_sped[4]={0};//转弯速度

u16 Handkey;
u8 Comd[2]={0x01,0x42};	//开始指令，请求数据
u8 Data[9]={0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}; //数据存储数组
u16 MASK[]={
    PSB_SELECT,
    PSB_L3,
    PSB_R3 ,
    PSB_START,
    PSB_PAD_UP,
    PSB_PAD_RIGHT,
    PSB_PAD_DOWN,
    PSB_PAD_LEFT,
    PSB_L2,
    PSB_R2,
    PSB_L1,
    PSB_R1 ,
    PSB_GREEN,
    PSB_RED,
    PSB_BLUE,
    PSB_PINK
};	//按键值与按键名

/*****************************************************************************
 * 函 数 名  : PS2_Init
 * 负 责 人  : jishubao
 * 创建日期  : 2018年11月19日
 * 函数功能  : 手柄硬件初始化
 * 输入参数  : void  无
 * 输出参数  : 无
 * 返 回 值  : 
 * 调用关系  : 
 * 其    它  : 

*****************************************************************************/
void PS2_Init(void)
{
  GPIO_InitTypeDef  GPIO_InitStructure;

  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);//使能GPIOF时钟

  //
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//普通输出模式
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;//100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;//下拉
  GPIO_Init(GPIOB, &GPIO_InitStructure);//初始化
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;//普通输入模式
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;//100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;//下拉
  GPIO_Init(GPIOB, &GPIO_InitStructure);//初始化
	
	GPIO_ResetBits(GPIOB,GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15);//初始状态为低电平

}

//向手柄发送命令
void PS2_Cmd(u8 CMD)
{
	volatile u16 ref=0x01;
	Data[1] = 0;
	for(ref=0x01;ref<0x0100;ref<<=1)
	{
		if(ref&CMD)
		{
			DO_H;                   //输出高电平
		}
		else DO_L;

		CLK_H;                        //时钟拉高
		delay_us(50);
		CLK_L;
		delay_us(50);
		CLK_H;
		if(DI)
			Data[1] = ref|Data[1];
	}
}

//判断是否为红灯模式
//返回值：
//			0，红灯模式
//		  其他，其他模式
u8 PS2_RedLight(void)
{
	CS_L;
	PS2_Cmd(Comd[0]);  //开始命令
	PS2_Cmd(Comd[1]);  //请求数据
	CS_H;
	if( Data[1] == 0X73)   
		return 0 ;
	else return 1;

}
//读取手柄数据
void PS2_ReadData(void)
{
	volatile u8 byte=0;
	volatile u16 ref=0x01;

	CS_L;

	PS2_Cmd(Comd[0]);  //开始命令
	PS2_Cmd(Comd[1]);  //请求数据

	for(byte=2;byte<9;byte++)          //开始接受数据
	{
		for(ref=0x01;ref<0x100;ref<<=1)
		{
			CLK_H;
			delay_us(50);
			CLK_L;
			delay_us(50);
			CLK_H;
		      if(DI)
		      Data[byte] = ref|Data[byte];
		}
        delay_us(50);
	}
	CS_H;	
}

//对读出来的PS2数据进行处理，只处理按键部分  默认数据是红灯模式   只有一个按键按下时
//按下为0， 未按下为1
u8 PS2_DataKey()
{
	u8 index;

	PS2_ClearData();
	PS2_ReadData();

	Handkey=(Data[4]<<8)|Data[3];     //这是16个按键，按下为0，未按下为1
	for(index=0;index<16;index++)
	{	    
		if((Handkey&(1<<(MASK[index]-1)))==0)
		return index+1;
	}
	return 0;          //没有任何按键按下
}

//得到一个遥杆的模拟量 范围0~256
u8 PS2_AnologData(u8 button)
{
	return Data[button];
}

//清除数据缓存区
void PS2_ClearData()
{
	u8 a;
	for(a=0;a<9;a++)
		Data[a]=0x00;
}
/******************************************************
Function:    void PS2_Vibration(u8 motor1, u8 motor2)
Description: 手柄震动函数
Calls:		 void PS2_Cmd(u8 CMD);
Input: motor1:右侧小震动电机 0x00，关，其他，开
	   motor2:左侧大震动电机 0x40~0xFF 电机开，值越大，震动越大
******************************************************/
void PS2_Vibration(u8 motor1, u8 motor2)
{
	CS_L;
	delay_us(16);
    PS2_Cmd(0x01);  //开始命令
	PS2_Cmd(0x42);  //请求数据
	PS2_Cmd(0X00);
	PS2_Cmd(motor1);
	PS2_Cmd(motor2);
	PS2_Cmd(0X00);
	PS2_Cmd(0X00);
	PS2_Cmd(0X00);
	PS2_Cmd(0X00);
	CS_H;
	delay_us(16);  
}
//short poll
void PS2_ShortPoll(void)
{
	CS_L;
	delay_us(16);
	PS2_Cmd(0x01);  
	PS2_Cmd(0x42);  
	PS2_Cmd(0X00);
	PS2_Cmd(0x00);
	PS2_Cmd(0x00);
	CS_H;
	delay_us(16);	
}
//进入配置
void PS2_EnterConfing(void)
{
    CS_L;
	delay_us(16);
	PS2_Cmd(0x01);  
	PS2_Cmd(0x43);  
	PS2_Cmd(0X00);
	PS2_Cmd(0x01);
	PS2_Cmd(0x00);
	PS2_Cmd(0X00);
	PS2_Cmd(0X00);
	PS2_Cmd(0X00);
	PS2_Cmd(0X00);
	CS_H;
	delay_us(16);
}
//发送模式配置
void PS2_TurnOnAnalogMode(void)
{
	CS_L;
	PS2_Cmd(0x01);  
	PS2_Cmd(0x44);  
	PS2_Cmd(0X00);
	PS2_Cmd(0x00); //analog=0x01;digital=0x00  软件设置为发送模式
	PS2_Cmd(0xEE); //Ox03锁存设置 即不可通过按键“MODE”设置模式
								 //0xEE不锁存设置 可通过按键“MODE”设置模式
	PS2_Cmd(0X00);
	PS2_Cmd(0X00);
	PS2_Cmd(0X00);
	PS2_Cmd(0X00);
	CS_H;
	delay_us(16);
}
//震动设置
void PS2_VibrationMode(void)
{
	CS_L;
	delay_us(16);
	PS2_Cmd(0x01);  
	PS2_Cmd(0x4D);  
	PS2_Cmd(0X00);
	PS2_Cmd(0x00);
	PS2_Cmd(0X01);
	CS_H;
	delay_us(16);	
}
//完成并保存设置
void PS2_ExitConfing(void)
{
    CS_L;
	delay_us(16);
	PS2_Cmd(0x01);  
	PS2_Cmd(0x43);  
	PS2_Cmd(0X00);
	PS2_Cmd(0x00);
	PS2_Cmd(0x5A);
	PS2_Cmd(0x5A);
	PS2_Cmd(0x5A);
	PS2_Cmd(0x5A);
	PS2_Cmd(0x5A);
	CS_H;
	delay_us(16);
}
//手柄初始化
void PS2_SetInit(void)
{
	PS2_ShortPoll();
	PS2_ShortPoll();
	PS2_ShortPoll();
	PS2_EnterConfing();		//进入配置模式
	PS2_TurnOnAnalogMode();	//“红绿灯”配置模式，并选择保存
//	PS2_VibrationMode();	//开启震动模式
	PS2_ExitConfing();		//保存并保存配置
}


