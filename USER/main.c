/***********************************************************************************
 * 文 件 名   : main.c
 * 负 责 人   : jishubao
 * 创建日期   : 2018年11月19日
 * 文件描述   : 巡检机器人主函数
 * 版权说明   : Copyright (c) 2008-2018   杭州国辰机器人科技有限公司
 * 其    他   : 
 * 修改日志   : 
***********************************************************************************/
#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "key.h"
#include "CANOpenObjDictConfig.h"
#include "can_stm32.h"
#include "motor_control.h"
#include "ps2.h"
#include "usart3.h"
#include "timer.h"
#include "rs485.h"
#include "main.h"
#include "stdlib.h"
#include "infrared.h" 
u8 AbnormalReporting(u8 type,u8 group,u8 number);
char recvfromPCbuf[USART_REC_LEN];//上位机通信串口接收缓存
extern BattaryMSG battarymsg;
#define DEBUG_DATA_UP_LOAD
#define DEBUG_PROTECT_IO  0 //0为取消急停等保护开关，1为打开急停等保护开关
#define INSPECTIONROBOT_CODE_V2
//u16 num2timer=0;
/*****************************************************************************
 * 函 数 名  : main
 * 负 责 人  : jishubao
 * 创建日期  : 2018年11月21日
 * 函数功能  : 主函数
 * 输入参数  : void  无
 * 输出参数  : 无
 * 返 回 值  : 
 * 调用关系  : 
 * 其    它  : 

*****************************************************************************/
int main(void)
{ 
	static u16 cnt = 0;
	SYS_Init();

	while(1)
	{   
        canSend(CAN1,&Can_Msg_SYNCMY);		
	  	delay_ms(10);

//	Set_Speed((float)-10.0,(float)10.0);	//设置左右电机速度
        ModeSelection();
         cnt++;

        if(cnt%20==0)LED0=!LED0;
	}
	
}
u8 AbnormalReporting(u8 type,u8 group,u8 number)
{
		if((type == 1)&&(group == 1||group == 2||group == 3||group == 4||group == 5||group == 6||group == 7||group == 8||group == 9||group == 10))
		{
			Set_Alarm(2);		//设置三色灯状态为黄色
		}
		if((type == 2)&&(group == 1))
		{
			Set_Alarm(1);		//设置三色灯状态为红色
		}
	errcode[0] = type;
	errcode[1] = group;
	errcode[2] = number;
#ifdef DEBUG_DATA_UP_LOAD
	Date_Up_Load(ERROR_STATUS_CMD);
#endif

	return 0;
}
/*****************************************************************************
 * 函 数 名  : SYS_Init
 * 负 责 人  : jishubao
 * 创建日期  : 2018年11月19日
 * 函数功能  : 系统硬件初始化
 * 输入参数  : void  无参数
 * 输出参数  : 无
 * 返 回 值  : 
 * 调用关系  : 
 * 其    它  : 

*****************************************************************************/
void SYS_Init(void)
{
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//设置系统中断优先级分组2
	delay_init(168);    //初始化延时函数
	uart_init(115200);	//初始化串口波特率为15200
    usart3_init(115200); 
#if DEFINE_TEST_FRE
    TIM3_Int_Init(50000-1,8400-1);
#endif    
    BSP_GPIO_Init();
	MYRS485_Init(9600);		//初始化RS485串口2	
//	Adc_Init();
    delay_ms(5000);
	CanopenInit();//CANopen初始化

    SetCanopenParameter(0);
    SetCanopenParameter(1);

    MotorInit();
    PS2_Init(); 
#if DEBUG_PROTECT_IO
	RequestBatteryInformation(&battarymsg);
	while(PZ_CHECK == 0)
	{
		MotionHalts();
		AbnormalReporting(0x02,0x01,0x02);//上报异常	
	}
	while(EMERSTOP == 1)
	{
		MotionHalts();
		AbnormalReporting(0x02,0x01,0x01);//上报异常
	}
	if(PZ_CHECK == 1&&EMERSTOP == 0)
	{
		Set_Alarm(4);
	}
 #endif
}
/*****************************************************************************
 * 函 数 名  : GetValStatus
 * 负 责 人  : jishubao
 * 创建日期  : 2018年11月21日
 * 函数功能  : 将驱动器上报报文状态字数据保存在状态字缓存区
 * 输入参数  : u8 idnode  驱动器ID
 * 输出参数  : 无
 * 返 回 值  : 
 * 调用关系  : 
 * 其    它  : 

*****************************************************************************/
void GetValStatus(u8 idnode)
{
	if(idnode==1)
	{
		status_word_left[0]  = RxMSG.data[1];
		status_word_left[1]  = RxMSG.data[0];
	}
	if(idnode==2)
	{
		status_word_right[0]  = RxMSG.data[1];
		status_word_right[1]  = RxMSG.data[0];
	}
	if(RxMSG.data[1]&0x08)//bit3表示状态出现错误
		{;
		}	
}

/*****************************************************************************
 * 函 数 名  : GetValCurrent
 * 负 责 人  : jishubao
 * 创建日期  : 2018年11月21日
 * 函数功能  : 将驱动器上报报文电流数据保存在电流缓存             区
 * 输入参数  : u8 idnode  驱动器ID
 * 输出参数  : 无
 * 返 回 值  : 
 * 调用关系  : 
 * 其    它  : 

*****************************************************************************/
void GetValCurrent(u8 idnode)
{
	if(idnode==1)
	{

		current_valueleft[0] = RxMSG.data[3];
		current_valueleft[1] = RxMSG.data[2];
	}
	if(idnode==2)
	{
		current_valueright[0] = RxMSG.data[3];
		current_valueright[1] = RxMSG.data[2];
	}
}
/*****************************************************************************
 * 函 数 名  : GetValEncoder
 * 负 责 人  : jishubao
 * 创建日期  : 2018年11月21日
 * 函数功能  : 将驱动器上报报文编码器数据保存在编码器缓存区
 * 输入参数  : u8 idnode  驱动器ID
 * 输出参数  : 无
 * 返 回 值  : 
 * 调用关系  : 
 * 其    它  : 

*****************************************************************************/
void GetValEncoder(u8 idnode)
{
	if(idnode==1)
	{
		encoder_valueleft[0]=RxMSG.data[0];
		encoder_valueleft[1]=RxMSG.data[1];
		encoder_valueleft[2]=RxMSG.data[2];
		encoder_valueleft[3]=RxMSG.data[3];
	}
	if(idnode==2)
	{
		encoder_valueright[0]=RxMSG.data[0];
		encoder_valueright[1]=RxMSG.data[1];
		encoder_valueright[2]=RxMSG.data[2];
		encoder_valueright[3]=RxMSG.data[3];
	}

}

/*****************************************************************************
 * 函 数 名  : GetValVelocity
 * 负 责 人  : jishubao
 * 创建日期  : 2018年11月21日
 * 函数功能  : 将驱动器上传电机速度转换成轮子转速
 * 输入参数  : u8 idnode  驱动器ID
 * 输出参数  : 无
 * 返 回 值  : 
 * 调用关系  : 
 * 其    它  : 

*****************************************************************************/
void GetValVelocity(u8 idnode)
{
  s16 valueright,valueleft;
	float right,left;

	if(idnode==1)
	{
			memcpy(&valueleft, &RxMSG.data[4], sizeof(valueleft));
        /* 下位机直接反馈电机转速给上位机，不需要计算整车速度（可以不用考虑轮子直径标定值） */
            left = (float)valueleft/SPEED_COEFFICIENT;//(REDUCTION_RATIO*SPEED_COEFFICIENT);      //REDUCTION_RATIO 减速比28    SPEED_COEFFICIENT 电机转速系数 240  
			memcpy(velocity_valueleft, &left, sizeof(left));          
        /* 下位机根据轮子直径和当前转速得到车子整体速度 */
//			left = (float)valueleft*785/(240*28*1000);//velocity_valueright/240得到电机速度rps，除以减速比28得到1s内轮子转的圈数再乘圆周0.785m得到整车速度单位m/s
//			memcpy(velocity_valueleft, &left, sizeof(left));

	}
	if(idnode==2)
	{
		  memcpy(&valueright, &RxMSG.data[4], sizeof(valueright));
         /* 下位机直接反馈电机转速给上位机，不需要计算整车速度（可以不用考虑轮子直径标定值） */
            right = (float)valueright/SPEED_COEFFICIENT;//(REDUCTION_RATIO*SPEED_COEFFICIENT);
            memcpy(velocity_valueright, &right, sizeof(right));
        /* 下位机根据轮子直径和当前转速得到车子整体速度 */
//			right = (float)valueright*785/(240*28*1000);//velocity_valueright/240得到电机速度rps，除以减速比28得到1s内轮子转的圈数再乘圆周0.785m得到整车速度单位m/s
//			memcpy(velocity_valueright, &right, sizeof(right));
	}
}


/*****************************************************************************
 * 函 数 名  : AnalysisMessagefromDriver
 * 负 责 人  : jishubao
 * 创建日期  : 2018年11月21日
 * 函数功能  : 分析驱动器上报报文，得到左右轮编码器值-
                   ，速度值，状态字，电流值
 * 输入参数  : void  无
 * 输出参数  : 无
 * 返 回 值  : 
 * 调用关系  : 
 * 其    它  : 

*****************************************************************************/
void AnalysisMessagefromDriver(void)
{
	 uint32_t left_receivecnt = 0 ,right_receivecnt = 0;
    switch (RxMSG.cob_id)
    {
        case 0X0281://1号驱动器编码器及速度值报文
                    GetValEncoder(1);
                    GetValVelocity(1);
										right_receivecnt++;
            break;
        case 0X0381://1号驱动器状态字及电流值报文
                    GetValStatus(1);
                    GetValCurrent(1);
										right_receivecnt++;
            break;
        case 0X0282://2号驱动器编码器及速度值报文
                    GetValEncoder(2);
                    GetValVelocity(2);
										left_receivecnt++;
            break;
        case 0X0382://2号驱动器状态字及电流值报文
                    GetValStatus(2);
                    GetValCurrent(2);
										right_receivecnt++;
            break;
		case 0X0581:	//其他报文
		case 0X0582:
					if(RxMSG.data[0]!=0x60)
						startcmdstatus=1;
			break;

        default:
            break;            
    }
		if(abs(left_receivecnt-right_receivecnt)>10)//如果左右轮的返回值反馈时间差较大认为单轮失联
			{
				//单轮失联控制逻辑
				MotionHalts();//左右电机立即停止;
				AbnormalReporting(0x01,0x00,0x02);//上报异常
			}		
}


#define HEARTBEATTIME 50 //心跳超时时间，单位10ms
void HeartbeatDetection(uint16_t *time)
{
	if( *time > HEARTBEATTIME)//下位机与上位机失去连接
	{
		MotionHalts();//左右电机立即停止
		*time = 0;
		AbnormalReporting(1,1,1);

	}
}
 extern u16 timecnt;
u16 hw_dis[4]= {0};
/*****************************************************************************
 * 函 数 名  : ModeSelection
 * 负 责 人  : jishubao
 * 创建日期  : 2018年11月19日
 * 函数功能  : 手自动模式选择判断函数
 * 输入参数  : void  无
 * 输出参数  : 无
 * 返 回 值  : 
 * 调用关系  : 
 * 其    它  : 

*****************************************************************************/
void ModeSelection(void)
{
     static u32 cnt=0;
	static uint16_t Heartbeat = 0;
	Get_Swerve_Speed_PS2(); //处于手动遥控模式下
		LED1=!LED1;
#if 0	  
	if(AUTOMODE==0) //串口3接收到数据并处在自动模式下
	{      
        Date_Up_Load(UPLOAD_ENCODER_AND_VELOCITY); //封装编码器&速度数据并上传

        if(cnt%2==0)
        {
            Date_Up_Load(UPLOAD_STATUS_AND_CURRENT); //封装状态字&电流数据并上传
#if FRE_UPLOAD_DATA 					
        /* 计算上传频率 */	
		timecnt = TIM3->CNT;
		printf("\r\n[UPLOAD_ENCODER_AND_VELOCITY] cycle: %d ms  fre:%f\r\n",timecnt,1000.0f/timecnt);
        TIM3->CNT = 0; 
#endif
            StatusWordAnalysis();
			LED1=!LED1;
        }
        if(USART_RX_STA>>15)
        {
            memcpy(recvfromPCbuf,USART_RX_BUF,USART_REC_LEN);
            PC_Data_Analysis(recvfromPCbuf);
            USART_RX_STA=0;
						Heartbeat = 0;
        }
#if DEBUG_BATTERY
		if(cnt%225==0)//(100*60*10)) 30000大概上传周期为 400s周期
		{
			RequestBatteryInformation(&battarymsg);
			Date_Up_Load(BATTARY_MONITOR_MSGID_CMD);
		}
#endif
#if DEBUG_INFRARED_DIS
		if(cnt%30==0)//(100*60*10)) 30 大概上传周期为0.3s 
		{
			Get_Distance_Infrared(hw_dis);
			Date_Up_Load(INFRARED_DISTANCE_CMD);
		}
#endif
		HeartbeatDetection(&Heartbeat);
		cnt++;
	}
	else if(AUTOMODE==1)
	{
//		printf("\r\nAUTOMODE ing...\r\n");
		Get_Swerve_Speed_PS2(); //处于手动遥控模式下
		LED1=!LED1;
	}
#if DEBUG_PROTECT_IO
	while(PZ_CHECK == 0)
	{
		MotionHalts();
		AbnormalReporting(0x02,0x01,0x02);//上报异常	
	}
	while(EMERSTOP == 1)
	{
		MotionHalts();
		AbnormalReporting(0x02,0x01,0x01);//上报异常
	}
	if(PZ_CHECK == 1&&EMERSTOP == 0)
	{
		Set_Alarm(4);
	}
#endif
	Heartbeat++;
#endif
}
/* delete begin by jishubao, 2018-11-21, Mantis号:2*/
#if 0 //下位机调试时测试驱动器功能时使用
u8 spedl[4]={0xCD, 0xCC,0x4C,0x3E};  //浮点型速度0.2m/s
u8 spedr[4]={0x00, 0x00,0x00,0x00};  //浮点型速度0.2m/s
#endif

/* delete end by jishubao, 2018-11-21 */


/*****************************************************************************
 * 函 数 名  : PC_Data_Analysis
 * 负 责 人  : jishubao
 * 创建日期  : 2018年11月21日
 * 函数功能  : 上位机命令分析
 * 输入参数  : char *recvfromPCbuf  从上位机接收命令室的缓存首地址
 * 输出参数  : 无
 * 返 回 值  : 
 * 调用关系  : 
 * 其    它  : 

*****************************************************************************/
void PC_Data_Analysis(char *recvfromPCbuf)
{
    float vall = 0,valr = 0;
	u16 crccheck = 0;
    u16 temp_crc = 0;

	
    if((recvfromPCbuf[0]==0x55) && (recvfromPCbuf[1]==0xaa)) //帧头2字节
    {
			length = recvfromPCbuf[4];		//得到字长度  
			memcpy(&temp_crc, &recvfromPCbuf[2], sizeof(temp_crc)); //得到modbus crc-16校验位
			crccheck = GetCRC16((u8*)&recvfromPCbuf[6],length);		//计算当前数据包CRC校验值
    	if(crccheck!= temp_crc)			//判断CRC校验位是否正确
			{
				return;
			}	
        LED0 = !LED0; 
        switch (recvfromPCbuf[5])       //判断MsgID
        {
            case MOTOR_CTL_MSGID_CMD:  				//电机控制指令 0x01
								memcpy(&vall, &recvfromPCbuf[6], sizeof(vall));			//得到上位机命令的左电机速度
								memcpy(&valr, &recvfromPCbuf[10], sizeof(valr));		//得到上位机命令的右电机速度
								Set_Speed((float)vall,(float)valr);	//设置左右电机速度              
                break;
						case BATTARY_MONITOR_MSGID_CMD: 		//电池信息指令 0x02
								RequestBatteryInformation(&battarymsg);     				//得到电池信息
								Date_Up_Load(BATTARY_MONITOR_MSGID_CMD);
								break;	
//			case OPEN_YUNTAI_CMD:					//请求开启云台 0x07
//				Set_Yuntai(recvfromPCbuf[6]);  		//设置云台状态
//				break;
						case ALARM_CMD: 						//请求开启报警三色灯 0x08
							Set_Alarm(recvfromPCbuf[6]);		//设置三色灯状态
							break;
//            case ENABLE_CHARGE_CMD:							//无线充电使能指令 0x09
//           		Wireless_Charging_Status(recvfromPCbuf[6]);//设置无线充电状态  
//				break;
						default:
							break;
        }
    }

}

 
 
 
 
 
