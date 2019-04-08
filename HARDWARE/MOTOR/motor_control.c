/***********************************************************************************
 * 文 件 名   : motor_control.c
 * 负 责 人   : jishubao
 * 创建日期   : 2018年11月19日
 * 文件描述   : 电机控制函数源文件
 * 版权说明   : Copyright (c) 2008-2018   杭州国辰机器人科技有限公司
 * 其    他   : 
 * 修改日志   : 
***********************************************************************************/

#include "motor_control.h"
#include "sys.h"
#include "CANOpenObjDictConfig.h"
#include "can_stm32.h"
#include "ps2.h"
#include "delay.h"
#include "usart3.h"
#include "usart.h"
#include "key.h"
#include "led.h"
#include "can.h"
#include "timer.h"

 Message Can_Msg_ReadSwitch_NO[2];
 Message Can_Msg_SwitchOn_NO[2];
 Message Can_Msg_OperationEn_NO[2];
 Message Can_Msg_VelocityMode_NO[2];
 Message Can_Msg_TargetVelocity_NO[2];
 Message Can_Msg_SetAcc_NO[2];
 Message Can_Msg_SetDec_NO[2];
 Message Can_Msg_MotionStart_NO[2];
 Message Can_Msg_MotionHalts_NO[2];
 Message Can_Msg_DSPclearAlarm_NO[2];

/* 设置驱动器通讯参数Message */
 Message Can_Msg_TPDOTRANTYPE_NO[2][2];
 Message Can_Msg_ChangeMotionStatusToOperation[2];
 Message Can_Msg_ChangeMotionStatusToPreOperation[2];
 
 /* 设置驱动器映射参数Message */
 Message Can_Msg_Turn_Off_TPDOX[2][2];
 Message Can_Msg_Set_Number_Mapped_objectsTo0[2][2];
 Message Can_Msg_Map_Actual_value_to_TPDOX_objectX[2][4];
 Message Can_Msg_Set_Number_Mapped_objectsTo2[2][2];
 Message Can_Msg_Turn_On_TPDOX[2][2];
 
 //SYNC报文
 Message Can_Msg_SYNCMY;


 
 Message Can_Msg_ActPosition_NO[2];
 Message Can_Msg_ActVelocity_NO[2];
 Message Can_Msg_ActCurrent_NO[2];
 Message Can_Msg_MotionStatus_NO[2];
 

//驱动器TPDO通讯参数设置
UNS8 data_cmd_ChangeTPDOTransType180x[2][8]  = {{0x2F,0x01,0x18,0x02,0x01,0x00,0x00,0x00},	//ChangeTPDOTransType1801  TPDO2
											    {0x2F,0x02,0x18,0x02,0x02,0x00,0x00,0x00}	//ChangeTPDOTransType1802  TPDO3
												}; 
//驱动器节点状态切换为运行状态
UNS8 data_cmd_ChangeMotionStatusToOperation[2][2] = {{0x01,0x01},//将节点1状态切换到运行状态cmd+NodeID
													 {0x01,0x02}//将节点2状态切换到运行状态cmd+NodeID
													};
//驱动器节点状态切换为预运行状态
UNS8 data_cmd_ChangeMotionStatusToPreOperation[2][2] = {{0x80,0x01},//将节点1状态切换到运行状态cmd+NodeID
													 {0x80,0x02}//将节点2状态切换到运行状态cmd+NodeID
													};

//驱动器TPDO映射参数设置
UNS8 data_cmd_Turn_Off_TPDOX[2][8] = {{0x23,0x01,0x18,0x01,0x80,0x02,0x00,0x80},//关闭TPDO1
									  {0x23,0x02,0x18,0x01,0x80,0x03,0x00,0x80}//关闭TPDO2
};
UNS8 data_cmd_Set_Number_Mapped_objectsTo0[2][8] = {{0x2F,0x01,0x1A,0x00,0x00,0x00,0x00,0x00},//设置0X1A01映射对象个数为0
									  				{0x2F,0x02,0x1A,0x00,0x00,0x00,0x00,0x00}//设置0X1A02映射对象个数为0
};
UNS8 data_cmd_Map_Actual_value_to_TPDOX_objectX[4][8] = {{0x23,0x01,0x1A,0x01,0x20,0x00,0x0A,0x70},//设置编码器值映射到TPDO1的子索引01  4Bytes
  						  							  	 {0x23,0x01,0x1A,0x02,0x10,0x00,0x09,0x70},//设置速度值映射到TPDO1的子索引02		 		2Bytes
  						  							  	 {0x23,0x02,0x1A,0x01,0x10,0x00,0x41,0x60},//设置状态字映射到TPDO2的子索引01				2Bytes
  						  							  	 {0x23,0x02,0x1A,0x02,0x10,0x00,0x78,0x60}//设置电流值映射到TPDO2的子索引02		 		    2Bytes
  };
UNS8 data_cmd_Set_Number_Mapped_objectsTo2[2][8] = {{0x2F,0x01,0x1A,0x00,0x02,0x00,0x00,0x00},//设置0X1A01映射对象个数为2
									  				{0x2F,0x02,0x1A,0x00,0x02,0x00,0x00,0x00}//设置0X1A02映射对象个数为2
};
UNS8 data_cmd_Turn_On_TPDOX[2][8] = {{0x23,0x01,0x18,0x01,0x80,0x02,0x00,0x00},//打开TPDO1
									 {0x23,0x02,0x18,0x01,0x80,0x03,0x00,0x00}//打开TPDO2
};





//运行数据报文
UNS8 data_cmd_ReadSwitch[]      = {0x2B,0x40,0x60,0x00,0x06,0x00,0x00,0x00};   //Ready to Switch on
UNS8 data_cmd_SwitchOn[]        = {0x2B,0x40,0x60,0x00,0x07,0x00,0x00,0x00};   //Switched on
UNS8 data_cmd_OperationEn[]     = {0x2B,0x40,0x60,0x00,0x0F,0x01,0x00,0x00};    //Operation Enabled; Motion Halted
UNS8 data_cmd_VelocityMode[]    = {0x2F,0x60,0x60,0x00,0x03,0x00,0x00,0x00};    //Set to Profile Velocity Mode
UNS8 data_cmd_TargetVelocity[]  = {0x23,0xFF,0x60,0x00,0x00,0x00,0x00,0x00};    // Set Target Velocity to 10 rps
//UNS8 data_cmd_SetAcc[]          = {0x23,0x83,0x60,0x00,0x58,0x02,0x00,0x00};    //Set Acceleration to 100 rps/s
//UNS8 data_cmd_SetDec[]          = {0x23,0x84,0x60,0x00,0x58,0x02,0x00,0x00};    //Set Deceleration to 100 rps/s
//UNS8 data_cmd_SetAcc[]          = {0x23,0x83,0x60,0x00,0x2C,0x01,0x00,0x00};    //Set Acceleration to 50 rps/s
//UNS8 data_cmd_SetDec[]          = {0x23,0x84,0x60,0x00,0x2C,0x01,0x00,0x00};    //Set Deceleration to 50 rps/s
UNS8 data_cmd_SetAcc[]          = {0x23,0x83,0x60,0x00,0xA4,0x01,0x00,0x00};    //Set Acceleration to 50 rps/s
UNS8 data_cmd_SetDec[]          = {0x23,0x84,0x60,0x00,0xA4,0x01,0x00,0x00};    //Set Deceleration to 50 rps/s
UNS8 data_cmd_MotionStart[]     = {0x2B,0x40,0x60,0x00,0x0F,0x00,0x00,0x00};   //Motion Starts
//UNS8 data_cmd_MotionHalts[]     = {0x2B,0x40,0x60,0x00,0x0F,0x01,0x00,0x00};   //Motion Halts 
UNS8 data_cmd_MotionHalts[]     = {0x23,0xFF,0x60,0x00,0x00,0x00,0x00,0x00};;   //Motion Halts set velocity to 0

UNS8 data_cmd_GetPosition[]     = {0x43,0x0A,0x70,0x00,0x00,0x00,0x00,0x00};   //Request encoder value
UNS8 data_cmd_GetVelActVal[]    = {0x4B,0x09,0x70,0x00,0x00,0x00,0x00,0x00};   //Request velocity_actual_value; The value reading from driver should be divided 240 to change to rps unit.
UNS8 data_cmd_GetCurActValue[]  = {0x4B,0x78,0x60,0x00,0x00,0x00,0x00,0x00};  //current_actual_value;This object is only available on servo/step-servo drivers.The unit of this object is 0.01Amps
UNS8 data_cmd_MotionStatus[]    = {0x4B,0x41,0x60,0x00,0x00,0x00,0x00,0x00};  //Motion_Status
UNS8 data_cmd_DSPclearAlarm[]   = {0x2F,0x06,0x70,0x00,0x01,0x00,0x00,0x00};  //clear alarm of the drive






UNS16 id_motor_no[2] ={0x0601,0x0602};
UNS8 len = 0x08;
UNS8 rtr_my =0x00;
u8 encoder_valueleft[4] = {0};
u8 encoder_valueright[4] = {0};
u8 velocity_valueleft[4] = {0};
u8 velocity_valueright[4] = {0};
u8 current_valueleft[2] =  {0};
u8 current_valueright[2] =  {0};
u8 status_word_left[2] =  {0};
u8 status_word_right[2] =  {0};
u8 stopflag=0;
u8 startcmdstatus=0;

u8 errcode[3]={0};

u8 length=0;
u8 sequence=0;
u8 ConfigError[3];

//u8 CAN_Send_AndCheckreturnMessage(u8 id, u8 firstdata)
//{
//	
//	if(id ==1)
//	{
//		 while(RxMSG.cob_id!=0X0581&&num<0xffffff)
//		 {
//			 num++;
//		 };
//		 if(RxMSG.data[0]==firstdata)
//		 	return 0;
//		 else
//		 {
//		 	ConfigError[0] = RxMSG.data[3];
//			ConfigError[1] = RxMSG.data[2];
//			ConfigError[2] = RxMSG.data[4];
//			return 1;
//		 }
//		 
//		 
//	}
//}

#define delay_n 5
/*****************************************************************************
 * 函 数 名  : SetMotorPar
 * 负 责 人  : jishubao
 * 创建日期  : 2018年11月19日
 * 函数功能  : 发送驱动器参数至驱动器函数
 * 输入参数  : Message * canmsg  驱动器参数结构体指针
 * 输出参数  : 无
 * 返 回 值  : 
 * 调用关系  : 
 * 其    它  : 

*****************************************************************************/
void SetMotorPar(Message * canmsg)
{
	u8 countsend;
	countsend=0;
	do
	{
		canSend(CAN1,canmsg);  //发送命令
		delay_ms(delay_n);
		countsend++;
		if(countsend>=5)
		{
			startcmdstatus=0;
			errcode[0] =0x01;
			errcode[1] = canmsg->data[2];
			errcode[2] = canmsg->data[3];
			Date_Up_Load(0xff);
			return;
		}
	}while(startcmdstatus);

}
/*****************************************************************************
 * 函 数 名  : MotorInit
 * 负 责 人  : jishubao
 * 创建日期  : 2018年11月19日
 * 函数功能  : 电机CANopen网络软件初始化
 * 输入参数  : void  无
 * 输出参数  : 无
 * 返 回 值  : void
 * 调用关系  : 
 * 其    它  : 

*****************************************************************************/
 void MotorInit(void)
{
	u8 i;
	/* 确保在运行状态意外停止导致驱动器报错 分为三步 */

	/* 步骤1：确保驱动器处于预操作重新设置驱动器状态为预操作状态 */	 
	SetMotorPar(&Can_Msg_ChangeMotionStatusToPreOperation[0]);
	SetMotorPar(&Can_Msg_ChangeMotionStatusToPreOperation[1]);
    SetMotorPar(&Can_Msg_ChangeMotionStatusToPreOperation[0]);
	SetMotorPar(&Can_Msg_ChangeMotionStatusToPreOperation[1]);
	delay_ms(1500);
	/* 步骤2：清除由于运动状态下急停电机故障状态 */
	SetMotorPar(&Can_Msg_DSPclearAlarm_NO[0]); 	//清除NODE1电机DSP故障，该指令需要在预操作状态下
	SetMotorPar(&Can_Msg_DSPclearAlarm_NO[1]);	//清除NODE2电机DSP故障，该指令需要在预操作状态下
	/*步骤3： 设置为运行状态 */
	SetMotorPar(&Can_Msg_OperationEn_NO[0]);
	SetMotorPar(&Can_Msg_OperationEn_NO[1]);
	delay_ms(500);

	/* 重新设置驱动器状态为预操作状态 */
	SetMotorPar(&Can_Msg_ChangeMotionStatusToPreOperation[0]);
	SetMotorPar(&Can_Msg_ChangeMotionStatusToPreOperation[1]);

#if 1


	for(i=0;i<2;i++)
	{
		SetMotorPar(&Can_Msg_ChangeMotionStatusToPreOperation[i]);//设置为预操作状态
		
//		SetMotorPar(&Can_Msg_DSPclearAlarm_NO[i]);//清除 
		
		SetMotorPar(&Can_Msg_ReadSwitch_NO[i]); 
		
		SetMotorPar(&Can_Msg_SwitchOn_NO[i]);
		
		SetMotorPar(&Can_Msg_TPDOTRANTYPE_NO[i][0]);//设置TPDO2传输类型
		SetMotorPar(&Can_Msg_TPDOTRANTYPE_NO[i][1]);//设置TPDO3传输类型

		/* 映射设置分为6步 */
		
		/* 第1步  ：在预操作模式下   */
		SetMotorPar(&Can_Msg_ChangeMotionStatusToPreOperation[i]); //设置为预操作状态           

		/* 第2步	：关闭TPDO   */
		SetMotorPar(&Can_Msg_Turn_Off_TPDOX[i][0]); //关闭TPDO2
		SetMotorPar(&Can_Msg_Turn_Off_TPDOX[i][1]);	//关闭TPDO3

		/* 第3步	：设置映射对象个数为0 */
		SetMotorPar(&Can_Msg_Set_Number_Mapped_objectsTo0[i][0]); 	//设置TPDO2映射对象个数为 0
		SetMotorPar(&Can_Msg_Set_Number_Mapped_objectsTo0[i][1]);	//设置TPDO3映射对象个数为 0

		/* 第4步	：设置映射内容 */
		SetMotorPar(&Can_Msg_Map_Actual_value_to_TPDOX_objectX[i][0]);//编码器映射
		SetMotorPar(&Can_Msg_Map_Actual_value_to_TPDOX_objectX[i][1]);//速度映射
		SetMotorPar(&Can_Msg_Map_Actual_value_to_TPDOX_objectX[i][2]);//状态字映射
		SetMotorPar(&Can_Msg_Map_Actual_value_to_TPDOX_objectX[i][3]);//电流值映射

		/* 第5步	：设置映射对象个数 */
		SetMotorPar(&Can_Msg_Set_Number_Mapped_objectsTo2[i][0]);	//设置TPDO2映射对象个数为 2
		SetMotorPar(&Can_Msg_Set_Number_Mapped_objectsTo2[i][1]);	//设置TPDO3映射对象个数为 2

		/* 第6步	：打开TPDO   */
		SetMotorPar(&Can_Msg_Turn_On_TPDOX[i][0]);//打开TPDO2
		SetMotorPar(&Can_Msg_Turn_On_TPDOX[i][1]);//打开TPDO3

		SetMotorPar(&Can_Msg_OperationEn_NO[i]);	//设置操作使能

		SetMotorPar(&Can_Msg_VelocityMode_NO[i]);	//设置速度模式

		SetMotorPar(&Can_Msg_SetAcc_NO[i]);		//设置加速度

		SetMotorPar(&Can_Msg_SetDec_NO[i]);		//设置减速度

		SetMotorPar(&Can_Msg_ChangeMotionStatusToOperation[i]);//设置为操作状态

//		SetMotorPar(&Can_Msg_MotionHalts_NO[i]);//设置速度为 0

		SetMotorPar(&Can_Msg_TargetVelocity_NO[i]);//设置目标速度 0

		SetMotorPar(&Can_Msg_MotionStart_NO[i]);	//启动
	}
	#endif
	#if 0
	SetMotorPar(&Can_Msg_ChangeMotionStatusToPreOperation[0]);
	SetMotorPar(&Can_Msg_ChangeMotionStatusToPreOperation[1]);
	delay_ms(500);
	SetMotorPar(&Can_Msg_DSPclearAlarm_NO[0]);
	SetMotorPar(&Can_Msg_DSPclearAlarm_NO[1]);
	
	SetMotorPar(&Can_Msg_ReadSwitch_NO[0]);
	SetMotorPar(&Can_Msg_ReadSwitch_NO[1]);
	
	SetMotorPar(&Can_Msg_SwitchOn_NO[0]);
	SetMotorPar(&Can_Msg_SwitchOn_NO[1]);
	
	SetMotorPar(&Can_Msg_TPDOTRANTYPE_NO[0][0]);
	SetMotorPar(&Can_Msg_TPDOTRANTYPE_NO[0][1]);

	SetMotorPar(&Can_Msg_TPDOTRANTYPE_NO[1][0]);
	SetMotorPar(&Can_Msg_TPDOTRANTYPE_NO[1][1]);

	SetMotorPar(&Can_Msg_ChangeMotionStatusToPreOperation[0]);
	SetMotorPar(&Can_Msg_ChangeMotionStatusToPreOperation[1]);
	
	SetMotorPar(&Can_Msg_Turn_Off_TPDOX[0][0]);
	SetMotorPar(&Can_Msg_Turn_Off_TPDOX[0][1]);
	SetMotorPar(&Can_Msg_Turn_Off_TPDOX[1][0]);
	SetMotorPar(&Can_Msg_Turn_Off_TPDOX[1][1]);
	
	SetMotorPar(&Can_Msg_Set_Number_Mapped_objectsTo0[0][0]);
	SetMotorPar(&Can_Msg_Set_Number_Mapped_objectsTo0[0][1]);
	SetMotorPar(&Can_Msg_Set_Number_Mapped_objectsTo0[1][0]);
	SetMotorPar(&Can_Msg_Set_Number_Mapped_objectsTo0[1][1]);

	SetMotorPar(&Can_Msg_Map_Actual_value_to_TPDOX_objectX[0][0]);
	SetMotorPar(&Can_Msg_Map_Actual_value_to_TPDOX_objectX[0][1]);
	SetMotorPar(&Can_Msg_Map_Actual_value_to_TPDOX_objectX[0][2]);
	SetMotorPar(&Can_Msg_Map_Actual_value_to_TPDOX_objectX[0][3]);
	SetMotorPar(&Can_Msg_Map_Actual_value_to_TPDOX_objectX[1][0]);
	SetMotorPar(&Can_Msg_Map_Actual_value_to_TPDOX_objectX[1][1]);
	SetMotorPar(&Can_Msg_Map_Actual_value_to_TPDOX_objectX[1][2]);
	SetMotorPar(&Can_Msg_Map_Actual_value_to_TPDOX_objectX[1][3]);

	SetMotorPar(&Can_Msg_Set_Number_Mapped_objectsTo2[0][0]);
	SetMotorPar(&Can_Msg_Set_Number_Mapped_objectsTo2[0][1]);
	SetMotorPar(&Can_Msg_Set_Number_Mapped_objectsTo2[1][0]);
	SetMotorPar(&Can_Msg_Set_Number_Mapped_objectsTo2[1][1]);

	SetMotorPar(&Can_Msg_Turn_On_TPDOX[0][0]);
	SetMotorPar(&Can_Msg_Turn_On_TPDOX[0][1]);
	SetMotorPar(&Can_Msg_Turn_On_TPDOX[1][0]);
	SetMotorPar(&Can_Msg_Turn_On_TPDOX[1][1]);
	
	SetMotorPar(&Can_Msg_OperationEn_NO[0]);
	SetMotorPar(&Can_Msg_OperationEn_NO[1]);

	SetMotorPar(&Can_Msg_VelocityMode_NO[0]);
	SetMotorPar(&Can_Msg_VelocityMode_NO[1]);
	
	SetMotorPar(&Can_Msg_SetAcc_NO[0]);
	SetMotorPar(&Can_Msg_SetAcc_NO[1]);
	
	SetMotorPar(&Can_Msg_SetDec_NO[0]);
	SetMotorPar(&Can_Msg_SetDec_NO[1]);
	
	SetMotorPar(&Can_Msg_ChangeMotionStatusToOperation[0]);
	SetMotorPar(&Can_Msg_ChangeMotionStatusToOperation[1]);
	
	SetMotorPar(&Can_Msg_MotionHalts_NO[0]);
	SetMotorPar(&Can_Msg_MotionHalts_NO[1]);
	
	SetMotorPar(&Can_Msg_TargetVelocity_NO[0]);
	SetMotorPar(&Can_Msg_TargetVelocity_NO[1]);
	
	SetMotorPar(&Can_Msg_MotionStart_NO[0]);
	SetMotorPar(&Can_Msg_MotionStart_NO[1]);	
#endif	
#if 0	
	do
	{
   		canSend(CAN1,&Can_Msg_ChangeMotionStatusToPreOperation[0]);  //切换至1号电机至运行状态
	   	delay_ms(delay_n);
		countsend++;
		if(countsend>=5)
		{
			startcmdstatus=0;
			errcode[0] =0x01;
			errcode[1] = Can_Msg_DSPclearAlarm_NO->data[2];
			errcode[2] = Can_Msg_DSPclearAlarm_NO->data[3];
			Date_Up_Load(0xff);
			return;
		}
	}while(startcmdstatus);
			do{
	   canSend(CAN1,&Can_Msg_ChangeMotionStatusToPreOperation[1]);  //切换至2号电机至运行状态
		   delay_ms(delay_n);
		   	}while(startcmdstatus);

	do{canSend(CAN1,&Can_Msg_DSPclearAlarm_NO[0]);
    	delay_ms(delay_n);
		i++;
		if(i>=5)
		{
			errcode[0] =0x01;
			errcode[1] = Can_Msg_DSPclearAlarm_NO->data[2];
			errcode[2] = Can_Msg_DSPclearAlarm_NO->data[3];
			Date_Up_Load(0xff);
		}
		}
	while(startcmdstatus&&i<6);
	
	do{
	canSend(CAN1,&Can_Msg_DSPclearAlarm_NO[1]);
	delay_ms(delay_n);
		}while(startcmdstatus);
	do{
    canSend(CAN1,&Can_Msg_ReadSwitch_NO[0]);
    	delay_ms(delay_n);
				}while(startcmdstatus);
	do{			
    canSend(CAN1,&Can_Msg_ReadSwitch_NO[1]);
        delay_ms(delay_n);
	}while(startcmdstatus);

	do{
    canSend(CAN1,&Can_Msg_SwitchOn_NO[0]);
        delay_ms(delay_n);
		}while(startcmdstatus);
		do{
    canSend(CAN1,&Can_Msg_SwitchOn_NO[1]);
        delay_ms(delay_n);
			}while(startcmdstatus);
		
    /* 设置驱动器的SDO通讯参数中的传输类型 */
			do{
	canSend(CAN1,&Can_Msg_TPDOTRANTYPE_NO[0][0]);
	   delay_ms(delay_n);	
	   	}while(startcmdstatus);
		do{
    canSend(CAN1,&Can_Msg_TPDOTRANTYPE_NO[0][1]);
	   delay_ms(delay_n);
	   	}while(startcmdstatus);
		do{
	canSend(CAN1,&Can_Msg_TPDOTRANTYPE_NO[1][0]);
	   delay_ms(delay_n);
	   	}while(startcmdstatus);
		do{
    canSend(CAN1,&Can_Msg_TPDOTRANTYPE_NO[1][1]);
	   delay_ms(delay_n);
	   	}while(startcmdstatus);

	/* 确保驱动器处于预操作重新设置驱动器状态为预操作状态 */	   
		do{
	   canSend(CAN1,&Can_Msg_ChangeMotionStatusToPreOperation[0]);  //切换至1号电机至运行状态
		   delay_ms(delay_n);
		   	}while(startcmdstatus);
			do{
	   canSend(CAN1,&Can_Msg_ChangeMotionStatusToPreOperation[1]);  //切换至2号电机至运行状态
		   delay_ms(delay_n);
		   	}while(startcmdstatus);
   
 	/* 设置驱动器的SDO映射参数 */
   for(i=0;i<2;i++)
   {
   		j=0;
		do{
	   canSend(CAN1,&Can_Msg_Turn_Off_TPDOX[i][0]); 
	   delay_ms(delay_n);
	   	}while(startcmdstatus);
	   j++;
	   do{
	   canSend(CAN1,&Can_Msg_Turn_Off_TPDOX[i][1]); 
	   delay_ms(delay_n);
	   	}while(startcmdstatus);
   }
   for(i=0;i<2;i++)
   {
   do{
	   canSend(CAN1,&Can_Msg_Set_Number_Mapped_objectsTo0[i][0]); 
	   delay_ms(delay_n);
	   	}while(startcmdstatus);
		do{
	   canSend(CAN1,&Can_Msg_Set_Number_Mapped_objectsTo0[i][1]); 
	   delay_ms(delay_n);	
	   }while(startcmdstatus);
	   
   }
   for(i=0;i<2;i++)
   {
   		j=0;
		do{
	   canSend(CAN1,&Can_Msg_Map_Actual_value_to_TPDOX_objectX[i][0]); 
	   delay_ms(delay_n);
	   	}while(startcmdstatus);
	   j++;
	   do{
	   canSend(CAN1,&Can_Msg_Map_Actual_value_to_TPDOX_objectX[i][1]); 
	   delay_ms(delay_n);
	   	}while(startcmdstatus);
	   j++;
	   do{
	   canSend(CAN1,&Can_Msg_Map_Actual_value_to_TPDOX_objectX[i][2]); 
	   delay_ms(delay_n);
	   	}while(startcmdstatus);
	   j++;
	   do{
	   canSend(CAN1,&Can_Msg_Map_Actual_value_to_TPDOX_objectX[i][3]); 
	   delay_ms(delay_n);
	   	}while(startcmdstatus);
   }
   for(i=0;i<2;i++)
   {
   		j=0;
		do{
	   canSend(CAN1,&Can_Msg_Set_Number_Mapped_objectsTo2[i][0]); 
	   delay_ms(delay_n);
	   	}while(startcmdstatus);
	   j++;
	   do{
	   canSend(CAN1,&Can_Msg_Set_Number_Mapped_objectsTo2[i][1]); 
	   delay_ms(delay_n);
	   	}while(startcmdstatus);
   }
   for(i=0;i<2;i++)
   {
   		j=0;
		do{
	   		canSend(CAN1,&Can_Msg_Turn_On_TPDOX[i][0]); 
	   		delay_ms(delay_n);
			if()
	   	}while(startcmdstatus);
	   j++;
	   do{
	   canSend(CAN1,&Can_Msg_Turn_On_TPDOX[i][1]); 
	   delay_ms(delay_n);
	   	}while(startcmdstatus);
   }

//   canSend(CAN1,&Can_Msg_OperationEn_NO[0]); 
//	   delay_ms(5); 
//   canSend(CAN1,&Can_Msg_OperationEn_NO[1]); 
//	   delay_ms(5);    
//   canSend(CAN1,&Can_Msg_VelocityMode_NO[0]); 
//	   delay_ms(5);   
//   canSend(CAN1,&Can_Msg_VelocityMode_NO[1]); 
//	   delay_ms(5);    
   do{

    canSend(CAN1,&Can_Msg_OperationEn_NO[0]); 
        delay_ms(delay_n);
			}while(startcmdstatus);
			do{
    canSend(CAN1,&Can_Msg_OperationEn_NO[1]); 
        delay_ms(delay_n);  
			}while(startcmdstatus);
			do{
    canSend(CAN1,&Can_Msg_VelocityMode_NO[0]); 
        delay_ms(delay_n); 
			}while(startcmdstatus);
			do{
    canSend(CAN1,&Can_Msg_VelocityMode_NO[1]); 
        delay_ms(delay_n);
			}while(startcmdstatus);
			do{
    canSend(CAN1,&Can_Msg_SetAcc_NO[0]);
        delay_ms(delay_n);
			}while(startcmdstatus);
			do{
    canSend(CAN1,&Can_Msg_SetAcc_NO[1]);
        delay_ms(delay_n);
			}while(startcmdstatus);
			do{
    canSend(CAN1,&Can_Msg_SetDec_NO[0]);
        delay_ms(delay_n);
			}while(startcmdstatus);
			do{
    canSend(CAN1,&Can_Msg_SetDec_NO[1]);
        delay_ms(delay_n);	
		}while(startcmdstatus); 
		do{
    canSend(CAN1,&Can_Msg_ChangeMotionStatusToOperation[0]);  //切换至1号电机至运行状态
        delay_ms(delay_n);
			}while(startcmdstatus);
			do{
    canSend(CAN1,&Can_Msg_ChangeMotionStatusToOperation[1]);  //切换至2号电机至运行状态
        delay_ms(delay_n);
			}while(startcmdstatus);
            
     canSend(CAN1,&Can_Msg_MotionHalts_NO[0]);  //Start/Stop Motion
        delay_ms(2);
    canSend(CAN1,&Can_Msg_MotionHalts_NO[1]);  //Start/Stop Motion
        delay_ms(2);
        do{
	canSend(CAN1,&Can_Msg_TargetVelocity_NO[0]);
        delay_ms(delay_n);
			}while(startcmdstatus);
			do{
    canSend(CAN1,&Can_Msg_TargetVelocity_NO[1]);
        delay_ms(delay_n); 
		}while(startcmdstatus);
        
    canSend(CAN1,&Can_Msg_MotionStart_NO[0]);  //Start/Stop Motion
        delay_ms(2);
    canSend(CAN1,&Can_Msg_MotionStart_NO[1]);  //Start/Stop Motion
        delay_ms(2);
#endif

			

}

 void SetMotorTargetVelocity(s32 vel_left,s32 vel_right)
 {
 	u8 i;
 	s32 speedleft,speedright;

//	if(vel_left>50||vel_right>50)
//		return;
	speedleft = vel_left;
	speedright = vel_right;
	
 	data_cmd_TargetVelocity[4] = speedleft;
	data_cmd_TargetVelocity[5] = speedleft>>8;
	data_cmd_TargetVelocity[6] = speedleft>>16;
	data_cmd_TargetVelocity[7] = speedleft>>24;
	
	for(i=4;i<Can_Msg_TargetVelocity_NO[0].len;i++)
		 Can_Msg_TargetVelocity_NO[0].data[i] = data_cmd_TargetVelocity[i];

	data_cmd_TargetVelocity[4] = speedright;
	data_cmd_TargetVelocity[5] = speedright>>8;
	data_cmd_TargetVelocity[6] = speedright>>16;
	data_cmd_TargetVelocity[7] = speedright>>24;
	
	for(i=4;i<Can_Msg_TargetVelocity_NO[1].len;i++)
		Can_Msg_TargetVelocity_NO[1].data[i] = data_cmd_TargetVelocity[i];
 }
 
 void StartMotion(void)
 {
    canSend(CAN1,&Can_Msg_TargetVelocity_NO[0]);
    delay_ms(2);
    canSend(CAN1,&Can_Msg_TargetVelocity_NO[1]);
    delay_ms(2);
     if(stopflag==1)
     {
      canSend(CAN1,&Can_Msg_MotionStart_NO[0]);  //Start/Stop Motion
                 delay_ms(2);
      canSend(CAN1,&Can_Msg_MotionStart_NO[1]);  //Start/Stop Motion
         stopflag=0;
     }
 }
 
 void MotionHalts(void)
 {
    canSend(CAN1,&Can_Msg_MotionHalts_NO[0]);
    delay_ms(2);
    canSend(CAN1,&Can_Msg_MotionHalts_NO[1]);
    delay_ms(2);
     stopflag=1;
 }
 extern u8 AbnormalReporting(u8 type,u8 group,u8 number);
/*****************************************************************************
 * 函 数 名  : StatusWordAnalysis
 * 负 责 人  : jishubao
 * 创建日期  : 2018年11月19日
 * 函数功能  : 驱动器状态字分析函数
 * 输入参数  : void  无
 * 输出参数  : 无
 * 返 回 值  : void
 * 调用关系  : 
 * 其    它  : 

*****************************************************************************/
 void StatusWordAnalysis(void)
{
//	if((status_word_left[1]&(0x01<<2))==0)//判断是否处于操作状态
//	{
//		canSend(CAN1,&Can_Msg_ChangeMotionStatusToOperation[0]);  //切换至1号电机至运行状态
//		delay_ms(2);
//	}
//    if((status_word_right[1]&(0x01<<2))==0)//判断是否处于操作状态
//	{
//		canSend(CAN1,&Can_Msg_ChangeMotionStatusToOperation[1]);  //切换至2号电机至运行状态
//		delay_ms(2);
//	}
	if((status_word_left[1]&(0x01<<3))||(status_word_right[1]&(0x01<<3)))     //判断是否出现FAULT
	{
		MotionHalts();
		AbnormalReporting(1,1,3);
	}
		if((status_word_left[1]&(0x01<<7))||(status_word_right[1]&(0x01<<7)))     //判断是否出现FAULT
	{
		MotionHalts();
		AbnormalReporting(1,1,4);
	}
}
/*****************************************************************************
 * 函 数 名  : SetCanopenParameter
 * 负 责 人  : jishubao
 * 创建日期  : 2018年11月19日
 * 函数功能  : 驱动器CANopen通讯网络参数设置
 * 输入参数  : u8 id  电机编号 id
 * 输出参数  : 无
 * 返 回 值  : void
 * 调用关系  : 
 * 其    它  : 

*****************************************************************************/
 void SetCanopenParameter(u8 id)
 {
     u8 i;
	 /* 填充read switch结构体 */
    Can_Msg_ReadSwitch_NO[id].cob_id = id_motor_no[id];
	Can_Msg_ReadSwitch_NO[id].rtr = rtr_my;
	Can_Msg_ReadSwitch_NO[id].len = len;
	for(i=0;i<Can_Msg_ReadSwitch_NO[id].len;i++)
        Can_Msg_ReadSwitch_NO[id].data[i]  =  data_cmd_ReadSwitch[i];

	/* 填充switch on结构体 */
	Can_Msg_SwitchOn_NO[id].cob_id = id_motor_no[id];
	Can_Msg_SwitchOn_NO[id].rtr = rtr_my;
	Can_Msg_SwitchOn_NO[id].len = len;
	for(i=0;i<Can_Msg_SwitchOn_NO[id].len;i++)
        Can_Msg_SwitchOn_NO[id].data[i]  =  data_cmd_SwitchOn[i];

	/* 填充OPeration enable 结构体 */
	Can_Msg_OperationEn_NO[id].cob_id = id_motor_no[id];
	Can_Msg_OperationEn_NO[id].rtr = rtr_my;
	Can_Msg_OperationEn_NO[id].len = len;
	for(i=0;i<Can_Msg_OperationEn_NO[id].len;i++)
        Can_Msg_OperationEn_NO[id].data[i]  =  data_cmd_OperationEn[i];

	/* 填充 模式选择为速度模式 结构体 */
	Can_Msg_VelocityMode_NO[id].cob_id = id_motor_no[id];
	Can_Msg_VelocityMode_NO[id].rtr = rtr_my;
	Can_Msg_VelocityMode_NO[id].len = len;
	for(i=0;i<Can_Msg_VelocityMode_NO[id].len;i++)
        Can_Msg_VelocityMode_NO[id].data[i] =  data_cmd_VelocityMode[i];

	/* 填充 设置加速度 结构体 */
	Can_Msg_SetAcc_NO[id].cob_id = id_motor_no[id];
	Can_Msg_SetAcc_NO[id].rtr = rtr_my;
	Can_Msg_SetAcc_NO[id].len = len;
	for(i=0;i<Can_Msg_SetAcc_NO[id].len;i++)
        Can_Msg_SetAcc_NO[id].data[i] =  data_cmd_SetAcc[i];

	/* 填充 设置减速度 结构体 */
	Can_Msg_SetDec_NO[id].cob_id = id_motor_no[id];
	Can_Msg_SetDec_NO[id].rtr = rtr_my;
	Can_Msg_SetDec_NO[id].len = len;
	for(i=0;i<Can_Msg_SetDec_NO[id].len;i++)
        Can_Msg_SetDec_NO[id].data[i] =  data_cmd_SetDec[i];

	/* 填充 设置目标速度 结构体 */
	Can_Msg_TargetVelocity_NO[id].cob_id = id_motor_no[id];
	Can_Msg_TargetVelocity_NO[id].rtr = rtr_my;
	Can_Msg_TargetVelocity_NO[id].len = len;
	for(i=0;i<Can_Msg_TargetVelocity_NO[id].len;i++)
        Can_Msg_TargetVelocity_NO[id].data[i] =  data_cmd_TargetVelocity[i];

	/* 填充 设置开始运动 结构体 */
	Can_Msg_MotionStart_NO[id].cob_id = id_motor_no[id];
	Can_Msg_MotionStart_NO[id].rtr = rtr_my;
	Can_Msg_MotionStart_NO[id].len = len;
	for(i=0;i<Can_Msg_MotionStart_NO[id].len;i++)
		Can_Msg_MotionStart_NO[id].data[i] =  data_cmd_MotionStart[i];

	/* 填充 设置停止 结构体 */
	Can_Msg_MotionHalts_NO[id].cob_id = id_motor_no[id];
	Can_Msg_MotionHalts_NO[id].rtr = rtr_my;
	Can_Msg_MotionHalts_NO[id].len = len;
	for(i=0;i<Can_Msg_MotionHalts_NO[id].len;i++)
		Can_Msg_MotionHalts_NO[id].data[i] =  data_cmd_MotionHalts[i];

	/* 填充 请求实际编码器值 结构体 */
	Can_Msg_ActPosition_NO[id].cob_id = id_motor_no[id];
	Can_Msg_ActPosition_NO[id].rtr = rtr_my;
	Can_Msg_ActPosition_NO[id].len = len;
	for(i=0;i<Can_Msg_ActPosition_NO[id].len;i++)
        Can_Msg_ActPosition_NO[id].data[i] =  data_cmd_GetPosition[i];

	/* 填充 请求实际速度值 结构体 */
	Can_Msg_ActVelocity_NO[id].cob_id = id_motor_no[id];
	Can_Msg_ActVelocity_NO[id].rtr = rtr_my;
	Can_Msg_ActVelocity_NO[id].len = len;
	for(i=0;i<Can_Msg_ActVelocity_NO[id].len;i++)
		Can_Msg_ActVelocity_NO[id].data[i] =  data_cmd_GetVelActVal[i];
	
	/* 填充 请求实际电流值 结构体 */
	Can_Msg_ActCurrent_NO[id].cob_id = id_motor_no[id];
	Can_Msg_ActCurrent_NO[id].rtr = rtr_my;
	Can_Msg_ActCurrent_NO[id].len = len;
	for(i=0;i<Can_Msg_ActCurrent_NO[id].len;i++)
		Can_Msg_ActCurrent_NO[id].data[i] =  data_cmd_GetCurActValue[i];

	/* 填充 驱动器状态 结构体 */ 
	Can_Msg_MotionStatus_NO[id].cob_id = id_motor_no[id];
	Can_Msg_MotionStatus_NO[id].rtr = rtr_my;
	Can_Msg_MotionStatus_NO[id].len = len;
	for(i=0;i<Can_Msg_MotionStatus_NO[id].len;i++)
		Can_Msg_MotionStatus_NO[id].data[i] =  data_cmd_MotionStatus[i];

	
 	/* 填充 驱动器id的TPDO传输类型 结构体 */ 
	 Can_Msg_TPDOTRANTYPE_NO[id][0].cob_id = id_motor_no[id];
	 Can_Msg_TPDOTRANTYPE_NO[id][0].rtr = rtr_my;
	 Can_Msg_TPDOTRANTYPE_NO[id][0].len = len;
	 for(i=0;i<Can_Msg_TPDOTRANTYPE_NO[id][0].len;i++)
		 Can_Msg_TPDOTRANTYPE_NO[id][0].data[i] = data_cmd_ChangeTPDOTransType180x[0][i];
	 
	  	/* 填充 驱动器 id 的TPDO传输类型 结构体 */ 
	 Can_Msg_TPDOTRANTYPE_NO[id][1].cob_id = id_motor_no[id];
	 Can_Msg_TPDOTRANTYPE_NO[id][1].rtr = rtr_my;
	 Can_Msg_TPDOTRANTYPE_NO[id][1].len = len;
	 for(i=0;i<Can_Msg_TPDOTRANTYPE_NO[id][1].len;i++)
		 Can_Msg_TPDOTRANTYPE_NO[id][1].data[i] =data_cmd_ChangeTPDOTransType180x[1][i];

	 	 /* 填充 驱动器 id 的状态切换到运行状态 结构体 */ 
	 Can_Msg_ChangeMotionStatusToOperation[id].cob_id = 0x0000;
	 Can_Msg_ChangeMotionStatusToOperation[id].rtr = rtr_my;
	 Can_Msg_ChangeMotionStatusToOperation[id].len = 2;
	 for(i=0;i<Can_Msg_ChangeMotionStatusToOperation[id].len;i++)
		 Can_Msg_ChangeMotionStatusToOperation[id].data[i] =data_cmd_ChangeMotionStatusToOperation[id][i];
	 
		/* 填充 驱动器 id 的状态切换到预运行状态 结构体 */ 
	 Can_Msg_ChangeMotionStatusToPreOperation[id].cob_id = 0x0000;
	 Can_Msg_ChangeMotionStatusToPreOperation[id].rtr = rtr_my;
	 Can_Msg_ChangeMotionStatusToPreOperation[id].len = 2;
	 for(i=0;i<Can_Msg_ChangeMotionStatusToPreOperation[id].len;i++)
		 Can_Msg_ChangeMotionStatusToPreOperation[id].data[i] =data_cmd_ChangeMotionStatusToPreOperation[id][i];	 

	 	 /* 填充 驱动器 id 关闭TPDO1 结构体 */ 
	 Can_Msg_Turn_Off_TPDOX[id][0].cob_id = id_motor_no[id];
	 Can_Msg_Turn_Off_TPDOX[id][0].rtr = rtr_my;
	 Can_Msg_Turn_Off_TPDOX[id][0].len = len;
	 for(i=0;i<Can_Msg_Turn_Off_TPDOX[id][0].len;i++)
		 Can_Msg_Turn_Off_TPDOX[id][0].data[i] =data_cmd_Turn_Off_TPDOX[0][i];	
	 
	  /* 填充 驱动器 id 关闭TPDO2 结构体 */ 
	 Can_Msg_Turn_Off_TPDOX[id][1].cob_id = id_motor_no[id];
	 Can_Msg_Turn_Off_TPDOX[id][1].rtr = rtr_my;
	 Can_Msg_Turn_Off_TPDOX[id][1].len = len;
	 for(i=0;i<Can_Msg_Turn_Off_TPDOX[id][1].len;i++)
		 Can_Msg_Turn_Off_TPDOX[id][1].data[i] =data_cmd_Turn_Off_TPDOX[1][i];	 

	 	 /* 填充 驱动器 id 设置0X1A01映射对象个数为0 结构体 */ 
	 Can_Msg_Set_Number_Mapped_objectsTo0[id][0].cob_id = id_motor_no[id];
	 Can_Msg_Set_Number_Mapped_objectsTo0[id][0].rtr = rtr_my;
	 Can_Msg_Set_Number_Mapped_objectsTo0[id][0].len = len;
	 for(i=0;i<Can_Msg_Set_Number_Mapped_objectsTo0[id][0].len;i++)
		 Can_Msg_Set_Number_Mapped_objectsTo0[id][0].data[i] =data_cmd_Set_Number_Mapped_objectsTo0[0][i];	
	 
	  /* 填充 驱动器 id 设置0X1A02映射对象个数为0 结构体 */ 
	 Can_Msg_Set_Number_Mapped_objectsTo0[id][1].cob_id = id_motor_no[id];
	 Can_Msg_Set_Number_Mapped_objectsTo0[id][1].rtr = rtr_my;
	 Can_Msg_Set_Number_Mapped_objectsTo0[id][1].len = len;
	 for(i=0;i<Can_Msg_Set_Number_Mapped_objectsTo0[id][1].len;i++)
		 Can_Msg_Set_Number_Mapped_objectsTo0[id][1].data[i] =data_cmd_Set_Number_Mapped_objectsTo0[1][i];
	 
	/* 填充 驱动器 id 设置编码器值映射到TPDO1的子索引01 4Bytes 结构体 */ 
	 Can_Msg_Map_Actual_value_to_TPDOX_objectX[id][0].cob_id = id_motor_no[id];
	 Can_Msg_Map_Actual_value_to_TPDOX_objectX[id][0].rtr = rtr_my;
	 Can_Msg_Map_Actual_value_to_TPDOX_objectX[id][0].len = len;
	 for(i=0;i<Can_Msg_Map_Actual_value_to_TPDOX_objectX[id][0].len;i++)
		 Can_Msg_Map_Actual_value_to_TPDOX_objectX[id][0].data[i] =data_cmd_Map_Actual_value_to_TPDOX_objectX[0][i];	
	 
	  /* 填充 驱动器 id //设置速度值映射到TPDO1的子索引02		 		  2Bytes 结构体 */ 
	 Can_Msg_Map_Actual_value_to_TPDOX_objectX[id][1].cob_id = id_motor_no[id];
	 Can_Msg_Map_Actual_value_to_TPDOX_objectX[id][1].rtr = rtr_my;
	 Can_Msg_Map_Actual_value_to_TPDOX_objectX[id][1].len = len;
	 for(i=0;i<Can_Msg_Map_Actual_value_to_TPDOX_objectX[id][1].len;i++)
		 Can_Msg_Map_Actual_value_to_TPDOX_objectX[id][1].data[i] =data_cmd_Map_Actual_value_to_TPDOX_objectX[1][i];
	 
	 /* 填充 驱动器 id 设置状态字映射到TPDO2的子索引01 4Bytes 结构体 */ 
	  Can_Msg_Map_Actual_value_to_TPDOX_objectX[id][2].cob_id = id_motor_no[id];
	  Can_Msg_Map_Actual_value_to_TPDOX_objectX[id][2].rtr = rtr_my;
	  Can_Msg_Map_Actual_value_to_TPDOX_objectX[id][2].len = len;
	  for(i=0;i<Can_Msg_Map_Actual_value_to_TPDOX_objectX[id][2].len;i++)
		  Can_Msg_Map_Actual_value_to_TPDOX_objectX[id][2].data[i] =data_cmd_Map_Actual_value_to_TPDOX_objectX[2][i];	 
	  
	   /* 填充 驱动器 id //设置电流值映射到TPDO2的子索引02 			       2Bytes 结构体 */ 
	  Can_Msg_Map_Actual_value_to_TPDOX_objectX[id][3].cob_id = id_motor_no[id];
	  Can_Msg_Map_Actual_value_to_TPDOX_objectX[id][3].rtr = rtr_my;
	  Can_Msg_Map_Actual_value_to_TPDOX_objectX[id][3].len = len;
	  for(i=0;i<Can_Msg_Map_Actual_value_to_TPDOX_objectX[id][3].len;i++)
		  Can_Msg_Map_Actual_value_to_TPDOX_objectX[id][3].data[i] =data_cmd_Map_Actual_value_to_TPDOX_objectX[3][i];

	  /* 填充 驱动器 id 设置0X1A00映射对象个数为2 结构体 */ 
	  Can_Msg_Set_Number_Mapped_objectsTo2[id][0].cob_id = id_motor_no[id];
	  Can_Msg_Set_Number_Mapped_objectsTo2[id][0].rtr = rtr_my;
	  Can_Msg_Set_Number_Mapped_objectsTo2[id][0].len = len;
	  for(i=0;i<Can_Msg_Set_Number_Mapped_objectsTo2[id][0].len;i++)
		  Can_Msg_Set_Number_Mapped_objectsTo2[id][0].data[i] =data_cmd_Set_Number_Mapped_objectsTo2[0][i];  
	  
	   /* 填充 驱动器 id 设置0X1A01映射对象个数为2 结构体 */ 
	  Can_Msg_Set_Number_Mapped_objectsTo2[id][1].cob_id = id_motor_no[id];
	  Can_Msg_Set_Number_Mapped_objectsTo2[id][1].rtr = rtr_my;
	  Can_Msg_Set_Number_Mapped_objectsTo2[id][1].len = len;
	  for(i=0;i<Can_Msg_Set_Number_Mapped_objectsTo2[id][1].len;i++)
		  Can_Msg_Set_Number_Mapped_objectsTo2[id][1].data[i] =data_cmd_Set_Number_Mapped_objectsTo2[1][i];

		  /* 填充 驱动器 id 打开TPDO1 结构体 */ 
	  Can_Msg_Turn_On_TPDOX[id][0].cob_id = id_motor_no[id];
	  Can_Msg_Turn_On_TPDOX[id][0].rtr = rtr_my;
	  Can_Msg_Turn_On_TPDOX[id][0].len = len;
	  for(i=0;i<Can_Msg_Turn_On_TPDOX[id][0].len;i++)
		  Can_Msg_Turn_On_TPDOX[id][0].data[i] =data_cmd_Turn_On_TPDOX[0][i];  
	  
	   /* 填充 驱动器 id 打开TPDO2 结构体 */ 
	  Can_Msg_Turn_On_TPDOX[id][1].cob_id = id_motor_no[id];
	  Can_Msg_Turn_On_TPDOX[id][1].rtr = rtr_my;
	  Can_Msg_Turn_On_TPDOX[id][1].len = len;
	  for(i=0;i<Can_Msg_Turn_On_TPDOX[id][1].len;i++)
		  Can_Msg_Turn_On_TPDOX[id][1].data[i] =data_cmd_Turn_On_TPDOX[1][i];   
	  
	  /* 填充 驱动器 id      SYNC报文 结构体 */ 
	  Can_Msg_SYNCMY.cob_id = 0X0080;
	  Can_Msg_SYNCMY.rtr = rtr_my;
	  Can_Msg_SYNCMY.len = 0;

	   /* 填充 驱动器 id 清除错误 结构体 */ 
	  Can_Msg_DSPclearAlarm_NO[id].cob_id = id_motor_no[id];
	  Can_Msg_DSPclearAlarm_NO[id].rtr = rtr_my;
	  Can_Msg_DSPclearAlarm_NO[id].len = len;
	  for(i=0;i<Can_Msg_DSPclearAlarm_NO[id].len;i++)
		  Can_Msg_DSPclearAlarm_NO[id].data[i] =data_cmd_DSPclearAlarm[i];  

 }
 
 void Get_Swerve_Speed_PS2(void)
{
	u8 key;
	s16 swerve,speed;

	if(!PS2_RedLight())
	{
        LED0=!LED0;
		delay_ms(40);	 //延时是必要的，理论上至少是50ms，这里包含了其他延时故设置为10ms以保证延时>50ms
		key=PS2_DataKey();	 //读取按键值

		if(key == 11 || key == 12)
		{  
			swerve = (PS2_AnologData(PSS_LX)-128); //	speed
			speed = -(PS2_AnologData(PSS_LY)-127);	   //
			printf("\r\nswerve: %d speed: %d \r\n",swerve,speed);
			if(swerve>=65)  //右转
			{
				Turn_Right();

			}
			if(swerve<=-65)  //左转
			{

				Turn_Left();

			}
			if(speed==0&&swerve==0)   //摇杆没有动作，静止
			{			
				MotionHalts();
			}
			if(swerve<65&&swerve>-65)  //前进或后退
			{
				if(speed>0)//是否前行，>0为前行，反之后退
				{

					if(speed>10&&speed<=110) //一档 V=8rps
					{	
						Move_Forward(1);
					}
					if(speed>110&&speed<=128) //设置前进速度为 V=16rps
					{
						Move_Forward(2);
					}						
				}
				if(speed<0) //后退
				{		
					speed=-speed;
					
					if(speed>10&&speed<=110)//设置后退速度 V=8rps
					{
						Back_Off(1);
					}
					if(speed>110&&speed<=128)//设置速度为 V=16rps
					{
						Back_Off(2);
					}
				}
			}
		}
//        else
//             MotionHalts();

	}
    else 
          MotionHalts();

}
void Turn_Left(void)
{
	s32 vel_left, vel_right;
	vel_left = -3*240;
	vel_right = 3*240;
	SetMotorTargetVelocity(vel_left, vel_right);//设置当前行驶速度
    StartMotion();


}

void Turn_Right(void)
{
	s32 vel_left, vel_right;
	vel_left = 3*240;
	vel_right = -(3*240);
	SetMotorTargetVelocity(vel_left, vel_right);//设置当前行驶速度
    StartMotion();
}

void Move_Forward(u8 level)
{
	s32 vel_left, vel_right;
	if(level == 1)
	{
		vel_left = 6*240;
		vel_right = 6*240;
	}
	else if(level == 2)
	{
		vel_left = 12*240;
		vel_right = 12*240;
	}
	else return;
	SetMotorTargetVelocity(vel_left, vel_right);//设置当前行驶速度
    StartMotion();
}

void Back_Off(u8 level)
{
	s32 vel_left, vel_right;
	if(level == 1)
	{
		vel_left = -6*240;
		vel_right = -6*240;
	}
	else if(level == 2)
	{
		vel_left = -12*240;
		vel_right = -12*240;
	}
	else return;
	SetMotorTargetVelocity(vel_left, vel_right);//设置当前行驶速度
    StartMotion();
}
#define SPEEDCOEFFICIENT 240
u8 Set_Speed(float leftvalue,float rightvalue)
{
	s32 lspeedrps=0,rspeedrps=0;
	const float FLT_EPSILON=0.0001f;
    /* 上位机发送电机转速*/ 
 /* 上位机发送电机转速*/ 
      if(leftvalue>50.0f||rightvalue>50.0f)
         return SPEEDOUT_ERROR;
     else if((leftvalue>=-FLT_EPSILON&&leftvalue<=FLT_EPSILON)||(rightvalue>=-FLT_EPSILON&&rightvalue<=FLT_EPSILON))  //速度为零
	{
        MotionHalts();
	}
    else
    {
				lspeedrps = (s32)(leftvalue*SPEEDCOEFFICIENT); //  直接得到电机转速   
        rspeedrps = (s32)(rightvalue*SPEEDCOEFFICIENT);//  直接得到电机转速 
			
       // lspeedrps = (s32)(leftvalue*REDUCTION_RATIO*SPEED_COEFFICIENT); //  28为减速比  D=250mm   圆周长0.785m     
        //rspeedrps = (s32)(rightvalue*REDUCTION_RATIO*SPEED_COEFFICIENT);//  28为减速比  D=250mm   圆周长0.785m  
        
       SetMotorTargetVelocity(lspeedrps,rspeedrps);
       StartMotion();
    }   
    /*上位机发送整车速度*/    
//	if(leftvalue>1.4f||rightvalue>1.4f)
//		return SPEEDOUT_ERROR;
////    if(leftvalue==0||rightvalue==0)
////         MotionHalts();
//	else if((leftvalue>=-FLT_EPSILON&&leftvalue<=FLT_EPSILON)||(rightvalue>=-FLT_EPSILON&&rightvalue<=FLT_EPSILON))  //速度为零
//	{
//        MotionHalts();
//	}
//    else
//    {
//        lspeedrps = (s32)(leftvalue*28000*240/785); //  28为减速比  D=250mm   圆周长0.785m     
//        rspeedrps = (s32)(rightvalue*28000*240/785);//  28为减速比  D=250mm   圆周长0.785m  
//        
//       SetMotorTargetVelocity(lspeedrps,rspeedrps);
//       StartMotion();
//    }      
        return NO_ERROR;
}
 void Get_Encoder_Value(void)
{
	u32 num=0;
	delay_ms(2);
    canSend(CAN1,&Can_Msg_ActPosition_NO[0]);
	while(RxMSG.cob_id!=0X0581&&num<0xffffff)
	{
		num++;
	};
	if(RxMSG.cob_id==0X0581&&RxMSG.data[0]==data_cmd_GetPosition[0]&&RxMSG.data[1]==data_cmd_GetPosition[1]&&RxMSG.data[2]==data_cmd_GetPosition[2]&&RxMSG.data[3]==data_cmd_GetPosition[3])//判断接收到的报文是否为编码器值报文
 	{
		encoder_valueleft[0] = RxMSG.data[4];
        encoder_valueleft[1] = RxMSG.data[5];
        encoder_valueleft[2] = RxMSG.data[6];
        encoder_valueleft[3] = RxMSG.data[7]; //由于高字节在低地址，按大端方式取出
	}
	delay_ms(2); 
	num=0;
    canSend(CAN1,&Can_Msg_ActPosition_NO[1]);
	while(RxMSG.cob_id!=0X0582&&num<0xffffff)
	{
		num++;
	};
	if(RxMSG.cob_id==0X0582&&RxMSG.data[0]==data_cmd_GetPosition[0]&&RxMSG.data[1]==data_cmd_GetPosition[1]&&RxMSG.data[2]==data_cmd_GetPosition[2]&&RxMSG.data[3]==data_cmd_GetPosition[3])//判断接收到的报文是否为编码器值报文
	{
		encoder_valueright[0] = RxMSG.data[4];
        encoder_valueright[1] = RxMSG.data[5];
        encoder_valueright[2] = RxMSG.data[6];
        encoder_valueright[3] = RxMSG.data[7]; //由于高字节在低地址，按大端方式取出
	}
    Date_Up_Load(MOTOR_CTL_MSGID_CMD); //封装数据并上传	


}
void Get_CurSpeed_Value(void)
{
	u16 num=0;
    u16 valueright,valueleft;
	float right,left;
	canSend(CAN1,&Can_Msg_ActVelocity_NO[0]);
    while(RxMSG.cob_id!=0X0581&&num<0xffff)
	{
		num++;
	};
    if(RxMSG.data[0]==data_cmd_GetVelActVal[0]&&RxMSG.data[1]==data_cmd_GetVelActVal[1]&&RxMSG.data[2]==data_cmd_GetVelActVal[2]&&RxMSG.data[3]==data_cmd_GetVelActVal[3])//判断接收到的报文是否为编码器值报文
	{
		if(RxMSG.data[5]>>8)//速度值为负（后退方向）
		{
			valueleft = 0XFFFF-(RxMSG.data[4]+RxMSG.data[5]*0XFF); //由于高字节在低地址，按大端方式取出,单位为240
			left = (float)valueleft*785/(240*28*1000);//velocity_valueright/240得到电机速度rps，除以减速比28得到1s内轮子转的圈数再乘圆周0.785m得到整车速度单位m/s
			left =-left;
		}
		else//判断速度值是否为正（前进方向）
		{
			valueleft = RxMSG.data[4]+RxMSG.data[5]*0XFF; //由于高字节在低地址，按大端方式取出,单位为240
			left = (float)valueleft*785/(240*28*1000);//velocity_valueright/240得到电机速度rps，除以减速比28得到1s内轮子转的圈数再乘圆周0.785m得到整车速度单位m/s
		}
	
		velocity_valueleft[0] = *(char*)(&left);
		velocity_valueleft[1] = *((char*)(&left)+1);
		velocity_valueleft[2] = *((char*)(&left)+2);
		velocity_valueleft[3] = *((char*)(&left)+3);	
	}
	delay_ms(2);
	num=0;
	canSend(CAN1,&Can_Msg_ActVelocity_NO[1]);
    while(RxMSG.cob_id!=0X0582&&num<0xffff)
	{
		num++;
	};
    if(RxMSG.data[0]==data_cmd_GetVelActVal[0]&&RxMSG.data[1]==data_cmd_GetVelActVal[1]&&RxMSG.data[2]==data_cmd_GetVelActVal[2]&&RxMSG.data[3]==data_cmd_GetVelActVal[3])//判断接收到的报文是否为编码器值报文
	{
		if(RxMSG.data[5]>>8)//速度值为负（后退方向）
		{
			valueright = 0XFFFF-(RxMSG.data[4]+RxMSG.data[5]*0XFF); //由于高字节在低地址，按大端方式取出,单位为240
			right = (float)valueright*785/(240*28*1000);//velocity_valueright/240得到电机速度rps，除以减速比28得到1s内轮子转的圈数再乘圆周0.785m得到整车速度单位m/s
			right =-right;
		}
		else//判断速度值是否为正（前进方向）
		{
			valueright = RxMSG.data[4]+RxMSG.data[5]*0XFF; //由于高字节在低地址，按大端方式取出,单位为240
			right = (float)valueright*785/(240*28*1000);//velocity_valueright/240得到电机速度rps，除以减速比28得到1s内轮子转的圈数再乘圆周0.785m得到整车速度单位m/s
		}
		velocity_valueright[0] = *(char*)(&right);
		velocity_valueright[1] = *((char*)(&right)+1);
		velocity_valueright[2] = *((char*)(&right)+2);
		velocity_valueright[3] = *((char*)(&right)+3);	
	}	
}

void Get_CurElectric_Value(void)
{
	u8 num=0;
	canSend(CAN1,&Can_Msg_ActCurrent_NO[0]);
    while(RxMSG.cob_id!=0X0581&&num<0xffff)
	{
		num++;
	};
    if(RxMSG.data[0]==data_cmd_GetCurActValue[0]&&RxMSG.data[1]==data_cmd_GetCurActValue[1]&&RxMSG.data[2]==data_cmd_GetCurActValue[2]&&RxMSG.data[3]==data_cmd_GetCurActValue[3])//判断接收到的报文是否为编码器值报文
	{
		if(RxMSG.data[5]>>8)//速度值为负（后退方向）电流值为负
		{
			current_valueleft[0] = 0xFF - RxMSG.data[5];
			current_valueleft[1] = 0xFF - RxMSG.data[4]; //由于高字节在低地址，按大端方式取出,单位为0.01A
		}
		else//判断速度值是否为正（前进方向）电流值为正
		{
			current_valueleft[0] = RxMSG.data[5];
			current_valueleft[1] = RxMSG.data[4]; //由于高字节在低地址，按大端方式取出,单位为0.01A
		}
	}
	delay_ms(2);
	num=0;
	canSend(CAN1,&Can_Msg_ActCurrent_NO[1]);
    while(RxMSG.cob_id!=0X0582&&num<0xffff)
	{
		num++;
	};
    if(RxMSG.data[0]==data_cmd_GetCurActValue[0]&&RxMSG.data[1]==data_cmd_GetCurActValue[1]&&RxMSG.data[2]==data_cmd_GetCurActValue[2]&&RxMSG.data[3]==data_cmd_GetCurActValue[3])//判断接收到的报文是否为编码器值报文
	{
		current_valueright[0] = RxMSG.data[5];
		current_valueright[1] = RxMSG.data[4]; //由于高字节在低地址，按大端方式取出,单位为0.01A
	}

}
/* CRC-16MODBUS校验  */
unsigned short GetCRC16(u8 *ptr,u8 len)
{
//	unsigned char i;
//	unsigned short crc = 0xFFFF;
    u8 i;
    u16 crc = 0xffff;
	if(len==0)
	{
		len = 1;
	}
	while(len--)
	{
		crc ^= *ptr;
		for(i=0; i<8; i++)
		{
			if(crc&1)
			{
				crc >>= 1;
				crc ^= 0xA001;
			}
			else
				crc >>= 1;
		}
		ptr++;
	}
	return(crc);
}
extern u8 chargingstatus;
extern u8 current;
extern u8 voltage[2];
extern u8 charging_times[2];
extern u8 remaining_total_capacity[2];
extern u8 total_capacity[2];
extern u8 remaining_battery_percentage;
extern BattaryMSG battarymsg;
extern u16 hw_dis[4];
/*****************************************************************************
 * 函 数 名  : Date_Up_Load
 * 负 责 人  : jishubao
 * 创建日期  : 2018年11月21日
 * 函数功能  : 下位机数据上传至上位机
 * 输入参数  : u8 msgid  上传数据类型ID
 * 输出参数  : 无
 * 返 回 值  : 
 * 调用关系  : 
 * 其    它  : 

*****************************************************************************/
void Date_Up_Load(u8 msgid)
{
	u8 j = 0;
	u16 crccheck = 0;
	memset(USART_TX_BUF, 0, USART_REC_LEN);	//清空上位机通信串口接收缓存
	USART_TX_BUF[0] = 0xaa;
	USART_TX_BUF[1] = 0x55;
	
	USART_TX_BUF[4] = 0X00;		//length
	USART_TX_BUF[5] = msgid;  	//type

	switch ( msgid )
	{
	    case UPLOAD_ENCODER_AND_VELOCITY:
				USART_TX_BUF[4] = 16;  //数据长度 16字节
				memcpy(&USART_TX_BUF[6], encoder_valueleft, sizeof(encoder_valueleft));
				memcpy(&USART_TX_BUF[10], velocity_valueleft, sizeof(velocity_valueleft));
				memcpy(&USART_TX_BUF[14], encoder_valueright, sizeof(encoder_valueright));
				memcpy(&USART_TX_BUF[18], velocity_valueright, sizeof(velocity_valueright));
	        break;
	    case UPLOAD_STATUS_AND_CURRENT :
	       USART_TX_BUF[4] = 8;  //数据长度 8字节
					USART_TX_BUF[6] = status_word_left[1];
					USART_TX_BUF[7] = status_word_left[0];
					USART_TX_BUF[8] = current_valueleft[1];
					USART_TX_BUF[9] = current_valueleft[0];
			
					USART_TX_BUF[10] = status_word_right[1];
					USART_TX_BUF[11] = status_word_right[0];
					USART_TX_BUF[12] = current_valueright[1];
					USART_TX_BUF[13] = current_valueright[0];
//	        memcpy(&USART_TX_BUF[6], status_word_left, sizeof(status_word_left));
//					memcpy(&USART_TX_BUF[8], current_valueleft, sizeof(current_valueleft));
//	        memcpy(&USART_TX_BUF[10], status_word_right, sizeof(status_word_right));
//					memcpy(&USART_TX_BUF[12], current_valueright, sizeof(current_valueright));
	        break;
		case ERROR_STATUS_CMD :
	        USART_TX_BUF[4] = 0x03;  //数据长度
	        memcpy(&USART_TX_BUF[6], errcode, sizeof(errcode));
	        break;
		case BATTARY_MONITOR_MSGID_CMD:
				USART_TX_BUF[4] = 12;  //数据长度
				USART_TX_BUF[6] = battarymsg.chargingstatus ;
		
					USART_TX_BUF[7] = (u8)battarymsg.current;
					USART_TX_BUF[8] = battarymsg.current>>8;
					USART_TX_BUF[9] = (u8)battarymsg.voltage;
					USART_TX_BUF[10] = battarymsg.voltage>>8;
		
					USART_TX_BUF[11] = (u8)battarymsg.charging_times;
					USART_TX_BUF[12] = battarymsg.charging_times>>8;
		
					USART_TX_BUF[13] = (u8)battarymsg.remaining_total_capacity;
					USART_TX_BUF[14] = battarymsg.remaining_total_capacity>>8;
					USART_TX_BUF[15] = (u8)battarymsg.total_capacity;
					USART_TX_BUF[16] = battarymsg.total_capacity>>8;
		
					USART_TX_BUF[17] = battarymsg.remaining_battery_percentage;
//				memcpy(&USART_TX_BUF[7],&battarymsg.current,sizeof(battarymsg.current));
//				memcpy(&USART_TX_BUF[9],&battarymsg.voltage,sizeof(battarymsg.voltage));
//				memcpy(&USART_TX_BUF[11],&battarymsg.charging_times,sizeof(battarymsg.charging_times));
//				memcpy(&USART_TX_BUF[13],&battarymsg.remaining_total_capacity,sizeof(battarymsg.remaining_total_capacity));
//				memcpy(&USART_TX_BUF[15],&battarymsg.total_capacity,sizeof(battarymsg.total_capacity));								
//				USART_TX_BUF[17] = battarymsg.remaining_battery_percentage;
					break;
		case INFRARED_DISTANCE_CMD:
				USART_TX_BUF[4] = 8;  //数据长度
				memcpy(&USART_TX_BUF[6],&hw_dis[1],sizeof(hw_dis[0]));
				memcpy(&USART_TX_BUF[8],&hw_dis[0],sizeof(hw_dis[1]));
				memcpy(&USART_TX_BUF[10],&hw_dis[2],sizeof(hw_dis[2]));
				memcpy(&USART_TX_BUF[12],&hw_dis[3],sizeof(hw_dis[3]));
					break;
	    default:
	        break;
	}
	crccheck = GetCRC16(&USART_TX_BUF[6],USART_TX_BUF[4]);
	USART_TX_BUF[2]=(u8)crccheck;
	USART_TX_BUF[3]=crccheck>>8;

	#if 0
	if(msgid==UPLOAD_ENCODER_AND_VELOCITY) //上传编码器信息&速度 integer32 
	{
		USART_TX_BUF[3]=0x10;  //数据长度 16字节
		
		USART_TX_BUF[6]=encoder_valueleft[0];
		USART_TX_BUF[7]=encoder_valueleft[1];
		USART_TX_BUF[8]=encoder_valueleft[2];
		USART_TX_BUF[9]=encoder_valueleft[3];

		USART_TX_BUF[10]=velocity_valueleft[0];
		USART_TX_BUF[11]=velocity_valueleft[1];
		USART_TX_BUF[12]=velocity_valueleft[2];
		USART_TX_BUF[13]=velocity_valueleft[3];
		
		USART_TX_BUF[14]=encoder_valueright[0];
		USART_TX_BUF[15]=encoder_valueright[1];
		USART_TX_BUF[16]=encoder_valueright[2];
		USART_TX_BUF[17]=encoder_valueright[3];


        USART_TX_BUF[18]=velocity_valueright[0];
		USART_TX_BUF[19]=velocity_valueright[1];
		USART_TX_BUF[20]=velocity_valueright[2];
		USART_TX_BUF[21]=velocity_valueright[3];
		
		crccheck = GetCRC16(&USART_TX_BUF[6],16);
		USART_TX_BUF[5]=crccheck>>8;
		USART_TX_BUF[4]=(u8)crccheck;
	}
	else if(msgid==UPLOAD_STATUS_AND_CURRENT) //上传电机状态字&电流信息 integer16
	{
		USART_TX_BUF[3]=0x08;  //数据长度 8字节
		
		USART_TX_BUF[6]=status_word_left[0];
		USART_TX_BUF[7]=status_word_left[1];
		USART_TX_BUF[8]=current_valueleft[0];
		USART_TX_BUF[9]=current_valueleft[1];

		USART_TX_BUF[10]=status_word_right[0];
		USART_TX_BUF[11]=status_word_right[1];
		USART_TX_BUF[12]=current_valueright[0];
		USART_TX_BUF[13]=current_valueright[1];
		
		crccheck = GetCRC16(&USART_TX_BUF[6],8);
		USART_TX_BUF[5]=crccheck>>8;
		USART_TX_BUF[4]=(u8)crccheck;
	}
	else if(msgid==ERROR_STATUS_CMD) //状态错误编码
	{
		USART_TX_BUF[3]=0x03;  //数据长度
		USART_TX_BUF[6]=errcode[0];
		USART_TX_BUF[7]=errcode[1];
		USART_TX_BUF[8]=errcode[2];
		
		crccheck = GetCRC16(&USART_TX_BUF[6],3);
		USART_TX_BUF[4]=crccheck>>8;
		USART_TX_BUF[5]=(u8)crccheck;

	}
#endif
	if(USART_TX_BUF[4])
	{
		//USART_TX_BUF[3]+6中的 6是指2字节帧头+1字节数据类型+1字节数据包长度+2字节校验位
		for(j=0;j<USART_TX_BUF[4]+6;j++)
		{
			while(USART_GetFlagStatus(USART1,USART_FLAG_TC)==RESET);//循环发送，知道发送完毕
			USART_SendData(USART1,(uint8_t)USART_TX_BUF[j]);   
		}
	}
	memset(USART_TX_BUF, 0, USART_REC_LEN);	//清空上位机通信串口接收缓存
    /*2018.11.21修改，删除编码器缓存置零，置零导致编码器没有更新时上报的编码器值为零（错误状态）*/
//    memset(encoder_valueleft, 0, 4);
//    memset(encoder_valueright, 0, 4);
}

void Motion_Status(void)
{
     u16 num=0;
	 u8 statusmotion_flag=0;
	 canSend(CAN1,&Can_Msg_MotionStatus_NO[0]);
	 while(RxMSG.cob_id!=0X0581&&num<0xffff)
	{
		num++;
	};
    if(RxMSG.data[0]==data_cmd_MotionStatus[0]&&RxMSG.data[1]==data_cmd_MotionStatus[1]&&RxMSG.data[2]==data_cmd_MotionStatus[2]&&RxMSG.data[3]==data_cmd_MotionStatus[3])//判断接收到的报文是否为电机1状态报文
	{
		if(RxMSG.data[5]&0x08)//bit3表示状态出现错误
		{
			MotionHalts(); //停止左右电机
			errcode[0]=0x01;
			errcode[1]=0x00;
			errcode[2]=0x01;
			statusmotion_flag++;
		}
		
	}
	num=0;
	delay_ms(2);
	canSend(CAN1,&Can_Msg_MotionStatus_NO[1]);
	 while(RxMSG.cob_id!=0X0582&&num<0xffff)
	{
		num++;
	};
    if(RxMSG.data[0]==data_cmd_MotionStatus[0]&&RxMSG.data[1]==data_cmd_MotionStatus[1]&&RxMSG.data[2]==data_cmd_MotionStatus[2]&&RxMSG.data[3]==data_cmd_MotionStatus[3])//判断接收到的报文是否为电机2状态报文
	{
		if(RxMSG.data[5]&0x08)//bit3表示状态出现错误
		{
			MotionHalts(); //停止左右电机
			errcode[0]=0x01;
			errcode[2]+=0x02;
			statusmotion_flag++;	
		}	
	}
	if(statusmotion_flag)
	{
		Date_Up_Load(ERROR_STATUS_CMD);//上报电机状态故障
		statusmotion_flag=0;
	}
	memset(errcode,0,6);

}

/* 设置云台状态 */
void Set_Yuntai(s8 data)
{
	if(data==0x01)	//开启云台
		YUNTAI=1;
	else 			//关闭云台
		YUNTAI=0;

}

/* 设置三色灯状态 */
void Set_Alarm(s8 cmd)
{
	if(cmd ==0x00);
	else if(cmd == 0x01) {BJRED=1;	 	BJYELLOW=0;BJGREEN =0;BJBEEP =0;}//红灯亮
	else if(cmd == 0x02) {BJYELLOW=1;	BJBEEP =0;BJRED=0;BJBEEP =0;}	//黄灯亮
	else if(cmd == 0x04) {BJGREEN=1; 	BJBEEP =0;BJRED=0;BJYELLOW=0;}	//绿灯亮
	else if(cmd == 0x08) {BJBEEP=1;  	BJRED=0;BJYELLOW=0;BJGREEN =0;}	//蜂鸣器响
	else if(cmd == 0x09) {BJBEEP=1; 	BJRED=1;BJYELLOW=0;BJGREEN =0;}	//红灯亮&蜂鸣器响
	else if(cmd == 0x0a) {BJBEEP=1;		BJYELLOW=1;BJBEEP =0;BJRED=0;}	//黄灯亮&蜂鸣器响
	else 																//绿灯常亮
	{
		BJBEEP =0; 
		BJRED=0;
		BJYELLOW=0;
		BJGREEN =1;
	}

}
// /*无线充电设置函数
// * 返回值：0充电无故障，1无线充电故障
// *
//*/
//u8 Wireless_Charging_Status(s8 cmd)
//{
//    /*充电使能*/
//    if(cmd ==0x00) WXCD_EN=0;//无线充电结束，设置为非使能状态
//    else if(cmd == 0x01)WXCD_EN=1;//使能无线充电
//    return 0;
//}


