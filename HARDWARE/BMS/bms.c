#include "sys.h"		    
#include "bms.h"	 
#include "delay.h"


  /* 电池信息获取命令 */
u8 battery_cmdbuf[4][8]={	{0x7f,0x10,0x02,0x06,0x11,0x58,0x0d,0x0a},	//获取当前总电压值命令，单位为mV，UINT32型
							{0x3a,0x16,0x0a,0x00,0x20,0x00,0x0d,0x0a},	//获取当前总电流值命令,单位为mA，INT32型，负值放电，正值充电
							{0x3a,0x16,0x0f,0x00,0x25,0x00,0x0d,0x0a},	//获取当前剩余容量命令，单位mAh
							{0x3a,0x16,0x0d,0x00,0x23,0x00,0x0d,0x0a}	//获取当前剩余容量百分比命令,单位无
 };

const u8 bsmbuffer[6]={0x7f,0x10,0x02,0x06,0x11,0x58};
u8 chargingstatus;
u16 current;
u8 voltage[2];
u8 charging_times[2];
u8 remaining_total_capacity[2];
u8 total_capacity[2];
u8 remaining_battery_percentage;
//BattaryMSG *battarymsg;
extern BattaryMSG battarymsg;
void BattaryMSGAnalysis(u8 *rs485buf,BattaryMSG *battarymsg)
{
	/* 使用结构体指针，最终要采取的方式 */
     battarymsg->chargingstatus  = rs485buf[5];
     battarymsg->current         = rs485buf[9]+ rs485buf[10]*256;
     battarymsg->voltage         = rs485buf[15] + rs485buf[16]*256;
     battarymsg->charging_times  = rs485buf[19] + rs485buf[20]*256;
     battarymsg->remaining_total_capacity  = rs485buf[21] + (rs485buf[22]*256);
     battarymsg->total_capacity  = rs485buf[23] + (rs485buf[24]*256);
     battarymsg->remaining_battery_percentage         = (float)(battarymsg->remaining_total_capacity*100/battarymsg->total_capacity);
	
	/* 使用数组存储*/
//	   chargingstatus  = rs485buf[5];
//     current         = rs485buf[9];
//     voltage[0]      = rs485buf[15];
//		 voltage[1]      = rs485buf[16];
//     charging_times[0]  = rs485buf[19];
//		 charging_times[0]  = rs485buf[20];
//     remaining_total_capacity[0]  = rs485buf[21];
//		 remaining_total_capacity[1]  = rs485buf[22];
//			total_capacity[0]  = rs485buf[23];
//			total_capacity[1]  = rs485buf[24];
//     remaining_battery_percentage         = (float)((remaining_total_capacity[0]+remaining_total_capacity[1]*256)*100/(total_capacity[0]+total_capacity[1]));
		
}
void RequestBatteryInformation(BattaryMSG *msg)
{
	u8 key;
	u8 rs485buf[35]={0}; 
	
	RS485_Send_Data((u8 *)bsmbuffer,6);//发送5个字节
  delay_ms(50);
	RS485_Receive_Data(rs485buf,&key);
  BattaryMSGAnalysis(rs485buf,msg);
}


