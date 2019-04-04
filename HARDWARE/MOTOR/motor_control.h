#ifndef __MOTOR_CONTROL_H
#define __MOTOR_CONTROL_H
#include "sys.h"
 #include "can.h"
 #include "bms.h"
typedef enum{
	MOTOR_CTL_MSGID_CMD 			= 0X01,
	UPLOAD_STATUS_AND_CURRENT       = 0X02,	
	UPLOAD_ENCODER_AND_VELOCITY     = 0X03,	
	BATTARY_MONITOR_MSGID_CMD 		= 0X04,
	RESET_STATUS_MSGID_CMD	 		= 0X05,
	ENCODER_RESET_CMD	  			= 0X06,
	OPEN_YUNTAI_CMD	  				= 0X07,
	ALARM_CMD	  					= 0X08,
    ENABLE_CHARGE_CMD	  		    = 0X09,
    INFRARED_DISTANCE_CMD           = 0X0A,
	ERROR_STATUS_CMD	  			= 0XFF,
}CMD;
	

typedef enum{
		NO_ERROR =0x00,
		SPEEDOUT_ERROR = 0x01,
		BATTERY_ERROR =0x02
}etERROR;

#define REDUCTION_RATIO     28
#define SPEED_COEFFICIENT   240

extern u8 encoder_valueleft[4];
extern u8 encoder_valueright[4];
extern u8 velocity_valueleft[4],velocity_valueright[4];
extern u8 current_valueleft[2],current_valueright[2];
extern u8 errcode[3];
extern u8 status_word_left[2];
extern u8 status_word_right[2];
extern u8 startcmdstatus;

//SYNC报文
extern Message Can_Msg_SYNCMY;


extern u8 length;
extern u8 sequence;



void SetCanopenParameter(u8 id);
void SetMotorTargetVelocity(s32 vel_left,s32 vel_right);
void MotorInit(void);
void MotionHalts(void);
void StartMotion(void);
u8 Set_Speed(float leftvalue,float rightvalue);
void Turn_Left(void);
void Turn_Right(void);
void Move_Forward(u8 level);
void Back_Off(u8 level);
void Get_Encoder_Value(void);
void Get_CurElectric_Value(void);
void Get_CurSpeed_Value(void);
void Date_Up_Load(u8 msgid);
void Get_Swerve_Speed_PS2(void);
void Motion_Status(void);
void StatusWordAnalysis(void);
void AnalysisMessagefromDriver(void);

void Set_Yuntai(s8 data);
void Set_Alarm(s8 cmd);
//u8 Wireless_Charging_Status(s8 cmd);

unsigned short GetCRC16(unsigned char *ptr,unsigned char len);









#endif

