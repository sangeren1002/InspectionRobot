/***********************************************************************************
 * 文 件 名   : main.h
 * 负 责 人   : jishubao
 * 创建日期   : 2018年11月21日
 * 文件描述   : main.c 的头文件
 * 版权说明   : Copyright (c) 2008-2018   杭州国辰机器人科技有限公司
 * 其    他   : 
 * 修改日志   : 
***********************************************************************************/

#ifndef __MAIN_H__
#define __MAIN_H__


#ifdef __cplusplus
#if __cplusplus
extern "C"{
#endif
#endif /* __cplusplus */

#define DEFINE_TEST_FRE		0		//定义为1开始上传数据频率测试,0则关闭测试
#define BOARD_SELECTION		3		//定义为 1为正点原子开发板 定义为 2 为LIUGC201809自制电路板 定义为 3为LIUGC201903自制电路板
#define DEBUG_BATTERY 		0		//定义为0 为不请求485电池信息，定义为1为请求电池信息
#define DEBUG_INFRARED_DIS 	0		//定义为0 为不请求红外测距传感器数据，定义为1请求红外测距数据


extern void AnalysisMessagefromDriver(void);
extern void GetValCurrent(u8 idnode);
extern void GetValEncoder(u8 idnode);
extern void GetValStatus(u8 idnode);
extern void GetValVelocity(u8 idnode);
extern int main(void);
extern void ModeSelection(void);
extern void PC_Data_Analysis(char *recvfromPCbuf);
extern void SYS_Init(void);

#ifdef __cplusplus
#if __cplusplus
}
#endif
#endif /* __cplusplus */


#endif /* __MAIN_H__ */
