/***********************************************************************************
 * �� �� ��   : main.h
 * �� �� ��   : jishubao
 * ��������   : 2018��11��21��
 * �ļ�����   : main.c ��ͷ�ļ�
 * ��Ȩ˵��   : Copyright (c) 2008-2018   ���ݹ��������˿Ƽ����޹�˾
 * ��    ��   : 
 * �޸���־   : 
***********************************************************************************/

#ifndef __MAIN_H__
#define __MAIN_H__


#ifdef __cplusplus
#if __cplusplus
extern "C"{
#endif
#endif /* __cplusplus */

#define DEFINE_TEST_FRE 0 //����Ϊ1��ʼ�ϴ�����Ƶ�ʲ���,0��رղ���
//#define BOARD_SELECTION  //����Ϊ Ϊ����ԭ�ӿ����� ������ΪLIUGC201809���Ƶ�·��




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