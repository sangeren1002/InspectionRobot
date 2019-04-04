#ifndef __LED_H
#define __LED_H
#include "sys.h"

//////////////////////////////////////////////////////////////////////////////////	 
//±¾³ÌĞòÖ»¹©Ñ§Ï°Ê¹ÓÃ£¬Î´¾­×÷ÕßĞí¿É£¬²»µÃÓÃÓÚÆäËüÈÎºÎÓÃÍ¾
//ALIENTEK STM32F407¿ª·¢°å
//LEDÇı¶¯´úÂë	   
//ÕıµãÔ­×Ó@ALIENTEK
//¼¼ÊõÂÛÌ³:www.openedv.com
//´´½¨ÈÕÆÚ:2014/5/2
//°æ±¾£ºV1.0
//°æÈ¨ËùÓĞ£¬µÁ°æ±Ø¾¿¡£
//Copyright(C) ¹ãÖİÊĞĞÇÒíµç×Ó¿Æ¼¼ÓĞÏŞ¹«Ë¾ 2014-2024
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 	

#ifdef BEIDA_CODE_V1
//LED¶Ë¿Ú¶¨Òå
#define LED0 PGout(13)	// DS0
#define LED1 PGout(14)	// DS1	 
#endif
#ifdef INSPECTIONROBOT_CODE_V2
//LED×‹à šÖ¨Ó¥
#define LED0 PAout(11)	// DS0
#define LED1 PAout(12)	// DS1	 
#endif
void LED_Init(void);//³õÊ¼»¯		 				    
#endif
