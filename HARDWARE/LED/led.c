#include "led.h" 


//初始化PF9和PF10为输出口.并使能这两个口的时钟		    
//LED IO初始化
void LED_Init(void)
{    	 
  GPIO_InitTypeDef  GPIO_InitStructure;

#ifdef BEIDA_CODE_V1
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG, ENABLE);//使能GPIOF时钟

  //GPIOF9,F10初始化设置
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//普通输出模式
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
  GPIO_Init(GPIOG, &GPIO_InitStructure);//初始化
	
  GPIO_SetBits(GPIOG,GPIO_Pin_13 | GPIO_Pin_14);//GPIOF9,F10设置高，灯灭	
#endif
#ifdef INSPECTIONROBOT_CODE_V2
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);//使艤GPIOF时讚

  //GPIOF9,F10缘始郫狮變
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11 | GPIO_Pin_12;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//菚通摔远模式
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//螁维摔远
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//蕪-
  GPIO_Init(GPIOA, &GPIO_InitStructure);//缘始郫
	
  GPIO_SetBits(GPIOA,GPIO_Pin_11 | GPIO_Pin_12);//GPIOF9,F10狮變贌矛謫陌	
#endif
	
}






