#include "infrared.h" 

//初始化ADC															   
void  Adc_Init(void)
{    
  GPIO_InitTypeDef  GPIO_InitStructure;
	ADC_CommonInitTypeDef ADC_CommonInitStructure;
	ADC_InitTypeDef       ADC_InitStructure;
	
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB , ENABLE);//使能GPIOA时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC , ENABLE);//使能GPIOA时钟
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE); //使能ADC1时钟

  //先初始化ADC1通道5 IO口
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;//PB0 ADC1_IN8  PB1 ADC1_IN9
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;//模拟输入
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN ;//不带上下拉
  GPIO_Init(GPIOB, &GPIO_InitStructure);//初始化  
	
	  //先初始化ADC1通道
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 |GPIO_Pin_4;//PC4  ADC1_IN14   PC5  ADC1_IN15
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;//模拟输入
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;//不带上下拉
  GPIO_Init(GPIOC, &GPIO_InitStructure);//初始化  

	RCC_APB2PeriphResetCmd(RCC_APB2Periph_ADC1,ENABLE);	  //ADC1复位
	RCC_APB2PeriphResetCmd(RCC_APB2Periph_ADC1,DISABLE);	//复位结束	 
 
	
  ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;//独立模式
  ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;//两个采样阶段之间的延迟5个时钟
  ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled; //DMA失能
  ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div4;//预分频4分频。ADCCLK=PCLK2/4=84/4=21Mhz,ADC时钟最好不要超过36Mhz 
  ADC_CommonInit(&ADC_CommonInitStructure);//初始化
	
  ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;//12位模式
  ADC_InitStructure.ADC_ScanConvMode = DISABLE;//非扫描模式	
  ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;//关闭连续转换
  ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;//禁止触发检测，使用软件触发
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;//右对齐	
  ADC_InitStructure.ADC_NbrOfConversion = 1;//1个转换在规则序列中 也就是只转换规则序列1 
  ADC_Init(ADC1, &ADC_InitStructure);//ADC初始化
	
 
	ADC_Cmd(ADC1, ENABLE);//开启AD转换器	

}				  
//获得ADC值
//ch: @ref ADC_channels 
//通道值 0~16取值范围为：ADC_Channel_0~ADC_Channel_16
//返回值:转换结果
u16 Get_Adc(u8 ch)   
{
	  	//设置指定ADC的规则组通道，一个序列，采样时间
	ADC_RegularChannelConfig(ADC1, ch, 1, ADC_SampleTime_480Cycles );	//ADC1,ADC通道,480个周期,提高采样时间可以提高精确度			    
  
	ADC_SoftwareStartConv(ADC1);		//使能指定的ADC1的软件转换启动功能	
	 
	while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC ));//等待转换结束

	return ADC_GetConversionValue(ADC1);	//返回最近一次ADC1规则组的转换结果
}
//冒泡排序
void bubble_sort(u32 a[], int n)
{
    int i, j;
		u32 temp;
    for (j = 0; j < n - 1; j++)
        for (i = 0; i < n - 1 - j; i++)
        {
            if(a[i] > a[i + 1])
            {
                temp = a[i];
                a[i] = a[i + 1];
                a[i + 1] = temp;
            }
        }
}
//获取通道ch的转换值，取times次,然后平均 
//ch:通道编号
//times:获取次数
//返回值:通道ch的times次转换结果平均值
u16 Get_Adc_Average(u8 ch,u8 times)
{
	u32 temp_val[20];
	u8 t;
	for(t=0;t<times;t++)
	{
		temp_val[t]=Get_Adc(ch);
		//delay_ms(5);
	}
	//bubble_sort(temp_val,times); //从小到大排序
	return temp_val[times/2];
}
void Get_Distance_Infrared(u16* dis)
{
	u8 i;
	u16 adcx[4] = {0};
	double temp;
	double Inverse_number;
	
	adcx[0]=Get_Adc_Average(ADC_Channel_8,1);//获取通道5的转换值，5次取中位数
	adcx[1]=Get_Adc_Average(ADC_Channel_9,1);//获取通道5的转换值，5次取中位数
	adcx[2]=Get_Adc_Average(ADC_Channel_14,1);//获取通道5的转换值，5次取中位数
	adcx[3]=Get_Adc_Average(ADC_Channel_15,1);//获取通道5的转换值，5次取中位数
	for(i=0;i<4;i++)
	{
		
		temp=(double)adcx[i]*(3.3/4096);          //获取计算后的带小数的实际电压值，比如3.1111

		if((double)temp<0.393)														//超出量程
				Inverse_number=0.006666667;
		else if(temp>=0.393&&temp<1.525)					//40-150cm
				Inverse_number=0.0162*temp + 0.0003;	//y=0.0162*x + 0.0003
		else if(temp>=1.525&&temp<1.995)					//30-40cm
				Inverse_number=0.0177*temp - 0.0002;	//y=0.0177*x - 0.0002
		else if(temp>=1.995&&temp<2.525)					//20-30cm
				Inverse_number=0.0314*temp - 0.0294;	//y=0.0314*x - 0.0294
		else if(temp>=2.525&&temp<=3.3)						//15-20cm
				Inverse_number=0.0741*temp - 0.137;	//y=0.0741*x - 0.137
			dis[i]=1/Inverse_number;
	}

}

