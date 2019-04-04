#ifndef __BMS_H
#define __BMS_H			 
#include "sys.h"	 			
#include "rs485.h"


typedef struct{
    u16  voltage;
    u16  current;
    u8  remaining_battery_percentage;
    u16  total_capacity;
    u16  remaining_total_capacity;
    u8  chargingstatus;
    u16  charging_times; 
}BattaryMSG;



void BattaryMSGAnalysis(u8 *rs485buf,BattaryMSG *battarymsg);
void RequestBatteryInformation(BattaryMSG *msg);
#endif	   
















