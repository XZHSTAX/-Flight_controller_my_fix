#ifndef _DRV_TIME_H_
#define _DRV_TIME_H_

#include "common.h"

#define TICK_PER_SECOND	1000
#define TICK_US	(1000000/TICK_PER_SECOND)

extern volatile uint32_t sysTickUptime;

void SysTick_Configuration(void);
uint32_t GetSysTime_us(void);
void Delay_us(uint32_t);
void Delay_ms(uint32_t);
void sys_time(void);
u32 SysTick_GetTick(void);

#endif

/******************* (C) COPYRIGHT 2018 DY EleTe *****END OF FILE************/
