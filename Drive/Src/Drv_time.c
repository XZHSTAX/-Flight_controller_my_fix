/******************** (C) COPYRIGHT 2018 DY EleTe ********************************
 * 作者    ：徳研电科
 * 官网    ：www.gototi.com
 * 描述    ：定时器驱动和滴答配置
**********************************************************************************/
#include "Drv_time.h"
#include "include.h"
#include "Drv_led.h"

volatile uint32_t sysTickUptime = 0;

void  SysTick_Configuration ( void )
{ 
  // 设置计数器数满多少数重置 这里为120000000/1000= 120000，系统一秒数120000000，所以数完120000用时1ms
  MAP_SysTickPeriodSet(g_ui32SysClock/1000);    // 时间基准：1ms 
  MAP_SysTickIntEnable();                       // 运行中断，中断触发后会自动重装载
  MAP_SysTickEnable();                          // 使能SysTick，开始计数
}

// 返回当前系统时间（us）
uint32_t GetSysTime_us ( void )
{
    register uint32_t ms;
    u32 value;
    ms = sysTickUptime;
    value = ms * TICK_US + ( (g_ui32SysClock/1000) - SysTickValueGet() ) * TICK_US / (g_ui32SysClock/1000);		//ms*1000+(重装载值-当前倒计数值)/重装载值[对应1ms]*1000=SysTime_us
    return value;
}

void Delay_us ( uint32_t us )
{
    uint32_t now = GetSysTime_us();
    while ( GetSysTime_us() - now < us );
}

void Delay_ms ( uint32_t ms )
{
    while ( ms-- )
        Delay_us ( 1000 );
}

u32 systime_ms;

void sys_time()
{
	systime_ms++;
}

// 返回系统当前时间
u32 SysTick_GetTick(void)
{
	return systime_ms;
}

// 每1ms进入一次中断
void SysTick_Handler(void)
{  
  sysTickUptime++;      // 记录中断次数，对应毫秒数
  sys_time();           // 变量systime_ms++
  LED_1ms_DRV();        // 控制LED灯
}

/******************* (C) COPYRIGHT 2018 DY EleTe *****END OF FILE************/
