/******************** (C) COPYRIGHT 2018 DY EleTe ********************************
 * 作者    ：徳研电科
 * 官网    ：www.gototi.com
 * 描述    ：PWM输入捕获
**********************************************************************************/
#include "Drv_pwm_in.h"
#include "DY_RC.h"
#include "include.h"

u32 DY_Rc_Pwm_In[8];

u16 Rc_Pwm_In[8];

void PWM_IN_Init ()
{
  u32 ValuePWMDivFactor;
  u32 ValueTemp;
  u32 ValueLoadSetLower,ValueLoadSetHiger;
  
  MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
  MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1);
  MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER2);
  MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER3);
  MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER4);
  MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER5);
  
  MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
  MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
  MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOM);
  
  MAP_GPIOPinConfigure(GPIO_PD0_T0CCP0);
  MAP_GPIOPinConfigure(GPIO_PD2_T1CCP0);
  MAP_GPIOPinConfigure(GPIO_PM0_T2CCP0);
  MAP_GPIOPinConfigure(GPIO_PD4_T3CCP0);
  MAP_GPIOPinConfigure(GPIO_PD6_T4CCP0);
  MAP_GPIOPinConfigure(GPIO_PB2_T5CCP0);
  
  MAP_GPIOPinTypeTimer(GPIO_PORTB_BASE, GPIO_PIN_2);
  MAP_GPIOPinTypeTimer(GPIO_PORTD_BASE, GPIO_PIN_0);
  MAP_GPIOPinTypeTimer(GPIO_PORTD_BASE, GPIO_PIN_2);
  MAP_GPIOPinTypeTimer(GPIO_PORTD_BASE, GPIO_PIN_4);
  MAP_GPIOPinTypeTimer(GPIO_PORTD_BASE, GPIO_PIN_6);
  MAP_GPIOPinTypeTimer(GPIO_PORTM_BASE, GPIO_PIN_0);
  
  MAP_TimerConfigure(TIMER0_BASE, TIMER_CFG_SPLIT_PAIR | TIMER_CFG_A_CAP_TIME_UP);
  MAP_TimerConfigure(TIMER1_BASE, TIMER_CFG_SPLIT_PAIR | TIMER_CFG_A_CAP_TIME_UP);
  MAP_TimerConfigure(TIMER2_BASE, TIMER_CFG_SPLIT_PAIR | TIMER_CFG_A_CAP_TIME_UP);
  MAP_TimerConfigure(TIMER3_BASE, TIMER_CFG_SPLIT_PAIR | TIMER_CFG_A_CAP_TIME_UP);
  MAP_TimerConfigure(TIMER4_BASE, TIMER_CFG_SPLIT_PAIR | TIMER_CFG_A_CAP_TIME_UP);
  MAP_TimerConfigure(TIMER5_BASE, TIMER_CFG_SPLIT_PAIR | TIMER_CFG_A_CAP_TIME_UP);
  
  MAP_TimerControlEvent(TIMER0_BASE, TIMER_A, TIMER_EVENT_POS_EDGE);
  MAP_TimerControlEvent(TIMER1_BASE, TIMER_A, TIMER_EVENT_POS_EDGE);
  MAP_TimerControlEvent(TIMER2_BASE, TIMER_A, TIMER_EVENT_POS_EDGE);
  MAP_TimerControlEvent(TIMER3_BASE, TIMER_A, TIMER_EVENT_POS_EDGE);
  MAP_TimerControlEvent(TIMER4_BASE, TIMER_A, TIMER_EVENT_POS_EDGE);
  MAP_TimerControlEvent(TIMER5_BASE, TIMER_A, TIMER_EVENT_POS_EDGE);
  
  MAP_TimerControlStall(TIMER0_BASE, TIMER_A, true);
  MAP_TimerControlStall(TIMER1_BASE, TIMER_A, true);
  MAP_TimerControlStall(TIMER2_BASE, TIMER_A, true);
  MAP_TimerControlStall(TIMER3_BASE, TIMER_A, true);
  MAP_TimerControlStall(TIMER4_BASE, TIMER_A, true);
  MAP_TimerControlStall(TIMER5_BASE, TIMER_A, true);
  
  MAP_TimerPrescaleSet(TIMER0_BASE, TIMER_A, 1);
  MAP_TimerPrescaleSet(TIMER1_BASE, TIMER_A, 1);
  MAP_TimerPrescaleSet(TIMER2_BASE, TIMER_A, 1);
  MAP_TimerPrescaleSet(TIMER3_BASE, TIMER_A, 1);
  MAP_TimerPrescaleSet(TIMER4_BASE, TIMER_A, 1);
  MAP_TimerPrescaleSet(TIMER5_BASE, TIMER_A, 1);
  
  ValuePWMDivFactor = (g_ui32SysClock/400);
  ValueTemp = ValuePWMDivFactor - 1;
  ValueLoadSetLower = ValueTemp & 0xFFFF;
  ValueLoadSetHiger = ValueTemp & 0xFFFF0000;
  ValueLoadSetHiger = ValueLoadSetHiger >> 16;
  
  MAP_TimerPrescaleSet(TIMER0_BASE, TIMER_A, ValueLoadSetHiger);
  MAP_TimerLoadSet(TIMER0_BASE, TIMER_A, ValueLoadSetLower);
  MAP_TimerPrescaleSet(TIMER1_BASE, TIMER_A, ValueLoadSetHiger);
  MAP_TimerLoadSet(TIMER1_BASE, TIMER_A, ValueLoadSetLower);
  MAP_TimerPrescaleSet(TIMER2_BASE, TIMER_A, ValueLoadSetHiger);
  MAP_TimerLoadSet(TIMER2_BASE, TIMER_A, ValueLoadSetLower);
  MAP_TimerPrescaleSet(TIMER3_BASE, TIMER_A, ValueLoadSetHiger);
  MAP_TimerLoadSet(TIMER3_BASE, TIMER_A, ValueLoadSetLower);
  MAP_TimerPrescaleSet(TIMER4_BASE, TIMER_A, ValueLoadSetHiger);
  MAP_TimerLoadSet(TIMER4_BASE, TIMER_A, ValueLoadSetLower);
  MAP_TimerPrescaleSet(TIMER5_BASE, TIMER_A, ValueLoadSetHiger);
  MAP_TimerLoadSet(TIMER5_BASE, TIMER_A, ValueLoadSetLower);
  
  MAP_TimerIntEnable(TIMER0_BASE, TIMER_CAPA_EVENT);
  MAP_TimerIntEnable(TIMER1_BASE, TIMER_CAPA_EVENT);
  MAP_TimerIntEnable(TIMER2_BASE, TIMER_CAPA_EVENT);
  MAP_TimerIntEnable(TIMER3_BASE, TIMER_CAPA_EVENT);
  MAP_TimerIntEnable(TIMER4_BASE, TIMER_CAPA_EVENT);
  MAP_TimerIntEnable(TIMER5_BASE, TIMER_CAPA_EVENT);
  
  MAP_IntEnable(INT_TIMER0A);
  MAP_IntEnable(INT_TIMER1A);
  MAP_IntEnable(INT_TIMER2A);
  MAP_IntEnable(INT_TIMER3A);
  MAP_IntEnable(INT_TIMER4A);
  MAP_IntEnable(INT_TIMER5A);
  
  MAP_TimerEnable(TIMER0_BASE, TIMER_A);
  MAP_TimerEnable(TIMER1_BASE, TIMER_A);
  MAP_TimerEnable(TIMER2_BASE, TIMER_A);
  MAP_TimerEnable(TIMER3_BASE, TIMER_A);
  MAP_TimerEnable(TIMER4_BASE, TIMER_A);
  MAP_TimerEnable(TIMER5_BASE, TIMER_A);
}

void TIMER0A_IRQHandler(void)           //PD0
{
  u32 count1;
  static u32 temp_cnt1,temp_cnt1_2;
  
  ch_watch_dog_feed(CH1);
  
  MAP_TimerIntClear(TIMER0_BASE, TIMER_CAPA_EVENT);
  
  count1 = MAP_TimerValueGet(TIMER0_BASE, TIMER_A);
  
  if(MAP_GPIOPinRead(GPIO_PORTD_BASE, GPIO_PIN_0))
  {
    MAP_TimerControlEvent(TIMER0_BASE, TIMER_A, TIMER_EVENT_NEG_EDGE);
    temp_cnt1 = count1;
  }
  else
  {
    MAP_TimerControlEvent(TIMER0_BASE, TIMER_A, TIMER_EVENT_POS_EDGE);
    temp_cnt1_2 = count1;
    if(temp_cnt1_2 >= temp_cnt1)
    {
      DY_Rc_Pwm_In[0] = (temp_cnt1_2 - temp_cnt1);
      Rc_Pwm_In[0] = DY_Rc_Pwm_In[0]/120;
    }
    else
    {
      DY_Rc_Pwm_In[0] = ((g_ui32SysClock/400) - temp_cnt1 + temp_cnt1_2);
      Rc_Pwm_In[0] = DY_Rc_Pwm_In[0]/120;
    }
  }
}

void TIMER1A_IRQHandler(void)           //PD2
{
  u32 count2;
  static u32 temp_cnt2,temp_cnt2_2;
  
  ch_watch_dog_feed(CH2);
  
  MAP_TimerIntClear(TIMER1_BASE, TIMER_CAPA_EVENT);
  
  count2 = MAP_TimerValueGet(TIMER1_BASE, TIMER_A);
  
  if(MAP_GPIOPinRead(GPIO_PORTD_BASE, GPIO_PIN_2))
  {
    MAP_TimerControlEvent(TIMER1_BASE, TIMER_A, TIMER_EVENT_NEG_EDGE);
    temp_cnt2 = count2;
  }
  else
  {
    MAP_TimerControlEvent(TIMER1_BASE, TIMER_A, TIMER_EVENT_POS_EDGE);
    temp_cnt2_2 = count2;
    if(temp_cnt2_2 >= temp_cnt2)
    {
      DY_Rc_Pwm_In[1] = (temp_cnt2_2 - temp_cnt2);
      Rc_Pwm_In[1] = DY_Rc_Pwm_In[1]/120;
    }
    else
    {
      DY_Rc_Pwm_In[1] = ((g_ui32SysClock/400) - temp_cnt2 + temp_cnt2_2);
      Rc_Pwm_In[1] = DY_Rc_Pwm_In[1]/120;
    }
  }
}

void TIMER3A_IRQHandler(void)           //PD4
{
  u32 count3;
  static u32 temp_cnt3,temp_cnt3_2;
  
  ch_watch_dog_feed(CH3);
  
  MAP_TimerIntClear(TIMER3_BASE, TIMER_CAPA_EVENT);
  
  count3 = MAP_TimerValueGet(TIMER3_BASE, TIMER_A);
  
  if(MAP_GPIOPinRead(GPIO_PORTD_BASE, GPIO_PIN_4))
  {
    MAP_TimerControlEvent(TIMER3_BASE, TIMER_A, TIMER_EVENT_NEG_EDGE);
    temp_cnt3 = count3;
  }
  else
  {
    MAP_TimerControlEvent(TIMER3_BASE, TIMER_A, TIMER_EVENT_POS_EDGE);
    temp_cnt3_2 = count3;
    if(temp_cnt3_2 >= temp_cnt3)
    {
      DY_Rc_Pwm_In[2] = (temp_cnt3_2 - temp_cnt3);
      Rc_Pwm_In[2] = DY_Rc_Pwm_In[2]/120;
    }
    else
    {
      DY_Rc_Pwm_In[2] = ((g_ui32SysClock/400) - temp_cnt3 + temp_cnt3_2);
      Rc_Pwm_In[2] = DY_Rc_Pwm_In[2]/120;
    }
  }
}

void TIMER4A_IRQHandler(void)           //PD6
{
  u32 count4;
  static u32 temp_cnt4,temp_cnt4_2;
  
  ch_watch_dog_feed(CH4);
  
  MAP_TimerIntClear(TIMER4_BASE, TIMER_CAPA_EVENT);
  
  count4 = MAP_TimerValueGet(TIMER4_BASE, TIMER_A);
  
  if(MAP_GPIOPinRead(GPIO_PORTD_BASE, GPIO_PIN_6))
  {
    MAP_TimerControlEvent(TIMER4_BASE, TIMER_A, TIMER_EVENT_NEG_EDGE);
    temp_cnt4 = count4;
  }
  else
  {
    MAP_TimerControlEvent(TIMER4_BASE, TIMER_A, TIMER_EVENT_POS_EDGE);
    temp_cnt4_2 = count4;
    if(temp_cnt4_2 >= temp_cnt4)
    {
      DY_Rc_Pwm_In[3] = (temp_cnt4_2 - temp_cnt4);
      Rc_Pwm_In[3] = DY_Rc_Pwm_In[3]/120;
    }
    else
    {
      DY_Rc_Pwm_In[3] = ((g_ui32SysClock/400) - temp_cnt4 + temp_cnt4_2);
      Rc_Pwm_In[3] = DY_Rc_Pwm_In[3]/120;
    }
  }
}

void TIMER2A_IRQHandler(void)           //PM0
{
  u32 count5;
  static u32 temp_cnt5,temp_cnt5_2;
  
  ch_watch_dog_feed(CH5);
  
  MAP_TimerIntClear(TIMER2_BASE, TIMER_CAPA_EVENT);
  
  count5 = MAP_TimerValueGet(TIMER2_BASE, TIMER_A);
  
  if(MAP_GPIOPinRead(GPIO_PORTM_BASE, GPIO_PIN_0))
  {
    MAP_TimerControlEvent(TIMER2_BASE, TIMER_A, TIMER_EVENT_NEG_EDGE);
    temp_cnt5 = count5;
  }
  else
  {
    MAP_TimerControlEvent(TIMER2_BASE, TIMER_A, TIMER_EVENT_POS_EDGE);
    temp_cnt5_2 = count5;
    if(temp_cnt5_2 >= temp_cnt5)
    {
      DY_Rc_Pwm_In[4] = (temp_cnt5_2 - temp_cnt5);
      Rc_Pwm_In[4] = DY_Rc_Pwm_In[4]/120;
    }
    else
    {
      DY_Rc_Pwm_In[4] = ((g_ui32SysClock/400) - temp_cnt5 + temp_cnt5_2);
      Rc_Pwm_In[4] = DY_Rc_Pwm_In[4]/120;
    }
  }
}

void TIMER5A_IRQHandler(void)           //PB2
{
  u32 count6;
  static u32 temp_cnt6,temp_cnt6_2;
  
  ch_watch_dog_feed(CH6);
  
  MAP_TimerIntClear(TIMER5_BASE, TIMER_CAPA_EVENT);
  
  count6 = MAP_TimerValueGet(TIMER5_BASE, TIMER_A);
  
  if(MAP_GPIOPinRead(GPIO_PORTB_BASE, GPIO_PIN_2))
  {
    MAP_TimerControlEvent(TIMER5_BASE, TIMER_A, TIMER_EVENT_NEG_EDGE);
    temp_cnt6 = count6;
  }
  else
  {
    MAP_TimerControlEvent(TIMER5_BASE, TIMER_A, TIMER_EVENT_POS_EDGE);
    temp_cnt6_2 = count6;
    if(temp_cnt6_2 >= temp_cnt6)
    {
      DY_Rc_Pwm_In[5] = (temp_cnt6_2 - temp_cnt6);
      Rc_Pwm_In[5] = DY_Rc_Pwm_In[5]/120;
    }
    else
    {
      DY_Rc_Pwm_In[5] = ((g_ui32SysClock/400) - temp_cnt6 + temp_cnt6_2);
      Rc_Pwm_In[5] = DY_Rc_Pwm_In[5]/120;
    }
  }
}

/******************* (C) COPYRIGHT 2018 DY EleTe *****END OF FILE************/
