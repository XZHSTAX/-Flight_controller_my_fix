/******************** (C) COPYRIGHT 2018 DY EleTe ********************************
 * 作者    ：徳研电科
 * 官网    ：www.gototi.com
 * 描述    ：PWM输出
**********************************************************************************/
#include "Drv_pwm_out.h"
#include "include.h"

/*初始化高电平时间1000us*/
#define INIT_DUTY 4000*1.875

/*设置飞控控制信号转换比例*/
#define PWM_RADIO 4*1.875

void PWM_Out_Init (void) //400hz	周期2.5ms               //PF0 PF1 PF2 PF3
{
  MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);
  while(!(MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_PWM0)));
  
  MAP_PWMClockSet(PWM0_BASE,PWM_SYSCLK_DIV_16);             //7,500,000
  
  MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
  while(!MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF));
  
  MAP_GPIOPinConfigure(GPIO_PF0_M0PWM0);
  MAP_GPIOPinConfigure(GPIO_PF1_M0PWM1);
  MAP_GPIOPinConfigure(GPIO_PF2_M0PWM2);
  MAP_GPIOPinConfigure(GPIO_PF3_M0PWM3);
  
  MAP_GPIOPadConfigSet(GPIO_PORTF_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3, GPIO_STRENGTH_4MA, GPIO_PIN_TYPE_STD); 
  
  MAP_GPIOPinTypePWM(GPIO_PORTF_BASE, (GPIO_PIN_0 | GPIO_PIN_1));
  MAP_GPIOPinTypePWM(GPIO_PORTF_BASE, (GPIO_PIN_2 | GPIO_PIN_3));
  
  MAP_PWMGenConfigure(PWM0_BASE, PWM_GEN_0, PWM_GEN_MODE_UP_DOWN |
                        PWM_GEN_MODE_NO_SYNC);
  MAP_PWMGenConfigure(PWM0_BASE, PWM_GEN_1, PWM_GEN_MODE_UP_DOWN |
                        PWM_GEN_MODE_NO_SYNC);
  
  MAP_PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, 18750);
  MAP_PWMGenPeriodSet(PWM0_BASE, PWM_GEN_1, 18750);
  
  MAP_PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, 0);
  MAP_PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1, 0);
  MAP_PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, 0);
  MAP_PWMPulseWidthSet(PWM0_BASE, PWM_OUT_3, 0);
  
  MAP_PWMGenEnable(PWM0_BASE, PWM_GEN_0);
  MAP_PWMGenEnable(PWM0_BASE, PWM_GEN_1);
  
  MAP_PWMOutputState(PWM0_BASE, PWM_OUT_0_BIT | PWM_OUT_1_BIT, true);
  MAP_PWMOutputState(PWM0_BASE, PWM_OUT_2_BIT | PWM_OUT_3_BIT, true);
}

void SetPwm ( int16_t pwm[MOTORSNUM] )
{
  u32 PWM_Duty1,PWM_Duty2,PWM_Duty3,PWM_Duty4;
  
  PWM_Duty1 = (u32)(PWM_RADIO * ( pwm[0] ) + INIT_DUTY);
  PWM_Duty2 = (u32)(PWM_RADIO * ( pwm[1] ) + INIT_DUTY);
  PWM_Duty3 = (u32)(PWM_RADIO * ( pwm[2] ) + INIT_DUTY);
  PWM_Duty4 = (u32)(PWM_RADIO * ( pwm[3] ) + INIT_DUTY);
  
  MAP_PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, PWM_Duty1);
  MAP_PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1, PWM_Duty2);
  MAP_PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, PWM_Duty3);
  MAP_PWMPulseWidthSet(PWM0_BASE, PWM_OUT_3, PWM_Duty4);
}

/******************* (C) COPYRIGHT 2018 DY EleTe *****END OF FILE************/
