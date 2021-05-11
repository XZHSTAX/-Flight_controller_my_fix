#include "Drv_adc.h"

u32 adc0_value[1];

void ADC0_Init(void)
{
  MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
  while(!MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_ADC0))
  {
  }
  MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
  while(!MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOD))
  {
  }
  
  MAP_GPIOPinTypeADC(GPIO_PORTD_BASE, GPIO_PIN_7);
  
  ADCClockConfigSet(ADC0_BASE, ADC_CLOCK_SRC_PLL | ADC_CLOCK_RATE_HALF, 30);    //480MHz/30=16MHz
  MAP_ADCSequenceConfigure(ADC0_BASE, 3, ADC_TRIGGER_ALWAYS, 0);
  MAP_ADCSequenceStepConfigure(ADC0_BASE, 3, 0, ADC_CTL_CH4 | ADC_CTL_IE | ADC_CTL_END);
  MAP_ADCSequenceEnable(ADC0_BASE, 3);
  
  MAP_ADCIntClear(ADC0_BASE, 3);
  MAP_ADCIntEnableEx(ADC0_BASE, ADC_INT_SS3);
  MAP_IntEnable(INT_ADC0SS3);
}

void ADC0SS3_IRQHandler(void)
{
  MAP_ADCIntClearEx(ADC0_BASE, ADC_INT_SS3);
  MAP_ADCSequenceDataGet(ADC0_BASE, 3, adc0_value);
}
