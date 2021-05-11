#ifndef _LED_H_
#define	_LED_H_

#include "common.h"

#define LED1_ON     MAP_GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_0, ~GPIO_PIN_0);
#define LED1_OFF    MAP_GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_0, GPIO_PIN_0);
#define LED2_ON     MAP_GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_3, ~GPIO_PIN_3);
#define LED2_OFF    MAP_GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_3, GPIO_PIN_3);
#define LED3_ON     MAP_GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_1, ~GPIO_PIN_1);
#define LED3_OFF    MAP_GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_1, GPIO_PIN_1);
#define LED4_ON     MAP_GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_2, ~GPIO_PIN_2);
#define LED4_OFF    MAP_GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_2, GPIO_PIN_2);

enum  //led编号
{
	X_led = 0,
	B_led,
	R_led,
	G_led,
	LED_NUM,
};

extern u8 LED_state;

void Drv_LED_Init(void);
void LED_1ms_DRV(void );
void LED_Task(u8);

#endif
