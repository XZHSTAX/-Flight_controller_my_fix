/******************** (C) COPYRIGHT 2018 DY EleTe ********************************
 * 作者    ：徳研电科
 * 官网    ：www.gototi.com
 * 描述    ：串口驱动
**********************************************************************************/
#ifndef _DRV_USART_H_
#define _DRV_USART_H_

#include "common.h"
#include "BSP_Init.h"

//void Uart2_Init(u32 bound);
//void Uart2_Send(u8 *DataToSend, u8 data_num);
void Uart2_Init(u32 bound);

void Uart3_Init(u32 bound);
void Uart3_Send(u8 *DataToSend, u8 data_num);

void Uart4_Init(u32 bound);
void Uart4_Send(u8 *DataToSend, u8 data_num);

void Uart5_Init(u32 bound);
void Uart5_Send(u8 *DataToSend, u8 data_num);

#endif

/******************* (C) COPYRIGHT 2018 DY EleTe *****END OF FILE************/
