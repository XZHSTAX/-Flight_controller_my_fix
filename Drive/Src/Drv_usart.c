/******************** (C) COPYRIGHT 2018 DY EleTe ********************************
 * 作者    ：徳研电科
 * 官网    ：www.gototi.com
 * 描述    ：串口驱动
**********************************************************************************/
#include "Drv_usart.h"
#include "uartstdio.h"
#include "DY_DT.h"
#include "DY_OF.h"
//#include "DY_OF_Ctrl.h"
#include "DY_Tracking.h"
#include "DY_FcData.h"

extern _flag flag;
u8 counter=0;
u8 data_brake[20] ={0};
u8 Wanning[] = {"\nNO,I will crash!\n"};

u8 Note[] = {"\nHave shut down the engine!\n"};


void zigbee_data_process(u8 data,u8 data_story[],u8 i);

void Uart2_Init(u32 bound)      //PA6、PA7
{
  MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART2);
  while(!MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_UART2))
  {
  }
  
  MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
  while(!MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOA))
  {
  }
  MAP_GPIOPinConfigure(GPIO_PA6_U2RX);
  MAP_GPIOPinConfigure(GPIO_PA7_U2TX);
  MAP_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_6 | GPIO_PIN_7);
  
  MAP_UARTConfigSetExpClk(UART2_BASE, g_ui32SysClock, bound, UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE);
  MAP_UARTTxIntModeSet(UART2_BASE, UART_TXINT_MODE_EOT);
  
  MAP_UARTFIFODisable(UART2_BASE);
  
  MAP_UARTIntEnable(UART2_BASE, UART_INT_RX);
  MAP_IntEnable(INT_UART2);
}

u8 Tx2Buffer[256];
u8 Tx2Counter=0;
u8 count2=0;

void UART2_IRQHandler(void)
{
  u32 uart2_status;
  u8 comdata;
  
  uart2_status = MAP_UARTIntStatus(UART2_BASE, true);
  
  if(uart2_status == 0x00000030)        //错误处理
  {
    comdata = (u8)MAP_UARTCharGet(UART2_BASE);
  }
  
  if(uart2_status == UART_INT_RX)
  {
//    MAP_UARTIntClear(UART2_BASE, uart2_status);
    
    /*****接收中断服务函数*****/
//    while(MAP_UARTCharsAvail(UART2_BASE))
//    {
      comdata = (u8)MAP_UARTCharGet(UART2_BASE);
//      DY_OF_Data_Receive_Prepare(comdata);
//    }
  }
  if(uart2_status == UART_INT_TX)
  {
//    MAP_UARTIntClear(UART2_BASE, uart2_status);
    /*****发送中断服务函数*****/
    MAP_UARTCharPut(UART2_BASE, Tx2Buffer[Tx2Counter++]);
    if(Tx2Counter == count2)
    {
      MAP_UARTIntDisable(UART2_BASE,UART_INT_TX);
    }
  }
}

void Uart2_Send(u8 *DataToSend, u8 data_num)
{
  u8 i;
  for(i=0;i<data_num;i++)
  {
    Tx2Buffer[count2++] = *(DataToSend + i);
  }
  if(!(MAP_UARTIntStatus(UART2_BASE, true) & 0x00000020))
  {
    MAP_UARTIntEnable(UART2_BASE, UART_INT_TX);
  }
}

/**********使用UARTprintf**********/
//void Uart2_Init(u32 bound)      //PA6、PA7
//{
//  MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART2);
//  while(!MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_UART2))
//  {
//  }
//  
//  MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
//  while(!MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOA))
//  {
//  }
//  MAP_GPIOPinConfigure(GPIO_PA6_U2RX);
//  MAP_GPIOPinConfigure(GPIO_PA7_U2TX);
//  MAP_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_6 | GPIO_PIN_7);
//  
//  UARTStdioConfig(2, bound, g_ui32SysClock);
//}

void Uart3_Init(u32 bound)      //PJ0、PJ1
{
  MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART3);
  while(!MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_UART3))
  {
  }
  
  MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOJ);
  while(!MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOJ))
  {
  }
  MAP_GPIOPinConfigure(GPIO_PJ0_U3RX);
  MAP_GPIOPinConfigure(GPIO_PJ1_U3TX);
  MAP_GPIOPinTypeUART(GPIO_PORTJ_BASE, GPIO_PIN_0 | GPIO_PIN_1);
  
  MAP_UARTConfigSetExpClk(UART3_BASE, g_ui32SysClock, bound, UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE);
  MAP_UARTTxIntModeSet(UART3_BASE, UART_TXINT_MODE_EOT);
  
  MAP_UARTFIFODisable(UART3_BASE);
  
  MAP_UARTIntEnable(UART3_BASE, UART_INT_RX);
  MAP_IntEnable(INT_UART3);
}

u8 Tx3Buffer[256];
u8 Tx3Counter=0;
u8 count3=0;

void UART3_IRQHandler(void)
{
  u32 uart3_status;
  u8 comdata;
  
  uart3_status = MAP_UARTIntStatus(UART3_BASE, true);
  
  if(uart3_status == 0x00000030)        //错误处理
  {
    comdata = (u8)MAP_UARTCharGet(UART3_BASE);
    DY_DT_Data_Receive_Prepare(comdata);
  }
  
  if(uart3_status == UART_INT_RX)
  {
//    MAP_UARTIntClear(UART3_BASE, uart3_status);
    
    /*****接收中断服务函数*****/
//    while(MAP_UARTCharsAvail(UART3_BASE))
//    {
      comdata = (u8)MAP_UARTCharGet(UART3_BASE);
      DY_DT_Data_Receive_Prepare(comdata);
//    }
  }
  if(uart3_status == UART_INT_TX)
  {
//    MAP_UARTIntClear(UART3_BASE, uart3_status);
    /*****发送中断服务函数*****/
    MAP_UARTCharPut(UART3_BASE, Tx3Buffer[Tx3Counter++]);
    if(Tx3Counter == count3)
    {
      MAP_UARTIntDisable(UART3_BASE,UART_INT_TX);
    }
  }
}

void Uart3_Send(u8 *DataToSend, u8 data_num)
{
  u8 i;
  for(i=0;i<data_num;i++)
  {
    Tx3Buffer[count3++] = *(DataToSend + i);
  }
  if(!(MAP_UARTIntStatus(UART3_BASE, true) & 0x00000020))
  {
    MAP_UARTIntEnable(UART3_BASE, UART_INT_TX);
  }
}

void Uart4_Init(u32 bound)      //PK0、PK1
{
  MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART4);
  while(!MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_UART4))
  {
  }
  
  MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOK);
  while(!MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOK))
  {
  }
  MAP_GPIOPinConfigure(GPIO_PK0_U4RX);
  MAP_GPIOPinConfigure(GPIO_PK1_U4TX);
  MAP_GPIOPinTypeUART(GPIO_PORTK_BASE, GPIO_PIN_0 | GPIO_PIN_1);
  
  MAP_UARTConfigSetExpClk(UART4_BASE, g_ui32SysClock, bound, UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE);
  MAP_UARTTxIntModeSet(UART4_BASE, UART_TXINT_MODE_EOT);
  
  MAP_UARTFIFODisable(UART4_BASE);
  
  MAP_UARTIntEnable(UART4_BASE, UART_INT_RX);
  MAP_IntEnable(INT_UART4);
}

u8 Tx4Buffer[256];
u8 Tx4Counter=0;
u8 count4=0;

void UART4_IRQHandler(void)
{
  u32 uart4_status;
  u8 comdata;
  u8 i=0;
  
  uart4_status = MAP_UARTIntStatus(UART4_BASE, true);
  
  if(uart4_status == 0x00000030)        //错误处理
  {
    comdata = (u8)MAP_UARTCharGet(UART4_BASE);
    
  }
  
  if(uart4_status == UART_INT_RX)
  {
//    MAP_UARTIntClear(UART4_BASE, uart4_status);
    
    /*****接收中断服务函数*****/
//    while(MAP_UARTCharsAvail(UART4_BASE))
//    {
      comdata = (u8)MAP_UARTCharGet(UART4_BASE);
      // DY_OF_GetOneByte(comdata);
      MAP_UARTCharPut(UART4_BASE, comdata);
      if(comdata != 0xff)
      {
        zigbee_data_process(comdata,data_brake,counter);
        counter++;
      }
      else
      {
        for(i=0;i<counter;i++)
        {
          MAP_UARTCharPut(UART4_BASE,data_brake[i]);
        }
        if(data_brake[6] == 0x66)
        {
          Uart4_Send(Wanning,sizeof(Wanning));
          flag.fly_ready = 0;
          Uart4_Send(Note,sizeof(Note));
          MAP_UARTCharPut(UART4_BASE,flag.fly_ready);
        }
        MAP_UARTCharPut(UART4_BASE,flag.fly_ready);
        counter = 0;
      }
      
      // MAP_UARTCharPut(UART4_BASE,comdata);
      
//    }
  }
  if(uart4_status == UART_INT_TX)
  {
//    MAP_UARTIntClear(UART4_BASE, uart4_status);
    /*****发送中断服务函数*****/
    MAP_UARTCharPut(UART4_BASE, Tx4Buffer[Tx4Counter++]);
    if(Tx4Counter == count4)
    {
      MAP_UARTIntDisable(UART4_BASE,UART_INT_TX);
    }
  }
}

void Uart4_Send(u8 *DataToSend, u8 data_num)
{
  u8 i;
  for(i=0;i<data_num;i++)
  {
    Tx4Buffer[count4++] = *(DataToSend + i);
  }
  if(!(MAP_UARTIntStatus(UART4_BASE, true) & 0x00000020))
  {
    MAP_UARTIntEnable(UART4_BASE, UART_INT_TX);
  }
}

void Uart5_Init(u32 bound)      //PC6、PC7
{
  MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART5);
  while(!MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_UART5))
  {
  }
  
  MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
  while(!MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOC))
  {
  }
  MAP_GPIOPinConfigure(GPIO_PC6_U5RX);
  MAP_GPIOPinConfigure(GPIO_PC7_U5TX);
  MAP_GPIOPinTypeUART(GPIO_PORTC_BASE, GPIO_PIN_6 | GPIO_PIN_7);
  
  MAP_UARTConfigSetExpClk(UART5_BASE, g_ui32SysClock, bound, UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE);
  MAP_UARTTxIntModeSet(UART5_BASE, UART_TXINT_MODE_EOT);
  
  MAP_UARTFIFODisable(UART5_BASE);
  
  MAP_UARTIntEnable(UART5_BASE, UART_INT_RX);
  MAP_IntEnable(INT_UART5);
}

u8 Tx5Buffer[256];
u8 Tx5Counter=0;
u8 count5=0;

void UART5_IRQHandler(void)
{
  u32 uart5_status;
  u8 comdata;
  
  uart5_status = MAP_UARTIntStatus(UART5_BASE, true);
  
  if(uart5_status == 0x00000030)        //错误处理
  {
    comdata = (u8)MAP_UARTCharGet(UART5_BASE);
    
  }
  
  if(uart5_status == UART_INT_RX)
  {
//    MAP_UARTIntClear(UART5_BASE, uart5_status);
    
    /*****接收中断服务函数*****/
//    while(MAP_UARTCharsAvail(UART5_BASE))
//    {
      comdata = (u8)MAP_UARTCharGet(UART5_BASE);
      
//    }
  }
  if(uart5_status == UART_INT_TX)
  {//    MAP_UARTIntClear(UART5_BASE, uart5_status);
    /*****发送中断服务函数*****/
    MAP_UARTCharPut(UART5_BASE, Tx5Buffer[Tx5Counter++]);
    if(Tx5Counter == count5)
    {
      MAP_UARTIntDisable(UART5_BASE,UART_INT_TX);
    }
  }
}

void Uart5_Send(u8 *DataToSend, u8 data_num)
{
  u8 i;
  for(i=0;i<data_num;i++)
  {
    Tx5Buffer[count5++] = *(DataToSend + i);
  }
  if(!(MAP_UARTIntStatus(UART5_BASE, true) & 0x00000020))
  {
    MAP_UARTIntEnable(UART5_BASE, UART_INT_TX);
  }
}


void zigbee_data_process(u8 data,u8 data_story[],u8 i)
{
  data_story[i] = data;
}

// 用于发送一些数据，data是要发送的数据，size_of_data是数据个数,会直接通过串口4，然后zigbee发给32
// 注意data 中不可以有ff或fe(没做转义)
void zigbee_data_Sent(u8 data[],u8 size_of_data)
{
  u8 data_processed[20]={0};
  u8 i=0;
  data_processed[0] = 0xfe;
  data_processed[1] = size_of_data+4;
  data_processed[2] = 0x91;
  data_processed[3] = 0x90;
  data_processed[4] = 0x3e;
  data_processed[5] = 0x49;
  for(i=6;i<6+size_of_data;i++)
  {
    data_processed[i] = data[i-6];
  }
  data_processed[i] = 0xFF;
  
  for(i=0;i<7+size_of_data;i++)
  {
    MAP_UARTCharPut(UART4_BASE, data_processed[i]);
    // printf("%x ",data_processed[i]);
  }

}

/******************* (C) COPYRIGHT 2018 DY EleTe *****END OF FILE************/
