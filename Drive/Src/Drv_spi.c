/******************** (C) COPYRIGHT 2018 DY EleTe ********************************
 * 作者    ：徳研电科
 * 官网    ：www.gototi.com
 * 描述    ：SPI驱动
**********************************************************************************/
#include "Drv_spi.h"

void Drv_SPI_Init(void)
{
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOQ);
    while(!(MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOQ)))
    {
    }
    MAP_GPIOPinConfigure(GPIO_PQ0_SSI3CLK);
    MAP_GPIOPinConfigure(GPIO_PQ2_SSI3XDAT0);
    MAP_GPIOPinConfigure(GPIO_PQ3_SSI3XDAT1);
    MAP_GPIOPinTypeSSI(GPIO_PORTQ_BASE, (GPIO_PIN_0 | GPIO_PIN_2 | GPIO_PIN_3));

    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI3);
    while(!(MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_SSI3)))
    {
    }
    
    MAP_SSIConfigSetExpClk(SSI3_BASE, g_ui32SysClock, SSI_FRF_MOTO_MODE_0,
                           SSI_MODE_MASTER, (g_ui32SysClock/120), 8);        //1MbHz
    
    MAP_SSIEnable(SSI3_BASE);
}

u8 Drv_SPI_RW(u8 dat)
{
    uint32_t getData;

    /* Send the data from the SSI-0 Master */
    MAP_SSIDataPut(SSI3_BASE, dat);
     
    /* Wait for the data to be transmitted out of the SSI3 by checking on
     * the busy status from the SSI controller*/
    while(MAP_SSIBusy(SSI3_BASE));
    
    MAP_SSIDataGet(SSI3_BASE, &getData);
    
    return (u8)getData;
}

void Drv_SPI2_init(void)
{
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    while(!(MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOA)))
    {
    }
    
    MAP_GPIOPinConfigure(GPIO_PA2_SSI0CLK);
    MAP_GPIOPinConfigure(GPIO_PA4_SSI0XDAT0);
    MAP_GPIOPinConfigure(GPIO_PA5_SSI0XDAT1);
    MAP_GPIOPinTypeSSI(GPIO_PORTA_BASE, (GPIO_PIN_2 | GPIO_PIN_4 | GPIO_PIN_5));
    
    GPIOA->PUR |= (GPIO_PIN_4 | GPIO_PIN_5);

    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI0);
    while(!(MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_SSI0)))
    {
    }
      //Polarity Phase       Mode
      //  0       0   SSI_FRF_MOTO_MODE_0
      //  0       1   SSI_FRF_MOTO_MODE_1
      //  1       0   SSI_FRF_MOTO_MODE_2
      //  1       1   SSI_FRF_MOTO_MODE_3
    MAP_SSIConfigSetExpClk(SSI0_BASE, g_ui32SysClock, SSI_FRF_MOTO_MODE_0,
                           SSI_MODE_MASTER, (g_ui32SysClock/12), 8);        //10MHz
    
    MAP_SSIEnable(SSI0_BASE);
}

u8 Drv_SPI2_RW(u8 dat)
{
    uint32_t getData = 0x00;

    /* Send the data from the SSI-0 Master */
    MAP_SSIDataPut(SSI0_BASE, dat);
     
    /* Wait for the data to be transmitted out of the SSI0 by checking on
     * the busy status from the SSI controller*/
    while(MAP_SSIBusy(SSI0_BASE));
    
    MAP_SSIDataGet(SSI0_BASE, &getData);
    
    return getData;
}

void Drv_SPI2_Transmit(uint8_t *pData, uint16_t Size)
{
	uint16_t i = 0;

	for(i = 0; i < Size; i++)
	{
		Drv_SPI2_RW(pData[i]);
	}
}

void Drv_SPI2_Receive(uint8_t *pData, uint16_t Size)
{
	uint16_t i = 0;

	for(i = 0; i < Size; i++)
	{
		pData[i] = Drv_SPI2_RW(0);
	}
}

/******************* (C) COPYRIGHT 2018 DY EleTe *****END OF FILE************/
