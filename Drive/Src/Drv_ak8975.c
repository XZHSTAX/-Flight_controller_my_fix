/******************** (C) COPYRIGHT 2018 DY EleTe ********************************
 * 作者    ：徳研电科
 * 官网    ：www.gototi.com
 * 描述    ：电子罗盘驱动
**********************************************************************************/
#include "Drv_ak8975.h"
#include "include.h"
#include "Drv_spi.h"

void Drv_AK8975CSPin_Init(void)
{
    MAP_SysCtlPeripheralEnable(AK8975_CS_SYSCTL);
    while(!(MAP_SysCtlPeripheralReady(AK8975_CS_SYSCTL)))
    {
    }
    GPIOPinTypeGPIOOutput(AK8975_CS_GPIO, AK8975_CS_PIN);
    
    GPIOPinTypeGPIOOutput(AK8975_CS_GPIO, AK8975_CS_PIN);
}

static void ak8975_enable(u8 ena)
{
    if(ena)
	GPIOPinWrite(AK8975_CS_GPIO, AK8975_CS_PIN, ~AK8975_CS_PIN);
    else
	GPIOPinWrite(AK8975_CS_GPIO, AK8975_CS_PIN, AK8975_CS_PIN);
}

static void ak8975_Trig(void)
{
    ak8975_enable(1);
    Drv_SPI2_RW(AK8975_CNTL_REG);
    Drv_SPI2_RW(0x01);
    ak8975_enable(0);
}

static u8 ak8975_buf[6];		//AK8975原始数据
void Drv_AK8975_Read(void)
{	
    u8 i = 0;
    
    ak8975_enable(1);
    Drv_SPI2_RW(AK8975_HXL_REG|0x80);
    for(i = 0; i < 6; i++)
	ak8975_buf[i] = Drv_SPI2_RW(0xff);
    ak8975_enable(0);

    ak8975_Trig();
}

void Mag_Get(s16 mag_val[3])
{
    s16 t[3];

    t[0] = ((((int16_t)ak8975_buf[1]) << 8) | ak8975_buf[0]) ;
    t[1] = ((((int16_t)ak8975_buf[3]) << 8) | ak8975_buf[2]) ;
    t[2] = ((((int16_t)ak8975_buf[5]) << 8) | ak8975_buf[4]) ;

    /*转换坐标轴为ANO坐标*/
    mag_val[0] = +t[0];
    mag_val[1] = -t[1];
    mag_val[2] = -t[2];
}

/******************* (C) COPYRIGHT 2018 DY EleTe *****END OF FILE************/
