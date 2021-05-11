#ifndef _DRV_SOFT_I2C_H_
#define _DRV_SOFT_I2C_H_

#include "common.h"

#define SDA_OUT()   {GPIODirModeSet(GPIO_PORTL_BASE, GPIO_PIN_0, GPIO_DIR_MODE_OUT);}
#define SDA_IN()    {GPIODirModeSet(GPIO_PORTL_BASE, GPIO_PIN_0, GPIO_DIR_MODE_IN);}

#define SCL_H   MAP_GPIOPinWrite(GPIO_PORTL_BASE, GPIO_PIN_1, GPIO_PIN_1);
#define SCL_L   MAP_GPIOPinWrite(GPIO_PORTL_BASE, GPIO_PIN_1, ~GPIO_PIN_1);
#define SDA_H   MAP_GPIOPinWrite(GPIO_PORTL_BASE, GPIO_PIN_0, GPIO_PIN_0);
#define SDA_L   MAP_GPIOPinWrite(GPIO_PORTL_BASE, GPIO_PIN_0, ~GPIO_PIN_0);

#define SCL_read    MAP_GPIOPinRead(GPIO_PORTL_BASE, GPIO_PIN_1)>>1
#define SDA_read    MAP_GPIOPinRead(GPIO_PORTL_BASE, GPIO_PIN_0)

void I2C_Soft_Init(void);

u8 IIC_Write_1Byte(u8 SlaveAddress,u8 REG_Address,u8 REG_data);
u8 IIC_Read_1Byte(u8 SlaveAddress,u8 REG_Address,u8 *REG_data);
u8 IIC_Write_nByte(u8 SlaveAddress, u8 REG_Address, u8 len, u8 *buf);
u8 IIC_Read_nByte(u8 SlaveAddress, u8 REG_Address, u8 len, u8 *buf);

#endif
