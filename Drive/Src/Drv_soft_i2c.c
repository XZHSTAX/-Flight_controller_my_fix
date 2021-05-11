#include "Drv_soft_i2c.h"

void I2C_Soft_Init(void)
{
  /*使能VL53L0X时钟*/
  MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOL);
  while(!MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOL));
  /*SCL-PL1 SDA-PL0*/
  MAP_GPIOPinTypeGPIOOutput(GPIO_PORTL_BASE, GPIO_PIN_0 | GPIO_PIN_1);
//  GPIOL->PUR |= (GPIO_PIN_0 | GPIO_PIN_1);
  
  MAP_GPIOPinWrite(GPIO_PORTL_BASE, GPIO_PIN_0, GPIO_PIN_0);
  MAP_GPIOPinWrite(GPIO_PORTL_BASE, GPIO_PIN_1, GPIO_PIN_1);
}

/*产生I2C起始信号*/
static void I2c_Soft_Start(void)
{
  SDA_OUT();
  SDA_H;
  SCL_H;
  SysCtlDelay(160);
  SDA_L;
  SysCtlDelay(160);
  SCL_L;
}

/*产生I2C停止信号*/
static void I2c_Soft_Stop(void)
{
  SDA_OUT();
  SCL_L;
  SDA_L;
  SysCtlDelay(160);
  SCL_H;
  SDA_H;
  SysCtlDelay(160);
}

/*产生ACK应答*/
void I2c_Soft_Ask(void)
{
  SCL_L;
  SDA_OUT();
  SDA_L;
  SysCtlDelay(40);
  SCL_H;
  SysCtlDelay(40);
  SCL_L;
}

/*不产生ACK应答*/
void I2c_Soft_NoAsk()
{
    SCL_L;
    SDA_OUT();
    SDA_H;
    SysCtlDelay(40);
    SCL_H;
    SysCtlDelay(40);
    SCL_L;
}

/*等待应答信号；   返回为:=1接收应答失败,=0接受应答成功*/
static u8 I2c_Soft_WaitAsk ( void )
{
    u8 ErrTime = 0;
    SDA_IN();
    SDA_H;
    SysCtlDelay(40);
    SCL_H;
    SysCtlDelay(40);
    while ( SDA_read )
    {
        ErrTime++;
        if ( ErrTime > 250 )
        {
            I2c_Soft_Stop();
            return 1;
        }
    }
    SCL_L;
    return 0;
}

/*I2C写一个字节*/
static void I2c_Soft_SendByte ( u8 SendByte ) //数据从高位到低位
{
	u8 i = 8;
    SDA_OUT();
    SCL_L;
    while ( i-- )
    {
        if ( SendByte & 0x80 )
        {
            SDA_H;
        }
        else
        {
            SDA_L;
        }
        SendByte <<= 1;
        SysCtlDelay(40);
        SCL_H;
        SysCtlDelay(40);
        SCL_L;
        SysCtlDelay(40);
    }
}

/*读1个字节，ack=1时，发送ACK，ack=0，发送NACK*/
static u8 I2c_Soft_ReadByte ( u8 ask ) //数据从高位到低位//
{
    u8 i = 8;
    u8 ReceiveByte = 0;
    SDA_IN();
    while ( i-- )
    {
        SCL_L;
        SysCtlDelay(40);
        SCL_H;
        SysCtlDelay(40);
        ReceiveByte <<= 1;
        if ( SDA_read )
        {
            ReceiveByte |= 0x01;
        }
        SysCtlDelay(40);
    }

    if ( ask )
        I2c_Soft_Ask();
    else
        I2c_Soft_NoAsk();
    return ReceiveByte;
}

// IIC写一个字节数据
u8 IIC_Write_1Byte ( u8 SlaveAddress, u8 REG_Address, u8 REG_data )
{
    I2c_Soft_Start();
    I2c_Soft_SendByte ( SlaveAddress  );
    if ( I2c_Soft_WaitAsk() )
    {
        I2c_Soft_Stop();
        return 1;
    }
    I2c_Soft_SendByte ( REG_Address );
    I2c_Soft_WaitAsk();
    I2c_Soft_SendByte ( REG_data );
    I2c_Soft_WaitAsk();
    I2c_Soft_Stop();
    return 0;
}

// IIC读1字节数据
u8 IIC_Read_1Byte ( u8 SlaveAddress, u8 REG_Address, u8 *REG_data )
{
    I2c_Soft_Start();
    I2c_Soft_SendByte ( SlaveAddress  );
    if ( I2c_Soft_WaitAsk() )
    {
        I2c_Soft_Stop();
        return 1;
    }
    I2c_Soft_SendByte ( REG_Address );
    I2c_Soft_WaitAsk();
    I2c_Soft_Start();
    I2c_Soft_SendByte ( SlaveAddress << 1 | 0x01 );
    I2c_Soft_WaitAsk();
    *REG_data = I2c_Soft_ReadByte ( 0 );
    I2c_Soft_Stop();
    return 0;
}

// IIC写n字节数据
u8 IIC_Write_nByte ( u8 SlaveAddress, u8 REG_Address, u8 len, u8 *buf )
{
    I2c_Soft_Start();
    I2c_Soft_SendByte ( SlaveAddress  );
    if ( I2c_Soft_WaitAsk() )
    {
        I2c_Soft_Stop();
        return 1;
    }
    I2c_Soft_SendByte ( REG_Address );
    I2c_Soft_WaitAsk();
    while ( len-- )
    {
        I2c_Soft_SendByte ( *buf++ );
        I2c_Soft_WaitAsk();
    }
    I2c_Soft_Stop();
    return 0;
}

// IIC读n字节数据
u8 IIC_Read_nByte ( u8 SlaveAddress, u8 REG_Address, u8 len, u8 *buf )
{
    I2c_Soft_Start();
    I2c_Soft_SendByte ( SlaveAddress  );
    if ( I2c_Soft_WaitAsk() )
    {
        I2c_Soft_Stop();
        return 1;
    }
    I2c_Soft_SendByte ( REG_Address );
    I2c_Soft_WaitAsk();

    I2c_Soft_Start();
    I2c_Soft_SendByte ( SlaveAddress | 0x01 );
    I2c_Soft_WaitAsk();
    while ( len )
    {
        if ( len == 1 )
        {
            *buf = I2c_Soft_ReadByte ( 0 );
        }
        else
        {
            *buf = I2c_Soft_ReadByte ( 1 );
        }
        buf++;
        len--;
    }
    I2c_Soft_Stop();
    return 0;
}
