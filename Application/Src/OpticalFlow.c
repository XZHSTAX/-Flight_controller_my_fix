#include "OpticalFlow.h"
#include "Drv_spi.h"
#include "Drv_usart.h"
#include "uartstdio.h"
//#include "stdio.h"
#include "math.h"
#include "DY_Imu.h"
#include "Drv_vl53l0x.h"
#include "DY_FcData.h"

#define NCS_PIN_H GPIOPinWrite(GPIO_PORTQ_BASE, GPIO_PIN_1, GPIO_PIN_1);
#define NCS_PIN_L GPIOPinWrite(GPIO_PORTQ_BASE, GPIO_PIN_1, ~GPIO_PIN_1);

// DY or ATK-PMW3901 光流驱动，使用SPI接收光流信息

motionBurst_t motionBurst;

//光流电源控制
void OpticalFlowPowerControl(bool state)
{
	if(state == true)
		GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, GPIO_PIN_7);
	else
		GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, 0);
}

static void RegisterWrite(uint8_t reg, uint8_t value)
{
	// 最高位为1 写寄存器
	reg |= 0x80u;
	
	NCS_PIN_L;
    MAP_SysCtlDelay(2000);   //50us
	
	Drv_SPI_RW(reg);
	MAP_SysCtlDelay(2000);   //50us
	Drv_SPI_RW(value);
	MAP_SysCtlDelay(2000);   //50us

	NCS_PIN_H;
	MAP_SysCtlDelay(8000);   //200us
}

static uint8_t RegisterRead(uint8_t reg)
{
	uint8_t data = 0;

	// 最高位为0 读寄存器
	reg &= ~0x80u;
	
	NCS_PIN_L;
	MAP_SysCtlDelay(2000);      //50us
	
	Drv_SPI_RW(reg);
	MAP_SysCtlDelay(20000);     //500us
	data = Drv_SPI_RW(0x00);
	MAP_SysCtlDelay(2000);      //50us
	
	NCS_PIN_H;
	MAP_SysCtlDelay(8000);      //200us

	return data;
}

static void RegisterRead_nByte(uint8_t *pData, uint8_t reg, uint16_t Size)
{	
	NCS_PIN_L;
	MAP_SysCtlDelay(2000);              //50us
	
	Drv_SPI_RW(reg);
	MAP_SysCtlDelay(2000);              //50us
	
	for(uint16_t i=0; i<Size; i++)
    {
        pData[i] = Drv_SPI_RW(0x00);
		MAP_SysCtlDelay(2000);          //50us
    }
	
	NCS_PIN_H;
	MAP_SysCtlDelay(8000);              //200us
}

static void InitRegisters(void)
{	
	RegisterWrite(0x7F, 0x00);
	RegisterWrite(0x61, 0xAD);
	RegisterWrite(0x7F, 0x03);
	RegisterWrite(0x40, 0x00);
	RegisterWrite(0x7F, 0x05);
	RegisterWrite(0x41, 0xB3);
	RegisterWrite(0x43, 0xF1);
	RegisterWrite(0x45, 0x14);
	RegisterWrite(0x5B, 0x32);
	RegisterWrite(0x5F, 0x34);
	RegisterWrite(0x7B, 0x08);
	RegisterWrite(0x7F, 0x06);
	RegisterWrite(0x44, 0x1B);
	RegisterWrite(0x40, 0xBF);
	RegisterWrite(0x4E, 0x3F);
	RegisterWrite(0x7F, 0x08);
	RegisterWrite(0x65, 0x20);
	RegisterWrite(0x6A, 0x18);
	RegisterWrite(0x7F, 0x09);
	RegisterWrite(0x4F, 0xAF);
	RegisterWrite(0x5F, 0x40);
	RegisterWrite(0x48, 0x80);
	RegisterWrite(0x49, 0x80);
	RegisterWrite(0x57, 0x77);
	RegisterWrite(0x60, 0x78);
	RegisterWrite(0x61, 0x78);
	RegisterWrite(0x62, 0x08);
	RegisterWrite(0x63, 0x50);
	RegisterWrite(0x7F, 0x0A);
	RegisterWrite(0x45, 0x60);
	RegisterWrite(0x7F, 0x00);
	RegisterWrite(0x4D, 0x11);
	RegisterWrite(0x55, 0x80);
	RegisterWrite(0x74, 0x1F);
	RegisterWrite(0x75, 0x1F);
	RegisterWrite(0x4A, 0x78);
	RegisterWrite(0x4B, 0x78);
	RegisterWrite(0x44, 0x08);
	RegisterWrite(0x45, 0x50);
	RegisterWrite(0x64, 0xFF);
	RegisterWrite(0x65, 0x1F);
	RegisterWrite(0x7F, 0x14);
	RegisterWrite(0x65, 0x67);
	RegisterWrite(0x66, 0x08);
	RegisterWrite(0x63, 0x70);
	RegisterWrite(0x7F, 0x15);
	RegisterWrite(0x48, 0x48);
	RegisterWrite(0x7F, 0x07);
	RegisterWrite(0x41, 0x0D);
	RegisterWrite(0x43, 0x14);
	RegisterWrite(0x4B, 0x0E);
	RegisterWrite(0x45, 0x0F);
	RegisterWrite(0x44, 0x42);
	RegisterWrite(0x4C, 0x80);
	RegisterWrite(0x7F, 0x10);
	RegisterWrite(0x5B, 0x02);
	RegisterWrite(0x7F, 0x07);
	RegisterWrite(0x40, 0x41);
	RegisterWrite(0x70, 0x00);

	MAP_SysCtlDelay(400000);   //10ms

	RegisterWrite(0x32, 0x44);
	RegisterWrite(0x7F, 0x07);
	RegisterWrite(0x40, 0x40);
	RegisterWrite(0x7F, 0x06);
	RegisterWrite(0x62, 0xF0);
	RegisterWrite(0x63, 0x00);
	RegisterWrite(0x7F, 0x0D);
	RegisterWrite(0x48, 0xC0);
	RegisterWrite(0x6F, 0xD5);
	RegisterWrite(0x7F, 0x00);
	RegisterWrite(0x5B, 0xA0);
	RegisterWrite(0x4E, 0xA8);
	RegisterWrite(0x5A, 0x50);
	RegisterWrite(0x40, 0x80);
}

_PMW3901_pixel_flow pmw_pixel_flow;

static float lpfValue = 0.15f;						/*低通系数*/
static float pmw_lastOutX=0.f, pmw_lastOutY=0.f;	/*上一次的补偿像素*/

int16_t DY_PMW_OF_DX2, DY_PMW_OF_DY2;
int16_t DY_PMW_OF_DX2FIX, DY_PMW_OF_DY2FIX;

/*复位光流数据*/
static void ResetOpticalFlowData(void)
{
	pmw_lastOutX = 0;
	pmw_lastOutY = 0;
}

/*初始化光流模块*/
u8 OpticalFlow_Init(void)
{
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
    while(!(MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOC)))
    {
    }
    GPIOPinTypeGPIOOutput(GPIO_PORTC_BASE, GPIO_PIN_7);
    
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOQ);
    while(!(MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOQ)))
    {
    }
    GPIOPinTypeGPIOOutput(GPIO_PORTQ_BASE, GPIO_PIN_1);
	
    ResetOpticalFlowData();
	
   //  OpticalFlowPowerControl(true);	/*打开电源*/
    MAP_SysCtlDelay(2000000);   //50ms
	
	NCS_PIN_H
	Drv_SPI_Init();
	MAP_SysCtlDelay(1600000);   //40ms
	
	uint8_t chipId = RegisterRead(0x00);
	uint8_t invChipId = RegisterRead(0x5f);
    
    if(chipId != 0x49)
      return 0;
    
//    printf("chipId:%d\r\n",chipId);
//    printf("invChipId:%d\r\n",invChipId);
	
//	UARTprintf("chipId:0x%x\r\n",chipId);
//	UARTprintf("invChipId:0x%x\r\n",invChipId);

	// 上电复位
	RegisterWrite(0x3a, 0x5a);
	MAP_SysCtlDelay(200000);   //5ms
	
	InitRegisters();
	MAP_SysCtlDelay(200000);   //5ms
    return 1;
}

u8 OpticalFlow_RawData[12];

void PMW3901_Read(void)
{
	RegisterRead_nByte(OpticalFlow_RawData, Motion_Burst, 12);
}

void PMW3901_Data_Prepare(void)
{
    PMW3901_Read();
    
	motionBurst.shutter = ((((uint16_t)OpticalFlow_RawData[10]) << 8) | OpticalFlow_RawData[11]);
	motionBurst.minRawData = OpticalFlow_RawData[9];
	motionBurst.maxRawData = OpticalFlow_RawData[8];
	motionBurst.rawDataSum = OpticalFlow_RawData[7];
	motionBurst.squal = OpticalFlow_RawData[6];
	motionBurst.deltaY = (int16_t)((((uint16_t)OpticalFlow_RawData[5]) << 8) | OpticalFlow_RawData[4]);
	motionBurst.deltaX = (int16_t)((((uint16_t)OpticalFlow_RawData[3]) << 8) | OpticalFlow_RawData[2]);
	motionBurst.observation = OpticalFlow_RawData[1];
	motionBurst.motion = OpticalFlow_RawData[0];
}

void OpticalFlow_DataFusion_Task(void)
{
    PMW3901_Data_Prepare();
  
    /*连续2帧之间的像素变化，根据实际安装方向调整 (pitch:x)  (roll:y)*/
    int16_t pixelDx = motionBurst.deltaY;
    int16_t pixelDy = motionBurst.deltaX;
    
    pmw_pixel_flow.SumX += pixelDx;
    pmw_pixel_flow.SumY += pixelDy;
    pmw_pixel_flow.LpfX += (pmw_pixel_flow.SumX - pmw_pixel_flow.LpfX)*lpfValue;
    pmw_pixel_flow.LpfY += (pmw_pixel_flow.SumY - pmw_pixel_flow.LpfY)*lpfValue;
    
    float pmw_tempCoefficient = PMW_RESOLUTION * tof_height_mm * 0.001f;        //tof_height_mm
    float pmw_tanRoll = tanf(imu_data.rol * DEG_TO_RAD);
	float pmw_tanPitch = tanf(imu_data.pit * DEG_TO_RAD);
    
    pmw_pixel_flow.CompX += (555.f * pmw_tanPitch - pmw_pixel_flow.CompX)*lpfValue;     /*倾角补偿*/
    pmw_pixel_flow.CompY += (605.f * pmw_tanRoll - pmw_pixel_flow.CompY)*lpfValue;
    pmw_pixel_flow.DataOutX = (pmw_pixel_flow.LpfX - pmw_pixel_flow.CompX);	            /*实际输出像素*/
	pmw_pixel_flow.DataOutY = (pmw_pixel_flow.LpfY - pmw_pixel_flow.CompY);
    
    pmw_pixel_flow.DataDx = pmw_tempCoefficient * (pmw_pixel_flow.DataOutX - pmw_lastOutX);	/*2帧之间位移变化量，单位 cm*/
	pmw_pixel_flow.DataDy = pmw_tempCoefficient * (pmw_pixel_flow.DataOutY - pmw_lastOutY);	
	pmw_lastOutX = pmw_pixel_flow.DataOutX;	        /*上一次实际输出像素*/
	pmw_lastOutY = pmw_pixel_flow.DataOutY;
	pmw_pixel_flow.Vx = 100.f * (pmw_pixel_flow.DataDx);	/*速度 cm/s*/
	pmw_pixel_flow.Vy = 100.f * (pmw_pixel_flow.DataDy);
    pmw_pixel_flow.VxFix += (pmw_pixel_flow.Vx - pmw_pixel_flow.VxFix) * 0.08f;	/*速度LPF*/
	pmw_pixel_flow.VyFix += (pmw_pixel_flow.Vy - pmw_pixel_flow.VyFix) * 0.08f;	/*速度LPF*/
    
    DY_PMW_OF_DX2 = (int16_t)pmw_pixel_flow.Vx;
    DY_PMW_OF_DY2 = (int16_t)pmw_pixel_flow.Vy;
    DY_PMW_OF_DX2FIX = (int16_t)pmw_pixel_flow.VxFix;
    DY_PMW_OF_DY2FIX = (int16_t)pmw_pixel_flow.VyFix;
}
