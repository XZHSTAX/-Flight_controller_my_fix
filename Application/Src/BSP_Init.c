/******************** (C) COPYRIGHT 2018 DeYan Electronic Technology ********************************
 * 作者    ：徳研电科
 * 官网    ：www.gototi.com
 * 描述    ：系统初始化
******************************************************************************************************/
#include "include.h"
#include "Drv_pwm_out.h"
#include "Drv_led.h"
#include "Drv_spi.h"
#include "Drv_icm20602.h"
#include "Drv_ak8975.h"
#include "Drv_spl06.h"
#include "Drv_w25qxx.h"
#include "Drv_soft_i2c.h"
#include "Drv_vl53l0x.h"
#include "DY_FlightCtrl.h"
#include "Drv_adc.h"

#include "OpticalFlow.h"

u32 g_ui32SysClock;

void BootLoader_Setup(void)
{
  HWREG(0xE000ED08) = 0x00008000;       //设置中断向量表偏移地址
}

u8 All_Init()
{
    // 返回值为系统实际时钟，即g_ui32SysClock = 120000000
    g_ui32SysClock = MAP_SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ | SYSCTL_OSC_MAIN |
                                          SYSCTL_USE_PLL | SYSCTL_CFG_VCO_480),
                                          120000000);       
                                        //初始化系统时钟120MHz
                                        // 参数的意义(外部时钟25M,使用外部时钟,使用PLL,PLL输出480M),配置系统时钟为120000000

    SysTick_Configuration(); 		    //滴答时钟，提供系统运行时间及延时函数
    
    Drv_LED_Init();					    //LED+RGB初始化
    
    Flash_Init();             		    //板载FLASH芯片(W25Q32)初始化，SPI1通信10MHz
    
    Uart3_Init(500000);                 //数传模块
    Delay_ms(10);
    Para_Data_Init();		            //参数数据初始化，读Flash
	
    PWM_IN_Init();			            //输入捕获

    PWM_Out_Init();                     //电机PWM输出400Hz
    Delay_ms(100);
    
    Drv_SPI2_init();          		    //SPI0初始化，用于读取飞控板上所有传感器，SPI通信10MHz
    Drv_Icm20602CSPin_Init(); 		    //ICM_CS初始化
    Drv_AK8975CSPin_Init();   		    //AK_CS初始化
    Drv_SPL06CSPin_Init();    		    //SPL_CS初始化
    
    sens_hd_check.gyro_ok = sens_hd_check.acc_ok = 
    Drv_Icm20602Reg_Init();   		    //ICM20602初始化，若初始化成功，则将陀螺仪和加速度的初始化成功标志位赋值
    sens_hd_check.mag_ok = 1;           //标记磁力计OK	
    sens_hd_check.baro_ok = Drv_Spl0601_Init();       		//SPL0601初始化，若初始化成功，则将气压计的初始化成功标志位赋值
    

    MAP_UARTCharPut(UART3_BASE, 0x00);  //UART3中断发送数据初始化（无效数据）
    Uart4_Init(500000);                 //光流模块
    Delay_ms(10);
    MAP_UARTCharPut(UART4_BASE, 0x00);  //UART4中断发送数据初始化（无效数据）
    Uart5_Init(500000);                 //OpenMv
    Delay_ms(10);
    MAP_UARTCharPut(UART5_BASE, 0x00);  //UART5中断发送数据初始化（无效数据）
    
    I2C_Soft_Init();          		    //I2C初始化，飞控外接VL53L0X激光测距模块，使用I2C通信
    Drv_Vl53_Init();          		    //TOF传感器初始化，使用VL53L0X激光测距模块
    
    sens_hd_check.dy_pmw3901_ok =  OpticalFlow_Init();      //ATK-PMW3901光流模块初始化，若初始化成功，则将ATK-PMW3901光流模块的初始化成功标志位赋值

    ADC0_Init();		                //A/D采集电池电压
    Delay_ms(100);

    All_PID_Init();               	    //PID参数初始化
    
    MAP_IntPriorityGroupingSet(3);              //中断优先级分组
    MAP_IntPrioritySet(INT_TIMER0A, 0x20);      //使用1个字节的高3位表示，001 00000
    MAP_IntPrioritySet(INT_TIMER1A, 0x20);
    MAP_IntPrioritySet(INT_TIMER3A, 0x20);
    MAP_IntPrioritySet(INT_TIMER4A, 0x20);
    MAP_IntPrioritySet(INT_TIMER2A, 0x20);
    MAP_IntPrioritySet(INT_TIMER5A, 0x20);
    MAP_IntPrioritySet(INT_UART3,   0xA0);      //101 00000
    MAP_IntPrioritySet(INT_UART4,   0x40);      //010 00000
    MAP_IntPrioritySet(INT_UART5,   0x80);      //100 00000
    MAP_IntPrioritySet(INT_ADC0SS3, 0xC0);      //110 00000
    
    DY_DT_SendString("SYS init OK!",sizeof("SYS init OK!"));		//系统初始化完毕
    Delay_ms(100);
    
	return (1);
}

/******************* (C) COPYRIGHT 2018 DeYan Electronic Technology **********************************/
