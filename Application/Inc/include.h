#ifndef _INCLUDE_H_
#define _INCLUDE_H_

#include "DY_FcData.h"
#include "DY_Scheduler.h"
#include "BSP_Init.h"
#include "DY_DT.h"
#include "DY_Parameter.h"
#include "Drv_time.h"
#include "Drv_pwm_in.h"
#include "Drv_usart.h"
#include "uartstdio.h"

//================系统===================
#define HW_TYPE	    05
#define HW_VER	    1
#define SOFT_VER    32
#define BL_VER	    0
#define PT_VER	    400

#define DY_DT_USE_USART3 				//开启UART3数传功能

//================传感器===================
//以下为板载各个传感器的使能引脚配置
#define ICM20602_CS_SYSCTL		SYSCTL_PERIPH_GPION
#define ICM20602_CS_GPIO		GPIO_PORTN_BASE
#define ICM20602_CS_PIN			GPIO_PIN_2

#define AK8975_CS_SYSCTL		SYSCTL_PERIPH_GPIOP
#define AK8975_CS_GPIO			GPIO_PORTP_BASE
#define AK8975_CS_PIN			GPIO_PIN_2

#define SPL06_CS_SYSCTL			SYSCTL_PERIPH_GPION
#define SPL06_CS_GPIO			GPIO_PORTN_BASE
#define SPL06_CS_PIN			GPIO_PIN_3

//=========================================

#endif
