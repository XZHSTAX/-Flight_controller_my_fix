/******************** (C) COPYRIGHT 2018 DY EleTe ********************************
 * 作者    ：徳研电科
 * 官网    ：www.gototi.com
 * 描述    ：SPL06气压计驱动
**********************************************************************************/
#ifndef _DRV_SPL06_H_
#define _DRV_SPL06_H_

#include "common.h"

#define CONTINUOUS_PRESSURE     1
#define CONTINUOUS_TEMPERATURE  2
#define CONTINUOUS_P_AND_T      3
#define PRESSURE_SENSOR     0
#define TEMPERATURE_SENSOR  1

struct spl0601_calib_param_t {	
    int16_t c0;
    int16_t c1;
    int32_t c00;
    int32_t c10;
    int16_t c01;
    int16_t c11;
    int16_t c20;
    int16_t c21;
    int16_t c30;       
};

struct spl0601_t {	
    struct spl0601_calib_param_t calib_param;/**<calibration data*/	
    u8 			chip_id; /**<chip id*/	
    int32_t 	i32rawPressure;
    int32_t 	i32rawTemperature;
    int32_t 	i32kP;    
    int32_t 	i32kT;
};

void Drv_SPL06CSPin_Init(void);
u8 Drv_Spl0601_Init(void);
float Drv_Spl0601_Read(void);

void spl0601_start_temperature ( void );
    
#endif
    
/******************* (C) COPYRIGHT 2018 DY EleTe *****END OF FILE************/
