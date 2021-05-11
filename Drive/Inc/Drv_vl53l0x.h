#ifndef _DRV_VL53L0X_H_
#define _DRV_VL53L0X_H_

#include "common.h"

extern u8 VL53L0X_LINKOK;
extern u16 tof_height_mm;

void Drv_Vl53_Init(void);
void Drv_Vl53_RunTask(void);

#endif
