// 激光的驱动
#include "Drv_vl53l0x.h"
#include "vl53l0x_def.h"
#include "vl53l0x_platform.h"

#include "DY_FcData.h"

u8 VL53L0X_LINKOK = 0;
u16 tof_height_mm;

void Drv_Vl53_Init(void)
{  
  if(VL53L0X_Init() == VL53L0X_ERROR_NONE)
  {
  	VL53L0X_LINKOK = 1;
  }
  else
  {
  	VL53L0X_LINKOK = 0;
  }
  sens_hd_check.tof_ok = VL53L0X_LINKOK;
}

static u16 err_delay;
void Drv_Vl53_RunTask(void)
{
	if(!VL53L0X_LINKOK)
		return;
	
	tof_height_mm = VL53L0X_FastRead();
	
	if(tof_height_mm == 0)
	{
		err_delay++;
		if(err_delay>10)
		{
			sens_hd_check.tof_ok = VL53L0X_LINKOK = 0;
		}
	}
	else
	{
		err_delay = 0;
	}
}