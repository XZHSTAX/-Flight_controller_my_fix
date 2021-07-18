#include "DY_MotorCtrl.h"
#include "DY_Math.h"
#include "DY_RC.h"
#include "Drv_icm20602.h"
#include "Drv_spl06.h"
#include "DY_Imu.h"
#include "Drv_pwm_out.h"
#include "DY_MotionCal.h"
#include "DY_Filter.h"
#include "DY_Navigate.h"

/*
四轴：
      机头
   m2     m1
     \   /
      \ /
      / \
     /   \
   m3     m4
      屁股
*/
s16 motor[MOTORSNUM];
s16 motor_step[MOTORSNUM];

static u16 motor_prepara_cnt;
_mc_st mc;

#define IDLING 200

void Motor_Ctrl_Task(u8 dT_ms)
{
	u8 i;
	
	if(flag.fly_ready)
	{		
		if(flag.motor_preparation == 0)
		{
			motor_prepara_cnt += dT_ms;
			
			if(flag.motor_preparation == 0)
			{			
				if(motor_prepara_cnt<300)
				{
					motor_step[m1] = IDLING;
				}
				else if(motor_prepara_cnt<600)
				{
					motor_step[m2] = IDLING;
				}
				else if(motor_prepara_cnt<900)
				{
					motor_step[m3] = IDLING;
				}	
				else if(motor_prepara_cnt<1200)
				{	
					motor_step[m4] = IDLING;
				}
				else
				{
					flag.motor_preparation = 1;
					motor_prepara_cnt = 0;
				}
			}
			
		}	
	}
	else
	{
		flag.motor_preparation = 0;
	}
    
    if(flag.motor_preparation == 1)
	{		
		motor_step[m1] = mc.ct_val_thr  +mc.ct_val_yaw -mc.ct_val_rol +mc.ct_val_pit;
		motor_step[m2] = mc.ct_val_thr  -mc.ct_val_yaw +mc.ct_val_rol +mc.ct_val_pit;
		motor_step[m3] = mc.ct_val_thr  +mc.ct_val_yaw +mc.ct_val_rol -mc.ct_val_pit;
		motor_step[m4] = mc.ct_val_thr  -mc.ct_val_yaw -mc.ct_val_rol -mc.ct_val_pit;
	}
    
    for(i=0;i<MOTORSNUM;i++)
	{
		if(flag.fly_ready)
		{
			if(flag.motor_preparation == 1)
			{
				motor_step[i] = LIMIT(motor_step[i],IDLING,1000);	//限幅 200 ~ 1000
			}
	
		}
		else
		{		
			motor_step[i] = 0;
		}
		motor[i] = motor_step[i];
	}
    
	SetPwm(motor);
}
// 在哪改限幅