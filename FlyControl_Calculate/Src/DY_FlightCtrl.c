/******************** (C) COPYRIGHT 2018 DY EleTe ********************************
 * 作者    ：徳研电科
 * 官网    ：www.gototi.com
 * 描述    ：飞行控制
**********************************************************************************/
#include "DY_FlightCtrl.h"
#include "DY_Imu.h"
#include "Drv_icm20602.h"
#include "DY_MagProcess.h"
#include "Drv_spl06.h"
#include "DY_MotionCal.h"
#include "DY_AttCtrl.h"
#include "DY_LocCtrl.h"
#include "DY_AltCtrl.h"
#include "DY_MotorCtrl.h"
#include "Drv_led.h"
#include "DY_RC.h"
#include "Drv_vl53l0x.h"
#include "DY_OF.h"

/*PID参数初始化*/
void All_PID_Init(void)
{
	/*姿态控制，角速度PID初始化*/
	Att_1level_PID_Init();
	
	/*姿态控制，角度PID初始化*/
	Att_2level_PID_Init();
	
	/*高度控制，高度速度PID初始化*/
	Alt_1level_PID_Init();	
	
	/*高度控制，高度PID初始化*/
	Alt_2level_PID_Init();
	
	/*位置速度控制PID初始化*/
	Loc_1level_PID_Init();
	
}

/*控制参数改变任务*/
void ctrl_parameter_change_task()
{	
	if(0)
	{
		Set_Att_2level_Ki(0);
		
	}
	else
	{
		if(flag.auto_take_off_land ==AUTO_TAKE_OFF)
		{

			Set_Att_1level_Ki(2);
		}
		else
		{

			Set_Att_1level_Ki(1);
		}
		
		Set_Att_2level_Ki(1);
	}
}

/*一键翻滚（暂无）*/
void one_key_roll()
{
			if(flag.flying && flag.auto_take_off_land == AUTO_TAKE_OFF_FINISH)
			{	
				if(rolling_flag.roll_mode==0)
				{
					rolling_flag.roll_mode = 1;
				}
			}
}

static u16 one_key_taof_start;
/*一键起飞任务（主要功能为延迟）*/
void one_key_take_off_task(u16 dt_ms)
{
	if(one_key_taof_start != 0)
	{
		one_key_taof_start += dt_ms;
		
		
		if(one_key_taof_start > 1400 && flag.motor_preparation == 1)		//1400*10=14000ms=14s
		{
			one_key_taof_start = 0;
				if(flag.auto_take_off_land == AUTO_TAKE_OFF_NULL)
				{
					flag.auto_take_off_land = AUTO_TAKE_OFF;
					//解锁、起飞

					flag.taking_off = 1;
				}
			
		}
	}

}

/*一键起飞*/
void one_key_take_off()
{
	if(flag.unlock_en)
	{	
		one_key_taof_start = 1;
		flag.fly_ready = 1;
	}
}

/*一键降落*/
void one_key_land()
{
	flag.auto_take_off_land = AUTO_LAND;
}

// 飞行状态结构体变量
_flight_state_st fs;

s16 flying_cnt,landing_cnt;

extern s32 ref_height_get;

float stop_baro_hpf;

/*降落检测*/

static s16 ld_delay_cnt ;
void land_discriminat(s16 dT_ms)
{	
	/*油门归一值小于0.1并且垂直方向加速度小于阈值  或者启动自动降落*/
	if((fs.speed_set_h_norm[Z] < 0.1f && imu_data.w_acc[Z]<200) || flag.auto_take_off_land == AUTO_LAND)
	{
		if(ld_delay_cnt>0)		//200ms
		{
			ld_delay_cnt -= dT_ms;
		}
	}
	else
	{
		ld_delay_cnt = 200;
	}
	
	/*意义是：如果向上推了油门，就需要等垂直方向加速度小于200cm/s2 保持200ms才开始检测*/	
	if(ld_delay_cnt <= 0 && (flag.thr_low || flag.auto_take_off_land == AUTO_LAND) )
	{
		/*油门最终输出量小于250并且没有在手动解锁上锁过程中，持续1.5秒，认为着陆，然后上锁*/
		if(mc.ct_val_thr<250 && flag.fly_ready == 1 && flag.locking != 2)//ABS(wz_spe_f1.out <20 ) //还应当 与上速度条件，速度小于正20厘米每秒。
		{
			if(landing_cnt<1500)
			{
				landing_cnt += dT_ms;
			}
			else
			{

				flying_cnt = 0;
				flag.taking_off = 0;
                
				landing_cnt =0;	
				flag.fly_ready =0;

				flag.flying = 0;
			}
		}
		else
		{
			landing_cnt = 0;
		}
	}
	else
	{
		landing_cnt  = 0;
	}
}

/*飞行状态任务*/
/***************PIT、ROL控制变量初始化+OpenMv***************/
s16 dy_pit = 0,dy_rol = 0;
/***********************************************************/
// 飞行任务控制（遥控器？）
// 传入的第一个参数表示这个程序执行的间隔
// 第二个参数是DY_RC中的全局数组，暂时不知道是干什么用的，可能是飞行器的姿态角？
// 此函数输出的或许是系统的设定值？
void Flight_State_Task(u8 dT_ms,s16 *CH_N)
{
	s16 thr_deadzone;
	static float max_speed_lim;
	/*设置油门摇杆量*/
	thr_deadzone = (flag.wifi_ch_en != 0) ? 0 : 50;                                  // 如果wifi_ch_en标志位不等于0  thr_deadzone=0
	fs.speed_set_h_norm[Z] = my_deadzone(CH_N[CH_THR],0,thr_deadzone) *0.0023f;     //-1.035~1.035	油门归一值,油门遥感中间位置；thr表示俯仰角，即z方向
	fs.speed_set_h_norm_lpf[Z] += 0.2f *(fs.speed_set_h_norm[Z] - fs.speed_set_h_norm_lpf[Z]);
	
	/*推油门起飞*/
	if(flag.fly_ready)
	{	
		if(fs.speed_set_h_norm[Z]>0.01f && flag.motor_preparation == 1)
		{
			flag.taking_off = 1;
		}
	}
	
	if(flag.taking_off)
	{
		if(flying_cnt<1000)     //1s
		{
			flying_cnt += dT_ms;
		}
		else
		{
			/*起飞后1秒，认为已经在飞行*/
			flag.flying = 1;
		}
		
		if(fs.speed_set_h_norm[Z]>0)
		{
			/*设置上升速度*/
			fs.speed_set_h[Z] = fs.speed_set_h_norm_lpf[Z] *MAX_Z_SPEED_UP;
		}
		else
		{
			/*设置下降速度*/
			fs.speed_set_h[Z]  = fs.speed_set_h_norm_lpf[Z] *MAX_Z_SPEED_DW;
		}
	}
	else
	{
		fs.speed_set_h[Z] = 0 ;
	}
    
	float speed_set_tmp[2];
    
	/*速度设定量，正负参考ANO坐标参考方向*/
	fs.speed_set_h_norm[X] = (my_deadzone(+CH_N[CH_PIT],0,50) *0.0022f); // 对 X方向的速度进行控制，pitch表示
	fs.speed_set_h_norm[Y] = (my_deadzone(-CH_N[CH_ROL],0,50) *0.0022f); // 对 y方向的速度进行控制，roll表示
		
	LPF_1_(3.0f,dT_ms*1e-3f,fs.speed_set_h_norm[X],fs.speed_set_h_norm_lpf[X]); // 滤波
	LPF_1_(3.0f,dT_ms*1e-3f,fs.speed_set_h_norm[Y],fs.speed_set_h_norm_lpf[Y]);
	
	max_speed_lim = MAX_SPEED; // 最大水平速度
	
	if(switchs.of_flow_on || switchs.dy_pmw3901_on)		//使用光流模块，就修改最大水平速度
	{
		max_speed_lim = 1.5f *wcz_hei_fus.out;
		max_speed_lim = LIMIT(max_speed_lim,50,150);
	}
	
	speed_set_tmp[X] = max_speed_lim *fs.speed_set_h_norm_lpf[X];
	speed_set_tmp[Y] = max_speed_lim *fs.speed_set_h_norm_lpf[Y];
	
	length_limit(&speed_set_tmp[X],&speed_set_tmp[Y],max_speed_lim,fs.speed_set_h_cms);// 这里做了个限制幅度？

/***************OpenMv控制模式***************/
	if(DY_Debug_Mode == 1)
	{
		fs.speed_set_h[X] = dy_pit;
		fs.speed_set_h[Y] = dy_rol;
	}
	else
	{
		fs.speed_set_h[X] = fs.speed_set_h_cms[X];
		fs.speed_set_h[Y] = fs.speed_set_h_cms[Y];
	}
/******************************************************/	

	/*调用检测着陆的函数*/
	land_discriminat(dT_ms);
	
	/*倾斜过大上锁*/
	if(rolling_flag.rolling_step == ROLL_END)
	{
		if(imu_data.z_vec[Z]<0.25f)//75度  ////////////////////////////////////////*************************** 倾斜过大上锁，慎用。
		{

			flag.fly_ready = 0;
		}
	}
    
	/*校准中，复位重力方向*/
	if(sensor.gyr_CALIBRATE != 0 || sensor.acc_CALIBRATE != 0 ||sensor.acc_z_auto_CALIBRATE)
	{
		imu_state.G_reset = 1;
	}
	
	/*复位重力方向时，认为传感器失效*/
	if(imu_state.G_reset == 1)
	{
		flag.sensor_ok = 0;
		WCZ_Data_Reset();
	}
	else if(imu_state.G_reset == 0)
	{	
		if(flag.sensor_ok == 0)
		{
			flag.sensor_ok = 1;
			DY_DT_SendString("IMU OK!",sizeof("IMU OK!"));
		}
	}
	
	/*飞行状态复位*/
	// 如果有错误导致flag.flying=0，那么需要把其他标志位也复位
	if(flag.fly_ready == 0)
	{
		flag.flying = 0;
		landing_cnt = 0;
		flag.taking_off = 0;
		flying_cnt = 0;
		
		//复位融合
		if(flag.taking_off == 0)
		{
          
		}
	}
}

static void LED_Switch()
{
	if( (LED_state != 0 && LED_state <= 115) )
	{
		return;
	}
	
	if(flag.flight_mode == ATT_STAB)
	{
		if(flag.fly_ready)
		{
			LED_state = 131;
		}
		else
		{
			LED_state = 121;
		}
	}
	else if(flag.flight_mode == LOC_HOLD)
	{
		if(flag.fly_ready)
		{
			LED_state = 132;
		}
		else
		{
			LED_state = 122;
		}		
	}
	else if(flag.flight_mode == RETURN_HOME)
	{
		if(flag.fly_ready)
		{
			LED_state = 133;
		}
		else
		{
			LED_state = 123;
		}	
	}

}

static u8 of_light_ok;
static u16 of_light_delay;
void Swtich_State_Task(u8 dT_ms)
{
	switchs.baro_on = 1;
    
    if(sens_hd_check.dy_pmw3901_ok)     //ATK-PMW3901光流模块
    {
      if(flag.flight_mode == LOC_HOLD)
      {
        if(switchs.dy_pmw3901_on == 0)
        {
          LED_state = 14 ;      //切换指示触发2下（2闪蓝）
        }
        switchs.dy_pmw3901_on = 1;
      }
      else
      {
        if(switchs.dy_pmw3901_on)
        {
          LED_state =  24 ;     //切换指示触发1下（2闪红）
        }
        switchs.dy_pmw3901_on = 0;
      }
    }
    else
    {
      switchs.dy_pmw3901_on = 0;
    }

	if(sens_hd_check.of_ok)     //匿名光流模块
	{
		if(OF_LIGHT>20 || flag.flying == 0)     //光流亮度大于20或者在飞行之前，认为光流可用，判定可用延迟时间为1秒
		{
			if(of_light_delay<1000)
			{
				of_light_delay += dT_ms;
			}
			else
			{
				of_light_ok = 1;
			}
		}
		else
		{
			of_light_delay =0;
			of_light_ok = 0;
		}
		
		if(OF_ALT<1900 && flag.flight_mode == LOC_HOLD)
		{		
			if(of_light_ok)
			{
				if(switchs.of_flow_on == 0)
				{
					LED_state = 13 ;        //切换指示触发1下（1闪蓝）
				}
				switchs.of_flow_on = 1;
			}
			else
			{
				if(switchs.of_flow_on )
				{
					LED_state = 23 ;        //切换指示触发1下（1闪红）
				}
				switchs.of_flow_on = 0;
			}
			
			if(switchs.of_tof_on == 0)
			{
				LED_state = 14 ;        //切换指示触发1下（2闪蓝）
			}
			switchs.of_tof_on = 1;
		}
		else
		{
			if(switchs.of_tof_on )
			{
				LED_state =  24 ;       //切换指示触发1下（2闪红）
			}
			switchs.of_tof_on = 0;
			
			
			switchs.of_flow_on = 0;
		}			
	}
	else
	{
		switchs.of_flow_on = switchs.of_tof_on = 0;
	}
	
	if(sens_hd_check.tof_ok)        //TOF模块
	{
		if(tof_height_mm<1900)
		{
			if(switchs.tof_on == 0)
			{
				LED_state = 14 ;        //切换指示触发1下（2闪蓝）
			}
			switchs.tof_on = 1;
		}
		else
		{
			if(switchs.tof_on )
			{
				LED_state = 24 ;        //切换指示触发1下（2闪红）
			}
			switchs.tof_on = 0;
		}
	}
	else
	{
		switchs.tof_on = 0;
	}
}

u8 speed_mode_old = 255;
u8 flight_mode_old = 255;

u8 DY_Debug_Mode = 0;   //启用OpenMv控制
u8 DY_Debug_Height_Mode = 0;
u8 DY_Debug_Yaw_Mode = 0;
u32 DY_Task_ExeTime = 0;
u8 DY_CountTime_Flag = 0;
u8 DY_Land_Flag = 0;

void Flight_Mode_Set(u8 dT_ms)
{
	LED_Switch();

	if(speed_mode_old != flag.speed_mode)
	{
		speed_mode_old = flag.speed_mode;
		
		if(flag.speed_mode == 1)
		{
			LED_state = 13;
		}
		else if(flag.speed_mode == 2)
		{
			LED_state = 14;
		}
		else
		{
			LED_state = 15;
		}
	}
	
	if(CH_N[AUX1]<-200)		        //-500`-200
	{
		flag.flight_mode = ATT_STAB;		//姿态模式
	}
	else if(CH_N[AUX1]<200)		//-200~200
	{
		flag.flight_mode = LOC_HOLD;		//定高悬停
	}
	else		                    //200~500
	{
		flag.flight_mode = LOC_HOLD;		//定高悬停
        one_key_land();		                //一键降落
	}
	
	
	if(flight_mode_old != flag.flight_mode) //状态改变
	{
		flight_mode_old = flag.flight_mode;
		
	}
	
	if(CH_N[AUX2]<-200)
	{
      if(DY_Debug_Height_Mode==0)
      {
        DY_Debug_Height_Mode = 1;
        one_key_take_off();
        dy_height = 30;
      }
      else
      {
        if(tof_height_mm>=1200 && DY_CountTime_Flag==0)
        {
          dy_height = 0;
          DY_CountTime_Flag = 1;
        }
        if(DY_CountTime_Flag)
        {
          DY_Task_ExeTime++;
          if(DY_Task_ExeTime>=1500 && DY_Land_Flag==0)
          {
            DY_Land_Flag = 1;
            one_key_land();     //一键降落
          }
//          if(DY_Task_ExeTime>=1000 && DY_Debug_Mode==0)
//          {
//            DY_Debug_Mode = 1;
//            MAP_UARTCharPut(UART5_BASE, 'H');     //OpenMv开始工作
//          }
        }
      }
	}
	else if(CH_N[AUX2]<200)
	{
		
	}
	else
	{
      
	}
	
	if(CH_N[AUX3]<-200)
	{
      
	}
	else if(CH_N[AUX3]<200)
	{
		
	}
	else
	{
      
	}
    
}

/******************* (C) COPYRIGHT 2018 DY EleTe *****END OF FILE************/
