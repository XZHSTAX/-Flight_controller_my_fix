#include "DY_AltCtrl.h"
#include "DY_Imu.h"
#include "Drv_icm20602.h"
#include "DY_MagProcess.h"
#include "Drv_spl06.h"
#include "DY_MotionCal.h"
#include "DY_FlightCtrl.h"
#include "DY_MotorCtrl.h"
#include "DY_AttCtrl.h"
#include "DY_LocCtrl.h"

#include "DY_FlightCtrl.h"

#include "operation.h"
// static s16 auto_taking_off_speed;
// 起飞时的附加速度，起飞后为0
s16 auto_taking_off_speed;

/***************高度控制变量初始化***************/
s16 dy_height = 0;
#define USE_SYETEM_HEIGH_CONTROL 0
#define AUTO_TAKE_OFF_KP 2.0f
////extern _filter_1_st wz_spe_f1;
// 一键起飞任务，判断标志位，为flag.auto_take_off_land标志位赋值
// 主要功能: 置位flag.auto_take_off_land;调用one_key_take_off_task
// 所需条件：1.flag.fly_ready=1
// ---------2.flag.taking_off = 1
// ---------3.flag.auto_take_off_land = AUTO_TAKE_OFF_NULL
// 主要功能： 为auto_taking_off_speed赋值
// 所需条件：flag.auto_take_off_land ==AUTO_TAKE_OFF
void Auto_Take_Off_Land_Task(u8 dT_ms)

{
	static u16 take_off_ok_cnt;
	
	one_key_take_off_task(dT_ms);
	
	if(flag.fly_ready)		//解锁
	{
		if(flag.taking_off)		//起飞
		{	
			if(flag.auto_take_off_land == AUTO_TAKE_OFF_NULL)
			{
				flag.auto_take_off_land = AUTO_TAKE_OFF;		
			}
		}

	}
	else
	{
		auto_taking_off_speed = 0;	
		flag.auto_take_off_land = AUTO_TAKE_OFF_NULL;	
	}
////////////////
	
	if(flag.auto_take_off_land ==AUTO_TAKE_OFF)
	{

		take_off_ok_cnt += dT_ms;
		
/***************刘德祥修改***************/
		auto_taking_off_speed = AUTO_TAKE_OFF_KP *(DY_Parame.set.auto_take_off_height - wcz_hei_fus.out);

/*******************************/
		
		auto_taking_off_speed = LIMIT(auto_taking_off_speed,0,150);		//限幅 0~150 
		
		// 当时间超过25s或者飞行高度与设定高度之差在5cm内，就认为飞机已经起飞
		if(take_off_ok_cnt>=2500 || (DY_Parame.set.auto_take_off_height - wcz_hei_fus.out <5))//(auto_ref_height>AUTO_TAKE_OFF_HEIGHT)
		{
			flag.auto_take_off_land = AUTO_TAKE_OFF_FINISH;
			
		}
		// 如果超过10s，且竖直方向有速度，就认为已经起飞
		if(take_off_ok_cnt >1000 && ABS(fs.speed_set_h_norm[Z])>0.1f)// 一定已经taking_off
		{
			flag.auto_take_off_land = AUTO_TAKE_OFF_FINISH;
		}
	
	}
	else 
	{
		take_off_ok_cnt = 0;
		
		if(flag.auto_take_off_land ==AUTO_TAKE_OFF_FINISH)
		{
			auto_taking_off_speed = 0;	
		}
	}


////////////
	
	if(flag.auto_take_off_land == AUTO_LAND)
	{
		auto_taking_off_speed = -60;

	}
}


_PID_arg_st alt_arg_2;
_PID_val_st alt_val_2;

/*高度环PID参数初始化*/
void Alt_2level_PID_Init()
{
	alt_arg_2.kp = DY_Parame.set.pid_alt_2level[KP];
	alt_arg_2.ki = DY_Parame.set.pid_alt_2level[KI];
	alt_arg_2.kd_ex = 0.00f;
	alt_arg_2.kd_fb = DY_Parame.set.pid_alt_2level[KD];
	alt_arg_2.k_ff = 0.0f;

}

void Alt_2level_Ctrl(float dT_s)
{
	Auto_Take_Off_Land_Task(1000*dT_s);
	
/***************OpenMv控制***************/
	if(DY_Debug_Height_Mode == 1)
	{
		fs.speed_set_h[Z] = dy_height;
	}
/*****************************************************/
	
	fs.alt_ctrl_speed_set = fs.speed_set_h[Z] + auto_taking_off_speed; //一旦flag.auto_take_off_land == AUTO_TAKE_OFF_FINISH， auto_taking_off_speed这一项就等于0，也就是说auto_taking_off_speed只在起飞或者降落过程中使用
	
	loc_ctrl_2.fb[Z] = wcz_hei_fus.out;
	
	if(fs.alt_ctrl_speed_set != 0)
	{
		flag.ct_alt_hold = 0;
	}
	else
	{
		if(ABS(loc_ctrl_1.exp[Z] - loc_ctrl_1.fb[Z])<20)
		{
			flag.ct_alt_hold = 1;
		}
	}
#if USE_SYETEM_HEIGH_CONTROL
	if(flag.taking_off == 1)
	{
		if(flag.ct_alt_hold == 1)		//定高悬停		flag.ct_alt_hold标志位由程序控制
		{
			PID_calculate(dT_s,            //周期（单位：秒）
						0,				//前馈值
						loc_ctrl_2.exp[Z],				//期望值（设定值）
						loc_ctrl_2.fb[Z],			//反馈值（）
						&alt_arg_2, //PID参数结构体
						&alt_val_2,	//PID数据结构体
						100,//积分误差限幅
						0			//integration limit，积分限幅									
						 );		//输出==>alt_val2.out
		}
		else
		{
			loc_ctrl_2.exp[Z] = loc_ctrl_2.fb[Z] + alt_val_2.err;
		}
	}
	else
	{
		loc_ctrl_2.exp[Z] = loc_ctrl_2.fb[Z];
		alt_val_2.out = 0;
		
	}
#else
	alt_val_2.out = 0;
#endif	
	alt_val_2.out  = LIMIT(alt_val_2.out,-150,150);
}

_PID_arg_st alt_arg_1;
_PID_val_st alt_val_1;

/*高度速度环PID参数初始化*/
void Alt_1level_PID_Init()
{
	alt_arg_1.kp = DY_Parame.set.pid_alt_1level[KP];
	alt_arg_1.ki = DY_Parame.set.pid_alt_1level[KI];
	alt_arg_1.kd_ex = 0.00f;
	alt_arg_1.kd_fb = DY_Parame.set.pid_alt_1level[KD];
	alt_arg_1.k_ff = 0.0f;

}

//static u8 thr_start_ok;
static float err_i_comp;
void Alt_1level_Ctrl(float dT_s)
{
	u8 out_en;
	out_en = (flag.taking_off != 0) ? 1 : 0;
	
	flag.thr_mode = THR_AUTO;//THR_MANUAL;
	
	loc_ctrl_1.exp[Z] = fs.alt_ctrl_speed_set + alt_val_2.out;
	
	if(0) //(flag.thr_mode == THR_MANUAL)
	{
		loc_ctrl_1.fb[Z] = 0;
	}
	else
	{
		loc_ctrl_1.fb[Z] = wcz_spe_fus.out;
	}
	
	PID_calculate( dT_s,            //周期（单位：秒）
					0,				//前馈值
					loc_ctrl_1.exp[Z],				//期望值（设定值）
					loc_ctrl_1.fb[Z] ,			//反馈值（）
					&alt_arg_1, //PID参数结构体
					&alt_val_1,	//PID数据结构体
					100,//积分误差限幅
					(THR_INTE_LIM *10 - err_i_comp)*out_en			//integration limit，积分限幅									
					 );
	
	if(flag.taking_off == 1)
	{
		LPF_1_(1.0f,dT_s,THR_START *10,err_i_comp);//err_i_comp = THR_START *10;			
	}
	else
	{
		err_i_comp = 0;
	}
	
	alt_val_1.out = LIMIT(alt_val_1.out,-err_i_comp,MAX_THR *10);
	
	loc_ctrl_1.out[Z] = out_en *FINAL_P *(alt_val_1.out + err_i_comp - 0.2f *imu_data.w_acc[Z]);
	
	mc.ct_val_thr = loc_ctrl_1.out[Z];
}
