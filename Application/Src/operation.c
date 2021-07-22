#include "operation.h"
#include "DY_Scheduler.h"
#include "include.h"
#include "DY_RC.h"
#include "DY_Parameter.h"
#include "Drv_time.h"
#include "Drv_led.h"
#include "Drv_pwm_in.h"
#include "Drv_icm20602.h"
#include "Drv_ak8975.h"
#include "Drv_spl06.h"
#include "DY_FlightCtrl.h"
#include "DY_FlightDataCal.h"
#include "DY_AttCtrl.h"
#include "DY_Imu.h"
#include "Drv_vl53l0x.h"
#include "DY_LocCtrl.h"
#include "DY_AltCtrl.h"
#include "DY_MotorCtrl.h"
#include "DY_MagProcess.h"
#include "DY_Power.h"
#include "DY_OF.h"
#include "OpticalFlow.h"        //ATK-PMW3901光流模块
#include "DY_Flight_Log.h"      //OpenmMv控制
#include "Drv_usart.h"
#include "Drv_vl53l0x.h"
#include "DY_FcData.h"
#include "DY_Filter.h"
#include "DY_Imu.h"
#include "DY_Math.h"
#include "DY_MotionCal.h"
#include "DY_Navigate.h"
#include "DY_Pid.h"


_our_flags our_flags={0,0,1};
_PID_arg_st our_height_pid;//自定义高度控制PID
_PID_val_st our_height_pid_val;//自定义高度控制PID数据
   
u32 our_delay_times[2] = {0,0};
//our_delay_times[0]用于起飞延时，计算总时间并降落
//our_delay_times[1]用于定高悬停计时

float Height_Set = 0.0f; //设定高度值


/*******************************************************
* Function name ：our_take_off
* Description   : 放入DY_scheduler.c 10ms线程中，延迟10s,模拟摇杆启动一键起飞任务
* Parameter     ：None
* Return        ：None
**********************************************************/
void our_take_off()
{   
    if(our_delay_times[0] > 1000)
    {
        CH_N[AUX2] = -210;

    }

}
/*******************************************************
* Function name ：our_mission_updown_repeat
* Description   : 放入DY_scheduler.c 10ms线程中，延迟fly_time启动一键起飞任务
                  如果飞机高度在upper_limit和lower_limit之内，每个5s转换方向（先降后升）
                  如果飞机高度在upper_limit和lower_limit之外，调整速度方向进入阈值
* Parameter     ：None
* Return        ：None
**********************************************************/
void our_mission_updown_repeat()
{
    if((our_delay_times[0] > fly_time) && (flag.auto_take_off_land == AUTO_TAKE_OFF_FINISH))
    {
        if(wcz_hei_fus.out>lower_limit&&wcz_hei_fus.out<upper_limit)
        {
            if((our_delay_times[0]%500)==0&&our_flags.UAV_up==0)
            { 
            dy_height = 5;
            our_flags.UAV_up == 1;
            }
            else if((our_delay_times[0]%500)==0&&our_flags.UAV_up==1)
            {
            dy_height = -5;
            our_flags.UAV_up == 0;
            }
            else
                ;
        }
        else if(wcz_hei_fus.out<=lower_limit)
        {
            dy_height = 10;
        }
        else if(wcz_hei_fus.out>=upper_limit)
        {
            dy_height = -10;
        }
        else
            dy_flag.stop == 1;     
    }
}

/*******************************************************
* Function name ：our_mission_height_control
* Description   : 放入DY_scheduler.c 10ms线程中，定高任务
* Parameter     ：None
* Return        ：None
**********************************************************/
void our_mission_height_control()
{   
    if((our_delay_times[0] > fly_time) && (flag.auto_take_off_land == AUTO_TAKE_OFF_FINISH) && (our_delay_times[0] <= fly_land_time))
    {
        if(our_delay_times[1] <= 1000 )
        {
            Height_Set = 50;
        }
        else if(our_delay_times[1] > 1000 &&  our_delay_times[1] < 2000)
        {
            Height_Set = 100;
        }
        else if(our_delay_times[1] > 2000 &&  our_delay_times[1] < 3000)
        {
            Height_Set = 150;
        }
        else if(our_delay_times[1] > 3000 &&  our_delay_times[1] < 4000)
        {
            Height_Set = 50;
        }
        else 
            Height_Set = 100;
        
        if(flag.ct_alt_hold)
        {
            our_delay_times[1] += 1;
        }
        PID_calculate(10e-3f,      //周期（单位：秒）
                0,				        //前馈值
                Height_Set,				//期望值（设定值）
                wcz_hei_     fus.out,	    //反馈值（）
                &our_height_pid, //PID参数结构体
                &our_height_pid_val,	//PID数据结构体
                100,        //积分误差限幅
                0			//integration limit，积分限幅									
                );	 
        dy_height =  our_height_pid_val.out; 
    }
}



/*******************************************************
* Function name ：our_height_pid_Init
* Description   : 放入All_PID_Init(void)中，初始化定高控制的PID参数
* Parameter     ：None
* Return        ：None
**********************************************************/
void our_height_pid_Init()
{   
    our_height_pid.kp = 1.0f;  //比例系数
    our_height_pid.ki = 3.0f;  //积分系数
    our_height_pid.kd_ex = 0.00f;  //微分系数（期望微分系数）
    our_height_pid.kd_fb = 0.01f;  //previous_d 微分先行（反馈微分系数）
    our_height_pid.k_ff = 0.0f;    //前馈系数
}


/*******************************************************
* Function name ：our_delay_time
* Description   : 放入DY_scheduler.c 10ms线程中，返回重要标志位给stm32
* Parameter     ：None
* Return        ：None
**********************************************************/
void our_delay_time()
{
    static u8 data[] = {6,6,6,6,6,6,6,6};
    our_delay_times[0] += 1;

    if(our_delay_times[0] %25 == 0) // 10ms*25=250ms=0.25s
    {
        data[0] = flag.fly_ready;
        data[1] = flag.taking_off;
        data[2] = flag.auto_take_off_land;//one_key_taof_start
        data[3] = one_key_taof_start;
        data[4] = (tof_height_mm&0xff00)>>8;
        data[5] = tof_height_mm&0x00ff;
        data[6] = (uint8_t)( ((uint16_t)auto_taking_off_speed&0xff00)>>8);
        data[7] = (auto_taking_off_speed&0x00ff);
        zigbee_data_Sent(data,sizeof(data));
    }
}


/*******************************************************
* Function name ：our_landing
* Description   : 放入DY_scheduler.c 10ms线程中，一键降落任务
* Parameter     ：None
* Return        ：None
**********************************************************/
void our_landing() 
{
    if(our_delay_times[0] > fly_land_time)
    {
        one_key_land();
    }
}


