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


_our_flags our_flags={0,0};

u32 our_delay_times[2] = {0};


void our_take_off()
{   
    if(our_delay_times[0] > 1000)
    {
        CH_N[AUX2] = -210;
        // DY_Debug_Height_Mode = 1;
        // one_key_take_off();
        // dy_height = 30;
    }
    // if(flag.auto_take_off_land != AUTO_TAKE_OFF_FINISH && our_delay_times[0] <200 ) // 10ms*2000  = 20s
    // {
    //     DY_Debug_Height_Mode = 1;
    //     one_key_take_off();
    //     dy_height = 30;
    // }
}


void our_delay_time()
{
    static u8 data[4] = {6,6,6,6};
    our_delay_times[0] += 1;

    if(our_delay_times[0] %25 == 0) // 10ms*25=250ms=0.25s
    {
        data[0] = flag.fly_ready;
        data[1] = flag.taking_off;
        data[2] = flag.auto_take_off_land;//one_key_taof_start
        data[3] = one_key_taof_start;
        zigbee_data_Sent(data,sizeof(data));
    }
 //   if(our_delay_times[0]==2000 && our_flags.delaying==0) //10ms*2000 = 20s
 //   {
 //       our_flags.delaying = 1;
 //   }   
}

void our_landing()
{
    if((our_flags.delaying)&&(flag.auto_take_off_land == AUTO_TAKE_OFF_FINISH))
    {
        our_flags.delaying = 0;
        one_key_land();
    }
}


