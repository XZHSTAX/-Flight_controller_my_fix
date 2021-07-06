#include "DY_FlightDataCal.h"
#include "DY_Imu.h"
#include "Drv_icm20602.h"
#include "DY_MagProcess.h"
#include "Drv_spl06.h"
#include "Drv_ak8975.h"
#include "DY_MotionCal.h"
#include "Drv_vl53l0x.h"
#include "Drv_led.h"
#include "DY_OF.h"


// 获取陀螺仪、加速度计、电子罗盘，气压计的原始数据
void Fc_Sensor_Get()
{
	static u8 cnt;
	if(flag.start_ok)   // 如果初始化OK
	{
		/*读取陀螺仪和加速度计数据，读完后会放到mpu_buffer中*/
		Drv_Icm20602_Read();     
		
		cnt ++;
		cnt %= 20;
		if(cnt==0)     // 相当于每20ms读取一次电子罗盘磁力计和气压计
		{
			/*读取电子罗盘磁力计数据，读完后放到ak8975_buf中*/
			Drv_AK8975_Read();
			/*读取气压计数据*/
			baro_height = (s32)Drv_Spl0601_Read();
		}
	}
}

extern s32 sensor_val_ref[];

static u8 reset_imu_f;
// 姿态计算，更新，融合，输入sensor.Gyro_rad sensor.Acc_cmss；输出：imu_data
void IMU_Update_Task(u8 dT_ms)
{
        ////////////////////////////////////////////////////////////////////////		
        /*如果准备飞行，复位重力复位标记和磁力计复位标记*/
        if(flag.fly_ready )
        {
            imu_state.G_reset = imu_state.M_reset = 0;
            reset_imu_f = 0;
        }
        else
        {
            if(flag.motionless == 0)
            {
              
            }	
            
            if(reset_imu_f==0 )//&& flag.motionless == 1)
            {
                imu_state.G_reset = 1;      //重力复位	
                sensor.gyr_CALIBRATE = 2;   //校准陀螺仪，不保存
                reset_imu_f = 1;            //已经置位复位标记
            }            
        }
        
        if(0) 
        {
            imu_state.gkp = 0.0f;
            imu_state.gki = 0.0f;
            
        }
        else
        {
            if(0)
            {
                imu_state.gkp = 0.2f;
            }
            else
            {
                /*设置重力互补融合修正kp系数*/
                imu_state.gkp = 0.3f;//0.4f;
            }
            
            /*设置重力互补融合修正ki系数*/
            imu_state.gki = 0.002f;
            
            /*设置罗盘互补融合修正ki系数*/
            imu_state.mkp = 0.2f;
        }
        
        imu_state.M_fix_en = sens_hd_check.mag_ok;		//磁力计修正使能
        
        /*姿态计算，更新，融合*/
        IMU_update(dT_ms *1e-3f, &imu_state,sensor.Gyro_rad, sensor.Acc_cmss, mag.val,&imu_data);//x3_dT_1[2] * 0.000001f
}

static s16 mag_val[3];
void Mag_Update_Task(u8 dT_ms)
{
	Mag_Get(mag_val);
	
	Mag_Data_Deal_Task(dT_ms,mag_val,imu_data.z_vec[Z],sensor.Gyro_deg[X],sensor.Gyro_deg[Z]);
}


s32 baro_height,baro_h_offset,ref_height_get_1,ref_height_get_2,ref_height_used;
s32 baro2tof_offset,tof2baro_offset;

float baro_fix1,baro_fix2,baro_fix;

static u8 wcz_f_pause;
float wcz_acc_use;			

void WCZ_Acc_Get_Task()//最小周期
{
	wcz_acc_use += 0.2f *(imu_data.w_acc[Z] - wcz_acc_use);
    
}

//void Baro_Get_Task()
//{
////			ref_height_get += LIMIT((s32)user_spl0601_get() - ref_height_get,-20,20 );
////	baro_height =(s32)user_spl0601_get();
//}

u16 ref_tof_height;
static u8 baro_offset_ok,tof_offset_ok;
void WCZ_Fus_Task(u8 dT_ms)
{
	if(flag.taking_off)
	{
		baro_offset_ok = 2;
	}
	else
	{
		if(baro_offset_ok == 2)
		{
			baro_offset_ok = 0;
		}
	}
	
	if(baro_offset_ok >= 1)//(flag.taking_off)
	{
		ref_height_get_1 = baro_height - baro_h_offset + baro_fix  + tof2baro_offset;//气压计相对高度，切换点跟随TOF
	}
	else
	{
		if(baro_offset_ok == 0 )
		{
			baro_h_offset = baro_height;
			if(flag.sensor_ok)
			{
				baro_offset_ok = 1;
			}
		}
	}
	
	if((flag.flying == 0) && flag.auto_take_off_land == AUTO_TAKE_OFF	)
	{
		wcz_f_pause = 1;
		
		baro_fix = 0;
	}
	else
	{
		wcz_f_pause = 0;
		
		if(flag.taking_off == 0)
		{
			baro_fix1 = 0;
			baro_fix2 = 0;
				
		}
		baro_fix2 = -BARO_FIX;

		
		baro_fix = baro_fix1 + baro_fix2 - BARO_FIX;//+ baro_fix3;
	}
	
	if((sens_hd_check.tof_ok || sens_hd_check.of_ok) && baro_offset_ok) //TOF或者OF硬件正常，且气压计记录相对值以后
	{
		if(switchs.tof_on || switchs.of_tof_on) //TOF数据有效
		{
			if(switchs.of_tof_on) //光流带TOF，光流优先
			{
				ref_tof_height = OF_ALT;
			}
			else
			{
				ref_tof_height = tof_height_mm/10;
			}
			
			if(tof_offset_ok == 1)
			{
				ref_height_get_2 = ref_tof_height + baro2tof_offset;//TOF参考高度，切换点跟随气压计
				
				ref_height_used = ref_height_get_2;
				
				tof2baro_offset += 0.5f *((ref_height_get_2 - ref_height_get_1) - tof2baro_offset);//记录气压计切换点，气压计波动大，稍微滤波一下
				//tof2baro_offset = ref_height_get_2 - ref_height_get_1;				
			}
			else
			{
				baro2tof_offset = ref_height_get_1 - ref_tof_height ; //记录TOF切换点
				
				tof_offset_ok = 1;
			}
		}
		else
		{
			tof_offset_ok = 0;
			
			ref_height_used = ref_height_get_1 ;
		}
	}
	else
	{
		ref_height_used = ref_height_get_1;
	}
	
	WCZ_Data_Calc(dT_ms,wcz_f_pause,(s32)wcz_acc_use,(s32)(ref_height_used));
}
