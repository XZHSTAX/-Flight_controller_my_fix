/******************** (C) COPYRIGHT 2018 DY EleTe ********************************
 * 作者    ：徳研电科
 * 官网    ：www.gototi.com
 * 描述    ：飞控数据
**********************************************************************************/
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef _DY_FCDATA_H_
#define _DY_FCDATA_H_

/* Includes ------------------------------------------------------------------*/
#include "common.h"
#include "config.h"

/* Exported types ------------------------------------------------------------*/

#define TRUE 1
#define FALSE 0 

enum
{
      AUTO_TAKE_OFF_NULL = 0,
      AUTO_TAKE_OFF,            //一键起飞
      AUTO_TAKE_OFF_FINISH,
      AUTO_LAND,                //一键降落
};

enum pwminmode_e
{
      PWM = 0,
      PPM,
      SBUS,
};

enum
{
      A_X = 0,
      A_Y ,
      A_Z ,
      G_X ,
      G_Y ,
      G_Z ,
      TEM ,
      MPU_ITEMS ,
};

	
enum
{
      CH_ROL = 0,
      CH_PIT ,
      CH_THR ,
      CH_YAW ,
      AUX1 ,
      AUX2 ,
      AUX3 ,
      AUX4 ,
      CH_NUM,
};

enum
{
      m1 = 0,           //电机
      m2,
      m3,
      m4,
      m5,
      m6,
      m7,
      m8,

};

enum
{
      X = 0,
      Y,
      Z,
      VEC_XYZ,
};

enum
{
      ROL = 0,
      PIT,
      YAW,
      VEC_RPY,
};

enum
{
      KP = 0,
      KI,
      KD,
      PID,
};

enum _power_alarm
{

      HIGH_POWER = 0,           //电池电量
      HALF_POWER,
      LOW_POWER ,
      LOWEST_POWER, 
	

};


enum _flight_mode
{
      ATT_STAB = 0,             //Attitude stabilization
      LOC_HOLD,
      RETURN_HOME,
	
};

//thr_mode
enum
{
      THR_MANUAL = 0,
      THR_AUTO,
	
};

typedef struct
{
      u8 first_f;
      float acc_offset[VEC_XYZ];
      float gyro_offset[VEC_XYZ];

      float surface_vec[VEC_XYZ];

      float mag_offset[VEC_XYZ];
      float mag_gain[VEC_XYZ];

} _save_st ;
extern _save_st save;

typedef struct
{
      //基本状态/传感器
      u8 start_ok;	//系统初始化OK
      u8 sensor_ok;
      u8 motionless;
      u8 power_state;
      u8 wifi_ch_en;
      u8 rc_loss;	
      u8 gps_ok;	
      u8 gps_signal_bad;


      //控制状态
      u8 manual_locked;
      u8 unlock_en;
      u8 fly_ready;  //unlocked 准备起飞
      u8 thr_low;
      u8 locking;
      u8 taking_off; //起飞
      u8 set_yaw;
      u8 ct_loc_hold;
      u8 ct_alt_hold;


      //飞行状态
      u8 flying;
      u8 auto_take_off_land;
      u8 home_location_ok;	
      u8 speed_mode;
      u8 thr_mode;	
      u8 flight_mode;
      u8 gps_mode_en;
      u8 motor_preparation;
      u8 locked_rotor;
	
}_flag;
extern _flag flag;

typedef struct
{
      u8 sonar_on;
      u8 tof_on;
      u8 of_flow_on;
      u8 of_tof_on;
      u8 baro_on;
      u8 gps_on;
      
      u8 dy_opticalflow_on;     //徳研光流
      u8 dy_pmw3901_on;
	
}_switch_st;
extern _switch_st switchs;

typedef struct
{
      u8 gyro_ok;
      u8 acc_ok;
      u8 mag_ok;
      u8 baro_ok;
      u8 gps_ok;
      u8 sonar_ok;
      u8 tof_ok;
      u8 of_ok;
      
      u8 dy_opticalflow_ok;     //徳研光流
      u8 dy_pmw3901_ok;
	
} _sensor_hd_check_st; //Hardware
extern _sensor_hd_check_st sens_hd_check;

/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

void data_save(void);
void Para_Data_Init(void);

#endif

/******************* (C) COPYRIGHT 2018 DY EleTe *****END OF FILE************/
