/******************** (C) COPYRIGHT 2018 DY EleTe ********************************
 * 作者    ：徳研电科
 * 官网    ：www.gototi.com
 * 描述    ：飞行控制
**********************************************************************************/
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef _FLIGHT_CTRL_H_
#define _FLIGHT_CTRL_H_
/* Includes ------------------------------------------------------------------*/
#include "DY_FcData.h"
#include "DY_Filter.h"
#include "DY_Math.h"

/* Exported types ------------------------------------------------------------*/

enum
{
   null=0,
   takeoff,
   landing,
	
    s_up_down_2, //小幅度上升下降2次
	
    s_yaw_pn_2,  //小幅度左右转2次
    b_yaw_pn_1,  //大幅度左右转1次

    s_rol_pn_2,  //小幅度左右2次
    b_rol_pn_1,  //大幅度左右1次
    
    s_pit_pn_2,  //小幅度前后2次
    b_pit_pn_1,  //大幅度前后1次

    yaw_n360,   //左转1圈
    yaw_p360,   //右转1圈

    roll_1,    //翻滚

    pit_jump_pn_2,  //前后跳2次
    rol_jump_pn_2,  //左右跳2次
    
    rol_up_down_2,  //蛇形上升下降	  
    yaw_up_dowm_1,  //旋转上升下降		2
    pit_rol_pn_2,   //蛇形前进后退		3
		
};

// 飞行状态
typedef struct
{
	s16 alt_ctrl_speed_set;            // 高度控制速度设置变量
	s16 speed_set_h[VEC_XYZ];	       // 速度设置向量
	float speed_set_h_cms[VEC_XYZ];    // 速度设置向量,单位：cm/s
	
	float speed_set_h_norm[VEC_XYZ];   // 速度设定值/最大速度值
	float speed_set_h_norm_lpf[VEC_XYZ];// 速度设定值/最大速度值 的滤波值
	
}_flight_state_st;
extern _flight_state_st fs;
/* Exported constants --------------------------------------------------------*/
extern float wifi_selfie_mode_yaw_vlue;

extern s16 dy_pit,dy_rol;
extern u8 DY_Debug_Mode;
extern u8 DY_Debug_Height_Mode;
extern u8 DY_Debug_Yaw_Mode;

/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
void user_fun(float dT,u8 action_num);

void All_PID_Init(void);

void one_key_take_off(void);
void one_key_land(void);

void one_key_roll(void);
void app_one_key_roll(void);
void app_one_key_roll_reset(void);
void one_key_take_off_task(u16 dt_ms);

void ctrl_parameter_change_task(void);
	
void Flight_State_Task(u8,s16 *CH_N);

void Flight_Mode_Set(u8 dT_ms);

void Swtich_State_Task(u8 dT_ms);
extern u16 one_key_taof_start;

#endif
/******************* (C) COPYRIGHT 2018 DY EleTe *****END OF FILE************/
