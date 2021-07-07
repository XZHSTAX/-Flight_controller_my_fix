#ifndef __DY_PID_H
#define __DY_PID_H
#include "DY_FcData.h"
/*=====================================================================================================================
						 *****
=====================================================================================================================*/
typedef struct
{
	u8 fb_d_mode;
	float kp;			 //比例系数
	float ki;			 //积分系数
	float kd_ex;		 	 //微分系数（期望微分系数）
	float kd_fb; //previous_d 微分先行（反馈微分系数）
//	float inc_hz;  //不完全微分低通系数
//	float k_inc_d_norm; //Incomplete 不完全微分 归一（0,1）
	float k_ff;		 //前馈 

}_PID_arg_st;

typedef struct
{
	float err;           // 误差，当前值
	float exp_old;       // 上次的期望值
	float feedback_old;  // 上次的反馈值
	
	float fb_d;          // 反馈的微分值
	float fb_d_ex;
	float exp_d;         // 期望的微分值
//	float err_d_lpf;
	float err_i;         // 误差累计值
	float ff;
	float pre_d;

	float out;
}_PID_val_st;

float PID_calculate( float T,            //周期
										float in_ff,				//前馈
										float expect,				//期望值（设定值）
										float feedback,			//反馈值
										_PID_arg_st *pid_arg, //PID参数结构体
										_PID_val_st *pid_val,	//PID数据结构体
										float inte_d_lim,
										float inte_lim			//integration limit，积分限幅
										   );			//输出

									
#endif
