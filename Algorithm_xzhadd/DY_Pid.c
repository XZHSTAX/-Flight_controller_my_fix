/******************** (C) COPYRIGHT 2016 ANO Tech ***************************
 * 作者		 ：匿名科创
 * 文件名  ：ANO_PID.c
 * 描述    ：PID函数
 * 官网    ：www.anotc.com
 * 淘宝    ：anotc.taobao.com
 * 技术Q群 ：190169595
*****************************************************************************/
#include "DY_Pid.h"
#include "DY_Math.h"
#include "DY_Filter.h"

// PID控制器
// 输出值为 PID的运算输出(微分先行)
// in_ff 前馈值
// pid_arg为pid的参数
// pid_val为PID运算过程中储存值
// 因此，传入对应的arg和val便可以用于不同环的控制
float PID_calculate( float dT_s,            //周期（单位：秒）
										float in_ff,				//前馈值
										float expect,				//期望值（设定值）
										float feedback,			//反馈值（）
										_PID_arg_st *pid_arg, //PID参数结构体
										_PID_val_st *pid_val,	//PID数据结构体
										float inte_d_lim,//积分误差限幅
										float inte_lim			//integration limit，积分限幅									
										 )	
{
	float differential,hz;
	hz = safe_div(1.0f,dT_s,0);  // 获取频率
	
//	pid_arg->k_inc_d_norm = LIMIT(pid_arg->k_inc_d_norm,0,1);
	
	//误差 = 期望 - 反馈
	pid_val->err = (expect - feedback);		//ERROR 误差
	
	//期望值的微分 = （期望 - 上次期望） * 频率		
	pid_val->exp_d = (expect - pid_val->exp_old) *hz;//如何正确理解微分定义？  dx/dt = lim  [x(t+dt) - x(t)]/dt
	
	// 如果启用微分先行算法
	if(pid_arg->fb_d_mode == 0)
	{
		//反馈值的微分 = （反馈 - 上次反馈） * 频率		 
		pid_val->fb_d = (feedback - pid_val->feedback_old) *hz;
	}
	else
	{
		pid_val->fb_d = pid_val->fb_d_ex;
	}
	// 微分先行算法
	//微分 = 期望微分系数 * 期望微分值 - 反馈微分系数 * 反馈微分值	
	differential = (pid_arg->kd_ex *pid_val->exp_d - pid_arg->kd_fb *pid_val->fb_d);		//偏差
	
//	if((s16)(100 *pid_arg->inc_hz)!=0)
//	{
//		LPF_1_(pid_arg->inc_hz,T,differential,pid_val->err_d_lpf );
//	}
//	else
//	{
//		pid_val->err_d_lpf = 0;
//	}
	// 计算误差累计值，即对误差积分，并且限值
		pid_val->err_i += pid_arg->ki *LIMIT((pid_val->err ),-inte_d_lim,inte_d_lim )*dT_s;//)*T;//+ differential/pid_arg->kp
	//pid_val->err_i += pid_arg->ki *(pid_val->err )*T;//)*T;//+ pid_arg->k_pre_d *pid_val->feedback_d
		pid_val->err_i = LIMIT(pid_val->err_i,-inte_lim,inte_lim);
	
	
	
	pid_val->out = pid_arg->k_ff *in_ff 
	    + pid_arg->kp *pid_val->err  
			+	differential
//	    + pid_arg->k_inc_d_norm *pid_val->err_d_lpf + (1.0f-pid_arg->k_inc_d_norm) *differential
    	+ pid_val->err_i;		//PID控制最后输出
	
	pid_val->feedback_old = feedback;
	pid_val->exp_old = expect;
	
	return (pid_val->out);
}

/******************* (C) COPYRIGHT 2016 ANO TECH *****END OF FILE************/
