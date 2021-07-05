/******************** (C) COPYRIGHT 2018 DeYan Electronic Technology ********************************
 * 作者    ：徳研电科
 * 官网    ：www.gototi.com
 * 描述    ：任务调度
******************************************************************************************************/
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

// test_dT_1000hz
// [0]:上次计时 时刻
// [1]：当前时刻  [2]：两次计时的间隔时间
u32 test_dT_1000hz[3];
// test_rT
// [3] 进入Loop_1000Hz的时刻
// [4] 执行完Loop_1000Hz的时刻
// [5] 本次执行Loop_1000Hz的时长
u32 test_rT[6];
static void Loop_1000Hz(void)	//1ms执行一次
{ 
	test_dT_1000hz[0] = test_dT_1000hz[1];
	test_rT[3] = test_dT_1000hz[1] = GetSysTime_us ();
	test_dT_1000hz[2] = (u32)(test_dT_1000hz[1] - test_dT_1000hz[0]) ;
    //////////////////////////////////////////////////////////////////////	
	/*-传感器数据读取*/
	Fc_Sensor_Get();            //读取ICM20602（1ms）、AK8975+SPL0601（20ms）原始数据
	
	/*-惯性传感器数据准备*/
	Sensor_Data_Prepare(1);     //ICM20602数据处理（滤波、转换）==>陀螺仪（度每秒、弧度每秒）+加速度计（厘米每平方秒）
	
	/*~~~姿态解算更新*/
	IMU_Update_Task(1);         //使用四元数法进行姿态更新
	
	/*~~~获取WC_Z加速度*/
	WCZ_Acc_Get_Task();         //地理坐标系下Z轴的运动加速度==>wcz_acc_use
	
	/*~~~飞行状态任务*/
	Flight_State_Task(1,CH_N);
	
	/*开关状态任务*/
	Swtich_State_Task(1);       //判断ATK-PMW3901光流模块、光流、TOF数据是否有效
	
	/*姿态角速度环控制*/
	Att_1level_Ctrl(1e-3f);
	
	/*电机输出控制*/
	Motor_Ctrl_Task(1);
		
	/*数传数据交换*/
	DY_DT_Data_Exchange();
    //////////////////////////////////////////////////////////////////////	
	test_rT[4]= GetSysTime_us ();
	test_rT[5] = (u32)(test_rT[4] - test_rT[3]) ;
}

static void Loop_500Hz(void)	//2ms执行一次
{	
	/*OpenMv控制*/
	DY_Flight_Control();
}

static void Loop_200Hz(void)	//5ms执行一次
{
  
}

static void Loop_100Hz(void)	//10ms执行一次
{
    test_rT[0]= GetSysTime_us ();
    //////////////////////////////////////////////////////////////////////	
	/*遥控器数据处理*/
	RC_duty_task(10);
	
	/*飞行模式设置任务*/
	Flight_Mode_Set(10);
	
	/*获取姿态角（ROLL PITCH YAW）*/
	calculate_RPY();
	
	/*姿态角度环控制*/
	Att_2level_Ctrl(10e-3f,CH_N);
	
	/*位置速度环控制*/
	Loc_1level_Ctrl(10,CH_N);
	
	/*高度数据融合任务*/
	WCZ_Fus_Task(10);
	
	/*高度速度环(内环)控制*/
	Alt_1level_Ctrl(10e-3f);
	
	/*高度环(外环)控制*/
	Alt_2level_Ctrl(10e-3f);
	
	/*--*/	
	DY_OF_DataAnl_Task(10);
    
    /*光流数据融合*/
    if(sens_hd_check.dy_pmw3901_ok)
    {
      OpticalFlow_DataFusion_Task();
    }

	/*灯光控制*/	
	LED_Task(10);
    //////////////////////////////////////////////////////////////////////		
    test_rT[1]= GetSysTime_us ();
    test_rT[2] = (u32)(test_rT[1] - test_rT[0]) ;	    
}

static void Loop_50Hz(void)	//20ms执行一次
{	
	/*罗盘数据处理任务*/
	Mag_Update_Task(20);
}

static void Loop_20Hz(void)	//50ms执行一次
{	
	/*TOF激光任务*/
	Drv_Vl53_RunTask();
	/*电压相关任务*/
	Power_UpdateTask(50);
}

static void Loop_2Hz(void)	//500ms执行一次
{
	/*延时存储任务*/
	DY_Parame_Write_task(500);
}

//系统任务配置，创建不同执行频率的“线程”
//task_func rate_hz interval_ticks last_run
static sched_task_t sched_tasks[] = 
{
	{Loop_1000Hz,1000,  0, 0},
	{Loop_500Hz , 500,  0, 0},
	{Loop_200Hz , 200,  0, 0},
	{Loop_100Hz , 100,  0, 0},
	{Loop_50Hz  ,  50,  0, 0},
	{Loop_20Hz  ,  20,  0, 0},
	{Loop_2Hz   ,   2,  0, 0},
};

//根据数组长度，判断线程数量
#define TASK_NUM (sizeof(sched_tasks)/sizeof(sched_task_t))


// 初始化sched_tasks结构体，即初始化任务表
void Scheduler_Setup(void)
{
	uint8_t index = 0;
	//初始化任务表
	for(index=0;index < TASK_NUM;index++)
	{
		//计算每个任务的延时周期数
		sched_tasks[index].interval_ticks = TICK_PER_SECOND/sched_tasks[index].rate_hz;
		//最短周期为1，也就是1ms
		if(sched_tasks[index].interval_ticks < 1)
		{
			sched_tasks[index].interval_ticks = 1;
		}
	}
}

//这个函数放到main函数的while(1)中，不停判断是否有线程应该执行
void Scheduler_Run(void)
{
	uint8_t index = 0;
	//循环判断所有线程，是否应该执行
	for(index=0;index < TASK_NUM;index++)
	{
		//获取系统当前时间，单位ms
		uint32_t tnow = SysTick_GetTick();

		//进行判断，如果当前时间减去上一次执行的时间，大于等于该线程的执行周期，则执行线程
		if(tnow - sched_tasks[index].last_run >= sched_tasks[index].interval_ticks)
		{
			//更新线程的执行时间，用于下一次判断
			sched_tasks[index].last_run = tnow;
			//执行线程函数，使用的是函数指针
			sched_tasks[index].task_func();
		}	 
	}
}
// 系统时钟为120M，执行1条指令用时为1ns左右，所以每个Loop_1000Hz实际执行时间很短，势必不会超过1ms
// 但tnow按ms计数，也就是说，每次1ms的程序都会被执行，而500ms得500ms后才会执行。
// 执行时间很短，直接用轮询系统不可以吗
// 是因为为了保证固定间隔时间姿态数据能返回，好做运算吗？


/******************* (C) COPYRIGHT 2018 DeYan Electronic Technology **********************************/
