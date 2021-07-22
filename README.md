# 文件树解析

## 整体文件夹

```
├─.vscode
├─Application
│  ├─Inc
│  └─Src
├─Debug
│  ├─Exe
│  ├─List
│  └─Obj
├─Drive
│  ├─Inc
│  ├─Src
│  └─Vl53l0x
│      ├─Inc
│      └─Src
├─figure
├─FlyControl_Algorithm
│  ├─Inc
│  └─src
├─FlyControl_Calculate
│  ├─Inc
│  └─Src
├─Library
├─Note
└─settings
```

## Application

```c
Application
├─Inc  
└─Src
        BSP_Init.c        // 初始化所有（底层驱动、中断配置）
        DY_DT.c           // 数据传输
        DY_Flight_Log.c
        DY_OF.c          // 匿名optical flow 光流的驱动(使用uart4中断接收数据) 本项目未用到
        DY_Parameter.c
        DY_power.c
        DY_RC.c          // 遥控器通道数据处理
        DY_Scheduler.c   // 任务调度（所有的任务及其调度方式）
        DY_Tracking.c
        main.c
        OpticalFlow.c    // DY or ATK-PMW3901 光流驱动，使用SPI接收光流信息
```

此文件夹存放了`main.c`文件，整个系统的启动、初始化、电源管理都在此处。`Inc`中存放的是对应的`.h`文件，这里不再打印。

## FlyControl_Calculate

```C
FlyControl_Calculate
├─Inc    
└─Src
        DY_AltCtrl.c
        DY_AttCtrl.c          // 角度控制
        DY_FlightCtrl.c
        DY_FlightDataCal.c    // 读取加速度计、陀螺仪的数据
        DY_LocCtrl.c
        DY_MagProcess.c
        DY_MotorCtrl.c        // 电机控制
```

## FlyControl_Algorithm

```C
FlyControl_Algorithm
├─Inc  
└─Src
        DY_FcData.c
        DY_Filter.c
        DY_Imu.c
        DY_Math.c
        DY_MotionCal.c
        DY_Navigate.c
        DY_Pid.c          // PID计算（控制器）
```

## Drive

```C
Drive
├─Inc     
├─Src
│      Drv_adc.c
│      Drv_ak8975.c       // 电子罗盘驱动
│      Drv_icm20602.c     // 姿态传感器的驱动
│      Drv_led.c
│      Drv_pwm_in.c       // 遥控器输入信号接收
│      Drv_pwm_out.c
│      Drv_soft_i2c.c
│      Drv_spi.c
│      Drv_spl06.c
│      Drv_time.c
│      Drv_usart.c
│      Drv_vl53l0x.c      // TOF激光测距模块驱动
│      Drv_w25qxx.c
│      uartstdio.c
│      
└─Vl53l0x                 // TOF激光测距模块库函数
    ├─Inc     
    └─Src
            vl53l0x_api.c
            vl53l0x_api_calibration.c
            vl53l0x_api_core.c
            vl53l0x_api_ranging.c
            vl53l0x_api_strings.c
            vl53l0x_i2c.c
            vl53l0x_platform.c
```



## 代码整体逻辑展示 

如图，摘自网络大佬博客，更详细、高清的框图请见大佬公开 [思维导图]: https://www.processon.com/view/link/5d374332e4b0b3e4dcd01d3a

![20190728002712888](figure\20190728002712888.png)



# 模块学习

## 1. ICM20602

姿态传感器，其中有1个3轴陀螺仪和1个3轴加速度计。使用I2C或SPI通讯，工程中使用的是SPI通讯，其接线图如下：

<img src="figure\2021-05-11-17-33-58.jpg" alt="2021-05-11-17-33-58" style="zoom:33%;" />

### API

源代码中提供的api文件名为`Drv_icm20602.c`，其中提供的函数如下：

```C
void Drv_Icm20602CSPin_Init(void); // 初始化 Icm20602 的CS引脚，输出1
u8   Drv_Icm20602Reg_Init(void);   //  初始化icm进入可用状态。
void Drv_Icm20602_Read(void);      // 把数据读入mpu_buffer数组中
void Sensor_Data_Prepare(u8 dT_ms);
void Center_Pos_Set(void);
```

## 2. 遥控器控制

此飞控的遥控器接收器只能使用PWM模式，**6通道信号**；数据通过`Drv_pwm_in.c`文件，接收到`Rc_Pwm_In`数组中，然后在`DY_RC.c`的`RC_duty_task`函数中变为+-500摇杆量存储在`CH_N`数组中。

## 3. 飞行状态控制 Flight_State_Task

<img src="figure\2021-07-18-12-07-11.jpg" alt="2021-07-18-12-07-11" style="zoom: 50%;" />

此函数接收摇杆量`CH_N`，转换为速度量`fs.speed_set_h`。作为后面环的控制。但如果启用OpenMV控制，则变为：

<img src="figure\2021-07-18-12-11-13.jpg" alt="2021-07-18-12-11-13" style="zoom:50%;" />

OpenMV模式下，`fs.speed_set_h[Z]`将在`DY_AltCtrl.c`中被赋值。

所以说，这个函数的主要功能就是把摇杆量转换为后续控制环所需设定值，这里所有的设定值皆为速度量。此外函数中还有对标志位的检测

## 4. PID模块

此程序中所有的PID计算都由`DY_Pid.c` 中的 `PID_calculate` 函数完成，其参数列表如下：

```C
float PID_calculate(float dT_s,            //周期（单位：秒）
					float in_ff,		   //前馈值
					float expect,		   //期望值（设定值）
					float feedback,		   //反馈值（）
					_PID_arg_st *pid_arg,  //PID参数结构体
					_PID_val_st *pid_val,  //PID数据结构体
					float inte_d_lim,      //积分误差限幅
					float inte_lim		   //integration limit，积分限幅)
```

其中`pid_arg`中储存的是如 P,I,D之类的参数；`pid_val`中储存的是如上次的误差，上次的反馈值等在位置式PID中需要用到的储存量。也就是说，给定特定的 arg和val就可组成特定的PID控制器。

其可以实现的是一个反馈-前馈的控制，结构上采用了微分先行的方式，实现上使用的是位置式PID。

### 位置式PID

PID的连续型公式为：
$$
u(t) = K_p [e(t) + \frac{1}{T_i} \int_{t}^{0}e(t)dt + T_d \frac{de(t)}{dt}]
$$
直接对其离散化，即可得到**位置式PID**：
$$
u(k) = k_p e(k) + K_i \sum_{i=0}e(i) + K_D[e(k) - e(k-1)]
$$
这里有对误差的求和项$\sum_{i=0} e(i)$，此项容易造成存储空间的占据，于是**增量式PID**诞生：
$$
\begin{align*}
\Delta u(k) &= u(k) - u(k-1)\\
            &= K_p[e(k) - e(k-1)] + K_I e(k) +K_D[e(k) - 2e(k-1) + e(k-2)]
\end{align*}
$$

### 微分先行

所谓微分先行，就是对反馈量直接微分，作为控制器输出的一部分：

<img src="figure\微分先行示意图.png" alt="微分先行示意图" style="zoom:75%;" />

结合微分先行和位置式PID，代码中展现如下：

```C
pid_val->out = pid_arg->k_ff *in_ff      // 前馈系数 * 前馈
	         + pid_arg->kp *pid_val->err // Kp      * 误差 
			 + differential              // 微分先行项
    	     + pid_val->err_i;		     //误差积分项（已乘Ki）
```

PS：

按理说，`differential` 应当为 $K_d × \frac{d(feedback)}{dt}$，但在代码中却展示为：$K_{d(sp)} \frac{d(setpoint)}{dt}-K_{d(fb)}  \frac{d(feedback)}{dt}$.不知所谓：

```c
//如何正确理解微分定义？  dx/dt = lim  [x(t+dt) - x(t)]/dt = [x(t+dt) - x(t)]/T
//期望值的微分 = （期望 - 上次期望） * 频率
pid_val->exp_d = (expect - pid_val->exp_old) *hz;

//反馈值的微分 = （反馈 - 上次反馈） * 频率		 
pid_val->fb_d = (feedback - pid_val->feedback_old) *hz;
// 微分先行算法
//微分 = 期望微分系数 * 期望微分值 - 反馈微分系数 * 反馈微分值	
differential = (pid_arg->kd_ex *pid_val->exp_d - pid_arg->kd_fb *pid_val->fb_d);
```
$$
\frac{dx}{dt} = \lim\limits_{t\to 0} \frac{x(t+dt)-x(t)}{dt} = \frac{x(t_2)-x(t_1)}{T} = [x(t_2)-x(t_1)]f
$$



## 5. 位置控制

位置控制环函数为`DY_LocCtrl.c`，主要控制函数如下：

```C
void Loc_1level_Ctrl(u16 dT_ms,s16 *CH_N)
```

其中参数`CH_N`，并未被用到，输入输出结果如下：

<img src="figure\2021-07-18-11-45-07.jpg" alt="2021-07-18-11-45-07" style="zoom: 67%;" />

这里的反馈值是光流传输回来的，暂且不清楚是什么。其会输出一个数组，这个数组将作为角度环的设定值。

## 6. 角度控制模块 

角度控制是通过串级控制实现的，实现的文件为`DY_AttCtrl.c`，主要函数为：`Att_2level_Ctrl`角度控制器和`Att_1level_Ctrl`角速度控制器，方框图如下：

![image-20210712211933002](figure\image-20210712211933002.png)

联系函数内容可以做出下图：

![image-20210712213041089](figure\2021-07-18-09-12-17.jpg)

设定值`CH_N[YAM]`来自遥控器的输入，是偏航角的设定值，但其余两个设定值暂不清楚。

## 7.  高度环控制

高度环控制由`DY_AltCtrl.c`中的，`Alt_2level_Ctrl`和`Alt_1level_Ctrl`两个函数完成。也是串级控制，内环为速度环，外环为高度环。

<img src="D:\IAR\savelocation\DeYan_UAV\figure\2021-07-22-14-39-05.jpg" alt="2021-07-22-14-39-05" style="zoom: 33%;" />

但是关于设定值`loc_ctrl_2.exp[Z]`却十分奇怪，在`flag.taking_off != 1`时，他被赋值为反馈值`loc_ctrl_2.fb[Z]`；

如果`flag.taking_off = 1`，且`flag.ct_alt_hold != 1`，那么`loc_ctrl_2.exp[Z] = loc_ctrl_2.fb[Z] + alt_val_2.err`。但此时因为没有进入PID环节，`alt_val_2.err`就是为0，所以设定值仍然等于反馈值。

<img src="figure\2021-07-22-15-57-56.jpg" alt="2021-07-22-15-57-56" style="zoom:50%;" />

这里表达的意思是，若飞机未进入定高悬停状态`flag.ct_alt_hold=0`，则高度环的主控制器不起作用，高度环控制器输出为0。当进入定高悬停状态时，则`loc_ctrl_2.exp[Z]`等于当前的高度值，高度环控制器开始工作，使得高度得到控制。

这里，进入定高的条件是高度速度环的反馈值和期望值接近。也就是说，飞机在最开始起飞时，是没有高度控制器的，只有速度控制器；当速度接近设定值时，高度控制器才投入运行，保持当前的飞行高度。

因此，想要无人机定高飞行，只需要修改`loc_ctrl_2.exp[Z]`即可达到目的。

### 高度速度环控制

其实本身没有什么好说的，但其设定值还是有点东西。下面这个框图可以说明。



## 8. 电机控制模块

### 初始化与判断

如果`flag.fly_ready==1`，那么飞控就会依次使4个电机达到怠速状态，然后置位`flag.motor_preparation`，表示电机已经准备完成。

如果`flag.fly_ready==0`，那么`flag.motor_preparation`只会为0，且`motor_step`一直为0，这样，电机就会停止转动。所以通过改变`flag.fly_ready`就可以控制电机立即刹车。

### 核心部分——控制分配

电机控制模块为：`DY_MotorCtrl.c`文件，其核心代码如下：

```C
if(flag.motor_preparation == 1)
{		
    motor_step[m1] = mc.ct_val_thr  +mc.ct_val_yaw -mc.ct_val_rol +mc.ct_val_pit;
    motor_step[m2] = mc.ct_val_thr  -mc.ct_val_yaw +mc.ct_val_rol +mc.ct_val_pit;
    motor_step[m3] = mc.ct_val_thr  +mc.ct_val_yaw +mc.ct_val_rol -mc.ct_val_pit;
    motor_step[m4] = mc.ct_val_thr  -mc.ct_val_yaw -mc.ct_val_rol -mc.ct_val_pit;
}
```

`motor_step`数组为4个电机转速对应的值，而`mc.ct_val_thr`和等式右的值来自高度环和角度环的输出，这些输出需要按一定的原则分配到4个电机上，才能实现控制任务，这里就是在完成控制任务的分配。

在完成控制任务分配后，对分配值进行限幅操作，赋值给数组`motor`，最后使用函数`SetPwm`把数值转化为占空比，输出到4个电机。

# 功能实现

## 一键起飞与降落

首先，原版的一键起飞和降落是通过摇杆的旋钮实现的，旋钮通道CH5和CH6的值就会改变。数据通过`Drv_pwm_in.c`文件，接收到`Rc_Pwm_In`数组中，然后在`DY_RC.c`的`RC_duty_task`函数中变为+-500摇杆量存储在`CH_N`数组中。

在文件`DY_FlightCtrl.c`的函数`Flight_Mode_Set`中，会通过判断`CH_N[AUX1]`来判断是否要一键降落，通过判断`CH_N[AUX2]`来判断是否要一键起飞。

```c
	if(CH_N[AUX2]<-200)
	{
	  // 若启用一键起飞后，OpemMV控制高度模式没有启动，则启动；
	  // 同时启用one_key_take_off，置位标志位one_key_taof_start和flag.fly_ready
      if(DY_Debug_Height_Mode==0)
      {
        DY_Debug_Height_Mode = 1;
        one_key_take_off();
        dy_height = 30;
      }
      // 若启用一键起飞后，OpemMV控制高度模式已经启动
      else
      {
		// 如果当前高度高于1.2m且没有开始计数，就使得高度设定值为0，开始计数
        if(tof_height_mm>=1200 && DY_CountTime_Flag==0)
        {
          dy_height = 0;
          DY_CountTime_Flag = 1;
        }
        if(DY_CountTime_Flag)
        {
          DY_Task_ExeTime++;
		  // 如果计数15s后，就启用一键降落(DY_Land_Flag为防止one_key_land被执行多次)
          if(DY_Task_ExeTime>=1500 && DY_Land_Flag==0) // 10ms*1500 = 15s
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
```

<img src="figure\2021-07-19-22-06-16.jpg" alt="2021-07-19-22-06-16" style="zoom:50%;" />

因为我们的遥控器没有CH5和CH6通道，所以我们必须手动进入这个判断。我们的期望是，当飞机启动一段时间后，在20s内完成一键起飞和降落的功能。

使用的起飞函数如下：

```C
void our_take_off()
{   
    if(flag.auto_take_off_land != AUTO_TAKE_OFF_FINISH && our_delay_times[0] <200)
    {
        // DY_Debug_Height_Mode = 1;
        one_key_take_off();
        dy_height = 30;
    }
}
```

`our_delay_times[0]`是一个计时器

# 重要参数与标志位

## 全局变量

C语言中，统一文件夹下的全局变量可以在不同的C文件中调用。

1. `CH_N`

初次定义于`DY_RC.c`中，为遥控器的遥感量，在`RC_duty_task`函数中被赋值。

被作为参数传入函数`Flight_State_Task(u8 dT_ms,s16 *CH_N)`中，或许作为设定值？暂时不太清楚。

## 标志位

```c
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
      u8 fly_ready;  //unlocked 准备起飞，非常重要
      u8 thr_low;
      u8 locking;
      u8 taking_off; //起飞
      u8 set_yaw;
      u8 ct_loc_hold;
      u8 ct_alt_hold;


      //飞行状态
      u8 flying;             // 正在飞行
      u8 auto_take_off_land;
      u8 home_location_ok;	
      u8 speed_mode;
      u8 thr_mode;	
      u8 flight_mode;
      u8 gps_mode_en;
      u8 motor_preparation;
      u8 locked_rotor;
}_flag;
```

`fly_ready=1`表示已经准备好起飞，此时`flag.taking_off`才能被置位。此外更重要的是：只有当`fly_ready==1`时，标志位`motor_preparation`才可能置位，才会开始对电机的控制，否则电机的输入PWM占空比为0，电机停转。下表为`fly_ready`赋值处及其意义。

| 赋值 |                             位置                             |                             意义                             |
| :--: | :----------------------------------------------------------: | :----------------------------------------------------------: |
|  0   |        文件`DY_FlightCtrl.c`，函数`land_discriminat`         | 油门最终输出量小于250并且没有在手动解锁上锁过程中，持续1.5秒，认为着陆，然后上锁 |
|  0   |        文件`DY_FlightCtrl.c`，函数`Flight_State_Task`        |                    机体倾角过大，需要停机                    |
|  1   |        文件`DY_FlightCtrl.c`，函数`one_key_take_off`         |                           一键起飞                           |
| 0/1  | 文件`DY_RC.c`，函数`unlock`,函数`stick_function_check_longpress` |    摇杆满足条件unlock_time时间后，才会执行锁定和解锁动作     |
|  1   |文件`DY_RC.c`，函数`unlock`|如果`flag.fly_ready == 2`(但好像不太可能？)|

只有`flag.taking_off`被置位时，才会设置垂直方向速度，否则就设置垂直方向速度为0；

当`flag.taking_off=1`被维持1s后，`flag.flying`被置位，表示飞机已经起飞。




# 学习日志

7.6 今日在看`DY_FlightCtrl.c`中的`Flight_State_Task`函数，明天需要看`DY_RC.c`来进一步了解。

--------

7.7 今天搞明白了遥控器是如何控制的，PID的如何运算的，其输入输出和配置如何，下一步就是要开始看每一个具体的环路控制了。

---

7.12 今天搞明白了飞行器角度的控制方法`DY_AttCtrl.c`，但角度控制中的设定值来源`loc_ctrl_1.out[Y]`，变量暂不清楚是何作用，其来自于`DY_LocCtrl.c`位置环控制，此外电机控制`DY_MotorCtrl.c`也初露端倪，下一步就是搞懂这两块的代码。

---

7.18 今日任务，读懂`DY_LocCtrl.c`，`DY_MotorCtrl.c`；搞明白光流，高度环和位置环，另外知道设定值到底是哪个变量，进行飞行实验。

今日完成`DY_MotorCtrl.c`，想要加入一个远程zigbee紧急停车模块，下一步要测试`Uart4_Init`等的波特率，看波特率到底如何设置，如何通信和如何解析接收数据。

---

7.19 1：完成紧急刹车的布置，当按下stm32板子上的按钮（现在为重启）后，无人机的电机会自动停止旋转。

实现原理是使得`fly_ready=0`，无人机上的zigbee和串口4连接，接收并解析stm32上zigbee传来的信息，当数据的数据包部分第一个数据为`0x66`，直接使得`fly_ready=0`，完成刹车。

2:想要完成一键起飞，但失败了，不知道是什么原因，下一步查看MV和控制器的互动。

---

7.20 1：完成了一键起飞的任务，并且添加了zigbee回传标志位的功能。新建飞行日志文件夹，存放每次飞行时的记录。

2：phs的高度飞行设计完成，添加了飞行高度`tof_height_mm`的回传，添加了飞行日志，随着数据可以回传，我们的进度有所加快。下一步预计进行前后作用的控制。

---

7.22：今天完成了定高任务，完成区域定高，取消遥控器控制。下一步测试X、Y方向的运动，回传高度融合数据。
