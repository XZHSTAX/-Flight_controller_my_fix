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
        DY_DT.c
        DY_Flight_Log.c
        DY_OF.c
        DY_Parameter.c
        DY_power.c
        DY_RC.c          // 遥控器通道数据处理
        DY_Scheduler.c   // 任务调度（所有的任务及其调度方式）
        DY_Tracking.c
        main.c
        OpticalFlow.c
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
│      Drv_vl53l0x.c
│      Drv_w25qxx.c
│      uartstdio.c
│      
└─Vl53l0x
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

## ICM20602

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

## 遥控器控制

此飞控的遥控器接收器只能使用PWM模式，**6通道信号**；数据通过`Drv_pwm_in.c`文件，接收到`Rc_Pwm_In`数组中，然后在`DY_RC.c`的`RC_duty_task`函数中变为+-500摇杆量存储在`CH_N`数组中。

## PID模块

此程序中所有的PID计算都由`DY_Pid.c` 中的 `PID_calculate` 函数完成，其参数列表如下：

```C
float PID_calculate(float dT_s,            //周期（单位：秒）
					float in_ff,		   //前馈值
					float expect,		   //期望值（设定值）
					float feedback,		   //反馈值（）
					_PID_arg_st *pid_arg,  //PID参数结构体
					_PID_val_st *pid_val,  //PID数据结构体
					float inte_d_lim,      //积分误差限幅
					float inte_lim		   //integration limit，积分限幅								)
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

## 角度控制模块

角度控制是通过串级控制实现的，实现的文件为`DY_AttCtrl.c`，主要函数为：`Att_2level_Ctrl`角度控制器和`Att_1level_Ctrl`角速度控制器，方框图如下：

![image-20210712211933002](C:\Users\谢祖浩\AppData\Roaming\Typora\typora-user-images\image-20210712211933002.png)

联系函数内容可以做出下图：

![image-20210712213041089](C:\Users\谢祖浩\AppData\Roaming\Typora\typora-user-images\image-20210712213041089.png)

设定值`CH_N[YAM]`来自遥控器的输入，是偏航角的设定值，但其余两个设定值暂不清楚。


# 重要参数与标志位

## 全局变量

C语言中，统一文件夹下的全局变量可以在不同的C文件中调用。

1. `CH_N`

初次定义于`DY_RC.c`中，为遥控器的遥感量，在`RC_duty_task`函数中被赋值。

被作为参数传入函数`Flight_State_Task(u8 dT_ms,s16 *CH_N)`中，或许作为设定值？暂时不太清楚。



# 学习日志

7.6 今日在看`DY_FlightCtrl.c`中的`Flight_State_Task`函数，明天需要看`DY_RC.c`来进一步了解。

--------

7.7 今天搞明白了遥控器是如何控制的，PID的如何运算的，其输入输出和配置如何，下一步就是要开始看每一个具体的环路控制了。

---

7.12 今天搞明白了飞行器角度的控制方法`DY_AttCtrl.c`，但角度控制中的设定值来源`loc_ctrl_1.out[Y]`，变量暂不清楚是何作用，其来自于`DY_LocCtrl.c`位置环控制，此外电机控制`DY_MotorCtrl.c`也初露端倪，下一步就是搞懂这两块的代码。





