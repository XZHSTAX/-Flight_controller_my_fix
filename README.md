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
        DY_RC.c
        DY_Scheduler.c   // 任务调度（所有的任务及其调度方式）
        DY_Tracking.c
        main.c
        OpticalFlow.c
```

此文件夹存放了`main.c`文件，整个习题的启动、初始化、电源管理都在此处。`Inc`中存放的是对应的`.h`文件，这里不再打印。

## FlyControl_Calculate

```C
Application
├─Inc    
└─Src
        DY_AltCtrl.c
        DY_AttCtrl.c
        DY_FlightCtrl.c
        DY_FlightDataCal.c    // 读取加速度计、陀螺仪的数据
        DY_LocCtrl.c
        DY_MagProcess.c
        DY_MotorCtrl.c
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
│      Drv_pwm_in.c
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

# 学习日志

7.6 今日在看`DY_FlightCtrl.c`中的`Flight_State_Task`函数，明天需要看`DY_RC.c`来进一步了解。
