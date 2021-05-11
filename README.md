# 模块学习

## ICM20602

姿态传感器，其中有1个3轴陀螺仪和1个3轴加速度计。使用I2C或SPI通讯，工程中使用的是SPI通讯，其接线图如下：

<img src="D:\IAR\savelocation\DeYan_UAV\figure\2021-05-11-17-33-58.jpg" alt="2021-05-11-17-33-58" style="zoom:33%;" />

### API

源代码中提供的api文件名为`Drv_icm20602.c`，其中提供的函数如下：

```C
void Drv_Icm20602CSPin_Init(void); // 初始化 Icm20602 的CS引脚，输出1
u8   Drv_Icm20602Reg_Init(void);   //  初始化icm进入可用状态。
void Drv_Icm20602_Read(void);      // 把数据读入mpu_buffer数组中
void Sensor_Data_Prepare(u8 dT_ms);
void Center_Pos_Set(void);
```

