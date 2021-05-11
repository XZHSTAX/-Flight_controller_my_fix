/******************** (C) COPYRIGHT 2018 DY EleTe ********************************
 * 作者    ：徳研电科
 * 官网    ：www.gototi.com
 * 描述    ：ICM20602驱动
**********************************************************************************/
#include "Drv_icm20602.h"
#include "DY_Filter.h"
#include "DY_Math.h"
#include "Drv_spi.h"
#include "Drv_led.h"

// 初始化 Icm20602 的CS引脚，输出1，即不选中
void Drv_Icm20602CSPin_Init(void)
{
    MAP_SysCtlPeripheralEnable(ICM20602_CS_SYSCTL);          // 开启传感器CS引脚对应时钟 CS:PN2
    while(!(MAP_SysCtlPeripheralReady(ICM20602_CS_SYSCTL)))
    {
    }
    
    GPIOPinTypeGPIOOutput(ICM20602_CS_GPIO, ICM20602_CS_PIN);// 设置PN2模式为输出
    
    GPIOPinWrite(ICM20602_CS_GPIO, ICM20602_CS_PIN, ICM20602_CS_PIN); // 设置设置PN2输出1
}

// 片选，ena = 1选中芯片 
static void icm20602_enable(u8 ena)
{
    if(ena)
	GPIOPinWrite(ICM20602_CS_GPIO, ICM20602_CS_PIN, ~ICM20602_CS_PIN);
    else
	GPIOPinWrite(ICM20602_CS_GPIO, ICM20602_CS_PIN, ICM20602_CS_PIN);
}

// 无论读还是写，开始前要把CS拉低，完成后把电平拉高
static void icm20602_readbuf(u8 reg, u8 length, u8 *data)
{
    icm20602_enable(1);    // 选中芯片
    Drv_SPI2_RW(reg|0x80);
    Drv_SPI2_Receive(data,length);
    icm20602_enable(0);
}

static u8 icm20602_writebyte(u8 reg, u8 data)
{
    u8 status;

    icm20602_enable(1);       // 选中芯片
    status = Drv_SPI2_RW(reg);
    Drv_SPI2_RW(data);
    icm20602_enable(0);
    
    return status;
}

/**************************实现函数********************************************
*功　　能:	  读 修改 写 指定设备 指定寄存器一个字节 中的1个位
reg	   寄存器地址
bitNum  要修改目标字节的bitNum位
data  为0 时，目标位将被清0 否则将被置位
*******************************************************************************/
static void icm20602_writeBit(u8 reg, u8 bitNum, u8 data) 
{
    u8 b;
    icm20602_readbuf(reg, 1, &b);
    b = (data != 0) ? (b | (1 << bitNum)) : (b & ~(1 << bitNum));
	icm20602_writebyte(reg, b);
}

/**************************实现函数********************************************
*功　　能:	    初始化icm进入可用状态。
*******************************************************************************/
u8 Drv_Icm20602Reg_Init(void)
{
    u8 tmp;

    icm20602_writebyte(MPU_RA_PWR_MGMT_1,0x80);
    MAP_SysCtlDelay(400000);
    icm20602_writebyte(MPU_RA_PWR_MGMT_1,0x01);
    MAP_SysCtlDelay(400000);

    icm20602_readbuf(MPUREG_WHOAMI, 1, &tmp);
    
    if(tmp != MPU_WHOAMI_20602) 
    return 0;

    /*复位reg*/
    icm20602_writebyte(MPU_RA_SIGNAL_PATH_RESET,0x03);
    MAP_SysCtlDelay(400000);
    /*复位reg*/
    icm20602_writebyte(MPU_RA_USER_CTRL,0x01);
    MAP_SysCtlDelay(400000);

    icm20602_writebyte(0x70,0x40);//dmp
    MAP_SysCtlDelay(400000);
    icm20602_writebyte(MPU_RA_PWR_MGMT_2,0x00);
    MAP_SysCtlDelay(400000);
    icm20602_writebyte(MPU_RA_SMPLRT_DIV,0);
    MAP_SysCtlDelay(400000);
    icm20602_writebyte(MPU_RA_CONFIG,ICM20602_LPF_20HZ);
    MAP_SysCtlDelay(400000);
    icm20602_writebyte(MPU_RA_GYRO_CONFIG,(3 << 3));
    MAP_SysCtlDelay(400000);
    icm20602_writebyte(MPU_RA_ACCEL_CONFIG,(2 << 3));
    MAP_SysCtlDelay(400000);
    /*加速度计LPF 20HZ*/
    icm20602_writebyte(0X1D,0x04);
    MAP_SysCtlDelay(400000);
    /*关闭低功耗*/
    icm20602_writebyte(0X1E,0x00);
    MAP_SysCtlDelay(400000);
    /*关闭FIFO*/
    icm20602_writebyte(0X23,0x00);
    MAP_SysCtlDelay(400000);

    /*设置重心相对传感器的偏移量*/
    Center_Pos_Set();

    sensor.acc_z_auto_CALIBRATE = 1;    //开机自动校准Z轴
    sensor.gyr_CALIBRATE = 2;           //开机自动校准陀螺仪
    return 1;
}

u8 mpu_buffer[14];		                //ICM20602原始数据
_center_pos_st center_pos;
_sensor_st sensor;

// 把数据读入mpu_buffer数组中
void Drv_Icm20602_Read()
{
	icm20602_readbuf(MPUREG_ACCEL_XOUT_H,14,mpu_buffer);
}

s32 sensor_val[6];
s32 sensor_val_rot[6];
s32 sensor_val_ref[6];

s32 sum_temp[7]={0,0,0,0,0,0,0};
s32 acc_auto_sum_temp[3];
s16 acc_z_auto[4];

u16 acc_sum_cnt = 0,gyro_sum_cnt = 0,acc_z_auto_cnt;

s16 g_old[VEC_XYZ];
float g_d_sum[VEC_XYZ] = {500,500,500};

void mpu_auto_az()
{
	u8 i = 0;
	if(sensor.acc_z_auto_CALIBRATE)
	{
		acc_z_auto_cnt++;

		acc_auto_sum_temp[0] += sensor_val_ref[A_X];
		acc_auto_sum_temp[1] += sensor_val_ref[A_Y];
		acc_auto_sum_temp[2] += sensor_val_rot[A_Z];

		if(acc_z_auto_cnt>=OFFSET_AV_NUM)
		{
			sensor.acc_z_auto_CALIBRATE = 0;
			acc_z_auto_cnt = 0;

			for(i = 0;i < 3; i++)
			{
				acc_z_auto[i] = acc_auto_sum_temp[i]/OFFSET_AV_NUM;

				acc_auto_sum_temp[i] = 0;
			}
			acc_z_auto[3] = my_sqrt( 4096*4096 - (my_pow(acc_z_auto[0]) + my_pow(acc_z_auto[1])) );
            save.acc_offset[Z] = acc_z_auto[2] - acc_z_auto[3];
		}
	}
}

void motionless_check(u8 dT_ms)
{
	u8 t = 0, i = 0;

	for(i = 0;i<3;i++)
	{
		g_d_sum[i] += 3*ABS(sensor.Gyro_Original[i] - g_old[i]) ;

		g_d_sum[i] -= dT_ms ;

		g_d_sum[i] = LIMIT(g_d_sum[i],0,200);

		if( g_d_sum[i] > 10)
		{
			t++;
		}

		g_old[i] = sensor.Gyro_Original[i];
	}

	if(t>=2)
	{
		flag.motionless = 0;
	}
	else
	{
		flag.motionless = 1;
	}
}

void MPU6050_Data_Offset()
{
	static u8 off_cnt;
	u8 i = 0;
	u8 j = 0;
	if(sensor.gyr_CALIBRATE || sensor.acc_CALIBRATE || sensor.acc_z_auto_CALIBRATE)
	{
        ///////////////////复位校准值///////////////////////////
		if(flag.motionless == 0 || sensor_val[A_Z]<1000)
		{
				gyro_sum_cnt = 0;
				acc_sum_cnt=0;
				acc_z_auto_cnt = 0;

				for(j = 0; j < 3; j ++)
				{
					acc_auto_sum_temp[j] = sum_temp[G_X+j] = sum_temp[A_X+j] = 0;
				}
				sum_temp[TEM] = 0;
		}
        ///////////////////////////////////////////////////////////
		off_cnt++;
		if(off_cnt>=10)
		{
			off_cnt=0;
            
			if(sensor.gyr_CALIBRATE)
			{
				LED_state = 2;
				gyro_sum_cnt++;

				for(i = 0;i<3;i++)
				{
					sum_temp[G_X+i] += sensor.Gyro_Original[i];
				}
				if( gyro_sum_cnt >= OFFSET_AV_NUM )
				{
					for(i = 0;i<3;i++)
					{
						save.gyro_offset[i] = (float)sum_temp[G_X+i]/OFFSET_AV_NUM;

						sum_temp[G_X + i] = 0;
					}
					gyro_sum_cnt =0;
					if(sensor.gyr_CALIBRATE == 1)
					{
						if(sensor.acc_CALIBRATE == 0)
						{
							data_save();
						}
					}
					sensor.gyr_CALIBRATE = 0;
					DY_DT_SendString("GYR init OK!",sizeof("GYR init OK!"));
				}
			}

			if(sensor.acc_CALIBRATE == 1)
			{
				LED_state = 3;
				acc_sum_cnt++;

				sum_temp[A_X] += sensor_val_rot[A_X];
				sum_temp[A_Y] += sensor_val_rot[A_Y];
				sum_temp[A_Z] += sensor_val_rot[A_Z] - G_1G;// - 65535/16;   // +-8G
				sum_temp[TEM] += sensor.Tempreature;

				if( acc_sum_cnt >= OFFSET_AV_NUM )
				{
					for(i = 0 ; i < 3; i++)
					{
						save.acc_offset[i] = sum_temp[A_X+i]/OFFSET_AV_NUM;

						sum_temp[A_X + i] = 0;
					}

					acc_sum_cnt =0;
					sensor.acc_CALIBRATE = 0;
					DY_DT_SendString("ACC init OK!",sizeof("ACC init OK!"));
					data_save();
				}
			}
		}
	}
}

s16 roll_gz_comp;
float wh_matrix[VEC_XYZ][VEC_XYZ] = 
{
	{1,0,0},
	{0,1,0},
	{0,0,1}

};

void Center_Pos_Set()
{
	center_pos.center_pos_cm[X] = DY_Parame.set.center_pos_cm[X];//+0.0f;
	center_pos.center_pos_cm[Y] = DY_Parame.set.center_pos_cm[Y];//-0.0f;
    center_pos.center_pos_cm[Z] = DY_Parame.set.center_pos_cm[Z];//+0.0f;
}

static float gyr_f1[VEC_XYZ],acc_f1[VEC_XYZ];

void Sensor_Data_Prepare(u8 dT_ms)
{
  	float hz = 0 ;
    
	if(dT_ms != 0) 
      hz = 1000/dT_ms;
    
	u8 i = 0;

	/*静止检测*/
	motionless_check(dT_ms);

	MPU6050_Data_Offset(); //校准函数

	/*读取buffer原始数据*/
	sensor.Acc_Original[X] = (s16)((((u16)mpu_buffer[0]) << 8) | mpu_buffer[1]);
	sensor.Acc_Original[Y] = (s16)((((u16)mpu_buffer[2]) << 8) | mpu_buffer[3]);
    sensor.Acc_Original[Z] = (s16)((((u16)mpu_buffer[4]) << 8) | mpu_buffer[5]);

	sensor.Gyro_Original[X] = (s16)((((u16)mpu_buffer[ 8]) << 8) | mpu_buffer[ 9]) ;
	sensor.Gyro_Original[Y] = (s16)((((u16)mpu_buffer[10]) << 8) | mpu_buffer[11]) ;
    sensor.Gyro_Original[Z] = (s16)((((u16)mpu_buffer[12]) << 8) | mpu_buffer[13]) ;

	sensor.Tempreature = ((((int16_t)mpu_buffer[6]) << 8) | mpu_buffer[7]);
	/*icm20602温度*/
	sensor.Tempreature_C = sensor.Tempreature/326.8f + 25 ;

	/*得出校准后的数据*/
	for(i = 0; i < 3; i++)
	{

		sensor_val[A_X+i] = sensor.Acc_Original[i] ;

		sensor_val[G_X+i] =(s32)(sensor.Gyro_Original[i] - save.gyro_offset[i]) ;
	}

	/*赋值*/
	for(i = 0;i<6;i++)
	{
		sensor_val_rot[i] = sensor_val[i];
	}

	/*数据坐标转90度*/
	sensor_val_ref[G_X] =  sensor_val_rot[G_Y] ;
	sensor_val_ref[G_Y] = -sensor_val_rot[G_X] ;
	sensor_val_ref[G_Z] =  sensor_val_rot[G_Z];


	sensor_val_ref[A_X] =  (sensor_val_rot[A_Y] - save.acc_offset[Y] ) ;
	sensor_val_ref[A_Y] = -(sensor_val_rot[A_X] - save.acc_offset[X] ) ;
    sensor_val_ref[A_Z] =  (sensor_val_rot[A_Z] - save.acc_offset[Z] ) ;

	/*单独校准z轴模长*/
	mpu_auto_az();

    //======================================================================
	/*软件滤波*/
	for(i=0;i<3;i++)
	{
		//0.24f，1ms ，50hz截止; 0.15f,1ms,28hz; 0.1f,1ms,18hz
		gyr_f1[X +i] += 0.12f *(sensor_val_ref[G_X + i] - sensor.Gyro[X +i]);
		acc_f1[X +i] += 0.12f *(sensor_val_ref[A_X + i] - sensor.Acc[X +i]);
	}

	/*旋转加速度补偿*/
    //======================================================================

	for(i=0;i<3;i++)
	{
		center_pos.gyro_rad_old[i] = center_pos.gyro_rad[i];
		center_pos.gyro_rad[i] =  gyr_f1[X + i] *RANGE_PN2000_TO_RAD;//0.001065f;
		center_pos.gyro_rad_acc[i] = hz *(center_pos.gyro_rad[i] - center_pos.gyro_rad_old[i]);
	}
    
    center_pos.linear_acc[X] = +center_pos.gyro_rad_acc[Z] *center_pos.center_pos_cm[Y] - center_pos.gyro_rad_acc[Y] *center_pos.center_pos_cm[Z];
	center_pos.linear_acc[Y] = -center_pos.gyro_rad_acc[Z] *center_pos.center_pos_cm[X] + center_pos.gyro_rad_acc[X] *center_pos.center_pos_cm[Z];
	center_pos.linear_acc[Z] = +center_pos.gyro_rad_acc[Y] *center_pos.center_pos_cm[X] - center_pos.gyro_rad_acc[X] *center_pos.center_pos_cm[Y];

    //======================================================================
	/*赋值*/
	for(i=0;i<3;i++)
	{
		sensor.Gyro[X+i] = gyr_f1[i];

		sensor.Acc[X+i] = acc_f1[i] - center_pos.linear_acc[i]/RANGE_PN8G_TO_CMSS;
	}

	/*转换单位*/
    for(i =0 ;i<3;i++)
    {
        /*陀螺仪转换到度每秒，量程+-2000度*/
        sensor.Gyro_deg[i] = sensor.Gyro[i] *0.06103f ;

        /*陀螺仪转换到弧度度每秒，量程+-2000度*/
        sensor.Gyro_rad[i] = sensor.Gyro[i] *RANGE_PN2000_TO_RAD ;

        /*加速度计转换到厘米每平方秒，量程+-8G*/
        sensor.Acc_cmss[i] = (sensor.Acc[i] *RANGE_PN8G_TO_CMSS);
    }
}

/******************* (C) COPYRIGHT 2018 DY EleTe *****END OF FILE************/
