#include "DY_Tracking.h"
#include "Drv_usart.h"

u8 DY_Direction;
s16 DY_Speed_S,dy_ov_pit,dy_ov_rol,dy_ov_height;
float DY_Speed_F,dy_ov_yaw;

static u8 OV_RxBuffer[256],OV_data_cnt = 0;

void DY_Tracking_Data_Receive_Prepare(u8 data)
{
	static u8 _data_len = 0;
	static u8 state = 0;
	
	if(state==0&&data==0xAA)	//帧头0xAA
	{
		state=1;
		OV_RxBuffer[0]=data;
	}
	else if(state==1)		//功能字
	{
		state = 2;
		OV_RxBuffer[1]=data;
	}
	else if(state==2)			//数据长度
	{
		state = 3;
		OV_RxBuffer[2]=data;
		_data_len = data;
		OV_data_cnt = 0;
	}
	else if(state==3&&_data_len>0)
	{
		_data_len--;
		OV_RxBuffer[3+OV_data_cnt++]=data;
		if(_data_len==0)
			state = 4;
	}
	else if(state==4)
	{
		state = 0;
		OV_RxBuffer[3+OV_data_cnt]=data;
		DY_Tracking_DataAnl(OV_RxBuffer,OV_data_cnt+4);
	}
	else
		state = 0;
}

void DY_Tracking_DataAnl(uint8_t *data_buf,uint8_t num)
{	

	u8 sum = 0;
	for(u8 i=0;i<(num-1);i++)
		sum += *(data_buf+i);
	if(!(sum==*(data_buf+num-1)))		return;		//校验
	
	DY_Direction = *(data_buf+3);
	
	if(*(data_buf+1)==0X89)
	{
		DY_Speed_S = (int16_t)(*(data_buf+5)<<8)|*(data_buf+4);
	}
	if(*(data_buf+1)==0X98)
	{
		DY_Speed_F = ((int16_t)(*(data_buf+5)<<8)|*(data_buf+4)) * 0.01;
	}
	
	if((DY_Direction == 'C') || (DY_Direction == 'F'))
	{
		if(DY_Direction == 'C')
			dy_flag.yaw_cw = 1;
		else if(DY_Direction == 'F')
			dy_flag.yaw_ccw = 1;
		dy_ov_yaw = DY_Speed_F;
	}
	else if((DY_Direction == 'G') || (DY_Direction == 'B'))
	{
		if(DY_Direction == 'G')
			dy_flag.pit_go = 1;
		else if(DY_Direction == 'B')
			dy_flag.pit_back = 1;
		dy_ov_pit = DY_Speed_S;
	}
	else if((DY_Direction == 'L') || (DY_Direction == 'R'))
	{
		if(DY_Direction == 'L')
			dy_flag.rol_left = 1;
		else if(DY_Direction == 'R')
			dy_flag.rol_right = 1;
		dy_ov_rol = DY_Speed_S;
	}
	else if((DY_Direction == 'U') || (DY_Direction == 'D'))
	{
		if(DY_Direction == 'U')
			dy_flag.up = 1;
		else if(DY_Direction == 'D')
			dy_flag.down = 1;
		dy_ov_height = DY_Speed_S;
	}
	else if(DY_Direction == 'S')
		dy_flag.stop = 1;
    else if(DY_Direction == 'E')
		dy_flag.land = 1;
	
}
