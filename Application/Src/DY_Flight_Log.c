#include "DY_Flight_Log.h"
#include "DY_Tracking.h"

#include "DY_AltCtrl.h"
#include "DY_AttCtrl.h"
#include "DY_FlightCtrl.h"
#include "DY_Math.h"
#include "Drv_time.h"
#include "Drv_usart.h"

_DY_flag dy_flag;
u8 DY_ControlConvert_Tracking_Flag=1;
void DY_Flight_Control(void)
{
    /***************飞行控制***************/
	if(dy_flag.pit_go == 1)
	{
		dy_flag.pit_go = 0;
        
        dy_pit = dy_ov_pit;
	}
	if(dy_flag.pit_back == 1)
	{
		dy_flag.pit_back = 0;
		
        dy_pit = -(dy_ov_pit);
	}
	if(dy_flag.rol_left == 1)
	{
		dy_flag.rol_left = 0;
		
        dy_rol = dy_ov_rol;
	}
	if(dy_flag.rol_right == 1)
	{
		dy_flag.rol_right = 0;
		
        dy_rol = -(dy_ov_rol);
	}
	if(dy_flag.yaw_ccw == 1)
	{
		dy_flag.yaw_ccw = 0;
		
        dy_yaw = dy_ov_yaw;
	}
	if(dy_flag.yaw_cw == 1)
	{
		dy_flag.yaw_cw = 0;
		
        dy_yaw = -(dy_ov_yaw);
	}
	if(dy_flag.up == 1)
	{
		dy_flag.up = 0;
		
		// dy_height = dy_ov_height;
	}
	if(dy_flag.down == 1)
	{
		dy_flag.down = 0;
		
		// dy_height = -(dy_ov_height);
	}
	if(dy_flag.stop == 1)
	{
		dy_flag.stop = 0;
		
		// dy_height = 0;
		dy_pit = 0;
		dy_rol = 0;
		dy_yaw = 0.0f;
	}
	if(dy_flag.land == 1)
	{
		dy_flag.land = 0;
		
		// dy_height = 0;
		dy_pit = 0;
		dy_rol = 0;
		dy_yaw = 0.0f;
		one_key_land();
	}
}
