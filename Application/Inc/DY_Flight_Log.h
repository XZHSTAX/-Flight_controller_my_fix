#ifndef _DY_FLIGHT_LOG_
#define _DY_FLIGHT_LOG_

#include "common.h"

typedef struct{
	u8 up;
	u8 down;
	u8 pit_go;
	u8 pit_back;
	u8 rol_left;
	u8 rol_right;
	u8 yaw_cw;
	u8 yaw_ccw;
	u8 stop;
    u8 land;
}_DY_flag;
extern _DY_flag dy_flag;

void DY_Flight_Control(void);

#endif
