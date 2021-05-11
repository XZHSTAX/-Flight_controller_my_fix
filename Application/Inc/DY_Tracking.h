#ifndef _DY_TRACKING_H_
#define _DY_TRACKING_H_

#include "common.h"
#include "DY_Flight_Log.h"

extern s16 dy_ov_pit,dy_ov_rol,dy_ov_height;
extern float dy_ov_yaw;

void DY_Tracking_Data_Receive_Prepare(u8 data);
void DY_Tracking_DataAnl(uint8_t *data_buf,uint8_t num);

#endif
