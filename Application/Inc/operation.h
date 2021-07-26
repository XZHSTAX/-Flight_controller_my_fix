#ifndef _OPERATION_H_
#define _OPERATION_H_

#include "common.h"

typedef struct{

	u8 delaying;
    u8 unlock_flag;
    u8 UAV_up;
}_our_flags;
extern _our_flags our_flags;

#define fly_time 4000 //5000*10ms = 40s起飞延迟时间
#define fly_land_time 12000 //12000*10ms = 120s降落延迟时间
#define upper_limit 140 //140cm
#define lower_limit 50  //50cm 

void our_delay_time(void);
void our_landing(void);
void our_take_off(void);
void our_mission_updown_repeat(void);
void our_mission_height_control(void);
void our_height_pid_Init(void);
void our_square_trajectory(void);

#endif