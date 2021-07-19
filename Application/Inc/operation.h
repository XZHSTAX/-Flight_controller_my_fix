#ifndef _OPERATION_H_
#define _OPERATION_H_

#include "common.h"

typedef struct{

	u8 delaying;
    u8 unlock_flag;

}_our_flags;
extern _our_flags our_flags;



void our_delay_time(void);
void our_landing(void);
void our_take_off(void);



#endif