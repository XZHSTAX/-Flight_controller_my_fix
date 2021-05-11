#ifndef _DY_FLYCONTROL_H_
#define _DY_FLYCONTROL_H_

#include "common.h"

void Flash_SectorsRead2(uint32_t address, uint8_t *buffer, uint16_t count);
void DY_DT_SendStr(char *str, u8 len);

#endif
