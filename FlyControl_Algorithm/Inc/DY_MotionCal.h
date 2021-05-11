/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __DY_MOTIONCAL_H
#define __DY_MOTIONCAL_H
/* Includes ------------------------------------------------------------------*/
#include "DY_FcData.h"
#include "DY_Filter.h"
/* Exported types ------------------------------------------------------------*/
extern _inte_fix_filter_st wcz_acc_fus;
extern _fix_inte_filter_st wcz_spe_fus,wcz_hei_fus;
extern s32 wcz_ref_height,wcz_ref_speed,wcz_ref_acc;

/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
void WCZ_Data_Calc(u8 dT_ms,u8 wcz_f_pause,s32 wcz_acc_get,s32 ref_height);
void WCZ_Data_Reset(void);
#endif

