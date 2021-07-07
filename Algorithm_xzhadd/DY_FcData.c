/******************** (C) COPYRIGHT 2018 DY EleTe ********************************
 * 作者    ：徳研电科
 * 官网    ：www.gototi.com
 * 描述    ：飞控数据
**********************************************************************************/
#include "DY_FcData.h"
#include "DY_Parameter.h"

_switch_st switchs;
_save_st save;
_flag flag;
_sensor_hd_check_st sens_hd_check;

void data_save(void)
{
  para_sta.save_en = !flag.fly_ready;
  para_sta.save_trig = 1;
}

void Para_Data_Init()
{
  DY_Parame_Read();
}

/******************* (C) COPYRIGHT 2018 DY EleTe *****END OF FILE************/
