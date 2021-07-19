/******************** (C) COPYRIGHT 2018 DeYan Electronic Technology ********************************
 * 作者    ：徳研电科
 * 官网    ：www.gototi.com
 * 描述    ：主函数
******************************************************************************************************/
#include "common.h"
#include "include.h"

int main(void)
{
  BootLoader_Setup();           //设置程序启动入口
  flag.start_ok = All_Init();		//进行所有设备的初始化，并将初始化结果保存
  Scheduler_Setup();			    //裸机系统，人工做了一个任务调度器

  while(1)
  {
    Scheduler_Run();			    //运行任务调度器，所有系统功能，除了中断服务函数外，都在任务调度器内完成
  }
}

/******************* (C) COPYRIGHT 2018 DeYan Electronic Technology **********************************/
