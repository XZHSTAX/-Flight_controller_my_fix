#ifndef _SCHEDULER_H_
#define _SCHEDULER_H_

#include "common.h"

typedef struct
{
    void(*task_func)(void);     // 定义一个函数指针
    uint16_t rate_hz;           // 线程的频率
    uint16_t interval_ticks;    // 1000/rate_hz 线程的时长(ms)
    uint32_t last_run;          // 该线程上次运行的时刻
}sched_task_t;

void Scheduler_Setup(void);
void Scheduler_Run(void);

#endif
