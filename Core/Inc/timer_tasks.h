#ifndef TIMER_TASKS_H
#define TIMER_TASKS_H

#include <stdint.h>
#include <stdbool.h>
#include "main.h"
#include "scara.h"
#include "can_uart_exchange.h"
#include "adc.h"
#include "FOC.h"
#include "tim.h"
#include "camera_platform.h"
#include "spi.h"
// ???????
typedef struct {
    bool flag_1ms;    // 1ms ????
    bool flag_10ms;   // 10ms ????
    bool flag_100ms;  // 100ms ????
    bool flag_1s;     // 1?????
    bool flag_10s;    // 10?????
} TimerFlags;

//static volatile uint32_t sys_cnt;
extern volatile uint32_t sys_cnt;

// ??????????
extern volatile TimerFlags timer_flags;

// ?????????
void TimerTasks_Init(void);

// ????????
void TimerTasks_Scheduler(void);

// ?????? (?????????????)
void TimerTasks_ISR_Handler(void);

#endif // TIMER_TASKS_H
