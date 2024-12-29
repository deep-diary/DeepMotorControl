#include "timer_tasks.h"
#include <stdio.h>

// ??????????
volatile TimerFlags timer_flags = {0};  // ????????0
volatile uint32_t sys_cnt = 0;   // ????? (?100us ?1)

// ??????
static void task_1ms(void) {
    //printf("Executing 1ms task\n");
		if(packRdyFromCan)
		{
			can_to_uart();
			packRdyFromCan = 0;
		}
		if(packRdyFromUart)
		{
			uart_to_can();
			uart_cmd();
			packRdyFromUart = 0;
		}
//		if(!SysEnable)
//		{
//			arm_reset(&scaraArm);
//		}
}

static void task_10ms(void) {
    //printf("Executing 10ms task\n");
    // update platform position
    camera_10ms_task();
}

static void task_100ms(void) {
    //printf("Executing 100ms task\n");
		get_vbus_temp();
		arm_set_pos(&scaraArm, scaraArm.req_x, scaraArm.req_y);
}

static void task_1s(void) {
    //printf("Executing 1s task\n");
}

static void task_10s(void) {
    //printf("Executing 10s task\n");
}

// ?????????
void TimerTasks_Init(void) {
    // ??????????????????????
}

// ??????,?????100us???????
void TimerTasks_ISR_Handler(void) {
    sys_cnt++;  // ??????100????

    // ?1ms(10???)??1ms??
    if (sys_cnt % 10 == 0) {
        timer_flags.flag_1ms = true;
    }

    // ?10ms(100???)??10ms??
    if (sys_cnt % 100 == 0) {
        timer_flags.flag_10ms = true;
    }

    // ?100ms(1000???)??100ms??
    if (sys_cnt % 1000 == 0) {
        timer_flags.flag_100ms = true;
    }

    // ?1?(10000???)??1???
    if (sys_cnt % 10000 == 0) {
        timer_flags.flag_1s = true;
//				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_RESET);  // Blue LED
//				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_SET);  // always en485
//				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_RESET);  // Blue LED
//				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_SET);  // always en485
					HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_2);
					HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_3);
					//HAL_GPIO_TogglePin(GPIOC, GPIO_CTL2_Pin);
    }

    // ?10?(100000???)??10???
    if (sys_cnt % 100000 == 0) {
        timer_flags.flag_10s = true;
    }
}

// ????????
void TimerTasks_Scheduler(void) {
    // ??1ms???
    if (timer_flags.flag_1ms) {
        timer_flags.flag_1ms = false;  // ?????
        task_1ms();  // ??1ms??
    }

    // ??10ms???
    if (timer_flags.flag_10ms) {
        timer_flags.flag_10ms = false;  // ?????
        task_10ms();  // ??10ms??
    }

    // ??100ms???
    if (timer_flags.flag_100ms) {
        timer_flags.flag_100ms = false;  // ?????
        task_100ms();  // ??100ms??
    }

    // ??1s???
    if (timer_flags.flag_1s) {
        timer_flags.flag_1s = false;  // ?????
        task_1s();  // ??1???
    }

    // ??10s???
    if (timer_flags.flag_10s) {
        timer_flags.flag_10s = false;  // ?????
        task_10s();  // ??10???
    }
}
