#ifndef __CAN_UART_EXCHANGE_H__
#define __CAN_UART_EXCHANGE_H__

#include "fdcan.h"
#include "usart.h"
#include "scara.h"
#include "camera_platform.h"

extern void can_to_uart(void);
extern void uart_to_can(void);
extern void uart_cmd(void);

#endif