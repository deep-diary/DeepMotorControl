/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.h
  * @brief   This file contains all the function prototypes for
  *          the usart.c file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
#include <stdio.h>
#include <string.h>
/* USER CODE END Header */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USART_H__
#define __USART_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern UART_HandleTypeDef hlpuart1;

extern UART_HandleTypeDef huart1;

/* USER CODE BEGIN Private defines */
#define RXBUFFERSIZE	255
typedef struct {
    uint32_t canID;        // 4??? CAN ID
    uint8_t data_length;   // 1???????
    uint8_t data[8];      // ??63?????
} Uart2CanFrame;
extern Uart2CanFrame uart2can_frame;  // ???? Uart2CanFrame ??
extern char RxBuffer[RXBUFFERSIZE];
extern uint8_t aRxBuffer;
extern uint8_t Uart1_Rx_Cnt;
extern uint8_t packRdyFromUart;

/* USER CODE END Private defines */

void MX_LPUART1_UART_Init(void);
void MX_USART1_UART_Init(void);

/* USER CODE BEGIN Prototypes */
extern void uart_to_can(void);
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __USART_H__ */

