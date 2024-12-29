/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32g4xx_hal.h"

#include "stm32g4xx_ll_spi.h"
#include "stm32g4xx_ll_bus.h"
#include "stm32g4xx_ll_cortex.h"
#include "stm32g4xx_ll_rcc.h"
#include "stm32g4xx_ll_system.h"
#include "stm32g4xx_ll_utils.h"
#include "stm32g4xx_ll_pwr.h"
#include "stm32g4xx_ll_gpio.h"
#include "stm32g4xx_ll_dma.h"

#include "stm32g4xx_ll_exti.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "timer_tasks.h"

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
extern uint8_t packRdyFromUart;
extern uint8_t packRdyFromCan;
/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define GPIO_CTL1_Pin GPIO_PIN_2
#define GPIO_CTL1_GPIO_Port GPIOC
#define GPIO_CTL2_Pin GPIO_PIN_3
#define GPIO_CTL2_GPIO_Port GPIOC
#define En485_Pin GPIO_PIN_4
#define En485_GPIO_Port GPIOA
#define WAKE_Pin GPIO_PIN_7
#define WAKE_GPIO_Port GPIOE
#define KEY1_Pin GPIO_PIN_10
#define KEY1_GPIO_Port GPIOB
#define KEY1_EXTI_IRQn EXTI15_10_IRQn
#define KEY2_Pin GPIO_PIN_2
#define KEY2_GPIO_Port GPIOD
#define KEY2_EXTI_IRQn EXTI2_IRQn

/* USER CODE BEGIN Private defines */
extern FDCAN_RxHeaderTypeDef RxHeader;
//FDCAN_TxHeaderTypeDef TxHeader;
extern uint8_t RxData[8];
extern void MX_FDCAN1_Init(void);
//uint8_t TxData[8];
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
