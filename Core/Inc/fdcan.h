/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    fdcan.h
  * @brief   This file contains all the function prototypes for
  *          the fdcan.c file
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
/* USER CODE END Header */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __FDCAN_H__
#define __FDCAN_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern FDCAN_HandleTypeDef hfdcan1;

/* USER CODE BEGIN Private defines */
/* USER CODE BEGIN 0 */
#define P_MIN -12.5f
#define P_MAX 12.5f
#define V_MIN -30.0f
#define V_MAX 30.0f
#define KP_MIN 0.0f
#define KP_MAX 500.0f
#define KD_MIN 0.0f
#define KD_MAX 5.0f
#define T_MIN -12.0f
#define T_MAX 12.0f

#define MASTER_ID 0X66

struct exCanIdInfo{
	uint32_t id:8;
	uint32_t data:16;
	uint32_t mode:5;
	uint32_t res:3;
};
extern FDCAN_RxHeaderTypeDef RxHeader;
extern FDCAN_TxHeaderTypeDef TxHeader;
extern struct exCanIdInfo txCanIdInfo;
extern struct exCanIdInfo rxCanIdInfo;
extern uint8_t RxData[8];
extern uint8_t TxData[8];

extern uint8_t runmode;
extern uint16_t index;  // cmd code
extern float ref;       // value
extern uint8_t packRdyFromCan;
#define txCanIdEx (((struct exCanIdInfo)&(TxHeader.Identifier)))
#define rxCanIdEx (((struct exCanIdInfo)&(RxHeader.Identifier))) 
typedef struct __attribute__((packed)){
		uint8_t AT[2];
    uint32_t canID;        // 4 byte CAN ID
    uint8_t data_length;   // 1 byte
    uint8_t data[8];      //  8 byte
		uint8_t end[2];
} Can2UartFrame;
extern Can2UartFrame can2uart_frame;  //  Can2UartFrame 

// #define can_txd() HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, TxData);
// #define can_rxd() HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0,&RxHeader,RxData);)

/* USER CODE END Private defines */

void MX_FDCAN1_Init(void);

/* USER CODE BEGIN Prototypes */
// extern can_txd();
// extern can_rxd();
extern void motor_setzero(uint8_t id, uint16_t master_id);
extern void motor_enable(uint8_t id, uint16_t master_id);
extern void motor_controlmode(uint8_t id, float torque, float MechPosition , float speed ,float kp , float kd);
extern void motor_reset(uint8_t id, uint16_t master_id);
extern void motor_modechange(uint8_t id, uint16_t master_id);
extern void motor_read(uint8_t id, uint16_t master_id);
extern void motor_write(uint8_t id, uint16_t master_id);
extern void update_rxCanIdInfo(void);
extern void can_rxd(void);
extern void can_txd(void);
extern void MX_FDCAN1_Init(void);
extern void motor_init(uint8_t id);
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __FDCAN_H__ */

