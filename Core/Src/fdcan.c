/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    fdcan.c
  * @brief   This file provides code for the configuration
  *          of the FDCAN instances.
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
#include <string.h>
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "fdcan.h"

/* USER CODE BEGIN 0 */
FDCAN_RxHeaderTypeDef RxHeader;
FDCAN_TxHeaderTypeDef TxHeader;
uint8_t RxData[8]={NULL};
uint8_t TxData[8]={0X66,0X66,0X66,0X66,0X66,0X66,0X66,0X66};
struct exCanIdInfo* pCanIdTx = (struct exCanIdInfo*)&TxHeader.Identifier;
struct exCanIdInfo* pCanIdRx = (struct exCanIdInfo*)&RxHeader.Identifier;

struct exCanIdInfo txCanIdInfo;
struct exCanIdInfo rxCanIdInfo;
uint8_t packRdyFromCan = 0;
// ?????????
Can2UartFrame can2uart_frame = {
    .AT = {0x41, 0x54},              // ?AT?????? 'A' ? 'T'
    .canID = 0,                    // ???CAN ID?0,?????
    .data_length = 0,              // ????????0
    .data = {0},                   // ????????0,????????
    .end = {0x0D, 0x0A}            // ?end?????? '\r' ? '\n' (0x0D?0x0A)
};

uint8_t runmode;
uint16_t index;  // cmd code
float ref;       // value

/* USER CODE END 0 */

FDCAN_HandleTypeDef hfdcan1;

/* FDCAN1 init function */
void MX_FDCAN1_Init(void)
{

  /* USER CODE BEGIN FDCAN1_Init 0 */

  /* USER CODE END FDCAN1_Init 0 */

  /* USER CODE BEGIN FDCAN1_Init 1 */

  /* USER CODE END FDCAN1_Init 1 */
  hfdcan1.Instance = FDCAN1;
  hfdcan1.Init.ClockDivider = FDCAN_CLOCK_DIV1;
  hfdcan1.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan1.Init.AutoRetransmission = DISABLE;
  hfdcan1.Init.TransmitPause = DISABLE;
  hfdcan1.Init.ProtocolException = DISABLE;
  hfdcan1.Init.NominalPrescaler = 10;
  hfdcan1.Init.NominalSyncJumpWidth = 1;
  hfdcan1.Init.NominalTimeSeg1 = 10;
  hfdcan1.Init.NominalTimeSeg2 = 5;
  hfdcan1.Init.DataPrescaler = 10;
  hfdcan1.Init.DataSyncJumpWidth = 1;
  hfdcan1.Init.DataTimeSeg1 = 10;
  hfdcan1.Init.DataTimeSeg2 = 5;
  hfdcan1.Init.StdFiltersNbr = 0;
  hfdcan1.Init.ExtFiltersNbr = 0;
  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN1_Init 2 */

  /* USER CODE END FDCAN1_Init 2 */

}

void HAL_FDCAN_MspInit(FDCAN_HandleTypeDef* fdcanHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};
  if(fdcanHandle->Instance==FDCAN1)
  {
  /* USER CODE BEGIN FDCAN1_MspInit 0 */

  /* USER CODE END FDCAN1_MspInit 0 */

  /** Initializes the peripherals clocks
  */
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_FDCAN;
    PeriphClkInit.FdcanClockSelection = RCC_FDCANCLKSOURCE_PCLK1;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    {
      Error_Handler();
    }

    /* FDCAN1 clock enable */
    __HAL_RCC_FDCAN_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**FDCAN1 GPIO Configuration
    PA11     ------> FDCAN1_RX
    PA12     ------> FDCAN1_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF9_FDCAN1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* FDCAN1 interrupt Init */
    HAL_NVIC_SetPriority(FDCAN1_IT0_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(FDCAN1_IT0_IRQn);
  /* USER CODE BEGIN FDCAN1_MspInit 1 */

  /* USER CODE END FDCAN1_MspInit 1 */
  }
}

void HAL_FDCAN_MspDeInit(FDCAN_HandleTypeDef* fdcanHandle)
{

  if(fdcanHandle->Instance==FDCAN1)
  {
  /* USER CODE BEGIN FDCAN1_MspDeInit 0 */

  /* USER CODE END FDCAN1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_FDCAN_CLK_DISABLE();

    /**FDCAN1 GPIO Configuration
    PA11     ------> FDCAN1_RX
    PA12     ------> FDCAN1_TX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_11|GPIO_PIN_12);

    /* FDCAN1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(FDCAN1_IT0_IRQn);
  /* USER CODE BEGIN FDCAN1_MspDeInit 1 */

  /* USER CODE END FDCAN1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
void FDCAN_Config(void)
{
    FDCAN_FilterTypeDef sFilterConfig;

    HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);

    sFilterConfig.IdType = FDCAN_EXTENDED_ID;
    sFilterConfig.FilterIndex = 0;
    sFilterConfig.FilterType = FDCAN_FILTER_RANGE;
    sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    sFilterConfig.FilterID1 = 0x00000000;
    sFilterConfig.FilterID2 = 0x01ffffff;
    HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig);

    TxHeader.Identifier = 0x1B;
    TxHeader.IdType = FDCAN_EXTENDED_ID;
    TxHeader.TxFrameType = FDCAN_DATA_FRAME;
    TxHeader.DataLength = FDCAN_DLC_BYTES_8;
    TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
    TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
    TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    TxHeader.MessageMarker = 0x52;

    HAL_FDCAN_Start(&hfdcan1);
}

int float_to_uint(float x, float x_min, float x_max, int bits){
	float span = x_max - x_min;
	float offset = x_min;
	if(x > x_max) x=x_max;
	else if(x < x_min) x= x_min;
	return (int) ((x-offset)*((float)((1<<bits)-1))/span);
	}

uint32_t swap_bytes(uint32_t value) {
    return ((value & 0x000000FF) << 24) |  // ?????????????
           ((value & 0x0000FF00) << 8)  |  // ?????????????
           ((value & 0x00FF0000) >> 8)  |  // ?????????????
           ((value & 0xFF000000) >> 24);   // ?????????????
}

	// ????,??? TxHeader.Identifier ????? txCanIdInfo
void update_txCanIdInfo(void) {
    memcpy(&txCanIdInfo, &TxHeader.Identifier, sizeof(struct exCanIdInfo));
}

// ????,??? RxHeader.Identifier ????? rxCanIdInfo
void update_rxCanIdInfo(void) {
    memcpy(&rxCanIdInfo, &RxHeader.Identifier, sizeof(struct exCanIdInfo));
}

	// type 3: enable run
void motor_enable(uint8_t id, uint16_t master_id)
{
  
	pCanIdTx->mode = 3;
	pCanIdTx->id = id;
	pCanIdTx->res = 0;
	pCanIdTx->data = master_id;
	can_txd();
}
	// type 1: motion contral
void motor_controlmode(uint8_t id, float torque, float MechPosition , float speed ,float kp , float kd)
{
	pCanIdTx->mode = 1;
	pCanIdTx->id = id;
	pCanIdTx->res = 0;
	pCanIdTx->data = float_to_uint(torque,T_MIN,T_MAX,16);
	
	TxData[0]=float_to_uint(MechPosition,P_MIN,P_MAX,16)>>8;
	TxData[1]=float_to_uint(MechPosition,P_MIN,P_MAX,16);
	TxData[2]=float_to_uint(speed,V_MIN,V_MAX,16)>>8;
	TxData[3]=float_to_uint(speed,V_MIN,V_MAX,16);
	TxData[4]=float_to_uint(kp,KP_MIN,KP_MAX,16)>>8;
	TxData[5]=float_to_uint(kp,KP_MIN,KP_MAX,16);
	TxData[6]=float_to_uint(kd,KD_MIN,KD_MAX,16)>>8;
	TxData[7]=float_to_uint(kd,KD_MIN,KD_MAX,16);
	can_txd();
}
	// type 4:stop
void motor_reset(uint8_t id, uint16_t master_id)
{
	pCanIdTx->mode = 4;
	pCanIdTx->id = id;
	pCanIdTx->res = 0;
	pCanIdTx->data = master_id;
	
	for(uint8_t i=0;i<8;i++)
	{
	TxData[i]=0;
	}
	can_txd();
}

	// type 6:set zero
void motor_setzero(uint8_t id, uint16_t master_id)
{
	pCanIdTx->mode = 6;
	pCanIdTx->id = id;
	pCanIdTx->res = 0;
	pCanIdTx->data = master_id;
	TxData[0]=1;
	for(uint8_t i=1;i<8;i++)
	{
	TxData[i]=0;
	}

	can_txd();
}

	// type 12: set mode
void motor_modechange(uint8_t id, uint16_t master_id)
{
	pCanIdTx->mode = 0x12;
	pCanIdTx->id = id;
	pCanIdTx->res = 0;
	pCanIdTx->data = master_id;
	
	for(uint8_t i=0;i<8;i++)
	{
	TxData[i]=0;
	}
	memcpy(&TxData[0],&index,2);
	memcpy(&TxData[4],&runmode, 1);
	can_txd();
}

// type 18: read param
void motor_read(uint8_t id, uint16_t master_id)
{
	pCanIdTx->mode = 0x11;
	pCanIdTx->id = id;
	pCanIdTx->res = 0;
	pCanIdTx->data = master_id;
	
	for(uint8_t i=0;i<8;i++)
	{
	TxData[i]=0;
	}
	memcpy(&TxData[0],&index,2);
	can_txd();
}

// type 18: write param
void motor_write(uint8_t id, uint16_t master_id)
{
	pCanIdTx->mode = 0x12;
	pCanIdTx->id = id;
	pCanIdTx->res = 0;
	pCanIdTx->data = master_id;
	
	for(uint8_t i=0;i<8;i++)
	{
	TxData[i]=0;
	}
	memcpy(&TxData[0],&index,2);
	memcpy(&TxData[4],&ref,4);
	can_txd();
	// HAL_Delay(1);
}

void can_txd(void) 
{
	HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, TxData);
	update_txCanIdInfo();
}
void can_rxd(void)
{
	HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0,&RxHeader,RxData);
	can2uart_frame.canID = swap_bytes(RxHeader.Identifier); // big and small byte are changed, eg. 0x028001fd will be 0xfd018002 without swap_bytes function
	// memcpy((uint8_t *)&can2uart_frame.canID,(uint8_t *)&RxHeader.Identifier,4);
	can2uart_frame.data_length = RxHeader.DataLength;
	memcpy((uint8_t *)&can2uart_frame.data,(uint8_t *)&RxData,RxHeader.DataLength);
	// update AT
	can2uart_frame.AT[0] = 0x41; // 'A'
	can2uart_frame.AT[1] = 0x54; // 'T'

	//  update end
	can2uart_frame.end[0] = 0x0D; // '\r'
	can2uart_frame.end[1] = 0x0A; // '\n'
	update_rxCanIdInfo();
}

void motor_init(uint8_t id)
{
	// step1: set zero
	motor_setzero(id, MASTER_ID);
	HAL_Delay(1);
	// step2: set position mode
	runmode = 1; // Position mode
	index = 0X7005  ;  // 
	motor_modechange(id, MASTER_ID);
	HAL_Delay(1);
	// step3: Enable
  motor_enable(id, MASTER_ID);
  HAL_Delay(1);
	// step4: Set speed 
	index = 0X7017 ;  // SPEED 0x7017
	ref = 1.0;       // value
  motor_write(id, MASTER_ID);
	HAL_Delay(1);
}


/* USER CODE END 1 */
