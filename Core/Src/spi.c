/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    spi.c
  * @brief   This file provides code for the configuration
  *          of the SPI instances.
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
/* Includes ------------------------------------------------------------------*/
#include "spi.h"

/* USER CODE BEGIN 0 */
uint8_t txBuffer[2] = {0xff,0xff};  // Command to read angle data
uint8_t rxBuffer[2],rxBufferDMA[2];                 // Buffer to receive data
float puAngle;

uint16_t puAngle_LL;
/* USER CODE END 0 */

/* SPI1 init function */
void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  LL_SPI_InitTypeDef SPI_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SPI1);

  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOB);
  /**SPI1 GPIO Configuration
  PA15   ------> SPI1_NSS
  PB3   ------> SPI1_SCK
  PB4   ------> SPI1_MISO
  PB5   ------> SPI1_MOSI
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_15;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_5;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_3;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_5;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_4;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_5;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_5;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_5;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* SPI1 DMA Init */

  /* SPI1_RX Init */
  LL_DMA_SetPeriphRequest(DMA1, LL_DMA_CHANNEL_2, LL_DMAMUX_REQ_SPI1_RX);

  LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_CHANNEL_2, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);

  LL_DMA_SetChannelPriorityLevel(DMA1, LL_DMA_CHANNEL_2, LL_DMA_PRIORITY_LOW);

  LL_DMA_SetMode(DMA1, LL_DMA_CHANNEL_2, LL_DMA_MODE_NORMAL);

  LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_CHANNEL_2, LL_DMA_PERIPH_NOINCREMENT);

  LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_CHANNEL_2, LL_DMA_MEMORY_INCREMENT);

  LL_DMA_SetPeriphSize(DMA1, LL_DMA_CHANNEL_2, LL_DMA_PDATAALIGN_HALFWORD);

  LL_DMA_SetMemorySize(DMA1, LL_DMA_CHANNEL_2, LL_DMA_MDATAALIGN_HALFWORD);

  /* SPI1_TX Init */
  LL_DMA_SetPeriphRequest(DMA1, LL_DMA_CHANNEL_3, LL_DMAMUX_REQ_SPI1_TX);

  LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_CHANNEL_3, LL_DMA_DIRECTION_MEMORY_TO_PERIPH);

  LL_DMA_SetChannelPriorityLevel(DMA1, LL_DMA_CHANNEL_3, LL_DMA_PRIORITY_LOW);

  LL_DMA_SetMode(DMA1, LL_DMA_CHANNEL_3, LL_DMA_MODE_NORMAL);

  LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_CHANNEL_3, LL_DMA_PERIPH_NOINCREMENT);

  LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_CHANNEL_3, LL_DMA_MEMORY_INCREMENT);

  LL_DMA_SetPeriphSize(DMA1, LL_DMA_CHANNEL_3, LL_DMA_PDATAALIGN_HALFWORD);

  LL_DMA_SetMemorySize(DMA1, LL_DMA_CHANNEL_3, LL_DMA_MDATAALIGN_HALFWORD);

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  SPI_InitStruct.TransferDirection = LL_SPI_FULL_DUPLEX;
  SPI_InitStruct.Mode = LL_SPI_MODE_MASTER;
  SPI_InitStruct.DataWidth = LL_SPI_DATAWIDTH_16BIT;
  SPI_InitStruct.ClockPolarity = LL_SPI_POLARITY_LOW;
  SPI_InitStruct.ClockPhase = LL_SPI_PHASE_2EDGE;
  SPI_InitStruct.NSS = LL_SPI_NSS_HARD_OUTPUT;
  SPI_InitStruct.BaudRate = LL_SPI_BAUDRATEPRESCALER_DIV16;
  SPI_InitStruct.BitOrder = LL_SPI_MSB_FIRST;
  SPI_InitStruct.CRCCalculation = LL_SPI_CRCCALCULATION_DISABLE;
  SPI_InitStruct.CRCPoly = 7;
  LL_SPI_Init(SPI1, &SPI_InitStruct);
  LL_SPI_SetStandard(SPI1, LL_SPI_PROTOCOL_MOTOROLA);
  LL_SPI_DisableNSSPulseMgt(SPI1);
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/* USER CODE BEGIN 1 */
// Short precise delay using a simple loop (replace with actual microsecond delay if needed)
static inline void short_delay(uint32_t cycles)
{
    while (cycles--) {
        __NOP();  // No Operation, CPU does nothing for 1 cycle
    }
}


void Read_AS5047P(void)
{
	
	LL_SPI_Enable(SPI1);
	LL_SPI_TransmitData16(SPI1, 0xffff);
	while(!LL_SPI_IsActiveFlag_RXNE(SPI1));
	puAngle_LL = LL_SPI_ReceiveData16(SPI1);
	LL_SPI_Disable(SPI1);	
	
	puAngle_LL &= 0x3FFF;
	puAngle = (float)puAngle_LL / 16384.0f;
//		
}



//// Read 16-bit data from AS5047P encoder
//void Read_AS5047P(void)
//{
////    uint8_t txBuffer[2] = {0x3F, 0xFF};  // Command to read angle data
////    uint8_t rxBuffer[2] = {0x00, 0x00};  // Buffer to receive data
//    uint16_t receivedData = 0;

//    // Pull NSS low to start communication
////    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);
////		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);
//	
//    // Transmit command to read angle data
//		if (HAL_SPI_TransmitReceive(&hspi1, txBuffer, rxBuffer, 1, HAL_MAX_DELAY) != HAL_OK)
//    {
//        // Handle error if transmission fails
//        Error_Handler();
//    }
//		
//		// Pull NSS high to end communication
//		__HAL_SPI_DISABLE(&hspi1);
////		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);
////    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);
//		
//		
//		// Combine the received bytes into a 16-bit value
//		receivedData = (rxBuffer[1] << 8) | rxBuffer[0];
//		// Mask the lower 14 bits (assuming sensor data is in the lower 14 bits)
//		receivedData &= 0x3FFF;
//		puAngle = (float)receivedData / 16384.0f;
//		
//		// DMA read
////		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);
////  		HAL_SPI_TransmitReceive_DMA(&hspi1, txBuffer, rxBufferDMA, 1);
////		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);

//}

//// Read 14-bit data from SC60228 encoder
//void Read_SC60228(void)
//{
////    uint8_t txBuffer[2] = {0x3F, 0xFF};  // Command to read angle data
////    uint8_t rxBuffer[2] = {0x00, 0x00};  // Buffer to receive data
//    uint16_t receivedData = 0;

//    // Pull NSS low to start communication
//    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);
//    __NOP();  // Use a few NOPs or a short delay loop if necessary

//    // Transmit command to read angle data
//		if (HAL_SPI_TransmitReceive(&hspi1, txBuffer, rxBuffer, 1, HAL_MAX_DELAY) != HAL_OK)
//   // if (HAL_SPI_TransmitReceive_DMA(&hspi1, txBuffer, rxBuffer, 2) != HAL_OK)
//    {
//        // Handle error if transmission fails
//        Error_Handler();
//    }
//		//__HAL_SPI_DISABLE(&hspi1);
//    // Pull NSS high to end communication
//    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);
//				// Combine the received bytes into a 16-bit value
//		receivedData = (rxBuffer[1] << 8) | rxBuffer[0];
//		
//		// Mask the lower 12 bits (assuming sensor data is in the lower 14 bits)
//		receivedData = receivedData>>4;
//		puAngle = (float)receivedData / 4096.0f;
//		
//		
//		// DMA read
////		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);
////		HAL_SPI_TransmitReceive_DMA(&hspi1, txBuffer, rxBufferDMA, 1);
////		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);

//}
/* USER CODE END 1 */
