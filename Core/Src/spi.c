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
/*
 * AS5047P磁编码器SPI配置说明：
 * - SPI模式：Mode 1 (CPOL=0, CPHA=1)
 * - 时钟频率：最高10MHz (160MHz/16=10MHz)
 * - 数据格式：16位，MSB first
 * - 命令格式：读取角度命令 0x3FFF
 * - 数据保护：奇偶校验
 * - 采样时序：在CLK下降沿采样MOSI数据
 */
uint16_t txBuffer = 0x7FFF;  // 地址0x3fff,读取角度命令, 偶校验, 16位
uint16_t rxBuffer = 0x0000;  // Buffer to receive data，16位
uint16_t rxBufferDMA = 0x0000;  // DMA接收缓冲区，16位
float puAngle,puAngleDMA;

uint16_t puAngle_LL;

// === 新增：DMA读取相关变量 ===
uint16_t spi_tx_buf = 0x3FFF; // 16位命令
uint16_t spi_rx_buf = 0;
volatile uint16_t as5047p_angle = 0;
volatile uint8_t as5047p_dma_ready = 0;
/* USER CODE END 0 */

SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_rx;
DMA_HandleTypeDef hdma_spi1_tx;

/* SPI1 init function */
void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_16BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_HARD_OUTPUT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

void HAL_SPI_MspInit(SPI_HandleTypeDef* spiHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(spiHandle->Instance==SPI1)
  {
  /* USER CODE BEGIN SPI1_MspInit 0 */

  /* USER CODE END SPI1_MspInit 0 */
    /* SPI1 clock enable */
    __HAL_RCC_SPI1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**SPI1 GPIO Configuration
    PA15     ------> SPI1_NSS
    PB3     ------> SPI1_SCK
    PB4     ------> SPI1_MISO
    PB5     ------> SPI1_MOSI
    */
    GPIO_InitStruct.Pin = GPIO_PIN_15;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* SPI1 DMA Init */
    /* SPI1_RX Init */
    hdma_spi1_rx.Instance = DMA1_Channel2;
    hdma_spi1_rx.Init.Request = DMA_REQUEST_SPI1_RX;
    hdma_spi1_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_spi1_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_spi1_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_spi1_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_spi1_rx.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    hdma_spi1_rx.Init.Mode = DMA_NORMAL;
    hdma_spi1_rx.Init.Priority = DMA_PRIORITY_LOW;
    if (HAL_DMA_Init(&hdma_spi1_rx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(spiHandle,hdmarx,hdma_spi1_rx);

    /* SPI1_TX Init */
    hdma_spi1_tx.Instance = DMA1_Channel3;
    hdma_spi1_tx.Init.Request = DMA_REQUEST_SPI1_TX;
    hdma_spi1_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_spi1_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_spi1_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_spi1_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_spi1_tx.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    hdma_spi1_tx.Init.Mode = DMA_NORMAL;
    hdma_spi1_tx.Init.Priority = DMA_PRIORITY_LOW;
    if (HAL_DMA_Init(&hdma_spi1_tx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(spiHandle,hdmatx,hdma_spi1_tx);

  /* USER CODE BEGIN SPI1_MspInit 1 */

  /* USER CODE END SPI1_MspInit 1 */
  }
}

void HAL_SPI_MspDeInit(SPI_HandleTypeDef* spiHandle)
{

  if(spiHandle->Instance==SPI1)
  {
  /* USER CODE BEGIN SPI1_MspDeInit 0 */

  /* USER CODE END SPI1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_SPI1_CLK_DISABLE();

    /**SPI1 GPIO Configuration
    PA15     ------> SPI1_NSS
    PB3     ------> SPI1_SCK
    PB4     ------> SPI1_MISO
    PB5     ------> SPI1_MOSI
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_15);

    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5);

    /* SPI1 DMA DeInit */
    HAL_DMA_DeInit(spiHandle->hdmarx);
    HAL_DMA_DeInit(spiHandle->hdmatx);
  /* USER CODE BEGIN SPI1_MspDeInit 1 */

  /* USER CODE END SPI1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
// Short precise delay using a simple loop (replace with actual microsecond delay if needed)
static inline void short_delay(uint32_t cycles)
{
    while (cycles--) {
        __NOP();  // No Operation, CPU does nothing for 1 cycle
    }
}

// 生成AS5047P命令帧（带偶校验）
uint16_t AS5047P_BuildCmd(uint16_t addr, uint8_t rw)
{
    uint16_t cmd = ((rw & 0x1) << 14) | (addr & 0x3FFF);
    uint16_t temp = cmd;
    uint8_t parity = 0;
    for (int i = 0; i < 15; i++) {
        if (temp & 0x1) parity++;
        temp >>= 1;
    }
    if (parity % 2) cmd |= 0x8000; // 奇数个1，bit15=1
    // 否则bit15=0
    return cmd;
}


// void Read_AS5047P(void)
// {
	
// 	LL_SPI_Enable(SPI1);
// 	LL_SPI_TransmitData16(SPI1, 0xffff);
// 	while(!LL_SPI_IsActiveFlag_RXNE(SPI1));
// 	puAngle_LL = LL_SPI_ReceiveData16(SPI1);
// 	LL_SPI_Disable(SPI1);	
	
// 	puAngle_LL &= 0x3FFF;
// 	puAngle = (float)puAngle_LL / 16384.0f;
// //		
// }


// 错误标志结构体
typedef struct {
    uint8_t PARERR;   // 奇偶校验错误
    uint8_t INVCOMM;  // 无效命令错误
    uint8_t FRERR;    // 帧错误
    uint16_t raw;     // 原始寄存器值
} AS5047P_ErrFlag_t;

// 读取AS5047P的ERRFL寄存器（0x0001），返回错误结构体
AS5047P_ErrFlag_t Read_AS5047P_ERRFL(void)
{
    AS5047P_ErrFlag_t err = {0};
    uint16_t tx_cmd = AS5047P_BuildCmd(0x0001, 1); // 读ERRFL命令
    uint16_t rx_data = 0;
    __HAL_SPI_DISABLE(&hspi1);
    // 延时150ns
    short_delay(150);

    if (HAL_SPI_TransmitReceive(&hspi1, (uint8_t*)&tx_cmd, (uint8_t*)&rx_data, 1, HAL_MAX_DELAY) == HAL_OK)
    {
        err.raw = rx_data & 0xFFFF;
        err.PARERR = (err.raw >> 2) & 0x01;
        err.INVCOMM = (err.raw >> 1) & 0x01;
        err.FRERR = (err.raw >> 0) & 0x01;
    } else {
        err.raw = 0xFFFF; // 通信失败
    }
    return err;
}

void Read_AS5047P_Complete(void)
{
    uint16_t rawData;
    uint8_t error_flag;
    uint8_t parity_bit;
    uint16_t data_bits;

    rxBuffer = 0x0000;

    if (HAL_SPI_TransmitReceive(&hspi1, (uint8_t*)&txBuffer, (uint8_t*)&rxBuffer, 1, HAL_MAX_DELAY) == HAL_OK)
    {
        rawData = rxBuffer;
        parity_bit = (rawData >> 15) & 0x01;
        error_flag = (rawData >> 14) & 0x01;
        data_bits = rawData & 0x3FFF;

        if (error_flag) {
            // 有错误，读取ERRFL寄存器
            AS5047P_ErrFlag_t errfl = Read_AS5047P_ERRFL();
            // 可在此处打印或处理errfl
            // 例如：
            // if (errfl.PARERR) ...
            // if (errfl.INVCOMM) ...
            // if (errfl.FRERR) ...
        } else {
            // 没有错误，判断奇偶校验
            uint8_t calculated_parity = 0;
            uint16_t temp = rawData & 0x7FFF;
            for (int i = 0; i < 15; i++) {
                if (temp & (1 << i)) {
                    calculated_parity ^= 1;
                }
            }
            if (calculated_parity == parity_bit) {
                puAngle_LL = data_bits;
                puAngle = (float)puAngle_LL / 16384.0f;
            } else {
                // 奇偶校验失败，可选：读取ERRFL进一步确认
                AS5047P_ErrFlag_t errfl = Read_AS5047P_ERRFL();
                // 处理errfl
            }
        }
    }
    __HAL_SPI_DISABLE(&hspi1);
}

// 简化的AS5047P读取函数，用于快速测试
void Read_AS5047P_Simple(void)
{
	uint16_t rawData;
	uint16_t simple_tx = 0xFFFF;  // 读取角度命令，16位
	uint16_t simple_rx = 0x0000;  // 清零接收缓冲区，16位
	
	// 简单的读取，不检查状态位
	if (HAL_SPI_TransmitReceive(&hspi1, (uint8_t*)&simple_tx, (uint8_t*)&simple_rx, 1, HAL_MAX_DELAY) == HAL_OK)
	{
		// 直接处理数据（simple_rx已经是16位）
		rawData = simple_rx;
		puAngle_LL = rawData & 0x3FFF;
		puAngle = (float)puAngle_LL / 16384.0f;
	}

  __HAL_SPI_DISABLE(&hspi1);
}



// === 新增：AS5047P DMA启动函数 ===
void AS5047P_Start_DMA(void)
{
    spi_tx_buf = 0xFFFF;
    // as5047p_dma_ready = 0;
    HAL_SPI_TransmitReceive_DMA(&hspi1, (uint8_t*)&spi_tx_buf, (uint8_t*)&spi_rx_buf, 1);
}

// === 新增：DMA回调函数 ===
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi->Instance == SPI1)
    {
        puAngle_LL = spi_rx_buf & 0x3FFF; // 取14位有效数据
        puAngleDMA = (float)puAngle_LL / 16384.0f;
        as5047p_dma_ready = 1;
    }
    __HAL_SPI_DISABLE(&hspi1);
}


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
