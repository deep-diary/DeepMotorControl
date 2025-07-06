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
uint8_t txBuffer[2] = {0x3F, 0xFF};  // 修正：AS5047P读取角度命令 0x3FFF
uint8_t rxBuffer[2],rxBufferDMA[2];                 // Buffer to receive data
float puAngle;

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
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;  // 修正：改为8位数据大小，因为使用8位数组
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;  // 修正：CPOL=0 (时钟空闲时为低电平)
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;      // 修正：CPHA=1 (第二个边沿采样，即下降沿)
  hspi1.Init.NSS = SPI_NSS_HARD_OUTPUT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;  // 修正：10MHz时钟，160MHz/16=10MHz
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;  // 修正：MSB first
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE; //SPI_NSS_PULSE_DISABLE;
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
    hdma_spi1_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;  // 修正：改为字节对齐
    hdma_spi1_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;      // 修正：改为字节对齐
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
    hdma_spi1_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;  // 修正：改为字节对齐
    hdma_spi1_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;      // 修正：改为字节对齐
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

void Read_AS5047P_HAL(void)
{
	uint16_t puAngle_HAL;
	
	// 发送读取角度命令 (0x3FFF) 并接收数据
	HAL_SPI_TransmitReceive(&hspi1, txBuffer, rxBuffer, 2, HAL_MAX_DELAY);
	
	// AS5047P返回的数据格式：高字节在前，低字节在后
	// 需要交换字节序来正确解析
	puAngle_HAL = (rxBuffer[0] << 8) | rxBuffer[1];
	
	// 提取14位角度数据 (去掉最高两位的状态位)
	puAngle_HAL &= 0x3FFF;
	
	// 转换为角度值 (0-1.0)
	puAngle = (float)puAngle_HAL / 16384.0f;
}

// 更完整的AS5047P读取函数，包含错误检查
void Read_AS5047P_Complete(void)
{
	uint16_t rawData;
	uint8_t status;
	uint8_t parity_bit;
	uint8_t data_bits;
	
	// 清零接收缓冲区
	rxBuffer[0] = 0x00;
	rxBuffer[1] = 0x00;
	
	// 发送读取角度命令 (0x3FFF) 并接收数据
	if (HAL_SPI_TransmitReceive(&hspi1, txBuffer, rxBuffer, 2, HAL_MAX_DELAY) == HAL_OK)
	{
		// 组合接收到的数据
		rawData = (rxBuffer[0] << 8) | rxBuffer[1];
		
		// 检查状态位 (bit 14-15)
		status = (rawData >> 14) & 0x03;
		
		// 如果状态正常 (00)，则处理角度数据
		if (status == 0x00)
		{
			// 提取14位角度数据
			puAngle_LL = rawData & 0x3FFF;
			
			// 奇偶校验检查 (根据芯片手册)
			parity_bit = (rawData >> 15) & 0x01;  // 最高位是奇偶校验位
			data_bits = (rawData >> 1) & 0x7FFF;  // 中间15位是数据
			
			// 计算奇偶校验 (15位数据的奇偶性)
			uint8_t calculated_parity = 0;
			for (int i = 0; i < 15; i++) {
				if (data_bits & (1 << i)) {
					calculated_parity ^= 1;
				}
			}
			
			// 如果奇偶校验通过，则使用角度数据
			if (calculated_parity == parity_bit) {
				puAngle = (float)puAngle_LL / 16384.0f;
			} else {
				// 奇偶校验失败，保持上一次的有效值
			}
		}
		else
		{
			// 状态错误处理
			// status = 0x01: 奇偶校验错误
			// status = 0x02: 命令无效
			// status = 0x03: 帧错误或通信问题
			// 保持上一次的有效值或设置默认值
		}
	}
}

// 简化的AS5047P读取函数，用于快速测试
void Read_AS5047P_Simple(void)
{
	uint16_t rawData;
	uint8_t simple_tx[2] = {0x3F, 0xFF};  // 读取角度命令
	uint8_t simple_rx[2] = {0x00, 0x00};  // 清零接收缓冲区
	
	// 简单的读取，不检查状态位
	if (HAL_SPI_TransmitReceive(&hspi1, simple_tx, simple_rx, 2, HAL_MAX_DELAY) == HAL_OK)
	{
		// 直接处理数据
		rawData = (simple_rx[0] << 8) | simple_rx[1];
		puAngle_LL = rawData & 0x3FFF;
		puAngle = (float)puAngle_LL / 16384.0f;
	}
}

// 分离的发送和接收函数，用于调试
void Read_AS5047P_Separate(void)
{
	uint16_t rawData;
	uint8_t tx_cmd[2] = {0x3F, 0xFF};  // 读取角度命令
	uint8_t rx_data[2] = {0x00, 0x00};  // 清零接收缓冲区
	
	// 手动控制片选信号
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);  // 片选拉低
	short_delay(10);  // 短暂延时
	
	// 先发送命令
	if (HAL_SPI_Transmit(&hspi1, tx_cmd, 2, HAL_MAX_DELAY) == HAL_OK)
	{
		// 再接收数据
		if (HAL_SPI_Receive(&hspi1, rx_data, 2, HAL_MAX_DELAY) == HAL_OK)
		{
			// 处理接收到的数据
			rawData = (rx_data[0] << 8) | rx_data[1];
			puAngle_LL = rawData & 0x3FFF;
			puAngle = (float)puAngle_LL / 16384.0f;
		}
	}
	
	// 片选拉高
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);
}

// 调试函数：测试SPI通信
void Test_SPI_Communication(void)
{
	uint8_t test_tx[2] = {0xAA, 0x55};  // 测试模式
	uint8_t test_rx[2] = {0x00, 0x00};
	
	// 清零接收缓冲区
	test_rx[0] = 0x00;
	test_rx[1] = 0x00;
	
	// 发送测试数据
	if (HAL_SPI_TransmitReceive(&hspi1, test_tx, test_rx, 2, HAL_MAX_DELAY) == HAL_OK)
	{
		// 检查接收到的数据
		// 如果MISO线浮空或连接有问题，可能接收到0x00或0xFF
		// 如果连接正常，应该接收到AS5047P的响应
	}
}

// 使用软件片选的AS5047P读取函数
void Read_AS5047P_Software_CS(void)
{
	uint16_t rawData;
	uint8_t tx_cmd[2] = {0x3F, 0xFF};
	uint8_t rx_data[2] = {0x00, 0x00};
	
	// 禁用硬件片选，使用软件控制
	//__HAL_SPI_DISABLE(&hspi1);
	// hspi1.Init.NSS = SPI_NSS_SOFT;
	//short_delay(20);  // 短暂延时
	// __HAL_SPI_ENABLE(&hspi1);
	
	// // 手动控制片选
	// HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);  // 片选拉低
	// short_delay(20);  // 短暂延时
	
	// 发送命令并接收数据
	if (HAL_SPI_TransmitReceive(&hspi1, tx_cmd, rx_data, 2, HAL_MAX_DELAY) == HAL_OK)
	{
		rawData = (rx_data[0] << 8) | rx_data[1];
		puAngle_LL = rawData & 0x3FFF;
		puAngle = (float)puAngle_LL / 16384.0f;
	}
	

	
	// // 恢复硬件片选
	 __HAL_SPI_DISABLE(&hspi1);
	
}

// 低时钟频率测试函数
void Read_AS5047P_Low_Speed(void)
{
	uint16_t rawData;
	uint8_t tx_cmd[2] = {0x3F, 0xFF};
	uint8_t rx_data[2] = {0x00, 0x00};
	
	// 临时降低时钟频率
	__HAL_SPI_DISABLE(&hspi1);
	hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;  // 降低到2.5MHz
	__HAL_SPI_ENABLE(&hspi1);
	
	// 发送命令并接收数据
	if (HAL_SPI_TransmitReceive(&hspi1, tx_cmd, rx_data, 2, HAL_MAX_DELAY) == HAL_OK)
	{
		rawData = (rx_data[0] << 8) | rx_data[1];
		puAngle_LL = rawData & 0x3FFF;
		puAngle = (float)puAngle_LL / 16384.0f;
	}
	
	// 恢复原始时钟频率
	__HAL_SPI_DISABLE(&hspi1);
	hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
	__HAL_SPI_ENABLE(&hspi1);
}

// === 新增：AS5047P DMA启动函数 ===
void AS5047P_Start_DMA(void)
{
    spi_tx_buf = 0x3FFF;
    as5047p_dma_ready = 0;
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET); // NSS拉低
    HAL_SPI_TransmitReceive_DMA(&hspi1, (uint8_t*)&spi_tx_buf, (uint8_t*)&spi_rx_buf, 1);
}

// === 新增：DMA回调函数 ===
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi->Instance == SPI1)
    {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET); // NSS拉高
        as5047p_angle = spi_rx_buf & 0x3FFF; // 取14位有效数据
        as5047p_dma_ready = 1;
    }
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
