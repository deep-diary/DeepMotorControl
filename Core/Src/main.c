/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "comp.h"
#include "dac.h"
#include "dma.h"
#include "fdcan.h"
#include "i2c.h"
#include "usart.h"
#include "opamp.h"
#include "spi.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#define ADC4_SAMPLINGTIME_COMMON_1 LL_ADC_SAMPLINGTIME_COMMON_1
#define ADC4_SAMPLINGTIME_COMMON_2 LL_ADC_SAMPLINGTIME_COMMON_2
#define ADC4_SAMPLINGTIME_COMMON_1 LL_ADC_SAMPLINGTIME_COMMON_1
#define ADC4_SAMPLINGTIME_COMMON_2 LL_ADC_SAMPLINGTIME_COMMON_2
#define ADC4_SAMPLINGTIME_COMMON_1 LL_ADC_SAMPLINGTIME_COMMON_1
#define ADC4_SAMPLINGTIME_COMMON_2 LL_ADC_SAMPLINGTIME_COMMON_2
#define ADC4_SAMPLINGTIME_COMMON_1 LL_ADC_SAMPLINGTIME_COMMON_1
#define ADC4_SAMPLINGTIME_COMMON_2 LL_ADC_SAMPLINGTIME_COMMON_2
#define ADC4_SAMPLINGTIME_COMMON_1 LL_ADC_SAMPLINGTIME_COMMON_1
#define ADC4_SAMPLINGTIME_COMMON_2 LL_ADC_SAMPLINGTIME_COMMON_2
#define ADC4_SAMPLINGTIME_COMMON_1 LL_ADC_SAMPLINGTIME_COMMON_1
#define ADC4_SAMPLINGTIME_COMMON_2 LL_ADC_SAMPLINGTIME_COMMON_2
#define ADC4_SAMPLINGTIME_COMMON_1 LL_ADC_SAMPLINGTIME_COMMON_1
#define ADC4_SAMPLINGTIME_COMMON_2 LL_ADC_SAMPLINGTIME_COMMON_2
#define ADC4_SAMPLINGTIME_COMMON_1 LL_ADC_SAMPLINGTIME_COMMON_1
#define ADC4_SAMPLINGTIME_COMMON_2 LL_ADC_SAMPLINGTIME_COMMON_2
#define ADC4_SAMPLINGTIME_COMMON_1 LL_ADC_SAMPLINGTIME_COMMON_1
#define ADC4_SAMPLINGTIME_COMMON_2 LL_ADC_SAMPLINGTIME_COMMON_2
#define ADC4_SAMPLINGTIME_COMMON_1 LL_ADC_SAMPLINGTIME_COMMON_1
#define ADC4_SAMPLINGTIME_COMMON_2 LL_ADC_SAMPLINGTIME_COMMON_2
#define ADC4_SAMPLINGTIME_COMMON_1 LL_ADC_SAMPLINGTIME_COMMON_1
#define ADC4_SAMPLINGTIME_COMMON_2 LL_ADC_SAMPLINGTIME_COMMON_2
#define ADC4_SAMPLINGTIME_COMMON_1 LL_ADC_SAMPLINGTIME_COMMON_1
#define ADC4_SAMPLINGTIME_COMMON_2 LL_ADC_SAMPLINGTIME_COMMON_2
#define ADC4_SAMPLINGTIME_COMMON_1 LL_ADC_SAMPLINGTIME_COMMON_1
#define ADC4_SAMPLINGTIME_COMMON_2 LL_ADC_SAMPLINGTIME_COMMON_2
#define ADC4_SAMPLINGTIME_COMMON_1 LL_ADC_SAMPLINGTIME_COMMON_1
#define ADC4_SAMPLINGTIME_COMMON_2 LL_ADC_SAMPLINGTIME_COMMON_2
#define ADC4_SAMPLINGTIME_COMMON_1 LL_ADC_SAMPLINGTIME_COMMON_1
#define ADC4_SAMPLINGTIME_COMMON_2 LL_ADC_SAMPLINGTIME_COMMON_2
#define ADC4_SAMPLINGTIME_COMMON_1 LL_ADC_SAMPLINGTIME_COMMON_1
#define ADC4_SAMPLINGTIME_COMMON_2 LL_ADC_SAMPLINGTIME_COMMON_2
#define ADC4_SAMPLINGTIME_COMMON_1 LL_ADC_SAMPLINGTIME_COMMON_1
#define ADC4_SAMPLINGTIME_COMMON_2 LL_ADC_SAMPLINGTIME_COMMON_2
#define ADC4_SAMPLINGTIME_COMMON_1 LL_ADC_SAMPLINGTIME_COMMON_1
#define ADC4_SAMPLINGTIME_COMMON_2 LL_ADC_SAMPLINGTIME_COMMON_2
#define ADC4_SAMPLINGTIME_COMMON_1 LL_ADC_SAMPLINGTIME_COMMON_1
#define ADC4_SAMPLINGTIME_COMMON_2 LL_ADC_SAMPLINGTIME_COMMON_2
#define ADC4_SAMPLINGTIME_COMMON_1 LL_ADC_SAMPLINGTIME_COMMON_1
#define ADC4_SAMPLINGTIME_COMMON_2 LL_ADC_SAMPLINGTIME_COMMON_2
#define ADC4_SAMPLINGTIME_COMMON_1 LL_ADC_SAMPLINGTIME_COMMON_1
#define ADC4_SAMPLINGTIME_COMMON_2 LL_ADC_SAMPLINGTIME_COMMON_2
#define ADC4_SAMPLINGTIME_COMMON_1 LL_ADC_SAMPLINGTIME_COMMON_1
#define ADC4_SAMPLINGTIME_COMMON_2 LL_ADC_SAMPLINGTIME_COMMON_2
#define ADC4_SAMPLINGTIME_COMMON_1 LL_ADC_SAMPLINGTIME_COMMON_1
#define ADC4_SAMPLINGTIME_COMMON_2 LL_ADC_SAMPLINGTIME_COMMON_2
#define ADC4_SAMPLINGTIME_COMMON_1 LL_ADC_SAMPLINGTIME_COMMON_1
#define ADC4_SAMPLINGTIME_COMMON_2 LL_ADC_SAMPLINGTIME_COMMON_2
#define ADC4_SAMPLINGTIME_COMMON_1 LL_ADC_SAMPLINGTIME_COMMON_1
#define ADC4_SAMPLINGTIME_COMMON_2 LL_ADC_SAMPLINGTIME_COMMON_2
#define ADC4_SAMPLINGTIME_COMMON_1 LL_ADC_SAMPLINGTIME_COMMON_1
#define ADC4_SAMPLINGTIME_COMMON_2 LL_ADC_SAMPLINGTIME_COMMON_2
#define ADC4_SAMPLINGTIME_COMMON_1 LL_ADC_SAMPLINGTIME_COMMON_1
#define ADC4_SAMPLINGTIME_COMMON_2 LL_ADC_SAMPLINGTIME_COMMON_2
#define ADC4_SAMPLINGTIME_COMMON_1 LL_ADC_SAMPLINGTIME_COMMON_1
#define ADC4_SAMPLINGTIME_COMMON_2 LL_ADC_SAMPLINGTIME_COMMON_2
#define ADC4_SAMPLINGTIME_COMMON_1 LL_ADC_SAMPLINGTIME_COMMON_1
#define ADC4_SAMPLINGTIME_COMMON_2 LL_ADC_SAMPLINGTIME_COMMON_2
#define ADC4_SAMPLINGTIME_COMMON_1 LL_ADC_SAMPLINGTIME_COMMON_1
#define ADC4_SAMPLINGTIME_COMMON_2 LL_ADC_SAMPLINGTIME_COMMON_2
#define ADC4_SAMPLINGTIME_COMMON_1 LL_ADC_SAMPLINGTIME_COMMON_1
#define ADC4_SAMPLINGTIME_COMMON_2 LL_ADC_SAMPLINGTIME_COMMON_2
#define ADC4_SAMPLINGTIME_COMMON_1 LL_ADC_SAMPLINGTIME_COMMON_1
#define ADC4_SAMPLINGTIME_COMMON_2 LL_ADC_SAMPLINGTIME_COMMON_2
#define ADC4_SAMPLINGTIME_COMMON_1 LL_ADC_SAMPLINGTIME_COMMON_1
#define ADC4_SAMPLINGTIME_COMMON_2 LL_ADC_SAMPLINGTIME_COMMON_2
#define ADC4_SAMPLINGTIME_COMMON_1 LL_ADC_SAMPLINGTIME_COMMON_1
#define ADC4_SAMPLINGTIME_COMMON_2 LL_ADC_SAMPLINGTIME_COMMON_2
#define ADC4_SAMPLINGTIME_COMMON_1 LL_ADC_SAMPLINGTIME_COMMON_1
#define ADC4_SAMPLINGTIME_COMMON_2 LL_ADC_SAMPLINGTIME_COMMON_2
#define ADC4_SAMPLINGTIME_COMMON_1 LL_ADC_SAMPLINGTIME_COMMON_1
#define ADC4_SAMPLINGTIME_COMMON_2 LL_ADC_SAMPLINGTIME_COMMON_2
#define ADC4_SAMPLINGTIME_COMMON_1 LL_ADC_SAMPLINGTIME_COMMON_1
#define ADC4_SAMPLINGTIME_COMMON_2 LL_ADC_SAMPLINGTIME_COMMON_2
#define ADC4_SAMPLINGTIME_COMMON_1 LL_ADC_SAMPLINGTIME_COMMON_1
#define ADC4_SAMPLINGTIME_COMMON_2 LL_ADC_SAMPLINGTIME_COMMON_2
#define ADC4_SAMPLINGTIME_COMMON_1 LL_ADC_SAMPLINGTIME_COMMON_1
#define ADC4_SAMPLINGTIME_COMMON_2 LL_ADC_SAMPLINGTIME_COMMON_2
#define ADC4_SAMPLINGTIME_COMMON_1 LL_ADC_SAMPLINGTIME_COMMON_1
#define ADC4_SAMPLINGTIME_COMMON_2 LL_ADC_SAMPLINGTIME_COMMON_2
#define ADC4_SAMPLINGTIME_COMMON_1 LL_ADC_SAMPLINGTIME_COMMON_1
#define ADC4_SAMPLINGTIME_COMMON_2 LL_ADC_SAMPLINGTIME_COMMON_2
#define ADC4_SAMPLINGTIME_COMMON_1 LL_ADC_SAMPLINGTIME_COMMON_1
#define ADC4_SAMPLINGTIME_COMMON_2 LL_ADC_SAMPLINGTIME_COMMON_2
#define ADC4_SAMPLINGTIME_COMMON_1 LL_ADC_SAMPLINGTIME_COMMON_1
#define ADC4_SAMPLINGTIME_COMMON_2 LL_ADC_SAMPLINGTIME_COMMON_2
#define ADC4_SAMPLINGTIME_COMMON_1 LL_ADC_SAMPLINGTIME_COMMON_1
#define ADC4_SAMPLINGTIME_COMMON_2 LL_ADC_SAMPLINGTIME_COMMON_2
#define ADC4_SAMPLINGTIME_COMMON_1 LL_ADC_SAMPLINGTIME_COMMON_1
#define ADC4_SAMPLINGTIME_COMMON_2 LL_ADC_SAMPLINGTIME_COMMON_2
#define ADC4_SAMPLINGTIME_COMMON_1 LL_ADC_SAMPLINGTIME_COMMON_1
#define ADC4_SAMPLINGTIME_COMMON_2 LL_ADC_SAMPLINGTIME_COMMON_2
#define ADC4_SAMPLINGTIME_COMMON_1 LL_ADC_SAMPLINGTIME_COMMON_1
#define ADC4_SAMPLINGTIME_COMMON_2 LL_ADC_SAMPLINGTIME_COMMON_2
#define ADC4_SAMPLINGTIME_COMMON_1 LL_ADC_SAMPLINGTIME_COMMON_1
#define ADC4_SAMPLINGTIME_COMMON_2 LL_ADC_SAMPLINGTIME_COMMON_2
#define ADC4_SAMPLINGTIME_COMMON_1 LL_ADC_SAMPLINGTIME_COMMON_1
#define ADC4_SAMPLINGTIME_COMMON_2 LL_ADC_SAMPLINGTIME_COMMON_2
#define ADC4_SAMPLINGTIME_COMMON_1 LL_ADC_SAMPLINGTIME_COMMON_1
#define ADC4_SAMPLINGTIME_COMMON_2 LL_ADC_SAMPLINGTIME_COMMON_2
#define ADC4_SAMPLINGTIME_COMMON_1 LL_ADC_SAMPLINGTIME_COMMON_1
#define ADC4_SAMPLINGTIME_COMMON_2 LL_ADC_SAMPLINGTIME_COMMON_2
#define ADC4_SAMPLINGTIME_COMMON_1 LL_ADC_SAMPLINGTIME_COMMON_1
#define ADC4_SAMPLINGTIME_COMMON_2 LL_ADC_SAMPLINGTIME_COMMON_2
#define ADC4_SAMPLINGTIME_COMMON_1 LL_ADC_SAMPLINGTIME_COMMON_1
#define ADC4_SAMPLINGTIME_COMMON_2 LL_ADC_SAMPLINGTIME_COMMON_2
#define ADC4_SAMPLINGTIME_COMMON_1 LL_ADC_SAMPLINGTIME_COMMON_1
#define ADC4_SAMPLINGTIME_COMMON_2 LL_ADC_SAMPLINGTIME_COMMON_2
#define ADC4_SAMPLINGTIME_COMMON_1 LL_ADC_SAMPLINGTIME_COMMON_1
#define ADC4_SAMPLINGTIME_COMMON_2 LL_ADC_SAMPLINGTIME_COMMON_2
#define ADC4_SAMPLINGTIME_COMMON_1 LL_ADC_SAMPLINGTIME_COMMON_1
#define ADC4_SAMPLINGTIME_COMMON_2 LL_ADC_SAMPLINGTIME_COMMON_2
#define ADC4_SAMPLINGTIME_COMMON_1 LL_ADC_SAMPLINGTIME_COMMON_1
#define ADC4_SAMPLINGTIME_COMMON_2 LL_ADC_SAMPLINGTIME_COMMON_2
#define ADC4_SAMPLINGTIME_COMMON_1 LL_ADC_SAMPLINGTIME_COMMON_1
#define ADC4_SAMPLINGTIME_COMMON_2 LL_ADC_SAMPLINGTIME_COMMON_2
#define ADC4_SAMPLINGTIME_COMMON_1 LL_ADC_SAMPLINGTIME_COMMON_1
#define ADC4_SAMPLINGTIME_COMMON_2 LL_ADC_SAMPLINGTIME_COMMON_2
#define ADC4_SAMPLINGTIME_COMMON_1 LL_ADC_SAMPLINGTIME_COMMON_1
#define ADC4_SAMPLINGTIME_COMMON_2 LL_ADC_SAMPLINGTIME_COMMON_2
#define ADC4_SAMPLINGTIME_COMMON_1 LL_ADC_SAMPLINGTIME_COMMON_1
#define ADC4_SAMPLINGTIME_COMMON_2 LL_ADC_SAMPLINGTIME_COMMON_2
#define ADC4_SAMPLINGTIME_COMMON_1 LL_ADC_SAMPLINGTIME_COMMON_1
#define ADC4_SAMPLINGTIME_COMMON_2 LL_ADC_SAMPLINGTIME_COMMON_2
#define ADC4_SAMPLINGTIME_COMMON_1 LL_ADC_SAMPLINGTIME_COMMON_1
#define ADC4_SAMPLINGTIME_COMMON_2 LL_ADC_SAMPLINGTIME_COMMON_2
#define ADC4_SAMPLINGTIME_COMMON_1 LL_ADC_SAMPLINGTIME_COMMON_1
#define ADC4_SAMPLINGTIME_COMMON_2 LL_ADC_SAMPLINGTIME_COMMON_2
#define ADC4_SAMPLINGTIME_COMMON_1 LL_ADC_SAMPLINGTIME_COMMON_1
#define ADC4_SAMPLINGTIME_COMMON_2 LL_ADC_SAMPLINGTIME_COMMON_2
#define ADC4_SAMPLINGTIME_COMMON_1 LL_ADC_SAMPLINGTIME_COMMON_1
#define ADC4_SAMPLINGTIME_COMMON_2 LL_ADC_SAMPLINGTIME_COMMON_2
#define ADC4_SAMPLINGTIME_COMMON_1 LL_ADC_SAMPLINGTIME_COMMON_1
#define ADC4_SAMPLINGTIME_COMMON_2 LL_ADC_SAMPLINGTIME_COMMON_2
#define ADC4_SAMPLINGTIME_COMMON_1 LL_ADC_SAMPLINGTIME_COMMON_1
#define ADC4_SAMPLINGTIME_COMMON_2 LL_ADC_SAMPLINGTIME_COMMON_2
#define ADC4_SAMPLINGTIME_COMMON_1 LL_ADC_SAMPLINGTIME_COMMON_1
#define ADC4_SAMPLINGTIME_COMMON_2 LL_ADC_SAMPLINGTIME_COMMON_2
#define ADC4_SAMPLINGTIME_COMMON_1 LL_ADC_SAMPLINGTIME_COMMON_1
#define ADC4_SAMPLINGTIME_COMMON_2 LL_ADC_SAMPLINGTIME_COMMON_2
#define ADC4_SAMPLINGTIME_COMMON_1 LL_ADC_SAMPLINGTIME_COMMON_1
#define ADC4_SAMPLINGTIME_COMMON_2 LL_ADC_SAMPLINGTIME_COMMON_2
#define ADC4_SAMPLINGTIME_COMMON_1 LL_ADC_SAMPLINGTIME_COMMON_1
#define ADC4_SAMPLINGTIME_COMMON_2 LL_ADC_SAMPLINGTIME_COMMON_2
#define ADC4_SAMPLINGTIME_COMMON_1 LL_ADC_SAMPLINGTIME_COMMON_1
#define ADC4_SAMPLINGTIME_COMMON_2 LL_ADC_SAMPLINGTIME_COMMON_2


#include "FOC.h"
#include "NVMemory.h"
#include "scara.h"
#include "camera_platform.h"
extern UART_HandleTypeDef huart1;
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
void FDCAN_Config(void);
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t Data[32] = "LED Toggle DMA\r\n";


#define ADC_OFFSET_CNT	10


uint16_t IA_Offset, IB_Offset, IC_Offset;
uint16_t adc1_in1, adc1_in2,adc1_in3;
uint8_t ADC_offset = 0;
uint8_t invertStat = 0;
uint16_t rawData;
uint16_t g_sensorData = 0;  // Global variable to store the sensor data
uint16_t DelayCnt = 1;
float load_data[6];
static uint8_t tempData[28] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0x80,0x7f};
uint8_t A,B,I;

	
uint32_t writeValue = 0x99999999,pValue=0;
uint16_t addr = 1;
uint8_t eepromWriteFlag = 1;
	

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// ???????????????????
uint8_t I2C_ReadRegister(uint16_t DevAddress, uint8_t RegAddress)
{
    uint8_t receivedData = 0;
    HAL_StatusTypeDef status;

    // ??????????
    status = HAL_I2C_Master_Transmit(&hi2c3, DevAddress << 1, &RegAddress, 1, HAL_MAX_DELAY);
    if (status != HAL_OK) 
    {
        // ?????????
        Error_Handler();
    }

    // ???????????
    status = HAL_I2C_Master_Receive(&hi2c3, DevAddress << 1, &receivedData, 1, HAL_MAX_DELAY);
    if (status != HAL_OK) 
    {
        // ?????????
        Error_Handler();
    }

    return receivedData;
}

void Write_I2C(uint8_t deviceAddress, uint8_t registerAddress, uint8_t data)
{
    // ??????????????????
    uint8_t buffer[2];
    buffer[0] = registerAddress; // ???????
    buffer[1] = data;            // ?????????

    // ??I2C?????????
    if (HAL_I2C_Master_Transmit(&hi2c3, (deviceAddress << 1), buffer, 2, HAL_MAX_DELAY) != HAL_OK)
    {
        // ??????,???????
        Error_Handler();
    }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  NV_Init();
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_OPAMP1_Init();
  MX_OPAMP2_Init();
  MX_OPAMP3_Init();
  MX_TIM1_Init();
  MX_COMP1_Init();
  MX_DAC3_Init();
  MX_I2C3_Init();
  MX_SPI1_Init();
  MX_FDCAN1_Init();
  MX_TIM4_Init();
  MX_TIM2_Init();
  MX_I2C2_Init();
  MX_LPUART1_UART_Init();
  /* USER CODE BEGIN 2 */
	HAL_OPAMP_Start(&hopamp1); // 使能运放
	HAL_OPAMP_Start(&hopamp2); // 使能运放
	HAL_OPAMP_Start(&hopamp3); // 使能运放
	HAL_UART_Receive_IT(&huart1, (uint8_t *)&aRxBuffer,1);
	HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
	HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7, GPIO_PIN_SET);  // 唤起invert
//	TIM1->PSC = 1000;
//	TIM1->ARR = 10000;
	TIM1->ARR = 8000-1;
	TIM1->CCR1 = 0;  // 比较值，最大是配置的8000
	TIM1->CCR2 = 0;
	TIM1->CCR3 = 0;
	TIM1->CCR4 = 8000-2;
	
	HAL_TIM_Base_Start(&htim1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
	
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);  // start interruput


	HAL_ADCEx_InjectedStart_IT(&hadc1);
	HAL_ADCEx_InjectedStart(&hadc2);
	HAL_DAC_Start(&hdac3,DAC_CHANNEL_1);  // 用于内部比较器，电流封波
	//HAL_DAC_Start(&hdac1,DAC_CHANNEL_1);  // 用于外部模拟量输出
	HAL_DAC_SetValue(&hdac3,DAC_CHANNEL_1,DAC_ALIGN_12B_R, 3000);
	//HAL_DAC_SetValue(&hdac1,DAC_CHANNEL_1,DAC_ALIGN_12B_R, 4095);
	
	// disable without high side power supply
	HAL_COMP_Start(&hcomp1);	
	invertStat = I2C_ReadRegister(0x47, 0x80);
	Write_I2C(0x47, 0x09, 0xFF);


	FDCAN_Config();
	
	
	SysEnable = 0;
	FOC_initialize();
	SpReqPU = 0.01F;
	

	camera_platform_init();
	
	// arm_init(&scaraArm); // init arm


//	arm_mov_line(&scaraArm, 0,100,0,330,5); // move line
//	arm_zero(&scaraArm);
//	arm_mov_line(&scaraArm, -150,200,150,200,5); // move line
//	
//	arm_reset(&scaraArm);
//	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		TimerTasks_Scheduler();
//		if(eepromWriteFlag)
//		{
//				NV_RequestWriteData(addr, writeValue);  //  anywhere
//				eepromWriteFlag = 0;
//		}
//		NV_Housekeeping();
//		NV_ReadData(addr, &pValue);  // anywhere


//			ref = 0 - ref;       // value
//			motor_write(1, MASTER_ID);
//			HAL_Delay(DelayCnt);	


					
	
		
//			memcpy(tempData,(uint8_t *)&load_data,sizeof(load_data));
//			HAL_UART_Transmit_DMA(&huart1, tempData, sizeof(tempData));
//			if((GPIOE->IDR & GPIO_PIN_9) !=0)
//			{
//				load_data[3] = 1.0f;
//			}
//			else
//			{
//				load_data[3] = 0.0f;
//			}
//			HAL_Delay(DelayCnt);  // 1ms
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 20;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */



// adc 注入完成中断
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    /* Prevent unused argument(s) compilation warning */
		static uint8_t cnt;
		real32_T IaAd;
		real32_T IbAd;
		real32_T IcAd;
		
		TimerTasks_ISR_Handler();

		/* Prevent unused argument(s) compilation warning */
		UNUSED(hadc);
		if(hadc == &hadc1)
		{
				if(ADC_offset == 0)
				{
						cnt++;
						adc1_in1 = hadc1.Instance->JDR1;
					  adc1_in3 = hadc1.Instance->JDR2;
						adc1_in2 = hadc2.Instance->JDR1;
						
						IA_Offset += adc1_in1;
						IB_Offset += adc1_in2;
						IC_Offset += adc1_in3;
						if(cnt >= ADC_OFFSET_CNT)
						{
								ADC_offset = 1;
								IA_Offset = IA_Offset / ADC_OFFSET_CNT;
								IB_Offset = IB_Offset / ADC_OFFSET_CNT;
								IC_Offset = IC_Offset / ADC_OFFSET_CNT;
						}
				}
				else
				{
						HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_15);
						Read_AS5047P();
						//Read_SC60228();
					  
					
					
						adc1_in1 = hadc1.Instance->JDR1;
						adc1_in3 = hadc1.Instance->JDR2;
					  adc1_in2 = hadc2.Instance->JDR1;
					


					
						IaAd = (adc1_in1 - IA_Offset) * 0.0193359375f;  // adc*3.3/(4096*0.005*(1+11/1.5))
						IbAd = (adc1_in2 - IB_Offset) * 0.0193359375f;  // adc*3.3/(4096*0.005*(1+11/1.5))	
						IcAd = (adc1_in3 - IC_Offset) * 0.0193359375f;  // adc*3.3/(4096*0.005*(1+11/1.5))
					
//						Ia = IaSim;  
//						Ib = IbSim;
//						Ic = IcSim;
					
						Ia = -IaAd;  
						Ib = -IcAd;
						Ic = -IbAd;
					
						
						EposReq = EposOpen;
					  FOC_step();
					
						TIM1->CCR1 = U;  // 比较值，最大是配置的8000
						TIM1->CCR2 = V;
						TIM1->CCR3 = W;
						// puAngle sys_cnt EXT0 EXT1 EXT2 VqReqPU
						load_data[0] = sys_cnt;
						load_data[1] = EXT0;
						load_data[2] = EXT1;
						load_data[3] = EXT2;  //Vpoten;
						load_data[4] = TIM4->CNT;   // Vbus;
						load_data[5] = TIM4->CCR1;   // Vbus;

						
						
						memcpy(tempData,(uint8_t *)&load_data,sizeof(load_data));
						//HAL_UART_Transmit_DMA(&huart1, tempData, sizeof(tempData));
				}
		}
		
    /* NOTE: This function should not be modified. When the callback is needed, 
       function HAL_ADCEx_InjectedConvCpltCallback must be implemented in the user file. */
}



// DMA Interruput
//void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
//{
//    
//		if (hspi->Instance == SPI1) {
//			g_sensorData = (rxBufferDMA[1] << 8) | rxBufferDMA[0];

//			//g_sensorData = g_sensorData>>4;
//			//puAngleDMA = (float)g_sensorData / 4096.0f;
//			
//			g_sensorData &= 0x3FFF;
//			puAngle = (float)g_sensorData / 16384.0f;
//			
//			__HAL_SPI_DISABLE(&hspi1);
//    }
//		
//}






/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
