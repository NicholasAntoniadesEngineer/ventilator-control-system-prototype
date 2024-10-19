/* USER CODE BEGIN Header */
/*
------------------------------------------------------------------------------
   Project  : Ventilator Control System
   File     : main.c
   Brief    : Source file for main.c developed on the NUCLEO F439ZI development board.
   Author   : Nicholas Antoniades
------------------------------------------------------------------------------
*/
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "rtc.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lib.h"
#include <string.h>
#include <stdio.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* Add any private typedef here if necessary */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* Macros are now defined in lib.h */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* Add any private macros here if necessary */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
uint8_t hmiTxBuffer[HMI_BUFFER_SIZE] = {0};  /**< UART transmit buffer */
uint8_t hmiRxBuffer[HMI_BUFFER_SIZE] = {0};  /**< UART receive buffer */
uint32_t adcValues[NUM_ADC_CHANNELS] = {0};  /**< ADC values buffer */
float pressure = 0.0f;                       /**< Pressure value */
uint8_t toggleValue = 0U;                    /**< Toggle value for DMA streams */
uint8_t runFlag = 0U;                        /**< Run flag */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
/* No additional private function prototypes required */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* No additional private user code required */
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  /* Initialization code can be added here */
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of peripherals, Initialize the Flash interface and the Systick */
  HAL_Init();

  /* USER CODE BEGIN Init */
  /* Additional initialization can be added here */
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /* Additional system clock configuration can be added here */
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_USART1_UART_Init();
  MX_RTC_Init();
  MX_TIM3_Init();

  /* USER CODE BEGIN 2 */
  /* Wait until the run flag is set */
  while (runFlag == 0U)
  {
    __NOP();
  }

  /* Configure TIM3 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 17000U;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_TIM_Base_Start_IT(&htim3);

  /* Set initial valve conditions */
  HAL_GPIO_WritePin(GPIOB, Valve1_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, Valve2_Pin, GPIO_PIN_SET);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    HAL_Delay(50U);

    /* Toggle DMA streams for messaging and reading ADC values */
    if (toggleValue == 1U)
    {
      HAL_ADC_Stop_DMA(&hadc1); /* Stop ADC DMA stream */
      UART_Send_DMA(pressure);  /* Send data over UART using DMA */
      toggleValue = 0U;
    }
    else
    {
      HAL_UART_DMAStop(&huart1); /* Stop UART DMA stream */
      HAL_ADC_Start_DMA(&hadc1, adcValues, NUM_ADC_CHANNELS); /* Start ADC DMA stream */
      toggleValue = 1U;
    }
  }
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
  /* Additional user code can be added here */
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  /* USER CODE BEGIN SystemClock_Config */

  /* System clock configuration code */
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators
  */
  RCC_OscInitStruct.OscillatorType       = RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState             = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue  = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState             = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState         = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource        = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM             = 8U;
  RCC_OscInitStruct.PLL.PLLN             = 180U;
  RCC_OscInitStruct.PLL.PLLP             = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ             = 4U;

  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB, and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                     RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the RTC clock source
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInitStruct.RTCClockSelection    = RCC_RTCCLKSOURCE_LSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE END SystemClock_Config */
}

/* USER CODE BEGIN 4 */
/* No additional user code required in main.c */
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add implementation to report the HAL error return state */
  while (1)
  {
    /* Infinite loop in case of error */
  }
  /* USER CODE END Error_Handler_Debug */
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
