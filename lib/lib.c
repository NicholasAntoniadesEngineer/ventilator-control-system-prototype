/* USER CODE BEGIN Header */
/*
------------------------------------------------------------------------------
   Project  : Ventilator Control System
   File     : lib.c
   Brief    : Source file containing the implementation of library functions.
   Author   : Nicholas Antoniades
------------------------------------------------------------------------------
*/
/* USER CODE END Header */

#include "lib.h"
#include "usart.h"
#include "adc.h"
#include "tim.h"
#include "gpio.h"

/* Private variables ---------------------------------------------------------*/
extern uint8_t hmiTxBuffer[HMI_BUFFER_SIZE];  /**< UART transmit buffer */
extern uint8_t hmiRxBuffer[HMI_BUFFER_SIZE];  /**< UART receive buffer */
extern uint32_t adcValues[NUM_ADC_CHANNELS];  /**< ADC values buffer */
extern float pressure;                        /**< Pressure value */
extern uint8_t toggleValue;                   /**< Toggle value for DMA streams */
extern uint8_t runFlag;                       /**< Run flag */

static uint8_t eightBitResult[4];             /**< 8-bit result array */

/* Function implementations --------------------------------------------------*/

void Convert_To_EightBit(uint8_t *resultArray, uint32_t value)
{
  /* Break down the 32-bit value into four 8-bit values */
  resultArray[0] = (uint8_t)(value & 0xFFU);
  resultArray[1] = (uint8_t)((value >> 8U) & 0xFFU);
  resultArray[2] = (uint8_t)((value >> 16U) & 0xFFU);
  resultArray[3] = (uint8_t)((value >> 24U) & 0xFFU);
}

void UART_Send_DMA(float pressureValue)
{
  /* Convert pressure to 8-bit result for UART transmission */
  Convert_To_EightBit(eightBitResult, (uint32_t)pressureValue);

  /* Prepare the transmit buffer */
  hmiTxBuffer[0]  = eightBitResult[0];
  hmiTxBuffer[1]  = eightBitResult[1];
  hmiTxBuffer[11] = UART_TX_COMPLETE_FLAG; /* Good message check */

  /* Transmit data over UART */
  HAL_UART_Transmit(&huart1, hmiTxBuffer, HMI_BUFFER_SIZE, HAL_MAX_DELAY);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  /* Handle messages received from HMI */
  if (huart->Instance == USART1)
  {
    switch (hmiRxBuffer[0])
    {
      case 0U:
        /* Start ventilation */
        break;

      case 1U:
        /* Stop and reset */
        break;

      case 2U:
        /* Update ventilation states */
        break;

      case 3U:
        /* Calibrate system */
        break;

      case 4U:
        /* Return current system state */
        break;

      default:
        break;
    }

    /* Stop UART DMA stream */
    HAL_UART_DMAStop(&huart1);
  }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  /* Implement functionality if required */
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* Toggle the valves when the timer period elapses */
  if (htim->Instance == TIM3)
  {
    HAL_GPIO_TogglePin(GPIOB, Valve1_Pin);
    HAL_GPIO_TogglePin(GPIOB, Valve2_Pin);
  }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
  /* Check if the ADC instance is ADC1 */
  if (hadc->Instance == ADC1)
  {
    /* Convert ADC value to pressure in cm H2O */
    pressure = (((float)adcValues[0] * 10.0f) / (4096.0f - 2055.0f) - 10.0686f) * 10.1972f;

    /* Stop ADC DMA stream */
    HAL_ADC_Stop_DMA(&hadc1);
  }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  /* Handle external interrupt for ON and OFF pins */
  switch (GPIO_Pin)
  {
    case ON_Pin:
      runFlag = 1U;
      break;

    case OFF_Pin:
      /* Implement OFF functionality if required */
      __NOP();
      break;

    default:
      break;
  }
}
