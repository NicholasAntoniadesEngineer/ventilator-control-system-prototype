/* USER CODE BEGIN Header */
/*
------------------------------------------------------------------------------
   Project  : Ventilator Control System
   File     : lib.h
   Brief    : Header file containing function prototypes and macros for lib.c.
   Author   : Nicholas Antoniades
------------------------------------------------------------------------------
*/
/* USER CODE END Header */

#ifndef LIB_H
#define LIB_H

#include "main.h"

/* Public macros -------------------------------------------------------------*/
#define HMI_BUFFER_SIZE        12U   /**< UART buffer size */
#define NUM_ADC_CHANNELS       1U    /**< Number of ADC channels */
#define UART_TX_COMPLETE_FLAG  255U  /**< UART transmission complete flag */

/* Public function prototypes ------------------------------------------------*/

/**
  * @brief  Converts a 32-bit value to four 8-bit values.
  * @param  resultArray: Pointer to the result array.
  * @param  value: 32-bit value to convert.
  * @retval None.
  */
void Convert_To_EightBit(uint8_t *resultArray, uint32_t value);

/**
  * @brief  Sends data over UART using DMA.
  * @param  pressureValue: Pressure value to send.
  * @retval None.
  */
void UART_Send_DMA(float pressureValue);

/**
  * @brief  UART receive complete callback.
  * @param  huart: UART handle.
  * @retval None.
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);

/**
  * @brief  UART transmission complete callback.
  * @param  huart: UART handle.
  * @retval None.
  */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart);

/**
  * @brief  Timer period elapsed callback.
  * @param  htim: TIM handle.
  * @retval None.
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);

/**
  * @brief  ADC conversion complete callback.
  * @param  hadc: ADC handle.
  * @retval None.
  */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc);

/**
  * @brief  EXTI line detection callbacks.
  * @param  GPIO_Pin: Specifies the pins connected EXTI line.
  * @retval None.
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);

#endif /* LIB_H */
