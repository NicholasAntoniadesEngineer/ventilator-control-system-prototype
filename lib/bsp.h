/* USER CODE BEGIN Header */
/*
------------------------------------------------------------------------------
   Project  : Ventilator Control System
   File     : bsp.h
   Brief    : Header file for Board Support Package (HAL) functions.
   Author   : Nicholas Antoniades
------------------------------------------------------------------------------
*/
/* USER CODE END Header */

#ifndef BSP_H
#define BSP_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "lib.h"
#include "usart.h"
#include "adc.h"
#include "tim.h"
#include "gpio.h"
#include "stm32f4xx_hal.h"  // Adjust according to your STM32 series

/** @brief Size of the HMI UART transmit buffer */
#define HMI_BUFFER_SIZE      12U  /**< Adjust as per your protocol */

/** @brief Number of ADC channels */
#define NUM_ADC_CHANNELS     3U   /**< Adjust based on your ADC configuration */

/** @brief UART transmit complete flag */
#define UART_TX_COMPLETE_FLAG 255U /**< Example flag value */


/* Forward declaration of BSP_Context */
typedef struct {
    uint8_t* hmiTxBuffer;
    uint8_t* hmiRxBuffer;
    uint32_t* adcValues;
    float* pressure;
    uint8_t* toggleValue;
    uint8_t* runFlag;
} BSP_Context;

/* Function Prototypes */

/**
 * @brief Initialize the BSP with the user application context.
 *
 * @param context Pointer to the BSP_Context structure containing pointers to user data.
 */
void BSP_Init(BSP_Context* context);

/**
 * @brief Send pressure data via UART using DMA.
 *
 * @param pressureValue The pressure value to send.
 * @param txBuffer Pointer to the UART transmit buffer.
 */
void BSP_UART_Send_DMA(float pressureValue, uint8_t* txBuffer);

/**
 * @brief Stop UART DMA stream.
 */
void BSP_UART_DMA_Stop(void);

/**
 * @brief Start ADC DMA stream.
 *
 * @param adcValues Pointer to the ADC values buffer.
 * @param num_channels Number of ADC channels.
 */
void BSP_ADC_Start_DMA(uint32_t* adcValues, uint32_t num_channels);

/**
 * @brief Stop ADC DMA stream.
 */
void BSP_ADC_Stop_DMA(void);

/**
 * @brief Set the TIM3 prescaler.
 *
 * @param prescaler The prescaler value to set.
 */
void BSP_TIM_Set_Prescaler(uint32_t prescaler);

/**
 * @brief Start TIM3 with interrupts.
 */
void BSP_TIM_Start_IT(void);

/**
 * @brief Write to a GPIO pin.
 *
 * @param Pin The GPIO pin number.
 * @param PinState The state to set the pin to (e.g., GPIO_PIN_SET or GPIO_PIN_RESET).
 */
void BSP_GPIO_WritePin(uint16_t Pin, GPIO_PinState PinState);

#ifdef __cplusplus
}
#endif

#endif /* BSP_H */
