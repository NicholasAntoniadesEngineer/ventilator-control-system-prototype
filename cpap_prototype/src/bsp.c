/*
------------------------------------------------------------------------------
   Project  : Ventilator Control System
   File     : bsp.c
   Brief    : Source file containing the implementation of HAL functions.
   Author   : Nicholas Antoniades
------------------------------------------------------------------------------
*/

#include "bsp.h"
#include "lib.h"
#include "usart.h"
#include "adc.h"
#include "tim.h"
#include "gpio.h"
#include "main.h" // For definitions like Valve1_Pin, Valve2_Pin

/* Private variables ---------------------------------------------------------*/
static BSP_Context* bsp_context_ptr = NULL;  /**< Pointer to user application context */

/* Function Implementations --------------------------------------------------*/

void BSP_Init(BSP_Context* context)
{
    bsp_context_ptr = context;
}

void BSP_UART_Send_DMA(float pressureValue, uint8_t* txBuffer)
{
    /* Convert pressure to 8-bit result for UART transmission */
    Convert_To_EightBit(txBuffer, (uint32_t)pressureValue);

    /* Set UART transmit buffer flags */
    txBuffer[11] = UART_TX_COMPLETE_FLAG; /* Good message check */

    /* Transmit data over UART using DMA */
    HAL_UART_Transmit_DMA(&huart1, txBuffer, HMI_BUFFER_SIZE);
}

void BSP_UART_DMA_Stop(void)
{
    HAL_UART_DMAStop(&huart1);
}

void BSP_ADC_Start_DMA(uint32_t* adcValues, uint32_t num_channels)
{
    HAL_ADC_Start_DMA(&hadc1, adcValues, num_channels);
}

void BSP_ADC_Stop_DMA(void)
{
    HAL_ADC_Stop_DMA(&hadc1);
}

void BSP_TIM_Set_Prescaler(uint32_t prescaler)
{
    /* Set TIM3 Prescaler */
    __HAL_TIM_SET_PRESCALER(&htim3, prescaler);

    /* Reinitialize TIM3 to apply prescaler */
    if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
    {
        Error_Handler();
    }
}

void BSP_TIM_Start_IT(void)
{
    HAL_TIM_Base_Start_IT(&htim3);
}

void BSP_GPIO_WritePin(uint16_t Pin, GPIO_PinState PinState)
{
    HAL_GPIO_WritePin(GPIOB, Pin, PinState);
}

/* HAL Callback Implementations ------------------------------------------------*/

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    /* Handle messages received from HMI */
    if (huart->Instance == USART1 && bsp_context_ptr != NULL)
    {
        switch (bsp_context_ptr->hmiRxBuffer[0])
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
    if (hadc->Instance == ADC1 && bsp_context_ptr != NULL)
    {
        /* Convert ADC value to pressure in cm H2O */
        *(bsp_context_ptr->pressure) = (((float)(bsp_context_ptr->adcValues[0]) * 10.0f) / (4096.0f - 2055.0f) - 10.0686f) * 10.1972f;

        /* Stop ADC DMA stream */
        HAL_ADC_Stop_DMA(&hadc1);
    }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    /* Handle external interrupt for ON and OFF pins */
    if (bsp_context_ptr != NULL)
    {
        switch (GPIO_Pin)
        {
            case ON_Pin:
                *(bsp_context_ptr->runFlag) = 1U;
                break;

            case OFF_Pin:
                /* Implement OFF functionality if required */
                __NOP();
                break;

            default:
                break;
        }
    }
}
