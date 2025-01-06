/*
------------------------------------------------------------------------------
   Project  : Ventilator Control System
   File     : bsp.c
   Brief    : Source file containing the implementation of HAL functions.
   Author   : Nicholas Antoniades
------------------------------------------------------------------------------
*/

#include "bsp.h"
#include "adc.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private defines */
#define ADC_TO_PRESSURE_FACTOR (10.0f / (4096.0f - 2055.0f))

/* Private state pointer for callbacks */
static bsp_state_t* current_bsp_state = NULL;

void bsp_init(bsp_state_t* bsp_state)
{
    current_bsp_state = bsp_state;

    /* Initialize peripherals */
    MX_GPIO_Init();
    MX_DMA_Init();
    MX_ADC1_Init();
    MX_USART1_UART_Init();
    MX_TIM3_Init();
}

void bsp_start_timer3(bsp_state_t* bsp_state, uint32_t prescaler)
{
    __HAL_TIM_SET_PRESCALER(&htim3, prescaler);
    HAL_TIM_Base_Start_IT(&htim3);
}

void bsp_set_valve_state(bsp_state_t* bsp_state, GPIO_TypeDef* port, uint16_t pin, GPIO_PinState state)
{
    HAL_GPIO_WritePin(port, pin, state);
}

void bsp_start_adc_dma(bsp_state_t* bsp_state)
{
    HAL_ADC_Start_DMA(&hadc1, bsp_state->adcValues, NUM_ADC_CHANNELS);
}

void bsp_stop_adc_dma(bsp_state_t* bsp_state)
{
    HAL_ADC_Stop_DMA(&hadc1);
}

void bsp_send_uart_dma(bsp_state_t* bsp_state)
{
    memcpy(bsp_state->hmiTxBuffer, &bsp_state->pressure, sizeof(float));
    bsp_state->hmiTxBuffer[HMI_BUFFER_SIZE - 1] = UART_TX_COMPLETE_FLAG;
    HAL_UART_Transmit_DMA(&huart1, bsp_state->hmiTxBuffer, HMI_BUFFER_SIZE);
}

void bsp_stop_uart_dma(bsp_state_t* bsp_state)
{
    HAL_UART_DMAStop(&huart1);
}

/* HAL Callbacks */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    if (current_bsp_state != NULL && hadc->Instance == ADC1)
    {
        current_bsp_state->pressure = (float)current_bsp_state->adcValues[0] * ADC_TO_PRESSURE_FACTOR;
        bsp_stop_adc_dma(current_bsp_state);
    }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (current_bsp_state != NULL && huart->Instance == USART1)
    {
        bsp_stop_uart_dma(current_bsp_state);
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (current_bsp_state != NULL && htim->Instance == TIM3)
    {
        if (current_bsp_state->pressure < current_bsp_state->pressure_threshold)
        {
            HAL_GPIO_TogglePin(GPIOB, Valve1_Pin);
            HAL_GPIO_WritePin(GPIOB, Valve2_Pin, GPIO_PIN_RESET);
        }
        else
        {
            HAL_GPIO_WritePin(GPIOB, Valve1_Pin, GPIO_PIN_RESET);
            HAL_GPIO_TogglePin(GPIOB, Valve2_Pin);
        }
    }
}
