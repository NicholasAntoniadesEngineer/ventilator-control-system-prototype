/**
  ******************************************************************************
  * @file           : stm32_bsp.c
  * @brief          : Combined Board Support Package implementation
  * @details        : Encapsulates HAL and BSP functions for all prototypes
  * @date           : 2024
  * @author         : Nicholas Antoniades
  ******************************************************************************
  */

#include "stm32_bsp.h"
#include "gpio.h"
#include "tim.h"
#include "dac.h"
#include "usart.h"
#include "adc.h"
#include "i2c.h"
#include "dma.h"

/* Initialization Functions */
void stm32_bsp_hal_init(void)
{
    HAL_Init();
    
    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_DMA_Init();
    MX_ADC1_Init();
    MX_I2C1_Init();
    MX_USART1_UART_Init();
    MX_USART2_UART_Init();
    MX_I2C3_Init();
    MX_DAC_Init();
    MX_TIM2_Init();
    MX_TIM3_Init();
    MX_TIM4_Init();
    MX_USART3_UART_Init();
}

void stm32_bsp_init_state(bsp_state_t* state)
{
    /* Initialize state structure with default values */
    memset(state, 0, sizeof(bsp_state_t));
    state->pressure_threshold = 0.0f;
    state->runFlag = 0;
    state->toggleValue = 0;
}

/* GPIO Functions */
void stm32_bsp_gpio_writepin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, GPIO_PinState PinState)
{
    HAL_GPIO_WritePin(GPIOx, GPIO_Pin, PinState);
}

GPIO_PinState stm32_bsp_gpio_readpin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
    return HAL_GPIO_ReadPin(GPIOx, GPIO_Pin);
}

/* Timer Functions */
void stm32_bsp_tim_base_start(TIM_HandleTypeDef* htim)
{
    HAL_TIM_Base_Start(htim);
}

void stm32_bsp_tim_base_stop(TIM_HandleTypeDef* htim)
{
    HAL_TIM_Base_Stop(htim);
}

void stm32_bsp_tim_base_init(TIM_HandleTypeDef* htim)
{
    HAL_TIM_Base_Init(htim);
}

void stm32_bsp_tim_pwm_init(TIM_HandleTypeDef* htim, uint32_t channel, uint32_t frequency)
{
    /* Configure timer for PWM mode */
    TIM_OC_InitTypeDef sConfigOC = {0};
    
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = frequency / 2;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    
    HAL_TIM_PWM_ConfigChannel(htim, &sConfigOC, channel);
    HAL_TIM_PWM_Start(htim, channel);
}

/* UART Functions */
void stm32_bsp_uart_transmit(UART_HandleTypeDef *huart, uint8_t *data, uint16_t size)
{
    HAL_UART_Transmit(huart, data, size, HAL_MAX_DELAY);
}

void stm32_bsp_uart_receive(UART_HandleTypeDef *huart, uint8_t *data, uint16_t size)
{
    HAL_UART_Receive(huart, data, size, HAL_MAX_DELAY);
}

void stm32_bsp_uart_transmit_dma(UART_HandleTypeDef *huart, uint8_t *data, uint16_t size)
{
    HAL_UART_Transmit_DMA(huart, data, size);
}

void stm32_bsp_uart_receive_dma(UART_HandleTypeDef *huart, uint8_t *data, uint16_t size)
{
    HAL_UART_Receive_DMA(huart, data, size);
}

/* I2C Functions */
void stm32_bsp_i2c_read(I2C_HandleTypeDef *hi2c, uint16_t dev_addr, uint16_t reg_addr, uint8_t *data, uint16_t size)
{
    HAL_I2C_Mem_Read(hi2c, dev_addr, reg_addr, I2C_MEMADD_SIZE_8BIT, data, size, HAL_MAX_DELAY);
}

void stm32_bsp_i2c_write(I2C_HandleTypeDef *hi2c, uint16_t dev_addr, uint16_t reg_addr, uint8_t *data, uint16_t size)
{
    HAL_I2C_Mem_Write(hi2c, dev_addr, reg_addr, I2C_MEMADD_SIZE_8BIT, data, size, HAL_MAX_DELAY);
}

/* ADC Functions */
void stm32_bsp_adc_start(ADC_HandleTypeDef* hadc)
{
    HAL_ADC_Start(hadc);
}

void stm32_bsp_adc_stop(ADC_HandleTypeDef* hadc)
{
    HAL_ADC_Stop(hadc);
}

void stm32_bsp_adc_start_dma(ADC_HandleTypeDef* hadc, uint32_t* buffer, uint32_t length)
{
    HAL_ADC_Start_DMA(hadc, buffer, length);
}

void stm32_bsp_adc_stop_dma(ADC_HandleTypeDef* hadc)
{
    HAL_ADC_Stop_DMA(hadc);
}

uint32_t stm32_bsp_adc_readvalue(ADC_HandleTypeDef* hadc)
{
    HAL_ADC_PollForConversion(hadc, HAL_MAX_DELAY);
    return HAL_ADC_GetValue(hadc);
}

/* System Functions */
uint32_t stm32_bsp_gettick(void)
{
    return HAL_GetTick();
}

void stm32_bsp_delay(uint32_t delay)
{
    HAL_Delay(delay);
} 