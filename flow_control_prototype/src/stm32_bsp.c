/**
  ******************************************************************************
  * @file           : stm32_bsp.c
  * @brief          : Board Support Package implementation
  * @details        : Encapsulates HAL and BSP functions for easier management
  * @date           : 2020
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

void BSP_HAL_Init(void)
{
    HAL_Init();
    
    // Initialize peripherals
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

void BSP_GPIO_WritePin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, GPIO_PinState PinState)
{
    HAL_GPIO_WritePin(GPIOx, GPIO_Pin, PinState);
}

GPIO_PinState BSP_GPIO_ReadPin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
    return HAL_GPIO_ReadPin(GPIOx, GPIO_Pin);
}

void BSP_TIM_Base_Start(TIM_HandleTypeDef* htim)
{
    HAL_TIM_Base_Start(htim);
}

void BSP_TIM_Base_Stop(TIM_HandleTypeDef* htim)
{
    HAL_TIM_Base_Stop(htim);
}

void BSP_TIM_Base_Init(TIM_HandleTypeDef* htim)
{
    HAL_TIM_Base_Init(htim);
}

void BSP_UART_Transmit(UART_HandleTypeDef *huart, uint8_t *data, uint16_t size)
{
    HAL_UART_Transmit(huart, data, size, HAL_MAX_DELAY);
}

void BSP_UART_Receive(UART_HandleTypeDef *huart, uint8_t *data, uint16_t size)
{
    HAL_UART_Receive(huart, data, size, HAL_MAX_DELAY);
}

void BSP_I2C_Read(I2C_HandleTypeDef *hi2c, uint16_t dev_addr, uint16_t reg_addr, uint8_t *data, uint16_t size)
{
    HAL_I2C_Mem_Read(hi2c, dev_addr, reg_addr, I2C_MEMADD_SIZE_8BIT, data, size, HAL_MAX_DELAY);
}

void BSP_I2C_Write(I2C_HandleTypeDef *hi2c, uint16_t dev_addr, uint16_t reg_addr, uint8_t *data, uint16_t size)
{
    HAL_I2C_Mem_Write(hi2c, dev_addr, reg_addr, I2C_MEMADD_SIZE_8BIT, data, size, HAL_MAX_DELAY);
}

void BSP_ADC_Start(ADC_HandleTypeDef* hadc)
{
    HAL_ADC_Start(hadc);
}

void BSP_ADC_Stop(ADC_HandleTypeDef* hadc)
{
    HAL_ADC_Stop(hadc);
}

uint32_t BSP_ADC_ReadValue(ADC_HandleTypeDef* hadc)
{
    HAL_ADC_PollForConversion(hadc, HAL_MAX_DELAY);
    return HAL_ADC_GetValue(hadc);
}

uint32_t BSP_GetTick(void)
{
    return HAL_GetTick();
}