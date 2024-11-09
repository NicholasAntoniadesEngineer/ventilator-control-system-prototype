/**
  ******************************************************************************
  * @file           : stm32_bsp.h
  * @brief          : Board Support Package header file
  * @details        : Encapsulates HAL and BSP functions for easier management
  * @date           : 2020
  * @author         : Nicholas Antoniades
  ******************************************************************************
  */

#ifndef STM32_BSP_H_
#define STM32_BSP_H_

#include "stm32f4xx_hal.h"
#include <stdint.h>

/* Function Prototypes */
void BSP_HAL_Init(void);
void BSP_GPIO_WritePin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, GPIO_PinState PinState);
GPIO_PinState BSP_GPIO_ReadPin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
void BSP_TIM_Base_Start(TIM_HandleTypeDef* htim);
void BSP_TIM_Base_Stop(TIM_HandleTypeDef* htim);
void BSP_TIM_Base_Init(TIM_HandleTypeDef* htim);
void BSP_UART_Transmit(UART_HandleTypeDef *huart, uint8_t *data, uint16_t size);
void BSP_UART_Receive(UART_HandleTypeDef *huart, uint8_t *data, uint16_t size);
void BSP_I2C_Read(I2C_HandleTypeDef *hi2c, uint16_t dev_addr, uint16_t reg_addr, uint8_t *data, uint16_t size);
void BSP_I2C_Write(I2C_HandleTypeDef *hi2c, uint16_t dev_addr, uint16_t reg_addr, uint8_t *data, uint16_t size);
void BSP_ADC_Start(ADC_HandleTypeDef* hadc);
void BSP_ADC_Stop(ADC_HandleTypeDef* hadc);
uint32_t BSP_ADC_ReadValue(ADC_HandleTypeDef* hadc);
uint32_t BSP_GetTick(void);

/* Handle Declarations */
extern ADC_HandleTypeDef hadc1;
extern I2C_HandleTypeDef hi2c1;
extern I2C_HandleTypeDef hi2c3;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern DAC_HandleTypeDef hdac;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;

#endif /* STM32_BSP_H_ */ 