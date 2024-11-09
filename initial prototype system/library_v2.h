/*
 * library.h
 *
 * Header file for utility functions.
 *
 * Created on: 2020
 *     Author: Nicholas Antoniades
 */

#ifndef LIBRARY_H_
#define LIBRARY_H_

#include "stm32f4xx_hal.h"
#include "stdint.h"

// GPIO Functions
void init_ports(void);
uint8_t check_button(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);

// PWM Functions
void init_pwm(TIM_HandleTypeDef *htim, uint32_t channel, uint32_t frequency);

// ADC Functions
void init_adc(ADC_HandleTypeDef *hadc);
uint32_t read_adc(ADC_HandleTypeDef *hadc);

// USART Functions
void init_usart(UART_HandleTypeDef *huart);
void usart_transmit(UART_HandleTypeDef *huart, uint8_t data);
uint8_t usart_receive(UART_HandleTypeDef *huart);

// Utility Functions
void delay_ms(uint32_t time);

#endif /* LIBRARY_H_ */ 