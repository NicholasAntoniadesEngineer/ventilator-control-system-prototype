/*
 * library.c
 *
 * General utility and helper functions.
 *
 * Created on: 2020
 *     Author: Nicholas Antoniades
 */

#include "library.h"
#include "gpio.h"
#include "adc.h"
#include "usart.h"
#include "tim.h"
#include "stm32f4xx_hal.h"

void init_ports(void)
{
    // Initialize GPIO ports and pins
    // Example for GPIOA
    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // Configure PA0 as input with pull-up
    GPIO_InitStruct.Pin = GPIO_PIN_0;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    // Configure other pins as needed
}

void init_pwm(TIM_HandleTypeDef *htim, uint32_t channel, uint32_t frequency)
{
    // Initialize PWM output on specified timer and channel
    TIM_OC_InitTypeDef sConfigOC = {0};

    htim->Init.Prescaler = 0;
    htim->Init.CounterMode = TIM_COUNTERMODE_UP;
    htim->Init.Period = frequency;
    htim->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    HAL_TIM_PWM_Init(htim);

    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = frequency / 2; // 50% duty cycle
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    HAL_TIM_PWM_ConfigChannel(htim, &sConfigOC, channel);

    HAL_TIM_PWM_Start(htim, channel);
}

void init_adc(ADC_HandleTypeDef *hadc)
{
    // Initialize ADC with desired settings
    ADC_ChannelConfTypeDef sConfig = {0};

    hadc->Instance = ADC1;
    hadc->Init.Resolution = ADC_RESOLUTION_12B;
    HAL_ADC_Init(hadc);

    // Configure ADC channel
    sConfig.Channel = ADC_CHANNEL_0;
    sConfig.Rank = 1;
    HAL_ADC_ConfigChannel(hadc, &sConfig);
}

uint32_t read_adc(ADC_HandleTypeDef *hadc)
{
    // Read ADC value
    HAL_ADC_Start(hadc);
    HAL_ADC_PollForConversion(hadc, HAL_MAX_DELAY);
    return HAL_ADC_GetValue(hadc);
}

void delay_ms(uint32_t time)
{
    // Delay in milliseconds
    HAL_Delay(time);
}

uint8_t check_button(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
    // Check if button is pressed
    if (HAL_GPIO_ReadPin(GPIOx, GPIO_Pin) == GPIO_PIN_RESET)
    {
        HAL_Delay(50); // Debounce delay
        if (HAL_GPIO_ReadPin(GPIOx, GPIO_Pin) == GPIO_PIN_RESET)
        {
            return 1;
        }
    }
    return 0;
}

void init_usart(UART_HandleTypeDef *huart)
{
    // Initialize USART with desired settings
    huart->Instance = USART1;
    huart->Init.BaudRate = 115200;
    huart->Init.WordLength = UART_WORDLENGTH_8B;
    huart->Init.StopBits = UART_STOPBITS_1;
    huart->Init.Parity = UART_PARITY_NONE;
    huart->Init.Mode = UART_MODE_TX_RX;
    huart->Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart->Init.OverSampling = UART_OVERSAMPLING_16;
    HAL_UART_Init(huart);
}

void usart_transmit(UART_HandleTypeDef *huart, uint8_t data)
{
    // Transmit data over USART
    HAL_UART_Transmit(huart, &data, 1, HAL_MAX_DELAY);
}

uint8_t usart_receive(UART_HandleTypeDef *huart)
{
    // Receive data over USART
    uint8_t data;
    HAL_UART_Receive(huart, &data, 1, HAL_MAX_DELAY);
    return data;
} 