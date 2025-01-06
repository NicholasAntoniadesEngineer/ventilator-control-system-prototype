/* USER CODE BEGIN Header */
/*
------------------------------------------------------------------------------
   Project  : Ventilator Control System
   File     : ventilator_control.h
   Brief    : Header file for ventilator control logic.
   Author   : Nicholas Antoniades
------------------------------------------------------------------------------
*/
/* USER CODE END Header */

#ifndef VENTILATOR_CONTROL_H
#define VENTILATOR_CONTROL_H

#include "stm32_bsp.h"

/* Hardware configuration type */
typedef struct {
    uint32_t timer_prescaler;
    uint32_t adc_channels;
    GPIO_TypeDef* valve_port;
    uint16_t valve1_pin;
    uint16_t valve2_pin;
    ADC_HandleTypeDef* hadc;      
    UART_HandleTypeDef* huart;    
    TIM_HandleTypeDef* htim;      
} hardware_config_t;

/* Breathing parameters type */
typedef struct {
    float pressure_min;
    float pressure_max;
    float pressure_default;
    uint32_t sample_rate_ms;
} breathing_config_t;

/* Communication parameters type */
typedef struct {
    uint32_t uart_buffer_size;
    uint8_t uart_tx_flag;
} comm_config_t;

/* Function prototypes */
void ventilator_control_init(app_state_t *app_state, const app_config_t *app_config);
void ventilator_control_start(app_state_t *app_state);
void ventilator_control_run(app_state_t *app_state);

#endif /* VENTILATOR_CONTROL_H */ 