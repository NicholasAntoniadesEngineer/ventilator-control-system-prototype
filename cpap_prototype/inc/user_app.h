/* USER CODE BEGIN Header */
/*
------------------------------------------------------------------------------
   Project  : Ventilator Control System
   File     : user_app.h
   Brief    : Header file for user application logic.
   Author   : Nicholas Antoniades
------------------------------------------------------------------------------
*/
/* USER CODE END Header */

#ifndef USER_APP_H
#define USER_APP_H

#include "bsp.h"

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

/* Application configuration type */
typedef struct {
    hardware_config_t hardware;
    breathing_config_t breathing;
    comm_config_t comm;
} app_config_t;

/* Application state type */
typedef struct {
    bsp_state_t bsp_state;         /* BSP state */
    hardware_config_t hardware;     /* Hardware configuration */
    breathing_config_t breathing;   /* Breathing parameters */
    comm_config_t comm;            /* Communication parameters */
} app_state_t;

/* Function prototypes */
void user_app_init(app_state_t *app_state, const app_config_t *app_config);
void user_app_start(app_state_t *app_state);
void user_app_run(app_state_t *app_state);

#endif /* USER_APP_H */
