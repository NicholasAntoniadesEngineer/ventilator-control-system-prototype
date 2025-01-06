/*
 ******************************************************************************
 * @file           : main.h
 * @brief          : Header file for CPAP ventilator control system
 * @date           : 2024
 * @author         : Nicholas Antoniades
 ******************************************************************************
 */
#ifndef MAIN_H_
#define MAIN_H_

#include "user_app.h"
#include "stm32_bsp.h"

/* Hardware configuration defaults */
#define HARDWARE_TIMER_PRESCALER    17000U
#define HARDWARE_ADC_CHANNELS       BSP_NUM_ADC_CHANNELS
#define HARDWARE_VALVE_PORT         GPIOB
#define HARDWARE_VALVE1_PIN         Valve1_Pin
#define HARDWARE_VALVE2_PIN         Valve2_Pin

/* Breathing parameters defaults */
#define BREATHING_PRESSURE_MIN      -10.0f
#define BREATHING_PRESSURE_MAX      20.0f
#define BREATHING_PRESSURE_DEFAULT  5.0f
#define BREATHING_SAMPLE_RATE_MS    50U

/* Communication parameters defaults */
#define COMM_BUFFER_SIZE           BSP_HMI_BUFFER_SIZE
#define COMM_TX_FLAG               BSP_UART_TX_COMPLETE_FLAG

/* System state type */
typedef struct {
    app_state_t app_state;
} system_state_t;

/* Function prototypes */
void Error_Handler(void);

#endif /* MAIN_H_ */

