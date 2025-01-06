#ifndef VENTILATOR_STATE_H_
#define VENTILATOR_STATE_H_

#include <stdint.h>
#include "stm32_bsp.h"

/* PWM state structure */
struct pwm_state 
{
    double arr_val;
    double arr_bottom;
    double arr_top;
    TIM_HandleTypeDef* timer;
    uint32_t channel;
};

/* Timing state structure */
struct timing_state 
{
    uint32_t pause_delay_ms;
    uint32_t plateau_delay_ms;
    uint32_t pause_delay_counter;
    uint32_t plateau_delay_counter;
};

/* Breathing control state */
struct breathing_control 
{
    uint32_t bpm_max;
    double bpm_control;
    double volume_control;
    uint32_t tidal_counter;
    uint32_t tidal_input_steps;
    uint32_t tidal_step_multiplier;
    uint8_t direction_indicator;
};

/* Sensor state */
struct sensor_state 
{
    int32_t bellow_pressure;
    int32_t flow_sensor;
    uint8_t initialized;
};

/* Communication state */
struct uart_state 
{
    uint32_t counter;
    uint32_t frequency;
    uint8_t tx_buffer[BSP_HMI_BUFFER_SIZE];
    uint8_t rx_buffer[BSP_HMI_BUFFER_SIZE];
};

/* Complete ventilator state */
struct ventilator_state 
{
    struct pwm_state pwm;
    struct timing_state timing;
    struct breathing_control breathing;
    struct sensor_state sensors;
    struct uart_state uart;
    bsp_state_t bsp;
};

#endif /* VENTILATOR_STATE_H_ */ 