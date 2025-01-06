/* USER CODE BEGIN Header */
/*
------------------------------------------------------------------------------
   Project  : Ventilator Control System
   File     : ventilator_control.c
   Brief    : Source file for ventilator control logic.
   Author   : Nicholas Antoniades
------------------------------------------------------------------------------
*/
/* USER CODE END Header */

#include "ventilator_control.h"
#include "bsp.h"

/**
 * @brief Initializes the application state and peripherals.
 *
 * @param state Pointer to the application state.
 * @param config Pointer to the application configuration.
 */
void ventilator_control_init(app_state_t *app_state, const app_config_t *app_config)
{
    /* Initialize the state structure values */
    memset(&app_state->bsp_state.hmiTxBuffer, 0, app_config->comm.uart_buffer_size);
    memset(&app_state->bsp_state.hmiRxBuffer, 0, app_config->comm.uart_buffer_size);
    memset(&app_state->bsp_state.adcValues, 0, sizeof(app_state->bsp_state.adcValues));
    app_state->bsp_state.pressure = 0.0f;
    app_state->bsp_state.toggleValue = 0U;
    app_state->bsp_state.runFlag = 0U;
}

void ventilator_control_start(app_state_t *app_state)
{
    app_state->bsp_state.runFlag = 1U;
    BSP_TIM_Start_IT(app_state->hardware.htim);
}

void ventilator_control_run(app_state_t *app_state)
{
    if (app_state->bsp_state.runFlag)
    {
        /* Update pressure calculations */
        float pressure = BSP_Calculate_Pressure(app_state->bsp_state.adcValues[0]);
        app_state->bsp_state.pressure = pressure;

        /* Control valves based on pressure */
        if (pressure > app_state->breathing.pressure_max)
        {
            BSP_GPIO_WritePin(app_state->hardware.valve_port, 
                            app_state->hardware.valve1_pin, GPIO_PIN_RESET);
        }
        else if (pressure < app_state->breathing.pressure_min)
        {
            BSP_GPIO_WritePin(app_state->hardware.valve_port, 
                            app_state->hardware.valve2_pin, GPIO_PIN_SET);
        }
    }
} 