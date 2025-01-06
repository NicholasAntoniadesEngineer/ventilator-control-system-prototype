/* USER CODE BEGIN Header */
/*
------------------------------------------------------------------------------
   Project  : Ventilator Control System
   File     : user_app.c
   Brief    : Source file for user application logic.
   Author   : Nicholas Antoniades
------------------------------------------------------------------------------
*/
/* USER CODE END Header */

#include "user_app.h"
#include "bsp.h"

/**
 * @brief Initializes the application state and peripherals.
 *
 * @param state Pointer to the application state.
 * @param config Pointer to the application configuration.
 */
void user_app_init(app_state_t *app_state, const app_config_t *app_config)
{
    /* Initialize the state structure values */
    memset(&app_state->bsp_state.hmiTxBuffer, 0, app_config->comm.uart_buffer_size);
    memset(&app_state->bsp_state.hmiRxBuffer, 0, app_config->comm.uart_buffer_size);
    memset(&app_state->bsp_state.adcValues, 0, sizeof(app_state->bsp_state.adcValues));
    app_state->bsp_state.pressure = 0.0f;
    app_state->bsp_state.toggleValue = 0U;
    app_state->bsp_state.runFlag = 0U;

    /* Copy config values to state */
    app_state->hardware = app_config->hardware;
    app_state->breathing = app_config->breathing;
    app_state->comm = app_config->comm;

    /* Initialize BSP with state */
    bsp_init(&app_state->bsp_state);
}

/**
 * @brief Starts the application, sets initial configurations and states.
 *
 * @param state Pointer to the application state.
 */
void user_app_start(app_state_t *app_state)
{
    /* Wait until the run flag is set by external conditions */
    while (app_state->bsp_state.runFlag == 0U)
    {
        __NOP();
    }

    /* Configure TIM3 for valve control and start its interrupt */
    bsp_start_timer3(&app_state->bsp_state, app_state->hardware.timer_prescaler);

    /* Set initial valve conditions */
    bsp_set_valve_state(&app_state->bsp_state, 
                       app_state->hardware.valve_port, 
                       app_state->hardware.valve1_pin, 
                       GPIO_PIN_RESET);
    bsp_set_valve_state(&app_state->bsp_state, 
                       app_state->hardware.valve_port, 
                       app_state->hardware.valve2_pin, 
                       GPIO_PIN_SET);
}

/**
 * @brief Main application loop that runs continuously.
 *
 * @param state Pointer to the application state.
 */
void user_app_run(app_state_t *app_state)
{
    if (app_state->bsp_state.toggleValue == 1U)
    {
        bsp_stop_adc_dma(&app_state->bsp_state);
        bsp_send_uart_dma(&app_state->bsp_state);
        app_state->bsp_state.toggleValue = 0U;
    }
    else
    {
        bsp_stop_uart_dma(&app_state->bsp_state);
        bsp_start_adc_dma(&app_state->bsp_state);
        app_state->bsp_state.toggleValue = 1U;
    }
}
