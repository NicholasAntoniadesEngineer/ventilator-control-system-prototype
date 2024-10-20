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
 * @brief Initializes the user application context and peripherals.
 *
 * @param context Pointer to the user application context.
 */
void user_app_init(UserAppContext *context)
{
    /* Initialize the context structure values */
    memset(context->hmiTxBuffer, 0, HMI_BUFFER_SIZE);
    memset(context->hmiRxBuffer, 0, HMI_BUFFER_SIZE);
    memset(context->adcValues, 0, sizeof(context->adcValues));
    context->pressure = 0.0f;
    context->toggleValue = 0U;
    context->runFlag = 0U;

    /* Additional user-specific initialization can be added here */

    /* Initialize hardware using BSP */
    bsp_init();
}

/**
 * @brief Starts the user application, sets initial configurations and states.
 *
 * @param context Pointer to the user application context.
 */
void user_app_start(UserAppContext *context)
{
    /* Wait until the run flag is set by external conditions */
    while (context->runFlag == 0U)
    {
        __NOP();  /* Do nothing, waiting for run flag to be set */
    }

    /* Configure TIM3 for valve control and start its interrupt */
    bsp_start_timer3(17000U);

    /* Set initial valve conditions */
    bsp_set_valve_state(GPIOB, Valve1_Pin, GPIO_PIN_RESET);
    bsp_set_valve_state(GPIOB, Valve2_Pin, GPIO_PIN_SET);
}

/**
 * @brief Main application loop that runs continuously.
 *
 * @param context Pointer to the user application context.
 */
void user_app_run(UserAppContext *context)
{
    while (1)
    {
        HAL_Delay(50U);

        /* Toggle between sending UART data and reading ADC values */
        if (context->toggleValue == 1U)
        {
            bsp_stop_adc_dma();
            bsp_send_uart_dma(context->pressure);  /* Send pressure over UART */
            context->toggleValue = 0U;
        }
        else
        {
            bsp_stop_uart_dma();
            bsp_start_adc_dma(context->adcValues, NUM_ADC_CHANNELS);  /* Read ADC values */
            context->toggleValue = 1U;
        }
    }
}
