/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main application code for CPAP ventilator control system
  * @date           : 2020
  * @author         : Nicholas Antoniades
  ******************************************************************************
  */

#include "main.h"
#include "stm32_bsp.h"
#include "user_app.h"
#include "adc.h"
#include "dma.h"
#include "rtc.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private variables */
static struct system_state state;

const struct app_config vent_config = {
    .hardware = {
        .timer_prescaler = 17000U,
        .adc_channels = NUM_ADC_CHANNELS,
        .valve1_pin = Valve1_Pin,
        .valve2_pin = Valve2_Pin,
        .valve_port = GPIOB,
        .hadc = &hadc1,
        .huart = &huart1,
        .htim = &htim3
    },
    .breathing = {
        .pressure_min = -10.0f,
        .pressure_max = 20.0f,
        .pressure_default = 5.0f,
        .sample_rate_ms = 50U
    },
    .communication = {
        .uart_buffer_size = HMI_BUFFER_SIZE,
        .uart_tx_flag = UART_TX_COMPLETE_FLAG
    }
};

static void system_init(void)
{
    /* MCU Configuration */
    HAL_Init();
    SystemClock_Config();

    /* Initialize BSP and peripherals */
    BSP_HAL_Init();
    
    /* Initialize application state with config */
    user_app_init(&state.user_app, &vent_config);
}

static void main_loop(void)
{
    /* Start the application */
    user_app_start(&state.user_app);
    
    /* Main loop */
    while (1)
    {
        /* Read sensors and update state */
        BSP_ADC_Start_DMA(state.user_app.adcValues, 
                         state.user_app.hardware.adc_channels);

        /* Update application state */
        user_app_run(&state.user_app);

        /* Handle communication */
        if (state.user_app.toggleValue)
        {
            BSP_UART_Send_DMA(state.user_app.pressure, 
                             state.user_app.hmiTxBuffer);
        }

        /* Delay for next sample */
        HAL_Delay(state.user_app.breathing.sample_rate_ms);
    }
}

int main(void)
{
    system_init();
    main_loop();
    return 0;
}

