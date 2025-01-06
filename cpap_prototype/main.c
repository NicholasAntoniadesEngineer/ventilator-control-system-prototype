/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main application code for CPAP ventilator control system
  * @date           : 2024
  * @author         : Nicholas Antoniades
  ******************************************************************************
  */

#include "main.h"
#include "stm32_bsp.h"
#include "ventilator_control.h"

/* Private variables */
static struct system_state state;

const struct ventilator_config vent_config = {
    .hardware = {
        .timer_prescaler = 17000U,
        .adc_channels = BSP_NUM_ADC_CHANNELS,
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
        .uart_buffer_size = BSP_HMI_BUFFER_SIZE,
        .uart_tx_flag = BSP_UART_TX_COMPLETE_FLAG
    }
};

static void system_init(void)
{
    /* Initialize BSP and peripherals */
    BSP_HAL_Init();
    
    /* Initialize application state with config */
    ventilator_control_init(&state.ventilator_control, &vent_config);
}

static void main_loop(void)
{
    /* Start the application */
    ventilator_control_start(&state.ventilator_control);
    
    /* Main loop */
    while (1)
    {
        /* Read sensors and update state */
        BSP_ADC_Start_DMA(&hadc1, (uint32_t*)state.ventilator_control.bsp_state.adcValues, 
                         BSP_NUM_ADC_CHANNELS);

        /* Update application state */
        ventilator_control_run(&state.ventilator_control);

        /* Handle communication */
        if (state.ventilator_control.bsp_state.toggleValue)
        {
            uint8_t tx_buffer[BSP_HMI_BUFFER_SIZE];
            memcpy(tx_buffer, &state.ventilator_control.bsp_state.pressure, sizeof(float));
            tx_buffer[BSP_HMI_BUFFER_SIZE - 1] = BSP_UART_TX_COMPLETE_FLAG;
            BSP_UART_Transmit_DMA(&huart1, tx_buffer, BSP_HMI_BUFFER_SIZE);
        }

        /* Delay for next sample */
        BSP_Delay(state.ventilator_control.breathing.sample_rate_ms);
    }
}

int main(void)
{
    system_init();
    main_loop();
    return 0;
}

