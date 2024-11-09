/*
 * main.c
 *
 * Main application code for the ventilator control system.
 *
 * Created on: 2020
 *     Author: Nicholas Antoniades
 */

#include "main.h"
#include "config.h"
#include "ventilator.h"
#include "sensors.h"
#include "uart.h"
#include "timers.h"

// Global configuration structure
VentilatorConfig ventilator_config;

// Function prototypes
void system_init(void);
void main_loop(void);

void system_init(void)
{
    // HAL initialization
    HAL_Init();
    SystemClock_Config();

    // Initialize peripherals
    MX_GPIO_Init();
    MX_DMA_Init();
    MX_ADC1_Init();
    MX_I2C1_Init();
    MX_USART1_UART_Init();
    MX_USART2_UART_Init();
    MX_I2C3_Init();
    MX_DAC_Init();
    MX_RTC_Init();
    MX_USART3_UART_Init();

    // Initialize configurations
    ventilator_init(&ventilator_config);

    // Initialize other modules
    sensors_init();
    uart_init();
    timers_init();
}

int main(void)
{
    // Initialize the system
    system_init();

    // Enter main loop
    main_loop();

    // Should never reach here
    return 0;
}

void main_loop(void)
{
    while (1)
    {
        // Read sensors
        sensors_read_flow(&ventilator_config);

        sensors_read_pressure(&ventilator_config);

        // Update ventilator state
        ventilator_update_state(&ventilator_config);

        // Handle UART communication
        uart_handle_communication();

        // Toggle between ADC and UART DMA streams
        toggle_dma_streams();
    }
}

