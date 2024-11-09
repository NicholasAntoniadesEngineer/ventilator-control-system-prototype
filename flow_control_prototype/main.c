/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main application code for ventilator control system
  * @date           : 2020
  * @author         : Nicholas Antoniades
  ******************************************************************************
  */

#include "main.h"

/* Private variables */
static struct system_state state;

/* Private function definitions */
static void system_init(void)
{
    BSP_HAL_Init();

    sensors_init(&state.sensors);
    uart_init(&state.uart);
    ventilator_init(&state.ventilator);
    timers_init(&state.timers);
}

static void main_loop(void)
{
    while (1)
    {
        sensors_read_flow(&state.sensors);
        sensors_read_pressure(&state.sensors);
        ventilator_update_state(&state.ventilator, &state.sensors);
        uart_handle_communication(&state.uart);
    }
}

/* Main function */
int main(void)
{
    system_init();
    main_loop();
    return 0;
}