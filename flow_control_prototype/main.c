/*
 * main.c
 *
 * Main application code for the ventilator control system.
 *
 * Created on: 2020
 *     Author: Nicholas Antoniades
 */

#include "stm32_bsp.h"
#include "sensors.h"
#include "uart.h"
#include "timers.h"
#include "ventilator.h"

struct system_state {
    struct sensor_state sensors;
    struct uart_state uart;
    struct ventilator_state ventilator;
    struct timer_state timers;
};

static struct system_state state;

static void system_init(void);
static void main_loop(void);

static void system_init(void)
{
    // Initialize BSP layer
    BSP_HAL_Init();

    // Initialize modules with their respective states
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

int main(void)
{
    system_init();
    main_loop();
    return 0;
}