/*
 * main.c
 * Created on: 24 Apr 2020
 * Author: Nicholas Antoniades
 */

/* Includes */
#include "stm32f0xx.h"
#include "ventilator_state.h"
#include "ventilator_control.h"
#include "stm32_bsp.h"

static struct ventilator_state state;

static const struct ventilator_config config = 
{
    .motor = {
        .steps_per_rev = 1620,
        .gear_ratio = 4,
        .system_clock = 16000000
    },

    .control = {
        .bpm_max = 100,
        .pause_delay_ms = 180,
        .plateau_delay_ms = 100
    }
};

int main(void) 
{
    ventilator_init(&state, &config);
    
    while (1) 
	{
        // Handle button states
        if (!lib_check_button_gpioa(0)) 
		{
            // Button released
            sensor_read_and_send(&state.sensors);
        } else {
            // Button pressed
            handle_state_machine(&state);
        }
    }
    return 0;
}
