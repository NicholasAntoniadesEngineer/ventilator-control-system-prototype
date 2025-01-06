/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main application code for ventilator control system
  * @date           : 2020
  * @author         : Nicholas Antoniades
  ******************************************************************************
  */

#include "main.h"
#include "stm32_bsp.h"
#include "uart.h"
#include "timers.h"
#include "ventilator_control.h"
#include "sfm3000.h"
#include "honeywell_i2c.h"

/* Private variables */
static struct system_state state;

const struct ventilator_config vent_config = {
	.hardware = {
		.gear_ratio = HARDWARE_GEAR_RATIO,
		.micro_step = HARDWARE_MICRO_STEP,
		.steps_per_cycle = HARDWARE_STEPS_PER_CYCLE,
		.default_direction = HARDWARE_DEFAULT_DIRECTION
	},
	.breathing = {
		.breaths_per_minute = BREATHING_BPM,
		.ie_ratio = BREATHING_IE_RATIO
	},
	.volume = {
		.max_volume = VOLUME_MAX,
		.desired_volume = VOLUME_DESIRED
	}
};

/* Private function definitions */
static void system_init(void)
{
	stm32_bsp_hal_init();

	timers_init(&state.timers);

	uart_init(&state.uart);

	sfm3000_init(&state.flow_sensor);

	honeywell_init(&state.pressure_sensor);

	ventilator_control_init(&state.ventilator, &vent_config);

}

static void main_loop(void)
{
    while (1)
    {
        sfm3000_read_flow(&state.flow_sensor);

        honeywell_read_pressure(&state.pressure_sensor);

        ventilator_control_update_state(&state.ventilator, 
                                      &state.flow_sensor,
                                      &state.pressure_sensor);

        uart_handle_communication(&state.uart);
    }
}

int main(void)
{
    system_init();

    main_loop();
	
    return 0;
}