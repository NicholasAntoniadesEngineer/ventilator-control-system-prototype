/*
 * ventilator.c
 *
 * Ventilator control functions.
 *
 * Created on: 2020
 *     Author: Nicholas Antoniades
 */

#include "ventilator.h"
#include "stm32_bsp.h"

/* Private function prototypes */
static void set_motor_direction(uint8_t direction);
static void step_motor(void);
static uint8_t expiration_complete(const struct ventilator_state *state);


void ventilator_init(struct ventilator_state *state, const struct ventilator_config *config)
{
    const uint32_t total_steps = CALC_TOTAL_STEPS(
        config->hardware.steps_per_cycle,
        config->hardware.micro_step,
        config->hardware.gear_ratio
    );

    const uint32_t total_cycle_time = CALC_CYCLE_TIME(config->breathing.breaths_per_minute);
    const float ie_factor = CALC_IE_FACTOR(config->breathing.ie_ratio);
    const uint32_t insp_time = CALC_INSP_TIME(total_cycle_time, ie_factor);
    const uint32_t exp_time = CALC_EXP_TIME(total_cycle_time, config->breathing.ie_ratio);
    const uint32_t required_steps = CALC_REQUIRED_STEPS(
        total_steps,
        config->volume.desired_volume,
        config->volume.max_volume
    );

    state->hardware = (struct hardware_state) {
        .gear_ratio = config->hardware.gear_ratio,
        .micro_step = config->hardware.micro_step,
        .total_steps_per_cycle = total_steps,
        .default_direction = config->hardware.default_direction,
        .step_counter = 0
    };

    state->breathing = (struct breathing_state) {
        .bpm = config->breathing.breaths_per_minute,
        .ie_ratio = config->breathing.ie_ratio,
        .cycle_stage = 1,
        .total_cycle_time = total_cycle_time,
        .insp_time = insp_time,
        .exp_time = exp_time,
        .insp_freq = required_steps / insp_time,
        .exp_freq = required_steps / exp_time
    };

    state->volume = (struct volume_state) {
        .vmax = config->volume.max_volume,
        .vdes = config->volume.desired_volume,
        .steps_half_cycle = CALC_STEPS_HALF_CYCLE(total_steps),
        .required_steps_half_cycle = required_steps
    };

    state->status = (struct status_state) {
        .state = 0,
        .update_indicator = 0,
        .error_flags = 0
    };

    // Initialize motor direction
    set_motor_direction(state->hardware.default_direction);
} 

/* State machine actions */
static void handle_inspiration(struct ventilator_state *state)
{
    if (state->hardware.step_counter < state->volume.required_steps_half_cycle)
    {
        step_motor();
        state->hardware.step_counter++;
    }
    else
    {
        state->breathing.cycle_stage = 2;  // Switch to expiration
        state->hardware.step_counter = 0;
        set_motor_direction(!state->hardware.default_direction);
    }
}

static void handle_expiration(struct ventilator_state *state)
{
    if (!expiration_complete(state))
    {
        step_motor();
        state->hardware.step_counter++;
    }
    else
    {
        state->breathing.cycle_stage = 1;  // Switch to inspiration
        state->hardware.step_counter = 0;
        set_motor_direction(state->hardware.default_direction);
    }
}

/* State machine implementation */
void ventilator_update_state(struct ventilator_state *state, const struct sensor_state *sensors)
{
    switch(state->breathing.cycle_stage)
    {
        case 1: // Inspiration
            handle_inspiration(state);
            break;

        case 2: // Expiration
            handle_expiration(state);
            break;

        default:
            // Handle error condition
            state->status.error_flags |= ERROR_FLAG_INVALID_STATE;
            state->breathing.cycle_stage = 1;  // Reset to known state
            state->hardware.step_counter = 0;
            set_motor_direction(state->hardware.default_direction);
            break;
    }
}

static void step_motor(void)
{
    BSP_GPIO_WritePin(MOTOR_STEP_GPIO_Port, MOTOR_STEP_Pin, GPIO_PIN_SET);
    HAL_Delay(1); // Short delay for pulse width
    BSP_GPIO_WritePin(MOTOR_STEP_GPIO_Port, MOTOR_STEP_Pin, GPIO_PIN_RESET);
    HAL_Delay(1);
}

static uint8_t expiration_complete(const struct ventilator_state *state)
{
    return (state->hardware.step_counter >= state->volume.required_steps_half_cycle);
}

/* Helper functions */
static void set_motor_direction(uint8_t direction)
{
    BSP_GPIO_WritePin(MOTOR_DIR_GPIO_Port, MOTOR_DIR_Pin, direction ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

