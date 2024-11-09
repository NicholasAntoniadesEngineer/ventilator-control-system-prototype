/*
 * ventilator.c
 *
 * Ventilator control functions.
 *
 * Created on: 2020
 *     Author: Nicholas Antoniades
 */

#include "stdint.h"
#include "sfm3000.h"
#include "honeywell_i2c.h"
#include "motor_control.h"
#include "ventilator.h"

/* Private function prototypes */
static void handle_inspiration(struct ventilator_state *state);
static void handle_expiration(struct ventilator_state *state);
static uint8_t expiration_complete(const struct ventilator_state *state);

uint8_t ventilator_init(struct ventilator_state *state, const struct ventilator_config *config)
{
    int8_t status;

    // Initialize motor control
    status = motor_init(&state->motor, &config->motor);
    if (status != 0) {
        return status;
    }

    const uint32_t total_cycle_time = CALC_CYCLE_TIME(config->breathing.breaths_per_minute);
    const float ie_factor = CALC_IE_FACTOR(config->breathing.ie_ratio);
    const uint32_t insp_time = CALC_INSP_TIME(total_cycle_time, ie_factor);
    const uint32_t exp_time = CALC_EXP_TIME(total_cycle_time, config->breathing.ie_ratio);
    const uint32_t required_steps = CALC_REQUIRED_STEPS(
        state->motor.total_steps_per_cycle,
        config->volume.desired_volume,
        config->volume.max_volume
    );

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
        .steps_half_cycle = CALC_STEPS_HALF_CYCLE(state->motor.total_steps_per_cycle),
        .required_steps_half_cycle = required_steps
    };

    state->status = (struct status_state) {
        .state = 0,
        .update_indicator = 0,
        .error_flags = 0
    };

    return 0;
}

/* State machine actions */
static void handle_inspiration(struct ventilator_state *state)
{
    if (motor_get_step_count(&state->motor) < state->volume.required_steps_half_cycle)
    {
        motor_step(&state->motor);
    }
    else
    {
        state->breathing.cycle_stage = 2;  // Switch to expiration
        motor_reset_counter(&state->motor);
        motor_set_direction(!state->motor.default_direction);
    }
}

static void handle_expiration(struct ventilator_state *state)
{
    if (!expiration_complete(state))
    {
        motor_step(&state->motor);
    }
    else
    {
        state->breathing.cycle_stage = 1;  // Switch to inspiration
        motor_reset_counter(&state->motor);
        motor_set_direction(state->motor.default_direction);
    }
}

/* State machine implementation */
void ventilator_update_state(struct ventilator_state *state, 
                           const struct sfm3000_state *flow_sensor,
                           const struct honeywell_data *pressure_sensor)
{
    // Check sensor status
    if (!sfm3000_check_status(flow_sensor) || !honeywell_check_status(pressure_sensor)) {
        state->status.error_flags |= ERROR_FLAG_SENSOR_FAILURE;
        return;
    }

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
            motor_reset_counter(&state->motor);
            motor_set_direction(state->motor.default_direction);
            break;
    }
}

static uint8_t expiration_complete(const struct ventilator_state *state)
{
    return (motor_get_step_count(&state->motor) >= state->volume.required_steps_half_cycle);
}

