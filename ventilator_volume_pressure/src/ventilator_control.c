#include "ventilator_control.h"
#include "motor_control.h"

/* State machine implementation */
uint8_t ventilator_control_update_state(struct ventilator_state *state, 
                                      const struct sfm3000_state *flow_sensor,
                                      const struct honeywell_data *pressure_sensor)
{
    // Check sensor status
    if (!sfm3000_check_status(flow_sensor) || !honeywell_check_status(pressure_sensor)) {
        state->status.error_flags |= ERROR_FLAG_SENSOR_FAILURE;
        return -1;
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
    
    return 0;
}

uint8_t ventilator_control_init(struct ventilator_state *state, const struct ventilator_config *config)
{
    // Initialize motor control
    struct motor_config motor_cfg = {
        .gear_ratio = config->hardware.gear_ratio,
        .micro_step = config->hardware.micro_step,
        .steps_per_cycle = config->hardware.steps_per_cycle,
        .default_direction = config->hardware.default_direction
    };
    
    if (motor_init(&state->motor, &motor_cfg) != 0) {
        return -1;
    }

    // Initialize breathing parameters
    state->breathing.breaths_per_minute = config->breathing.breaths_per_minute;
    state->breathing.ie_ratio = config->breathing.ie_ratio;
    state->breathing.cycle_stage = 1;  // Start with inspiration

    // Initialize volume parameters
    state->volume.max_volume = config->volume.max_volume;
    state->volume.desired_volume = config->volume.desired_volume;
    
    return 0;
} 