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

void ventilator_init(struct ventilator_state *state)
{
    // Initialize hardware parameters
    state->gear_ratio = 1.6f;
    state->micro_step = 16;
    state->total_steps_per_cycle = 200;
    state->default_direction = 0;

    // Initialize ventilation parameters
    state->bpm = 20;
    state->ie_ratio = 2.0f;
    state->vmax = 100;
    state->vdes = 100;
    state->step_counter = 0;
    state->cycle_stage = 1;
    state->update_indicator = 0;
    state->state = 0;

    // Calculate derived parameters
    state->total_steps_per_cycle *= (uint32_t)(state->micro_step * state->gear_ratio);
    state->steps_half_cycle = state->total_steps_per_cycle / 2;
    state->required_steps_half_cycle = (state->total_steps_per_cycle * state->vdes) / (state->vmax * 2);

    // Calculate timing parameters
    state->total_cycle_time = 60 / state->bpm;
    state->insp_time = (uint32_t)(state->total_cycle_time * (state->ie_ratio / (1 + state->ie_ratio)));
    state->exp_time = (uint32_t)(state->total_cycle_time * (1 / (1 + state->ie_ratio)));
    state->insp_freq = state->required_steps_half_cycle / state->insp_time;
    state->exp_freq = state->required_steps_half_cycle / state->exp_time;

    // Initialize motor direction
    set_motor_direction(state->default_direction);
}

void ventilator_update_state(struct ventilator_state *state, const struct sensor_state *sensors)
{
    switch(state->cycle_stage)
    {
        case 1: // Inspiration
            if (state->step_counter < state->required_steps_half_cycle)
            {
                step_motor();
                state->step_counter++;
            }
            else
            {
                state->cycle_stage = 2;
                state->step_counter = 0;
                set_motor_direction(!state->default_direction);
            }
            break;

        case 2: // Expiration
            if (!expiration_complete(state))
            {
                step_motor();
                state->step_counter++;
            }
            else
            {
                state->cycle_stage = 1;
                state->step_counter = 0;
                set_motor_direction(state->default_direction);
            }
            break;
    }
}

static void set_motor_direction(uint8_t direction)
{
    BSP_GPIO_WritePin(MOTOR_DIR_GPIO_Port, MOTOR_DIR_Pin, direction ? GPIO_PIN_SET : GPIO_PIN_RESET);
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
    return (state->step_counter >= state->required_steps_half_cycle);
} 