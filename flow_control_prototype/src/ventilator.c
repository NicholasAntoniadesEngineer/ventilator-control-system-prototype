/*
 * ventilator.c
 *
 * Ventilator control functions.
 *
 * Created on: 2020
 *     Author: Nicholas Antoniades
 */

#include "ventilator.h"
#include "config.h"
#include "timers.h"
#include "gpio.h"

/**
 * @brief Initialize the ventilator configuration and hardware.
 *
 * @param config Pointer to VentilatorConfig structure where configurations are stored.
 */
void ventilator_init(VentilatorConfig *config)
{
    // Hardware defined variables
    config->gear_ratio = 1.6f;
    config->micro_step = 16;
    config->total_steps_per_cycle = 200;
    config->default_direction = 0;

    // Ventilation variables
    config->bpm = 20;
    config->ie_ratio = 2.0f;
    config->vmax = 100;
    config->vdes = 100;
    config->step_counter = 0;
    config->cycle_stage = 1;
    config->update_indicator = 0;
    config->state = 0;

    // Calculations
    config->total_steps_per_cycle *= (uint32_t)(config->micro_step * config->gear_ratio);
    config->steps_half_cycle = config->total_steps_per_cycle / 2;
    config->required_steps_half_cycle = (config->total_steps_per_cycle * config->vdes) / (config->vmax * 2);

    config->total_cycle_time = 60 / config->bpm;
    config->insp_time = (uint32_t)(config->total_cycle_time * (config->ie_ratio / (1 + config->ie_ratio)));
    config->exp_time = (uint32_t)(config->total_cycle_time * (1 / (1 + config->ie_ratio)));
    config->insp_freq = config->required_steps_half_cycle / config->insp_time;
    config->exp_freq = config->required_steps_half_cycle / config->exp_time;

    // Initialize valves
    HAL_GPIO_WritePin(PINCH1_GPIO_Port, PINCH1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(PINCH2_GPIO_Port, PINCH2_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(PINCH3_GPIO_Port, PINCH3_Pin, GPIO_PIN_RESET);

    // Power up flow sensor
    HAL_GPIO_WritePin(FLOW_POWER_GPIO_Port, FLOW_POWER_Pin, GPIO_PIN_SET);

    // Initialize timers and interrupts
    timers_start();
}

/**
 * @brief Update the ventilator state machine based on sensor readings and control logic.
 *
 * @param config Pointer to VentilatorConfig structure containing current configurations.
 */
void ventilator_update_state(VentilatorConfig *config)
{
    // Update the ventilator state machine based on sensor readings and control logic
    switch (config->state)
    {
        case 0:
            // Initial state: Prepare for inspiration
            prepare_inspiration(config);
            config->state = 1;
            break;

        case 1:
            // Inspiration phase
            perform_inspiration(config);
            if (inspiration_complete(config))
            {
                config->state = 2;
            }
            break;

        case 2:
            // Transition to expiration
            prepare_expiration(config);
            config->state = 3;
            break;

        case 3:
            // Expiration phase
            perform_expiration(config);
            if (expiration_complete(config))
            {
                config->state = 0;
            }
            break;

        default:
            // Error handling or reset
            config->state = 0;
            break;
    }
}

/**
 * @brief Prepare ventilator for inspiration phase.
 *
 * @param config Pointer to VentilatorConfig structure containing current configurations.
 */
void prepare_inspiration(VentilatorConfig *config)
{
    // Open inspiration valve, close expiration valve
    HAL_GPIO_WritePin(PINCH1_GPIO_Port, PINCH1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(PINCH2_GPIO_Port, PINCH2_Pin, GPIO_PIN_RESET);

    // Set motor direction for inspiration
    set_motor_direction(config->default_direction);

    // Reset step counter
    config->step_counter = 0;
}

/**
 * @brief Perform inspiration by controlling motor steps.
 *
 * @param config Pointer to VentilatorConfig structure containing current configurations.
 */
void perform_inspiration(VentilatorConfig *config)
{
    // Control motor steps for inspiration
    if (config->step_counter < config->required_steps_half_cycle)
    {
        step_motor();
        config->step_counter++;
    }
}

/**
 * @brief Check if inspiration phase is complete.
 *
 * @param config Pointer to VentilatorConfig structure containing current configurations.
 * @return uint8_t Returns 1 if inspiration is complete, 0 otherwise.
 */
uint8_t inspiration_complete(VentilatorConfig *config)
{
    return (config->step_counter >= config->required_steps_half_cycle);
}

/**
 * @brief Prepare ventilator for expiration phase.
 *
 * @param config Pointer to VentilatorConfig structure containing current configurations.
 */
void prepare_expiration(VentilatorConfig *config)
{
    // Open expiration valve, close inspiration valve
    HAL_GPIO_WritePin(PINCH1_GPIO_Port, PINCH1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(PINCH2_GPIO_Port, PINCH2_Pin, GPIO_PIN_SET);

    // Set motor direction for expiration
    set_motor_direction(!config->default_direction);

    // Reset step counter
    config->step_counter = 0;
}

/**
 * @brief Perform expiration by controlling motor steps.
 *
 * @param config Pointer to VentilatorConfig structure containing current configurations.
 */
void perform_expiration(VentilatorConfig *config)
{
    // Control motor steps for expiration
    if (config->step_counter < config->required_steps_half_cycle)
    {
        step_motor();
        config->step_counter++;
    }
}

/**
 * @brief Check if expiration phase is complete.
 *
 * @param config Pointer to VentilatorConfig structure containing current configurations.
 * @return uint8_t Returns 1 if expiration is complete, 0 otherwise.
 */
uint8_t expiration_complete(VentilatorConfig *config)
{
    return (config->step_counter >= config->required_steps_half_cycle);
}

/**
 * @brief Set the motor direction.
 *
 * @param direction Motor direction, 0 or 1.
 */
void set_motor_direction(uint8_t direction)
{
    // Set motor direction pin
    if (direction)
    {
        HAL_GPIO_WritePin(MOTOR_DIR_GPIO_Port, MOTOR_DIR_Pin, GPIO_PIN_SET);
    }
    else
    {
        HAL_GPIO_WritePin(MOTOR_DIR_GPIO_Port, MOTOR_DIR_Pin, GPIO_PIN_RESET);
    }
}

/**
 * @brief Perform a single motor step.
 */
void step_motor(void)
{
    // Generate a pulse on the motor step pin
    HAL_GPIO_WritePin(MOTOR_STEP_GPIO_Port, MOTOR_STEP_Pin, GPIO_PIN_SET);
    HAL_Delay(1); // Short delay for pulse width
    HAL_GPIO_WritePin(MOTOR_STEP_GPIO_Port, MOTOR_STEP_Pin, GPIO_PIN_RESET);
    HAL_Delay(1);
} 