#ifndef MOTOR_CONTROL_H_
#define MOTOR_CONTROL_H_

#include <stdint.h>
#include "stm32_bsp.h"

/* Motor control configuration */
struct motor_config {
    float gear_ratio;
    uint8_t micro_step;
    uint32_t steps_per_cycle;
    uint8_t default_direction;
};

/* Motor state */
struct motor_state {
    float gear_ratio;
    uint8_t micro_step;
    uint32_t total_steps_per_cycle;
    uint8_t default_direction;
    uint32_t step_counter;
    uint32_t required_steps;
};

/**
 * @brief Initialize motor control hardware
 * 
 * @param state Pointer to motor state structure
 * @param config Pointer to motor configuration structure
 * @return int8_t Returns 0 on success, negative value on error
 */
int8_t motor_init(struct motor_state *state, const struct motor_config *config);

/**
 * @brief Step motor in current direction
 * 
 * @param state Pointer to motor state structure
 */
void motor_step(struct motor_state *state);

/**
 * @brief Set motor direction
 * 
 * @param direction Direction (0 or 1)
 */
void motor_set_direction(uint8_t direction);

/**
 * @brief Get current step count
 * 
 * @param state Pointer to motor state structure
 * @return uint32_t Current step count
 */
uint32_t motor_get_step_count(const struct motor_state *state);

/**
 * @brief Reset step counter
 * 
 * @param state Pointer to motor state structure
 */
void motor_reset_counter(struct motor_state *state);

#endif /* MOTOR_CONTROL_H_ */ 