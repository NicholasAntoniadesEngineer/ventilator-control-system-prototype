/*
 * ventilator_controller.h
 * Description: Header file for STM32F0 ventilator control system.
 */

#ifndef VENTILATOR_CONTROLLER_H
#define VENTILATOR_CONTROLLER_H

#include "stm32f0xx.h"
#include "ventilator_state.h"
#include "stm32_bsp.h"

/* System definitions */
#define SYSTEM_CORE_CLOCK 16000000    // 16 MHz core clock
#define MOTOR_STEPS_PER_REV 3200      // Steps per revolution for motor
#define GEAR_RATIO 4                  // Motor gear ratio
#define TIDAL_INITIAL_STEPS (MOTOR_STEPS_PER_REV / GEAR_RATIO)
#define TIDAL_TOTAL_STEPS (MOTOR_STEPS_PER_REV * GEAR_RATIO)

/* Configuration structure */
struct ventilator_config {
    struct {
        uint32_t steps_per_rev;
        uint32_t gear_ratio;
        uint32_t system_clock;
    } motor;

    struct {
        uint32_t bpm_max;
        uint32_t pause_delay_ms;
        uint32_t plateau_delay_ms;
    } control;
};

/* Public Function Prototypes */
void ventilator_init(struct ventilator_state *state, const struct ventilator_config *config);
void handle_state_machine(struct ventilator_state *state);
void sensor_read_and_send(struct sensor_state *sensors);

#endif /* VENTILATOR_CONTROLLER_H */
