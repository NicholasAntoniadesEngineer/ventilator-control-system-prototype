/*
 * ventilator.h
 *
 * Header file for ventilator control functions.
 *
 * Created on: 2020
 *     Author: Nicholas Antoniades
 */

#ifndef VENTILATOR_H_
#define VENTILATOR_H_

#include "state.h"
#include "stdint.h"

/* Hardware state */
struct hardware_state {
    float gear_ratio;
    uint8_t micro_step;
    uint32_t total_steps_per_cycle;
    uint8_t default_direction;
    uint32_t step_counter;
};

/* Breathing cycle state */
struct breathing_state {
    uint8_t bpm;
    float ie_ratio;
    uint8_t cycle_stage;
    uint32_t total_cycle_time;
    uint32_t insp_time;
    uint32_t exp_time;
    uint32_t insp_freq;
    uint32_t exp_freq;
};

/* Volume state */
struct volume_state {
    uint8_t vmax;
    uint8_t vdes;
    uint32_t steps_half_cycle;
    uint32_t required_steps_half_cycle;
};

/* System status */
struct status_state {
    uint8_t state;
    uint8_t update_indicator;
    uint32_t error_flags;
};

/* Main ventilator state structure */
struct ventilator_state {
    struct hardware_state hardware;
    struct breathing_state breathing;
    struct volume_state volume;
    struct status_state status;
};

/* Configuration structures for ventilator initialization */
struct hardware_config {
    float gear_ratio;
    uint8_t micro_step;
    uint32_t steps_per_cycle;
    uint8_t default_direction;
};

struct breathing_params {
    uint8_t breaths_per_minute;
    float ie_ratio;
};

struct volume_params {
    uint8_t max_volume;
    uint8_t desired_volume;
};

struct ventilator_config {
    struct hardware_config hardware;
    struct breathing_params breathing;
    struct volume_params volume;
};

/* Calculation macros for ventilator initialization */
#define CALC_TOTAL_STEPS(steps, micro_step, gear_ratio) \
    ((steps) * (uint32_t)((micro_step) * (gear_ratio)))

#define CALC_CYCLE_TIME(breaths_per_min) \
    (60 / (breaths_per_min))

#define CALC_IE_FACTOR(ie_ratio) \
    ((ie_ratio) / (1 + (ie_ratio)))

#define CALC_INSP_TIME(cycle_time, ie_factor) \
    ((uint32_t)((cycle_time) * (ie_factor)))

#define CALC_EXP_TIME(cycle_time, ie_ratio) \
    ((uint32_t)((cycle_time) * (1.0f / (1 + (ie_ratio)))))

#define CALC_STEPS_HALF_CYCLE(total_steps) \
    ((total_steps) / 2)

#define CALC_REQUIRED_STEPS(total_steps, desired_vol, max_vol) \
    ((total_steps) * (desired_vol) / ((max_vol) * 2))

/**
 * @brief Initialize the ventilator configuration and hardware.
 *
 * @param state Pointer to ventilator_state structure where state is stored.
 * @param config Pointer to ventilator_config structure containing initialization parameters.
 */
void ventilator_init(struct ventilator_state *state, const struct ventilator_config *config);

/**
 * @brief Update the ventilator state machine based on sensor readings and control logic.
 *
 * @param state Pointer to ventilator_state structure containing current state.
 * @param sensors Pointer to sensor_state structure containing sensor readings.
 */
void ventilator_update_state(struct ventilator_state *state, const struct sensor_state *sensors);

#endif /* VENTILATOR_H_ */ 