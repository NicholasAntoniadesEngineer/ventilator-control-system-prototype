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

#include "config.h"
#include "stdint.h"

/**
 * @brief Initialize the ventilator configuration and hardware.
 *
 * @param config Pointer to VentilatorConfig structure where configurations are stored.
 */
void ventilator_init(VentilatorConfig *config);

/**
 * @brief Update the ventilator state machine based on sensor readings and control logic.
 *
 * @param config Pointer to VentilatorConfig structure containing current configurations.
 */
void ventilator_update_state(VentilatorConfig *config);

#endif /* VENTILATOR_H_ */ 