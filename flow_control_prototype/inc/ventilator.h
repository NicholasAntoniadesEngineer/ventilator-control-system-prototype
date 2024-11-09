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

/**
 * @brief Initialize the ventilator configuration and hardware.
 *
 * @param state Pointer to ventilator_state structure where state is stored.
 */
void ventilator_init(ventilator_state *state);

/**
 * @brief Update the ventilator state machine based on sensor readings and control logic.
 *
 * @param state Pointer to ventilator_state structure containing current state.
 */
void ventilator_update_state(ventilator_state *state);

#endif /* VENTILATOR_H_ */ 