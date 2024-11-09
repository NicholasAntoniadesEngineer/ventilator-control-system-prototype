/*
 * sensors.h
 *
 * Header file for sensor functions.
 *
 * Created on: 2020
 *     Author: Nicholas Antoniades
 */

#ifndef SENSORS_H_
#define SENSORS_H_

#include "config.h"

/**
 * @brief Initialize all sensors used in the ventilator system.
 */
void sensors_init(void);

/**
 * @brief Read flow sensor data and update configuration.
 *
 * @param config Pointer to VentilatorConfig structure containing current configurations.
 */
void sensors_read_flow(VentilatorConfig *config);

/**
 * @brief Read pressure sensor data and update configuration.
 *
 * @param config Pointer to VentilatorConfig structure containing current configurations.
 */
void sensors_read_pressure(VentilatorConfig *config);

#endif /* SENSORS_H_ */ 