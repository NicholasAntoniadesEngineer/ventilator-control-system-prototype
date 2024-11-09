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

#include <stdint.h>

/* Sensor buffer sizes */
#define FLOW_HISTORY_SIZE 20
#define PRESSURE_BUFFER_SIZE 4
#define TIDAL_VOLUME_BUFFER_SIZE 2

/* Sensor I2C addresses */
#define SFM3000_I2C_ADDR     0x40
#define HONEYWELL_I2C_ADDR   0x28

/* Sensor register addresses */
#define FLOW_REG_ADDR        0x10
#define PRESSURE_REG_ADDR    0x00

/* Sensor conversion factors */
#define FLOW_CONVERSION_FACTOR 0.01f
#define PRESSURE_CONVERSION_FACTOR 0.01f

struct sensor_state {
    /* Current sensor readings */
    float current_flow;
    float current_pressure;
    
    /* Historical data */
    float flow_history[FLOW_HISTORY_SIZE];
    float tidal_volume[TIDAL_VOLUME_BUFFER_SIZE];
    
    /* Raw sensor data */
    uint8_t pressure_buffer[PRESSURE_BUFFER_SIZE];
    uint8_t status_honeywell;
    uint8_t pressure_honeywell;
    uint8_t temp_honeywell;
    
    /* Sensor status flags */
    uint8_t flow_sensor_initialized;
    uint8_t pressure_sensor_initialized;
    uint8_t sensor_error_flags;
};

/**
 * @brief Initialize all sensors used in the ventilator system.
 * 
 * @param state Pointer to sensor state structure
 * @return int8_t Returns 0 on success, negative value on error
 */
int8_t sensors_init(struct sensor_state *state);

/**
 * @brief Read flow sensor data and update state.
 * 
 * @param state Pointer to sensor state structure
 * @return int8_t Returns 0 on success, negative value on error
 */
int8_t sensors_read_flow(struct sensor_state *state);

/**
 * @brief Read pressure sensor data and update state.
 * 
 * @param state Pointer to sensor state structure
 * @return int8_t Returns 0 on success, negative value on error
 */
int8_t sensors_read_pressure(struct sensor_state *state);

/**
 * @brief Get the current flow rate
 * 
 * @param state Pointer to sensor state structure
 * @return float Current flow rate value
 */
float sensors_get_flow(const struct sensor_state *state);

/**
 * @brief Get the current pressure
 * 
 * @param state Pointer to sensor state structure
 * @return float Current pressure value
 */
float sensors_get_pressure(const struct sensor_state *state);

/**
 * @brief Check if sensors are functioning correctly
 * 
 * @param state Pointer to sensor state structure
 * @return uint8_t Returns 1 if all sensors are OK, 0 otherwise
 */
uint8_t sensors_check_status(const struct sensor_state *state);

#endif /* SENSORS_H_ */ 