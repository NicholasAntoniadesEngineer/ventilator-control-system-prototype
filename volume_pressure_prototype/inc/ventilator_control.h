#ifndef VENTILATOR_CONTROL_H_
#define VENTILATOR_CONTROL_H_

#include "sfm3000.h"
#include "honeywell_i2c.h"

/**
 * @brief Initialize the ventilator configuration and hardware.
 *
 * @param state Pointer to ventilator_state structure where state is stored.
 * @param config Pointer to ventilator_config structure containing initialization parameters.
 * @return int8_t Returns 0 on success, negative value on error
 */
uint8_t ventilator_control_init(struct ventilator_state *state, const struct ventilator_config *config);

/**
 * @brief Update the ventilator state machine based on sensor readings and control logic.
 *
 * @param state Pointer to ventilator_state structure containing current state.
 * @param flow_sensor Pointer to sfm3000_state structure containing flow sensor readings.
 * @param pressure_sensor Pointer to honeywell_data structure containing pressure sensor readings.
 * @return int8_t Returns 0 on success, negative value on error
 */
uint8_t ventilator_control_update_state(struct ventilator_state *state, 
                                      const struct sfm3000_state *flow_sensor,
                                      const struct honeywell_data *pressure_sensor);

#endif /* VENTILATOR_CONTROL_H_ */ 