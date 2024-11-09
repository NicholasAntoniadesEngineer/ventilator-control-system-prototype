/*
 * sensors.c
 *
 * Sensor reading and processing functions.
 *
 * Created on: 2020
 *     Author: Nicholas Antoniades
 */

#include "sensors.h"
#include "stm32_bsp.h"

/* Private function prototypes */
static void sfm3000_init(void);
static void honeywell_pressure_init(void);
static float sfm3000_read_flow_value(void);
static float honeywell_read_pressure_value(void);

void sensors_init(struct sensor_state *state)
{
    sfm3000_init();             
    honeywell_pressure_init();  
    
    // Initialize state
    for(int i = 0; i < FLOW_HISTORY_SIZE; i++) {
        state->flow_history[i] = 0.0f;
    }
    state->current_flow = 0.0f;
    state->current_pressure = 0.0f;
    state->tidal_volume[0] = 0.0f;
    state->tidal_volume[1] = 0.0f;
    
    // Power up flow sensor
    BSP_GPIO_WritePin(FLOW_POWER_GPIO_Port, FLOW_POWER_Pin, GPIO_PIN_SET);
    state->flow_sensor_initialized = 1;
    state->pressure_sensor_initialized = 1;
    state->sensor_error_flags = 0;
}

void sensors_read_flow(struct sensor_state *state)
{
    float flow = sfm3000_read_flow_value();
    state->current_flow = flow;
    
    // Update flow history
    for(int i = FLOW_HISTORY_SIZE - 1; i > 0; i--) {
        state->flow_history[i] = state->flow_history[i-1];
    }
    state->flow_history[0] = flow;
}

void sensors_read_pressure(struct sensor_state *state)
{
    float pressure = honeywell_read_pressure_value();
    state->current_pressure = pressure;
}

static void sfm3000_init(void)
{
    uint8_t init_data[] = {0x10, 0x00};
    BSP_I2C_Write(&hi2c1, SFM3000_I2C_ADDR, 0x00, init_data, sizeof(init_data));
    HAL_Delay(100); // Wait for sensor initialization
}

static void honeywell_pressure_init(void)
{
    uint8_t init_data[] = {0x01};
    BSP_I2C_Write(&hi2c1, HONEYWELL_I2C_ADDR, 0x00, init_data, sizeof(init_data));
    HAL_Delay(100); // Wait for sensor initialization
}

static float sfm3000_read_flow_value(void)
{
    uint8_t data[3];
    BSP_I2C_Read(&hi2c1, SFM3000_I2C_ADDR, FLOW_REG_ADDR, data, sizeof(data));
    
    // Convert raw data to flow value
    int16_t raw_flow = (data[0] << 8) | data[1];
    float flow = (float)raw_flow * FLOW_CONVERSION_FACTOR;
    
    return flow;
}

static float honeywell_read_pressure_value(void)
{
    uint8_t data[4];
    BSP_I2C_Read(&hi2c1, HONEYWELL_I2C_ADDR, PRESSURE_REG_ADDR, data, sizeof(data));
    
    // Convert raw data to pressure value
    uint16_t raw_pressure = (data[0] << 8) | data[1];
    float pressure = (float)raw_pressure * PRESSURE_CONVERSION_FACTOR;
    
    return pressure;
} 