/*
 * sensors.c
 *
 * Sensor reading and processing functions.
 *
 * Created on: 2020
 *     Author: Nicholas Antoniades
 */

#include "sensors.h"
#include "config.h"
#include "i2c.h"
#include "adc.h"
#include "rtc.h"

/**
 * @brief Initialize all sensors used in the ventilator system.
 */
void sensors_init(void)
{
    // Initialize I2C sensors
    sfm3000_init();             // Sensirion flow sensor initialization
    honeywell_pressure_init();  // Honeywell pressure sensor initialization

    // Initialize ADC for analog sensors
    init_adc_channels();
}

/**
 * @brief Read flow sensor data and update configuration.
 *
 * @param config Pointer to VentilatorConfig structure containing current configurations.
 */
void sensors_read_flow(VentilatorConfig *config)
{
    // Read flow from Sensirion flow sensor
    float flow = sfm3000_read_flow();
    config->current_flow = flow;

    // Additional processing if necessary
}

/**
 * @brief Read pressure sensor data and update configuration.
 *
 * @param config Pointer to VentilatorConfig structure containing current configurations.
 */
void sensors_read_pressure(VentilatorConfig *config)
{
    // Read pressure from Honeywell sensor
    float pressure = honeywell_read_pressure();
    config->current_pressure = pressure;

    // Additional processing if necessary
}

/**
 * @brief Initialize ADC channels for analog sensors.
 */
void init_adc_channels(void)
{
    ADC_ChannelConfTypeDef sConfig = {0};

    // Configure ADC channels
    sConfig.Channel = ADC_CHANNEL_0;     // Replace with appropriate channel
    sConfig.Rank = 1;
    HAL_ADC_ConfigChannel(&hadc1, &sConfig);

    // Add configurations for other channels as needed
}

/**
 * @brief Read value from an ADC channel.
 *
 * @param channel ADC channel number.
 * @return uint32_t ADC conversion result.
 */
uint32_t read_adc_channel(uint32_t channel)
{
    ADC_ChannelConfTypeDef sConfig = {0};
    sConfig.Channel = channel;
    sConfig.Rank = 1;
    HAL_ADC_ConfigChannel(&hadc1, &sConfig);

    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
    return HAL_ADC_GetValue(&hadc1);
} 