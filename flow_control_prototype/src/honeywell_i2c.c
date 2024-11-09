#include "honeywell_i2c.h"
#include "stm32_bsp.h"

int8_t honeywell_init(void)
{
    // Add any specific initialization sequence required for Honeywell sensor
    return 0;
}

int8_t honeywell_read_pressure(struct honeywell_data *data)
{
    int8_t status = BSP_I2C_Read(&hi2c1, HONEYWELL_I2C_ADDR, PRESSURE_REG_ADDR, 
                                data->buffer, sizeof(data->buffer));
    
    if (status == 0) {
        data->status = data->buffer[0];
        data->pressure = data->buffer[1];
        data->temperature = data->buffer[2];
    }
    
    return status;
}

float honeywell_get_pressure(const struct honeywell_data *data)
{
    return data->pressure * PRESSURE_CONVERSION_FACTOR;
}

uint8_t honeywell_check_status(const struct honeywell_data *data)
{
    return (data->status & 0xC0) == 0; // Check status bits
} 