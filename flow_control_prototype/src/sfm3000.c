#include "sfm3000.h"
#include "stm32_bsp.h"

int8_t sfm3000_init(void)
{
    uint8_t init_data = 0;
    // Add any specific initialization sequence required for SFM3000
    BSP_I2C_Write(&hi2c1, SFM3000_I2C_ADDR, 0x00, &init_data, 1);
    return 0;
}

int8_t sfm3000_read_flow(float *flow_value)
{
    uint8_t data[2];
    int8_t status = BSP_I2C_Read(&hi2c1, SFM3000_I2C_ADDR, FLOW_REG_ADDR, data, 2);
    
    if (status == 0) {
        uint16_t raw_flow = (data[0] << 8) | data[1];
        *flow_value = raw_flow * FLOW_CONVERSION_FACTOR;
    }
    
    return status;
}

uint8_t sfm3000_check_status(void)
{
    uint8_t status = 0;
    BSP_I2C_Read(&hi2c1, SFM3000_I2C_ADDR, 0x00, &status, 1);
    return (status == 0);
} 