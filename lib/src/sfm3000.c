#include "sfm3000.h"

/* Private I2C helper functions */
static uint8_t I2C_write_bytes(uint16_t dev_addr, uint8_t *buffer, uint8_t size) {
    return (uint8_t)HAL_I2C_Master_Transmit(SFM3000_I2C_INS, dev_addr, buffer, size, 0xFF);
}

static uint8_t I2C_read_bytes(uint16_t dev_addr, uint8_t *buffer, uint8_t size) {
    return (uint8_t)HAL_I2C_Master_Receive(SFM3000_I2C_INS, dev_addr, buffer, size, 0xFF);
}

int8_t sfm3000_init(struct sfm3000_state *state) {
    uint8_t buffer[SFM3000_DEVICE_I2C_COMMAND_SIZE];
    
    // Initialize state
    state->current_flow = 0.0f;
    state->error_flags = 0;
    for(int i = 0; i < FLOW_HISTORY_SIZE; i++) {
        state->flow_history[i] = 0.0f;
    }
    
    // Start flow measurement
    buffer[0] = READ_FLOW_DATA_BYTE_1;
    buffer[1] = READ_FLOW_DATA_BYTE_2;
    
    if(I2C_write_bytes(SFM3000_I2C_ADDRESS, buffer, SFM3000_DEVICE_I2C_COMMAND_SIZE) == HAL_OK) {
        state->initialized = 1;
        return 0;
    }
    
    return -1;
}

int8_t sfm3000_read_flow(struct sfm3000_state *state) {
    uint8_t buffer[SFM3000_READ_DATA_SIZE] = {0};
    int8_t status = I2C_read_bytes(SFM3000_I2C_ADDRESS, buffer, SFM3000_READ_DATA_SIZE);
    
    if(status == HAL_OK) {
        // Update flow history
        for(int i = FLOW_HISTORY_SIZE - 1; i > 0; i--) {
            state->flow_history[i] = state->flow_history[i-1];
        }
        
        // Convert raw data to flow value
        uint16_t raw_flow = (buffer[0] << 8) | buffer[1];
        float flow = ((float)raw_flow - SFM3000_OFFSET_PARAMETER) / SFM3000_SCALE_PARAMETER;
        
        state->current_flow = flow;
        state->flow_history[0] = flow;
        
        #ifdef CRC_ON
        if(sfm3000_crc_check(buffer[0], buffer[1], buffer[2]) != HAL_OK) {
            state->error_flags |= 0x01;  // Set CRC error flag
            return -1;
        }
        #endif
    }
    
    return status;
}

float sfm3000_get_flow(const struct sfm3000_state *state) {
    return state->current_flow;
}

uint8_t sfm3000_check_status(const struct sfm3000_state *state) {
    return state->initialized && !(state->error_flags);
}

uint32_t sfm3000_read_serial_id(void) {
    uint8_t buffer[SFM3000_SERIAL_ID_SIZE];
    buffer[0] = READ_SERIAL_ID_BYTE_1;
    buffer[1] = READ_SERIAL_ID_BYTE_2;
    
    if(I2C_write_bytes(SFM3000_I2C_ADDRESS, buffer, SFM3000_DEVICE_I2C_COMMAND_SIZE) == HAL_OK) {
        HAL_Delay(5);
        if(I2C_read_bytes(SFM3000_I2C_ADDRESS, buffer, SFM3000_SERIAL_ID_SIZE) == HAL_OK) {
            #ifdef CRC_ON
            if(sfm3000_crc_check(buffer[0], buffer[1], buffer[2]) == HAL_OK && 
               sfm3000_crc_check(buffer[3], buffer[4], buffer[5]) == HAL_OK) {
                return ((buffer[0] << 24) | (buffer[1] << 16) | (buffer[3] << 8) | buffer[4]);
            }
            return 0;
            #else
            return ((buffer[0] << 24) | (buffer[1] << 16) | (buffer[3] << 8) | buffer[4]);
            #endif
        }
    }
    return 0;
}

uint8_t sfm3000_reset(void) {
    uint8_t buffer[SFM3000_DEVICE_I2C_COMMAND_SIZE];
    buffer[0] = SOFT_RESET_BYTE_1;
    buffer[1] = SOFT_RESET_BYTE_2;
    return I2C_write_bytes(SFM3000_I2C_ADDRESS, buffer, SFM3000_DEVICE_I2C_COMMAND_SIZE);
}

uint8_t sfm3000_init_temp(void) {
    uint8_t buffer[SFM3000_DEVICE_I2C_COMMAND_SIZE];
    buffer[0] = READ_TEMP_DATA_BYTE_1;
    buffer[1] = READ_TEMP_DATA_BYTE_2;
    return I2C_write_bytes(SFM3000_I2C_ADDRESS, buffer, SFM3000_DEVICE_I2C_COMMAND_SIZE);
}

uint8_t sfm3000_crc8(const uint8_t data, uint8_t crc) {
    crc ^= data;
    for(uint8_t i = 8; i > 0; --i) {
        crc = (crc & 0x80) ? ((crc << 1) ^ 0x31) : (crc << 1);
    }
    return crc;
}

uint8_t sfm3000_crc_check(uint8_t in1, uint8_t in2, uint8_t crc_read) {
    uint8_t crc_temp = 0xFF;
    crc_temp = sfm3000_crc8(in1, crc_temp);
    return (crc_read == sfm3000_crc8(in2, crc_temp)) ? HAL_OK : HAL_ERROR;
} 