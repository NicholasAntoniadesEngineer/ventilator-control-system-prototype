#ifndef SFM3000_H_
#define SFM3000_H_

#include <stdint.h>
#include "i2c.h"

/* Device Configuration */
#define SFM3000_I2C_INS                 &hi2c1          // I2C HAL Instance
#define SFM3000_I2C_ADDRESS             0x40<<1         // Device I2C Address
#define SFM3000_OFFSET_PARAMETER        32000.0f        // Offset flow value
#define SFM3000_SCALE_PARAMETER         140.0f          // Scale factor for air
#define FLOW_HISTORY_SIZE               20              // Size of flow history buffer

/* Command Sizes */
enum SFM3000_COMMAND_SIZES {
    SFM3000_DEVICE_I2C_COMMAND_SIZE = 0x02,    // Size of commands
    SFM3000_SERIAL_ID_SIZE         = 0x06,     // 4 byte id + 2 byte CRC
    SFM3000_READ_DATA_SIZE         = 0x03      // 2 byte data + 1 byte CRC
};

/* Command Values */
enum SFM3000_COMMANDS {
    READ_TEMP_DATA_BYTE_1  = 0x10,
    READ_TEMP_DATA_BYTE_2  = 0x01,
    READ_FLOW_DATA_BYTE_1  = 0x10,
    READ_FLOW_DATA_BYTE_2  = 0x00,
    READ_SERIAL_ID_BYTE_1  = 0x31,
    READ_SERIAL_ID_BYTE_2  = 0xAE,
    SOFT_RESET_BYTE_1      = 0x20,
    SOFT_RESET_BYTE_2      = 0x00
};

/* Sensor State Structure */
struct sfm3000_state {
    float current_flow;
    float flow_history[FLOW_HISTORY_SIZE];
    uint8_t initialized;
    uint8_t error_flags;
};

/* Function Prototypes */
int8_t sfm3000_init(struct sfm3000_state *state);
int8_t sfm3000_read_flow(struct sfm3000_state *state);
float sfm3000_get_flow(const struct sfm3000_state *state);
uint8_t sfm3000_check_status(const struct sfm3000_state *state);
uint32_t sfm3000_read_serial_id(void);
uint8_t sfm3000_reset(void);
uint8_t sfm3000_init_temp(void);

/* CRC Functions */
uint8_t sfm3000_crc8(const uint8_t data, uint8_t crc);
uint8_t sfm3000_crc_check(uint8_t in1, uint8_t in2, uint8_t crc_read);

#endif /* SFM3000_H_ */ 