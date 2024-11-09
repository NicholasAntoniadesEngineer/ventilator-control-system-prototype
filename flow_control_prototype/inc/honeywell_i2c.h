#ifndef HONEYWELL_I2C_H_
#define HONEYWELL_I2C_H_

#include <stdint.h>

struct honeywell_data {
    float current_pressure;
    uint8_t initialized;
};

int8_t honeywell_init(struct honeywell_data *state);
int8_t honeywell_read_pressure(struct honeywell_data *state);
float honeywell_get_pressure(const struct honeywell_data *state);
uint8_t honeywell_check_status(const struct honeywell_data *state);

#endif /* HONEYWELL_I2C_H_ */ 