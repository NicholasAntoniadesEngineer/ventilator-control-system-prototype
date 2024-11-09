#ifndef SFM3000_H_
#define SFM3000_H_

#include <stdint.h>

struct sfm3000_state {
    float current_flow;
    float flow_history[20];  // Keeping FLOW_HISTORY_SIZE as 20
    uint8_t initialized;
};

int8_t sfm3000_init(struct sfm3000_state *state);
int8_t sfm3000_read_flow(struct sfm3000_state *state);
float sfm3000_get_flow(const struct sfm3000_state *state);
uint8_t sfm3000_check_status(const struct sfm3000_state *state);

#endif /* SFM3000_H_ */ 