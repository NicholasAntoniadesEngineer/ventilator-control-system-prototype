/*
 * timers.h
 *
 * Timer functions header.
 *
 * Created on: 2020
 *     Author: Nicholas Antoniades
 */

#ifndef TIMERS_H_
#define TIMERS_H_

#include <stdint.h>

struct timer_state {
    uint32_t last_tick;
    uint32_t period_ms;
    uint8_t timer_active;
    uint32_t overflow_count;
};

void timers_init(struct timer_state *state);
void timers_start(struct timer_state *state, uint32_t period_ms);
void timers_stop(struct timer_state *state);
uint8_t timers_elapsed(struct timer_state *state);
uint32_t timers_get_count(struct timer_state *state);

#endif /* TIMERS_H_ */ 