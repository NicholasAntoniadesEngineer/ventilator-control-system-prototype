/*
 * timers.c
 *
 * Timer functions implementation.
 *
 * Created on: 2020
 *     Author: Nicholas Antoniades
 */

#include "timers.h"
#include "stm32_bsp.h"

void timers_init(struct timer_state *state)
{
    state->last_tick = 0;
    state->period_ms = 0;
    state->timer_active = 0;
    state->overflow_count = 0;

    // Start the base timers
    BSP_TIM_Base_Start(&htim2);
    BSP_TIM_Base_Start(&htim3);
    BSP_TIM_Base_Start(&htim4);
}

void timers_start(struct timer_state *state, uint32_t period_ms)
{
    state->last_tick = BSP_GetTick();
    state->period_ms = period_ms;
    state->timer_active = 1;
}

void timers_stop(struct timer_state *state)
{
    state->timer_active = 0;
}

uint8_t timers_elapsed(struct timer_state *state)
{
    if (!state->timer_active) {
        return 0;
    }

    uint32_t current_tick = BSP_GetTick();
    
    if (current_tick - state->last_tick >= state->period_ms) {
        state->last_tick = current_tick;
        state->overflow_count++;
        return 1;
    }
    
    return 0;
}

uint32_t timers_get_count(struct timer_state *state)
{
    return state->overflow_count;
} 