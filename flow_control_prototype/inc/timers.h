/*
 * timers.h
 *
 * Header file for timer functions.
 *
 * Created on: 2020
 *     Author: Nicholas Antoniades
 */

#ifndef TIMERS_H_
#define TIMERS_H_

#include "tim.h"

/**
 * @brief Initialize timer peripherals for the ventilator system.
 */
void timers_init(void);

/**
 * @brief Start timers and enable interrupts.
 */
void timers_start(void);

/**
 * @brief Timer period elapsed callback function.
 *
 * @param htim Timer handle pointer.
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);

#endif /* TIMERS_H_ */ 