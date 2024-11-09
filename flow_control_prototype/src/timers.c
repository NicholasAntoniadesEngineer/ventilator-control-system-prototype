/*
 * timers.c
 *
 * Timer initialization and handling functions.
 *
 * Created on: 2020
 *     Author: Nicholas Antoniades
 */

#include "timers.h"
#include "tim.h"
#include "ventilator.h"
#include "config.h"

/**
 * @brief Initialize timer peripherals for the ventilator system.
 */
void timers_init(void)
{
    // Configure and initialize timer 3 for periodic interrupts
    htim3.Instance = TIM3;
    htim3.Init.Prescaler = 0;
    htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim3.Init.Period = 1000;  // Adjust as needed
    htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    HAL_TIM_Base_Init(&htim3);

    // Enable timer interrupt
    HAL_NVIC_SetPriority(TIM3_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM3_IRQn);
}

/**
 * @brief Start timers and enable interrupts.
 */
void timers_start(void)
{
    // Start timer with interrupt enabled
    HAL_TIM_Base_Start_IT(&htim3);
}

/**
 * @brief Timer period elapsed callback function.
 *
 * @param htim Timer handle pointer.
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM3)
    {
        // Handle timer interrupt for ventilator control
        ventilator_timer_callback();
    }
}

/**
 * @brief Ventilator timer callback function called from timer interrupt.
 */
void ventilator_timer_callback(void)
{
    // Update ventilator state machine
    ventilator_update_state(&ventilator_config);
} 