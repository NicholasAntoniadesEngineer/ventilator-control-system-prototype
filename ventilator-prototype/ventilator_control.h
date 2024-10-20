/*
 * ventilator_controller.h
 * Description: Header file for STM32F0 ventilator control system.
 */

#ifndef VENTILATOR_CONTROLLER_H
#define VENTILATOR_CONTROLLER_H

#include "stm32f0xx.h"

/* Function Prototypes */
void initialize_values(void);
void sensor_read_and_send(void);
void pwm_initialization(void);
void EXTI_state_1(void);
void EXTI_state_2(void);
void TIM6_DAC_IRQHandler(void);
void EXTI0_1_IRQHandler(void);
void state_0(void);
void state_1(void);
void state_2(void);

#endif /* VENTILATOR_CONTROLLER_H */
