/*
 * ventilator_controller.c
 * Created on: 24 Apr 2020
 * Author: Nicholas Antoniades
 * Description: STM32F0 ventilator control system with motor control, sensors, PWM, and UART communication.
 */

/* Includes */
#include "stm32f0xx.h"
#include <stdint.h>
#include "Library.h"
#include "lcd_stm32f0.h"
#include "ftoa.h"
#include <stm32f0xx_exti.h>

/* Definitions and Global Variables */
#define SYSTEM_CORE_CLOCK 16000000    // 16 MHz core clock
#define MOTOR_STEPS_PER_REV 3200      // Steps per revolution for motor
#define GEAR_RATIO 4                  // Motor gear ratio
#define TIDAL_INITIAL_STEPS (MOTOR_STEPS_PER_REV / GEAR_RATIO)
#define TIDAL_TOTAL_STEPS (MOTOR_STEPS_PER_REV * GEAR_RATIO)

int BPM_max = 100;
int pause_delay_ms = 40;
int plateau_delay_ms = 30;

/* Bit Masks for GPIOs */
#define PA8_MASK  0x0100
#define PA9_MASK  0x0200
#define PA10_MASK 0x0400
#define PB12_MASK 0x1000
#define PB13_MASK 0x2000
#define PB14_MASK 0x4000
#define PB15_MASK 0x8000

/* Global State Variables */
int state, pause_delay, pause_delay_counter, plateau_delay_counter;
int tidal_counter, tidal_input_steps, tidal_step_multiplier, direction_indicator;
int uart_counter, uart_frequency;

/* PWM and Sensor Variables */
double ARR_Val, ARR_bottom, ARR_top;
double volume_control, BPM_control;
int bellow_pressure, flow_sensor;

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

/* Function Definitions */
void initialize_values(void) {
    uart_counter = 0;
    state = 0;
    pause_delay_counter = 0;
    plateau_delay_counter = 0;
    tidal_counter = 0;
    tidal_input_steps = 0;
    direction_indicator = 0;
}

void sensor_read_and_send(void) {
    bellow_pressure = (int)ADC_input(5, 8); // PA5 ADC input
    char *text = "\n";
    USART1_transmit(20);
    USART1_transmit(*text);
    USART1_transmit(30);
}

void pwm_initialization(void) {
    volume_control = (int)ADC_input(8, 8);  // Volume control
    BPM_control = (int)ADC_input(7, 8);     // BPM control
    ARR_top = (60 * 256 * 256);
    ARR_bottom = (volume_control * TIDAL_TOTAL_STEPS) * (BPM_control * BPM_max);
    ARR_Val = (SYSTEM_CORE_CLOCK * ARR_top / ARR_bottom) - 1;
    init_PWM(ARR_Val);
    TIM2->ARR = ARR_Val;
    TIM2->CCR4 = ARR_Val / 2;
}

void TIM6_DAC_IRQHandler(void) {
    sensor_read_and_send();
    libs_ack_irq();  // Reset interrupt flag
}

void EXTI_state_1(void) {
    GPIOB->ODR = 0x4;  // Toggle direction pin PB2
    direction_indicator = 0;
    tidal_input_steps = TIDAL_INITIAL_STEPS;

    if (tidal_counter >= tidal_input_steps) {
        tidal_counter = 0;
        state = 0;
        RCC->APB1ENR &= ~RCC_APB1ENR_TIM2EN;  // Disable TIM2 after moving set steps
    } else {
        tidal_counter++;
    }
}

void EXTI_state_2(void) {
    TIM2->CR1 &= ~TIM_CR1_CEN;  // Disable PWM signal temporarily

    if (tidal_counter >= tidal_input_steps) {
        // Update PWM frequency based on new volume and BPM values
        volume_control = (int)ADC_input(8, 8);
        BPM_control = (int)ADC_input(7, 8);
        ARR_Val = (SYSTEM_CORE_CLOCK * ARR_top / ARR_bottom) - 1;

        // Toggle direction and update steps
        if (direction_indicator == 1) {
            GPIOB->ODR = 0x4;
            direction_indicator = 0;
            tidal_step_multiplier = TIDAL_TOTAL_STEPS / 256;
            tidal_input_steps = volume_control * tidal_step_multiplier;
        } else {
            GPIOB->ODR = 0;
            direction_indicator = 1;
        }

        tidal_counter = 0;
    } else {
        tidal_counter++;
    }

    TIM2->CR1 |= TIM_CR1_CEN;  // Enable PWM counter
}

void EXTI0_1_IRQHandler(void) {
    if (EXTI_GetITStatus(EXTI_Line1) != RESET) {
        if (state == 1) {
            EXTI_state_1();
        } else if (state == 2) {
            EXTI_state_2();
        }
        EXTI_ClearITPendingBit(EXTI_Line1);
    }
}

void state_0(void) {
    SystemCoreClockUpdate();
    init_ADC();
    init_Ports();
    init_EXTI();
    libs_init_TIM6(50, 8000);  // 1ms interrupt
    pwm_initialization();
    libs_init_USART1();
}

void state_1(void) {
    RCC->APB1ENR &= ~RCC_APB1ENR_TIM2EN;  // Disable PWM
    initialize_values();

    // Wait for start button press
    while (!libs_check_button_GPIOA(0));
    TIM2->CR1 |= TIM_CR1_CEN;

    // Wait for limit switch trigger
    while (!libs_check_button_GPIOA(11));
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;  // Enable TIM2
    state = 1;
}

void state_2(void) {
    while (!libs_check_button_GPIOA(0));
    tidal_step_multiplier = TIDAL_TOTAL_STEPS / 256;
    tidal_input_steps = ADC_input(8, 8) * tidal_step_multiplier;
    state = 2;
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
}

int main(void) {
    state_1();
    state_2();

    while (1) {
        if (uart_counter > uart_frequency) {
            sensor_read_and_send();
            uart_counter = 0;
        } else {
            uart_counter++;
        }

        while (!libs_check_button_GPIOA(0));
        state_1();
        state_2();
    }

    return 0;
}
