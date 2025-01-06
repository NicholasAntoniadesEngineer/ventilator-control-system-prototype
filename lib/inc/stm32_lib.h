/*
 * stm32_lib.h
 *
 * Created on: 24 Apr 2020
 * Author: Nicholas Antoniades
 */

#ifndef STM32_LIB_H_
#define STM32_LIB_H_

#include <stdint.h>
#define STM32F051
#include "stm32f0xx.h"

// Pin masks
#define PA8_MASK  0x0100
#define PA9_MASK  0x0200
#define PA10_MASK 0x0400
#define PB12_MASK 0x1000
#define PB13_MASK 0x2000
#define PB14_MASK 0x4000
#define PB15_MASK 0x8000

// Function prototypes
void lib_init_ports(void);
void lib_init_pwm_tim(TIM_HandleTypeDef *htim, uint32_t channel, uint32_t frequency);
void lib_init_pwm(int frequency);
void lib_init_adc(void);
void lib_init_usart(UART_HandleTypeDef *huart);
void lib_init_usart1(void);
void lib_init_tim6(uint32_t arr, uint32_t psc);
int lib_adc_input(int pot, int resolution);
void lib_adc_awd_8bit(int ADC_channel, int ADC_Low_threshhold, int ADC_High_threshhold);
int lib_adc_awd_check(void);
int lib_adc_data(void);
void lib_delay(int time);
int lib_check_button_gpioa(int button);
int lib_check_button_gpiob(int button);
void lib_debounce(void);
void lib_tim6_set_psc(uint32_t psc);
void lib_tim6_set_arr(uint32_t arr);
void lib_ack_irq(void);
void lib_init_exti(void);
void lib_usart1_transmit(unsigned char DataToTx);
unsigned char lib_usart1_receive(void);

#endif /* STM32_LIB_H_ */ 