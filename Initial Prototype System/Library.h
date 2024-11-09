/*
 * Library.h
 *
 *  Created on: 24 Apr 2020
 *      Author: Nicholas Antoniades
 */

#ifndef LIBRARY_H_
#define LIBRARY_H_

#include <stdint.h>
#define STM32F051
#include "stm32f0xx.h"

//Initialisations
void init_PWM(int frequency);
void init_ADC(void);
int ADC_input(int pot, int resolution);
int ADC_DATA(void);
void init_Ports(void);
void libs_delay(int time);
void libs_init_TIM6(uint32_t arr, uint32_t psc);
void libs_TIM6_PSC(uint32_t psc);
void libs_TIM6_ARR(uint32_t arr);
int libs_check_button_GPIOA(int button);
int libs_check_button_GPIOB(int button);
void libs_debounce(void);
void libs_ack_irq(void);
void init_EXTI(void);
void EXTIO_1_interrupt(int toggleVal, int counter, int countTo);
void TIM6_DAC_interrupt(int toggleVal);
void libs_init_USART1(void);
void USART1_transmit(unsigned char DataToTx);
unsigned char USART1_receive(void);
void ADC_AWD_8bit(int ADC_channel, int ADC_Low_threshhold, int ADC_High_threshhold);
int ADC_AWD_check(void);

#endif /* LIBRARY_H_ */