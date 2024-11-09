/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header file for main application
  * @details        : Contains main application structures and definitions
  * @date           : 2020
  * @author         : Nicholas Antoniades
  ******************************************************************************
  */

#ifndef MAIN_H_
#define MAIN_H_

#include "stm32_bsp.h"
#include "sensors.h"
#include "uart.h"
#include "timers.h"
#include "ventilator.h"

/* System state structure containing all module states */
struct system_state {
    struct sensor_state sensors;
    struct uart_state uart;
    struct ventilator_state ventilator;
    struct timer_state timers;
};

/* Function prototypes */
static void system_init(void);
static void main_loop(void);

#endif /* MAIN_H_ */ 