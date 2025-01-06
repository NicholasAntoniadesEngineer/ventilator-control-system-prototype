/*
 ******************************************************************************
 * @file           : main.h
 * @brief          : Header file for ventilator volume and pressure control system
 * @date           : 2020
 * @author         : Nicholas Antoniades
 ******************************************************************************
 */

#ifndef MAIN_H_
#define MAIN_H_

#include "uart.h"
#include "timers.h"
#include "ventilator.h"
#include "sfm3000.h"
#include "honeywell_i2c.h"

/* Hardware configuration defaults */
#define HARDWARE_GEAR_RATIO          1.6f
#define HARDWARE_MICRO_STEP          16
#define HARDWARE_STEPS_PER_CYCLE     200
#define HARDWARE_DEFAULT_DIRECTION   0

/* Breathing parameters defaults */
#define BREATHING_BPM               20
#define BREATHING_IE_RATIO          2.0f

/* Volume parameters defaults */
#define VOLUME_MAX                  100
#define VOLUME_DESIRED             100

/* System state structure */
struct system_state {
    struct sfm3000_state flow_sensor;
    struct honeywell_data pressure_sensor;
    struct uart_state uart;
    struct ventilator_state ventilator;
    struct timer_state timers;
};

/* Function prototypes */
static void system_init(void);
static void main_loop(void);

#endif /* MAIN_H_ */