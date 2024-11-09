#ifndef MAIN_H_
#define MAIN_H_

#include "stm32_bsp.h"
#include "sensors.h"
#include "uart.h"
#include "timers.h"
#include "ventilator.h"

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
    struct sensor_state sensors;
    struct uart_state uart;
    struct ventilator_state ventilator;
    struct timer_state timers;
};

/* Function prototypes */
static void system_init(void);
static void main_loop(void);

#endif /* MAIN_H_ */