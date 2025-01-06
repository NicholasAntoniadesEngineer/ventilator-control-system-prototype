#ifndef MAIN_H_
#define MAIN_H_

#include "ventilator_state.h"
#include "ventilator_control.h"
#include "stm32_bsp.h"

/* Motor configuration defaults */
#define MOTOR_STEPS_PER_REV    1620
#define MOTOR_GEAR_RATIO       4
#define MOTOR_SYSTEM_CLOCK     16000000

/* Control parameters defaults */
#define CONTROL_BPM_MAX        100
#define CONTROL_PAUSE_DELAY    180
#define CONTROL_PLATEAU_DELAY  100

/* System state structure */
struct ventilator_config {
    struct {
        uint32_t steps_per_rev;
        uint32_t gear_ratio;
        uint32_t system_clock;
    } motor;

    struct {
        uint32_t bpm_max;
        uint32_t pause_delay_ms;
        uint32_t plateau_delay_ms;
    } control;
};

/* Function prototypes */
static void ventilator_init(struct ventilator_state *state, const struct ventilator_config *config);
static void handle_state_machine(struct ventilator_state *state);
static void sensor_read_and_send(struct sensor_state *sensors);

#endif /* MAIN_H_ */ 