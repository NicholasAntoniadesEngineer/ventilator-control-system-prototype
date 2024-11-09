#include "motor_control.h"
#include "stm32_bsp.h"

int8_t motor_init(struct motor_state *state, const struct motor_config *config)
{
    state->gear_ratio = config->gear_ratio;
    state->micro_step = config->micro_step;
    state->total_steps_per_cycle = config->steps_per_cycle * 
                                  (uint32_t)(config->micro_step * config->gear_ratio);
    state->default_direction = config->default_direction;
    state->step_counter = 0;
    
    // Set initial direction
    motor_set_direction(state->default_direction);
    
    return 0;
}

void motor_step(struct motor_state *state)
{
    BSP_GPIO_WritePin(MOTOR_STEP_GPIO_Port, MOTOR_STEP_Pin, GPIO_PIN_SET);
    HAL_Delay(1); // Short delay for pulse width
    BSP_GPIO_WritePin(MOTOR_STEP_GPIO_Port, MOTOR_STEP_Pin, GPIO_PIN_RESET);
    HAL_Delay(1);
    
    state->step_counter++;
}

void motor_set_direction(uint8_t direction)
{
    BSP_GPIO_WritePin(MOTOR_DIR_GPIO_Port, MOTOR_DIR_Pin, 
                     direction ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

uint32_t motor_get_step_count(const struct motor_state *state)
{
    return state->step_counter;
}

void motor_reset_counter(struct motor_state *state)
{
    state->step_counter = 0;
} 