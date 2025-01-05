/*
 * ventilator_controller.c
 * Created on: 24 Apr 2020
 * Author: Nicholas Antoniades
 * Description: STM32F0 ventilator control system with motor control, sensors, PWM, and UART communication.
 */

#include "ventilator_control.h"

/* Single global pointer for interrupt handlers */
static struct ventilator_state *g_state = NULL;

void ventilator_init(struct ventilator_state *state, const struct ventilator_config *config) 
{
    g_state = state;  
    _initialize_state(state, config);
    
    // Initialize hardware
    BSP_HAL_Init();
    lib_init_pwm(state->pwm.frequency);
    lib_init_adc();
    lib_init_ports();
    lib_init_exti();
    lib_init_tim6(50, 8000);
    lib_init_usart1();
}

static void _initialize_state(struct ventilator_state *state, const struct ventilator_config *config) 
{
    // Initialize timing parameters
    state->timing.pause_delay_ms = config->pause_delay_ms;
    state->timing.plateau_delay_ms = config->plateau_delay_ms;
    state->timing.pause_delay_counter = 0;
    state->timing.plateau_delay_counter = 0;

    // Initialize breathing control
    state->breathing.bpm_max = config->bpm_max;
    state->breathing.tidal_counter = 0;
    state->breathing.tidal_input_steps = 0;
    state->breathing.tidal_step_multiplier = 0;
    state->breathing.direction_indicator = 0;
    state->breathing.volume_control = 0.0;
    state->breathing.bpm_control = 0.0;

    // Initialize PWM state
    state->pwm.arr_top = (60.0 * 256.0 * 256.0);
    state->pwm.arr_bottom = 0.0;
    state->pwm.arr_val = 0.0;
    state->pwm.timer = TIM2;
    state->pwm.channel = TIM_CHANNEL_4;

    // Initialize UART state
    state->uart.counter = 0;
    state->uart.frequency = 0;

    // Initialize sensor state
    state->sensors.bellow_pressure = 0;
    state->sensors.flow_sensor = 0;
    state->sensors.initialized = 0;

    // Initialize main state
    state->current_state = 0;
}

void handle_state_machine(struct ventilator_state *state) 
{
    switch(state->current_state) 
    {
        case 0:
            _handle_state_0(state);
            break;
        case 1:
            _handle_state_1(state);
            break;
        case 2:
            _handle_state_2(state);
            break;
        default:
            // Reset to known state
            state->current_state = 0;
            break;
    }
}

static void _handle_state_0(struct ventilator_state *state) 
{
    // Wait for start button press
    if (!lib_check_button_gpioa(0)) 
    {
        return;
    }

    // Initialize motor position
    state->pwm.timer->CR1 |= TIM_CR1_CEN;

    // Wait for limit switch trigger
    if (!lib_check_button_gpioa(11)) 
    {
        return;
    }

    // Enable PWM timer
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
    state->current_state = 1;
}

static void _handle_state_1(struct ventilator_state *state) 
{
    // Wait for button press
    if (!lib_check_button_gpioa(0)) 
    {
        return;
    }

    // Calculate tidal volume steps
    state->breathing.tidal_step_multiplier = TIDAL_TOTAL_STEPS / 256;
    state->breathing.tidal_input_steps = lib_adc_input(8, 8) * state->breathing.tidal_step_multiplier;
    
    state->current_state = 2;
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
}

static void _handle_state_2(struct ventilator_state *state) 
{
    // Check for button press to return to state 1
    if (lib_check_button_gpioa(0)) {
        state->current_state = 1;
    }
}

static void _handle_state_1_interrupt(struct ventilator_state *state) 
{
    // Set motor direction
    BSP_GPIO_WritePin(state->motor.direction_port, state->motor.direction_pin, GPIO_PIN_SET);
    state->breathing.direction_indicator = 0;
    state->breathing.tidal_input_steps = state->motor.initial_steps;

    if (state->breathing.tidal_counter >= state->breathing.tidal_input_steps) 
    {
        state->breathing.tidal_counter = 0;
        state->current_state = 0;
        // Disable PWM timer
        state->pwm.timer->CR1 &= ~TIM_CR1_CEN;
    } else {
        state->breathing.tidal_counter++;
    }
}

static void _handle_state_2_interrupt(struct ventilator_state *state) 
{
    // Disable PWM temporarily
    state->pwm.timer->CR1 &= ~TIM_CR1_CEN;

    if (state->breathing.tidal_counter >= state->breathing.tidal_input_steps) 
    {
        // Update PWM parameters
        _update_pwm_parameters(state);
        
        // Toggle direction and update steps
        if (state->breathing.direction_indicator == 1) 
        {
            BSP_GPIO_WritePin(state->motor.direction_port, state->motor.direction_pin, GPIO_PIN_SET);
            state->breathing.direction_indicator = 0;
            state->breathing.tidal_step_multiplier = TIDAL_TOTAL_STEPS / 256;
            state->breathing.tidal_input_steps = state->breathing.volume_control * 
                                               state->breathing.tidal_step_multiplier;
        } else {
            BSP_GPIO_WritePin(state->motor.direction_port, state->motor.direction_pin, GPIO_PIN_RESET);
            state->breathing.direction_indicator = 1;
        }

        state->breathing.tidal_counter = 0;
    } else {
        state->breathing.tidal_counter++;
    }

    // Re-enable PWM
    state->pwm.timer->CR1 |= TIM_CR1_CEN;
}

static void _update_pwm_parameters(struct ventilator_state *state) 
{
    // Read ADC values
    state->breathing.volume_control = (double)lib_adc_input(8, 8);
    state->breathing.bpm_control = (double)lib_adc_input(7, 8);
    
    // Calculate new PWM parameters
    state->pwm.arr_bottom = (state->breathing.volume_control * TIDAL_TOTAL_STEPS) * 
                           (state->breathing.bpm_control * state->breathing.bpm_max);
    
    state->pwm.arr_val = (SYSTEM_CORE_CLOCK * state->pwm.arr_top / state->pwm.arr_bottom) - 1;
    
    // Update timer registers
    state->pwm.timer->ARR = (uint32_t)state->pwm.arr_val;
    state->pwm.timer->CCR4 = (uint32_t)(state->pwm.arr_val / 2);
}

void sensor_read_and_send(struct sensor_state *sensors) 
{
    // Read sensor values
    sensors->bellow_pressure = lib_adc_input(5, 12);
    sensors->flow_sensor = lib_adc_input(4, 12);
    
    // Send data over UART
    lib_usart1_transmit((uint8_t)sensors->bellow_pressure);
    lib_usart1_transmit((uint8_t)sensors->flow_sensor);
}

// Interrupt Handlers
void TIM6_DAC_IRQHandler(void) 
{
    if (g_state != NULL) 
    {
        sensor_read_and_send(&g_state->sensors);
        lib_ack_irq();
    }
}

void EXTI0_1_IRQHandler(void) 
{
    if (EXTI_GetITStatus(EXTI_Line1) != RESET && g_state != NULL) 
    {
        switch (g_state->current_state) {
            case 1:
                _handle_state_1_interrupt(g_state);
                break;
            case 2:
                _handle_state_2_interrupt(g_state);
                break;
        }
        EXTI_ClearITPendingBit(EXTI_Line1);
    }
}
