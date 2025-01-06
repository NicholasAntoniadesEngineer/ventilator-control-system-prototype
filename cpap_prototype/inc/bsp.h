#ifndef BSP_H
#define BSP_H

#include <stdint.h>
#include "stm32f4xx_hal.h"

/* Configuration constants */
#define HMI_BUFFER_SIZE         12U
#define NUM_ADC_CHANNELS        3U
#define UART_TX_COMPLETE_FLAG   255U

/* BSP state type definition */
typedef struct {
    uint8_t hmiTxBuffer[HMI_BUFFER_SIZE];
    uint8_t hmiRxBuffer[HMI_BUFFER_SIZE];
    uint32_t adcValues[NUM_ADC_CHANNELS];
    float pressure;
    uint8_t toggleValue;
    uint8_t runFlag;
    float pressure_threshold;
} bsp_state_t;

/* BSP Function prototypes */
void bsp_init(bsp_state_t* bsp_state);
void bsp_start_timer3(bsp_state_t* bsp_state, uint32_t prescaler);
void bsp_set_valve_state(bsp_state_t* bsp_state, GPIO_TypeDef* port, uint16_t pin, GPIO_PinState state);
void bsp_start_adc_dma(bsp_state_t* bsp_state);
void bsp_stop_adc_dma(bsp_state_t* bsp_state);
void bsp_send_uart_dma(bsp_state_t* bsp_state);
void bsp_stop_uart_dma(bsp_state_t* bsp_state);

#endif /* BSP_H */
