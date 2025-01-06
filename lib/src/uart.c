/*
 * uart.c
 *
 * UART communication functions implementation.
 *
 * Created on: 2020
 *     Author: Nicholas Antoniades
 */

#include "uart.h"
#include "stm32_bsp.h"

void uart_init(struct uart_state *state)
{
    // Clear buffers and reset states
    for(int i = 0; i < UART_BUFFER_SIZE; i++) {
        state->tx_buffer[i] = 0;
        state->rx_buffer[i] = 0;
    }
    
    state->tx_size = 0;
    state->rx_size = 0;
    state->tx_complete = 1;
    state->rx_complete = 1;
}

void uart_handle_communication(struct uart_state *state)
{
    // Check if we have data to send
    if (!state->tx_complete && state->tx_size > 0) {
        BSP_UART_Transmit(&huart1, state->tx_buffer, state->tx_size);
        state->tx_complete = 1;
        state->tx_size = 0;
    }

    // Check if we should receive data
    if (state->rx_complete) {
        uart_receive_data(state);
    }
}

void uart_send_data(struct uart_state *state, const uint8_t *data, uint16_t size)
{
    if (size > UART_BUFFER_SIZE) {
        size = UART_BUFFER_SIZE;
    }

    // Copy data to transmit buffer
    for(uint16_t i = 0; i < size; i++) {
        state->tx_buffer[i] = data[i];
    }
    
    state->tx_size = size;
    state->tx_complete = 0;
}

void uart_receive_data(struct uart_state *state)
{
    state->rx_complete = 0;
    BSP_UART_Receive(&huart1, state->rx_buffer, 1); // Receive one byte at a time
    state->rx_complete = 1;
}