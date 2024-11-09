/*
 * uart.h
 *
 * UART communication functions header.
 *
 * Created on: 2020
 *     Author: Nicholas Antoniades
 */

#ifndef UART_H_
#define UART_H_

#include <stdint.h>

#define UART_BUFFER_SIZE 64
#define UART_TX_COMPLETE_FLAG 255

struct uart_state {
    uint8_t tx_buffer[UART_BUFFER_SIZE];
    uint8_t rx_buffer[UART_BUFFER_SIZE];
    uint16_t tx_size;
    uint16_t rx_size;
    uint8_t tx_complete;
    uint8_t rx_complete;
};

void uart_init(struct uart_state *state);
void uart_handle_communication(struct uart_state *state);
void uart_send_data(struct uart_state *state, const uint8_t *data, uint16_t size);
void uart_receive_data(struct uart_state *state);

#endif /* UART_H_ */ 