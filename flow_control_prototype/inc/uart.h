/*
 * uart.h
 *
 * Header file for UART communication functions.
 *
 * Created on: 2020
 *     Author: Nicholas Antoniades
 */

#ifndef UART_H_
#define UART_H_

#include "stdint.h"

#define UART_BUFFER_SIZE 64

extern uint8_t uart_tx_buffer[UART_BUFFER_SIZE];
extern uint8_t uart_rx_buffer[UART_BUFFER_SIZE];

/**
 * @brief Initialize UART communication interfaces.
 */
void uart_init(void);

/**
 * @brief Handle UART communication tasks.
 */
void uart_handle_communication(void);

/**
 * @brief UART transmit complete callback function.
 *
 * @param huart UART handle pointer.
 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart);

/**
 * @brief UART receive complete callback function.
 *
 * @param huart UART handle pointer.
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);

#endif /* UART_H_ */ 