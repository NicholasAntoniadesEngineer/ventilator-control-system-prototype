/*
 * uart.c
 *
 * UART communication handling functions.
 *
 * Created on: 2020
 *     Author: Nicholas Antoniades
 */

#include "uart.h"
#include "usart.h"
#include "dma.h"
#include "string.h"

uint8_t uart_tx_buffer[UART_BUFFER_SIZE];
uint8_t uart_rx_buffer[UART_BUFFER_SIZE];

/**
 * @brief Initialize UART communication interfaces.
 */
void uart_init(void)
{
    // Initialize UART DMA streams
    HAL_UART_Receive_DMA(&huart1, uart_rx_buffer, UART_BUFFER_SIZE);
    HAL_UART_Transmit_DMA(&huart1, uart_tx_buffer, UART_BUFFER_SIZE);
}

/**
 * @brief Handle UART communication tasks.
 */
void uart_handle_communication(void)
{
    // Check if new data has been received
    if (uart_new_data_available())
    {
        // Process received data
        process_uart_message(uart_rx_buffer);

        // Prepare response if necessary
        prepare_uart_response(uart_tx_buffer);

        // Transmit response
        HAL_UART_Transmit_DMA(&huart1, uart_tx_buffer, strlen((char *)uart_tx_buffer));
    }
}

/**
 * @brief UART transmit complete callback function.
 *
 * @param huart UART handle pointer.
 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        // Transmission complete, handle if needed
    }
}

/**
 * @brief UART receive complete callback function.
 *
 * @param huart UART handle pointer.
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        // Reception complete, set a flag or handle data
        uart_set_new_data_flag();
    }
}

/**
 * @brief Check if new UART data is available.
 *
 * @return uint8_t Returns 1 if new data is available, 0 otherwise.
 */
uint8_t uart_new_data_available(void)
{
    // Implement mechanism to check for new data
    // This could be a flag set in the receive callback
    return uart_data_received_flag;
}

/**
 * @brief Process received UART message.
 *
 * @param data Pointer to received data buffer.
 */
void process_uart_message(uint8_t *data)
{
    // Parse and handle the incoming message
    // Example: Extract command and parameters
}

/**
 * @brief Prepare UART response message.
 *
 * @param data Pointer to transmission data buffer.
 */
void prepare_uart_response(uint8_t *data)
{
    // Compose response message based on processed data
    strcpy((char *)data, "ACK");
}

/**
 * @brief Set flag indicating new UART data has been received.
 */
void uart_set_new_data_flag(void)
{
    // Set flag indicating new data is available
    uart_data_received_flag = 1;
}

/**
 * @brief Clear flag indicating new UART data has been processed.
 */
void uart_clear_new_data_flag(void)
{
    // Clear flag after processing data
    uart_data_received_flag = 0;
}

// Flag indicating new UART data has been received
static volatile uint8_t uart_data_received_flag = 0; 