/* USER CODE BEGIN Header */
/*
------------------------------------------------------------------------------
   Project  : Ventilator Control System
   File     : user_app.h
   Brief    : Header file for user application logic (user_init, user_start, user_run).
   Author   : Nicholas Antoniades
------------------------------------------------------------------------------
*/
/* USER CODE END Header */

#ifndef USER_APP_H
#define USER_APP_H

#include "bsp.h"  // User app will interact only with BSP for hardware functionality

/* Define the user application context structure */
typedef struct {
    uint8_t hmiTxBuffer[HMI_BUFFER_SIZE];  /**< UART transmit buffer */
    uint8_t hmiRxBuffer[HMI_BUFFER_SIZE];  /**< UART receive buffer */
    uint32_t adcValues[NUM_ADC_CHANNELS];  /**< ADC values buffer */
    float pressure;                        /**< Pressure value */
    uint8_t toggleValue;                   /**< Toggle value for DMA streams */
    uint8_t runFlag;                       /**< Run flag */
} UserAppContext;

/* Function prototypes */

/**
 * @brief Initializes the user application, peripherals, and context.
 *
 * @param context Pointer to the user application context.
 */
void user_app_init(UserAppContext *context);

/**
 * @brief Starts the user application, setting initial states and configurations.
 *
 * @param context Pointer to the user application context.
 */
void user_app_start(UserAppContext *context);

/**
 * @brief Main loop of the user application. Called repeatedly in the main loop.
 *
 * @param context Pointer to the user application context.
 */
void user_app_run(UserAppContext *context);

#endif /* USER_APP_H */
