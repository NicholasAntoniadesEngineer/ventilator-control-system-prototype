/* USER CODE BEGIN Header */
/*
------------------------------------------------------------------------------
   Project  : Ventilator Control System
   File     : lib.h
   Brief    : Header file for general library functions.
   Author   : Nicholas Antoniades
------------------------------------------------------------------------------
*/
/* USER CODE END Header */

#ifndef LIB_H
#define LIB_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* Function Prototypes --------------------------------------------------------*/

/**
 * @brief Breaks down a 32-bit value into four 8-bit values.
 *
 * @param resultArray Array to store the 8-bit results (must have at least 4 elements).
 * @param value The 32-bit value to convert.
 */
void Convert_To_EightBit(uint8_t *resultArray, uint32_t value);

#ifdef __cplusplus
}
#endif

#endif /* LIB_H */
