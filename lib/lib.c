/* USER CODE BEGIN Header */
/*
------------------------------------------------------------------------------
   Project  : Ventilator Control System
   File     : lib.c
   Brief    : Source file containing the implementation of general functions.
   Author   : Nicholas Antoniades
------------------------------------------------------------------------------
*/
/* USER CODE END Header */

#include "lib.h"

/* Function implementations --------------------------------------------------*/

void Convert_To_EightBit(uint8_t *resultArray, uint32_t value)
{
  /* Break down the 32-bit value into four 8-bit values */
  resultArray[0] = (uint8_t)(value & 0xFFU);
  resultArray[1] = (uint8_t)((value >> 8U) & 0xFFU);
  resultArray[2] = (uint8_t)((value >> 16U) & 0xFFU);
  resultArray[3] = (uint8_t)((value >> 24U) & 0xFFU);
}
