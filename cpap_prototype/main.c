/*
------------------------------------------------------------------------------
   Project  : Ventilator Control System
   File     : main.c
   Brief    : Source file for main.c developed on the NUCLEO F439ZI development board.
   Author   : Nicholas Antoniades
------------------------------------------------------------------------------
*/

#include "main.h"
#include "adc.h"
#include "dma.h"
#include "rtc.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"


#include "user_app.h" 

int main(void)
{
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_USART1_UART_Init();
  MX_RTC_Init();
  MX_TIM3_Init();


  user_app_init();

  user_app_start();

  while (1)
  {
    user_app_run();
  }
}
