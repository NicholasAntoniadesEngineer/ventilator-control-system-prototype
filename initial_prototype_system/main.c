/*
 * main.c
 * Created on: 24 Apr 2020
 * Author: Nicholas Antoniades
 */

/* Includes */
#include "stm32f0xx.h"
#include <stdint.h>
#include "Library.h"
#include "lcd_stm32f0.h"
#include "ftoa.h"
#include <stm32f0xx_exti.h>

// Set up variables
int BPMmax = 100;
int motorStepsPerRev = 1620; // 1/8 steps micro stepping
int gearRatio = 4;

// Make this based on input volume and control
int pauseDelayms = 180;
int plateuDelayms = 100;  // Updated this based on a function of bpm
int uartFrequency = 0.1;
#define SystemCoreClock  16000000 //8MHz crystal
#define tidalInitialSteps (motorStepsPerRev)
#define tidalTotalSteps motorStepsPerRev*gearRatio 			// Total steps multiplied by gear ratio

// Setting global variables
int pauseDelay;
int pauseDelayCounter;
int tidalCounter;
int tidalInputSteps;
int tidalStepMultiplier;
int directionIndicator;
int UART_counter;
int state;
// Frequency update variables
double ARR_Val;
double ARR_bottom;
double ARR_top;
// Sensor variables
int load_cell_1;
int load_cell_2;
int flow_sensor;
int bellow_pressure;
int peep_pressure;
// Control variables
double volume_control;
double BPM_control;
// Communications variables
int uartCounter;

// Defining bit masks
#define PA8_mask  0x0100
#define PA9_mask  0x0200
#define PA10_mask 0x0400
#define PB12_mask 0x1000
#define PB13_mask 0x2000
#define PB14_mask 0x4000
#define PB15_mask 0x8000

void initialiseValues(void){
	uartCounter = 0;
	state = 0;
	pauseDelay = 0;
	pauseDelayCounter = 0;
	tidalCounter = 0;
	tidalInputSteps = 0;
	directionIndicator = 0;
	GPIOB -> ODR = 0x8;
	UART_counter = 0;
	TIM2->ARR = ARR_Val;
	TIM2->CCR4 = ARR_Val/2;
}

void pwmInitialization(void){
	volume_control =(int)ADC_input(8,8);	 // Volume pot
	BPM_control =(int)ADC_input(7,8);  		 // Frequency pot
	ARR_top = (60*256*256);
	ARR_bottom = (volume_control*tidalTotalSteps)*(BPM_control*BPMmax);
	ARR_Val = (SystemCoreClock*ARR_top/ARR_bottom) - 1;
	init_PWM(ARR_Val);
	TIM2->ARR = ARR_Val;
	TIM2->CCR4 = ARR_Val/2;
}

void PWMupdate(void){					     // Updating frequency keeping BPM constant for change in volume
	volume_control =(int)ADC_input(8,8);	 // Volume pot
	BPM_control =(int)ADC_input(7,8);  		 // Frequency pot
	ARR_top = (60*256*256);					 /* Divide the read values by 256*/
	ARR_bottom = (volume_control*tidalTotalSteps)*(BPM_control*BPMmax);
	ARR_Val = SystemCoreClock*ARR_top/ARR_bottom - 1;
	// Zero value limiter
	if(ARR_Val <= 0){
		ARR_Val = 100;
	}
}

void sensorReadAndSend(void){
//	load_cell_1 = 	  (int)ADC_input(2,8); // PA2
	load_cell_2 =     (int)ADC_input(3,8); // PA3
//
//	flow_sensor = 	  (int)ADC_input(4,8); // PA4
	bellow_pressure = (int)ADC_input(5,8); // PA5
//   peep_pressure =   (int)ADC_input(6,8); // PA6
//	BPM_control =     (int)ADC_input(7,8); // PA7
//	volume_control =  (int)ADC_input(8,8); // PB0
	char *text = "\n";
	USART1_transmit(BPM_control);
	USART1_transmit(volume_control);
	USART1_transmit(bellow_pressure);
	USART1_transmit(load_cell_2);
	USART1_transmit(*text);
//	USART1_transmit(peep_pressure);
//	USART1_transmit(bellow_pressure);
//	USART1_transmit(flow_sensor);
//	USART1_transmit(load_cell_1);
//	USART1_transmit(load_cell_2);
}



void UARTdelay(int UARTdelayms){
	pauseDelay = UARTdelayms ;
	while (pauseDelay > 0) {
		if (pauseDelayCounter > pauseDelay/727){
			sensorReadAndSend(); 	// Reading and sending sensor information intermittently
			pauseDelayCounter = 0;
		}else{
			pauseDelayCounter++;
		}
		pauseDelay--;
	}
}

void TIM6_DAC_IRQHandler(void) {
	sensorReadAndSend(); 			// Reading and sending sensor information intermittently
	if(peep_pressure > 150)	{	    // Pip pressure not reached
		GPIOA -> ODR |= PA9_mask;
	}else{
		GPIOA -> ODR ^= PA9_mask;
	}
	libs_ack_irq(); // Set interrupt flag low
}

void tidalStepUpdate(void){
	tidalStepMultiplier = (tidalTotalSteps - tidalInitialSteps/2 )/256;
	tidalInputSteps = volume_control*tidalStepMultiplier;
}

void senesorCheck(void){
	//	flow_sensor = 	  (int)ADC_input(4,8); // PA4
		bellow_pressure = (int)ADC_input(5,8); // PA5
	//  peep_pressure =   (int)ADC_input(6,8); // PA6

	    if(bellow_pressure>200){\
	    	GPIOB -> ODR &= ~0x8;
	    }else{
	    	GPIOB -> ODR |= 0x8;
	    }
}

void EXTI_state_1(void){
	GPIOB -> ODR  |= 0x4;							//Toggle direction pin PB2
	directionIndicator = 0;
	tidalInputSteps = tidalInitialSteps; 		// Set this value to set amount to move away from limit switch
	if (tidalCounter >=tidalInputSteps){
	   tidalCounter = 0;
	   state = 0;
	   RCC->APB1ENR = ~RCC_APB1ENR_TIM2EN;		// TIM2 disable after moving set steps
	}else{
	   tidalCounter ++;
	}
}
int newCounter=0;
void EXTI_state_2(void){
    TIM2->CR1 = ~TIM_CR1_CEN; 			//Disable PWM
	if(uartCounter>(uartFrequency*727)){
		sensorReadAndSend();
		uartCounter = 0;
	}else{
		uartCounter++;
	}
    senesorCheck();
    if (tidalCounter>=tidalInputSteps){ // Set counter overflow value to that relatable to pwm signal
    	PWMupdate();
		// Up
		if (directionIndicator == 1){
			GPIOB -> ODR |= 0x4;			//Toggle direction pin PB2
			directionIndicator = 0;
			tidalStepUpdate();			// Update change in volume
			TIM2->ARR = ARR_Val;		//Setting breathing ratio
			TIM2->CCR4 = ARR_Val/2;
			UARTdelay(pauseDelayms);
		}
		// Down
		else{
			GPIOB -> ODR &= ~0x4;		//Toggle direction pin PB2
			directionIndicator=1; 		//Toggle direction indicator
			TIM2->ARR = ARR_Val/2;		//Setting breathing ratio
			TIM2->CCR4 = ARR_Val/4;
			UARTdelay(plateuDelayms);
		}
		tidalCounter = 0;
	}else{
		tidalCounter ++;
	}
	TIM2->CR1 = TIM_CR1_CEN; // Start PWM counter
}


void EXTI0_1_IRQHandler(void){
	if(EXTI_GetITStatus(EXTI_Line1) != RESET){

		// Initial, blank interrupt state
		if(state == 0){

		}
		// Set distance from limit switch
		if(state == 1){
			EXTI_state_1();
		}
		if(state == 2){
			EXTI_state_2();
		}
	}
	EXTI_ClearITPendingBit(EXTI_Line1); //CLear interrupt pending bit
}

void state_0(void){
	// Initialization functions
	SystemCoreClockUpdate();
	init_ADC();					 // Initialize Analog to digital converters
	init_Ports();				 // Initialize ports
	init_EXTI();				 // Initializing External Interrupts from pins
	libs_init_TIM6(50,8000);     // Initialize internal interrupt with 1ms delay
	pwmInitialization();   		 // Initialize Pulse Width Modulation
	libs_init_USART1();			 // Initialize UART communications
	// ADC_AWD_8bit(5,2 ,3);		 // Initialize ADC watch dog
	// GPIOA -> ODR |= PA8_mask;
}

void state_1(void){
	// Wait for start button push
	initialiseValues();
	while(1){
		if(libs_check_button_GPIOA(0)){		// PA0 switch
			TIM2->CR1 |= TIM_CR1_CEN;    	// PWM Counter enable
			break;
		}
	}
	//wait for limit switch trigger
	while(1){
		if(libs_check_button_GPIOA(11)){	    // PA11 connected to limit switch
			RCC->APB1ENR = RCC_APB1ENR_TIM2EN;	// TIM2 Enable
			state = 1;							// Change state in EXTI handler
			break;
		}
	}
}

void state_2(void){
	while(1){
		// Wait for start button push
		if(libs_check_button_GPIOA(0)){	      						// PA0 switch
			tidalStepUpdate();
			state = 2;												// Change state in EXTI handler
			RCC->APB1ENR = RCC_APB1ENR_TIM2EN;						// Enable PWM
			break;
		}
	}
}

int main(void){
    state_0();
	while (1){

		// Start button release
		while(1){
			sensorReadAndSend();
			if(!libs_check_button_GPIOA(0)){
				break;
			}
		}
		// Start button press
		while(1){
			if(libs_check_button_GPIOA(0)){	      	// PA0 switch
				state_1();
				state_2();
				break;
			}
		}
	}
	return 0; //keep compiler happy
}
