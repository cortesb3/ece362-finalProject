/**
  ******************************************************************************
  * @file    main.c
  * @author  Cortes, Bea
  * @date    2025 Feb 19
  * @brief   ECE 362 Final Project ADC Input
  ******************************************************************************
*/

#include "stm32f0xx.h"
#include <math.h>   // for M_PI
#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>

//Analog-to-digital converter for joystick position, x and y direction
void setup_adc()
{
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN; // Enable GPIOA clock

    //use PA1 for x and PA2 for y
    GPIOA->MODER &= ~(GPIO_MODER_MODER1 | GPIO_MODER_MODER2); // Clear PA1 and PA2 mode
    GPIOA->MODER |= GPIO_MODER_MODER1 | GPIO_MODER_MODER2; // Set PA1 and PA2 to analog mode

    RCC->APB2ENR |= RCC_APB2ENR_ADCEN; // Enable ADC clock
    RCC->CR2 |= RCC_CR2_HSI14ON; // Turn on 14MHz HSI oscillator
    while(!(RCC->CR2 & RCC_CR2_HSI14RDY)); // Wait until HSI14 is ready
    ADC1->CR |= ADC_CR_ADEN; // Enable ADC
    while(!(ADC1->ISR & ADC_ISR_ADRDY)); // Wait until ADC is ready
}

//============================================================================
// Variables for joystick and boxcar averaging.
//============================================================================
#define BCSIZE 32
uint32_t adc_x = 2048, adc_y = 2048;
int16_t x_position = 0, y_position = 0;
int bcsum_x, bcsum_y;
int boxcar_x[BCSIZE], boxcar_y[BCSIZE];
int bcn = 0;
uint8_t joystick_data;
//============================================================================
// Timer 2 ISR
//============================================================================
// Write the Timer 2 ISR here.  Be sure to give it the right name.
void TIM2_IRQHandler(void) {
    TIM2->SR &= ~TIM_SR_UIF;

    //start with channel 1 (PA1)
    ADC1->CHSELR = ADC_CHSELR_CHSEL1; // Select channel 1 (PA1)
    ADC1->CR |= ADC_CR_ADSTART; // Start a conversion.
    while(!(ADC1->ISR & ADC_ISR_EOC)); // Wait for the conversion to complete.
    bcsum_x -= boxcar_x[bcn];
    bcsum_x += boxcar_x[bcn] = ADC1->DR;

    //move to channel 2 (PA2)
    ADC1->CHSELR = ADC_CHSELR_CHSEL2; // Select channel 2 (PA2)
    ADC1->CR |= ADC_CR_ADSTART; 
    while(!(ADC1->ISR & ADC_ISR_EOC)); 
    bcsum_y -= boxcar_y[bcn];
    bcsum_y += boxcar_y[bcn] = ADC1->DR;

    bcn += 1;
    if (bcn >= BCSIZE)
        bcn = 0;
    
    adc_x = bcsum_x / BCSIZE;
    adc_y = bcsum_y / BCSIZE;

    x_position = (adc_x - 2048);// / 16;
    y_position = (adc_y - 2048);// / 16;

    uint8_t direction;
    int16_t magnitude;

    // determine direction
    // 00 = left, 01 = right, 10 = down, 11 = up
    if (abs(x_position) > abs(y_position)) {
        magnitude = abs(x_position);
        direction = (x_position > 0) ? 0b01 : 0b00;
    }
    else {
        magnitude = abs(y_position);
        direction = (y_position > 0) ? 0b11 : 0b10;
    }

    uint8_t scaled_magnitude = magnitude >> 6;
    joystick_data = (direction << 7) | (scaled_magnitude & 0x3f);
}


//============================================================================
// init_tim2()
//============================================================================
void init_tim2(void) {
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

    TIM2->PSC = 4800-1;
    TIM2->ARR = 1000-1;

    TIM2->DIER |= TIM_DIER_UIE;
    NVIC_EnableIRQ(TIM2_IRQn);
    NVIC_SetPriority(TIM2_IRQn, 3);
    TIM2->CR1 |= TIM_CR1_CEN;
}