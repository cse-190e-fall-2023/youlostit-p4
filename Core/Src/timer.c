/*
 * timer.h
 *
 *  Created on: Oct 5, 2023
 *      Author: schulman
 */

#ifndef TIMER_H_
#define TIMER_H_

/* Include the type definitions for the timer peripheral */
#include <stm32l475xx.h>

void timer_init(TIM_TypeDef* timer)
{
	timer->CR1 &= ~TIM_CR1_CEN; // reset count enable
	timer->CNT = 0; // reset counter

	RCC->APB1ENR1 |= RCC_APB1ENR1_TIM2EN; // enable TIM 2 clock
	timer->PSC = 7999; // set pre scaler to 3999 on APB
	timer->ARR = 49; // set auto-reload value to value

	NVIC_EnableIRQ(TIM2_IRQn);
	NVIC_SetPriority(TIM2_IRQn, 0); // set interrupt to highest priority

	timer->DIER |= TIM_DIER_UIE; // enable interrupt

	timer->CR1 |= TIM_CR1_CEN; // enable timer
}

void timer_reset(TIM_TypeDef* timer)
{
	timer->CNT = 0; //reset counter
	timer->SR &= ~TIM_SR_UIF; //reset update interrupt flag
}

void timer_set_ms(TIM_TypeDef* timer, uint16_t period_ms)
{
	uint32_t ClockFrequency = 4000; // default frequency in ms
	uint32_t timerPeriod = (period_ms * (ClockFrequency / (timer->PSC +1))) - 1;

	timer->ARR = timerPeriod;
}


#endif /* TIMER_H_ */
