/*
 * tim.c
 *
 *  Created on: Oct 30, 2024
 *      Author: Blake
 */

#include "main.h"
#include "tim.h"

void TIM2_Config(){
	RCC->APB1ENR1 |= (RCC_APB1ENR1_TIM2EN);
	TIM2->ARR = 0xFFFFFFFF;
	// Set initial timer
	TIM2->CCR1 = 1000;
	//Clear flags
	 TIM2->SR &= ~(TIM_SR_CC1IF | TIM_SR_UIF);
	 //Enable CCR1 interrupt
	 TIM2->DIER |= (TIM_DIER_CC1IE);
	 TIM2->CR1 |= (TIM_CR1_CEN);

}
