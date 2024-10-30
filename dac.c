/*
 * dac.c
 *
 *  Created on: Oct 30, 2024
 *      Author: Blake
 */

#include "dac.h"
#include "main.h"

int DAC_init(void){
	RCC->APB2ENR   |=  (RCC_APB2ENR_SPI1EN_Msk); //Enable SPI
	RCC->AHB2ENR   |=  (RCC_AHB2ENR_GPIOAEN);		//Enable GPIOA
	//Set PA4-7 to AF
	GPIOA->MODER 	 &= ~(GPIO_MODER_MODE4 | GPIO_MODER_MODE5 | GPIO_MODER_MODE6 | GPIO_MODER_MODE7);
	GPIOA->MODER	|= (GPIO_MODER_MODE4_1 | GPIO_MODER_MODE5_1 | GPIO_MODER_MODE6_1 | GPIO_MODER_MODE7_1);
	//Set PA4-7 AF to SPI1 (5)
	GPIOA->AFR[0] &= ~(GPIO_AFRL_AFRL4 | GPIO_AFRL_AFRL5 | GPIO_AFRL_AFRL6 | GPIO_AFRL_AFRL7);
	GPIOA->AFR[0] |= ((GPIO_AF5_SPI1 << GPIO_AFRL_AFSEL4_Pos)
		  | (GPIO_AF5_SPI1 << GPIO_AFRL_AFSEL5_Pos)
		  | (GPIO_AF5_SPI1 << GPIO_AFRL_AFSEL6_Pos)
		  | (GPIO_AF5_SPI1 << GPIO_AFRL_AFSEL7_Pos));

	//Setting Baud Rate to 1/2; SPI Mode to 00, RXOnly to 0, BIDIMODE 0, SSM 0
	//Clear LSBFIRST; DAC specifies MSB first (as per data sheet SPI Input Timing Data)
	SPI1->CR1 &= ~(SPI_CR1_BR_Msk
		  | SPI_CR1_CPOL_Msk
		  | SPI_CR1_CPHA_Msk
		  | SPI_CR1_RXONLY_Msk
		  | SPI_CR1_BIDIMODE_Msk
		  | SPI_CR1_SSM_Msk
		  | SPI_CR1_LSBFIRST_Msk);

	//Setting BIDIOE, MSTR, Enable SPI
	SPI1->CR1 |= (SPI_CR1_BIDIOE_Msk | SPI_CR1_MSTR_Msk);

	//Setting NSSP for chip select pulse.
	SPI1->CR2 |= (SPI_CR2_NSSP_Msk);

	//Setting DS & SSOE
	SPI1->CR2 |= (SPI_CR2_DS_Msk
			  | SPI_CR2_SSOE_Msk);

	SPI1->CR1 |= SPI_CR1_SPE_Msk;

	return 1;
}
