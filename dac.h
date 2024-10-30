/*
 * dac.h
 *
 *  Created on: Oct 30, 2024
 *      Author: Blake
 */


#ifndef SRC_DAC_H_

#define SRC_DAC_H_
#include "main.h"

#define CONF_MASK 0xF << 12u
#define CONF_VALUE 0x3 << 12u

int DAC_init(void);

static inline void DAC_write(uint16_t val){

	//clear conf bits of value, set conf bits
	val &= ~(CONF_MASK);
	val |= (CONF_VALUE);

	//wait for TX empty
	while (!(SPI1->SR & SPI_SR_TXE));

	SPI1->DR = val;
}


#endif /* SRC_DAC_H_ */
