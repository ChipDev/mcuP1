/*
 * keypad.c
 *
 *  Created on: Oct 30, 2024
 *      Author: Blake
 */
#include "keypad.h"
#include "main.h"

void setup_keypad_pins(void){
	RCC->AHB2ENR   |=  (RCC_AHB2ENR_GPIOCEN);
	//enable C
	//inputs PC5-8
	//reset MODE5-8 (input)
	GPIOC->MODER &= ~(GPIO_MODER_MODE5_Msk | GPIO_MODER_MODE6_Msk | GPIO_MODER_MODE7_Msk | GPIO_MODER_MODE8_Msk);

	//clear PUPD5-8, set all to Pull-down (10)
	GPIOC->PUPDR &= ~(GPIO_PUPDR_PUPD5_Msk | GPIO_PUPDR_PUPD6_Msk | GPIO_PUPDR_PUPD7_Msk | GPIO_PUPDR_PUPD8_Msk);
	GPIOC->PUPDR |= (GPIO_PUPDR_PUPD5_1 | GPIO_PUPDR_PUPD6_1 | GPIO_PUPDR_PUPD7_1 | GPIO_PUPDR_PUPD8_1);

	//outputs PC10-13
	//clear MODE10-13 and set all to Output (01)
	GPIOC->MODER &= ~(GPIO_MODER_MODE10_Msk | GPIO_MODER_MODE11_Msk | GPIO_MODER_MODE12_Msk | GPIO_MODER_MODE13_Msk);
	GPIOC->MODER |= (GPIO_MODER_MODE10_0 | GPIO_MODER_MODE11_0 | GPIO_MODER_MODE12_0 | GPIO_MODER_MODE13_0);

	//PC10-13 very high speed
	GPIOC->OSPEEDR |= (GPIO_OSPEEDR_OSPEED10_Msk | GPIO_OSPEEDR_OSPEED11_Msk | GPIO_OSPEEDR_OSPEED12_Msk | GPIO_OSPEEDR_OSPEED13_Msk);

	//sets outputs PC10-13 high for starting
	GPIOC->ODR |= (GPIO_ODR_OD10_Msk | GPIO_ODR_OD11_Msk | GPIO_ODR_OD12_Msk | GPIO_ODR_OD13_Msk);
}

//position of first set bit; used for keypad.
uint32_t first_set_bit_pos(uint32_t input) {
    uint8_t position = 0;
    while ((input & 1) == 0) {
		input >>= 1;
		position++;
		if (input == 0) {
			return 255; //error; input was 0.
		}
	}
    return position;
}

uint32_t lut[] = {1, 2, 3, 10, 4, 5, 6, 11, 7, 8, 9, 12, 13, 0, 14, 15, KEYPAD_NONE};
//LUT converts position of key to the value;
/*
 * 1 2 3 A           0  1  2  3            1  2  3  10
 * 4 5 6 B   from    4  5  6  7    to      4  5  6  11  for LEDs.
 * 7 8 9 C           8  9  10 11           7  8  9  12
 * * 0 # D           12 13 14 15           13 0  14 15
 */

uint32_t get_keypad_press(void) {
	//read inputs PC5-8, masked
	uint32_t inputs_masked = GPIOC->IDR & (GPIO_IDR_ID5_Msk | GPIO_IDR_ID6_Msk | GPIO_IDR_ID7_Msk | GPIO_IDR_ID8_Msk);
	//is keypad 0? return no press
	if(!inputs_masked) {
		return KEYPAD_NONE;
	}

	//loop through all 4 outputs 10-13 to check which is pressed
	for(uint32_t i = 10; i <= 13; i++){
		//clear output for PC10-13, then set i'th pin (10...13)
		GPIOC->ODR &= ~(GPIO_ODR_OD10_Msk | GPIO_ODR_OD11_Msk | GPIO_ODR_OD12_Msk | GPIO_ODR_OD13_Msk);
		GPIOC->ODR |= (0x1 << i);

		uint32_t temp_inputs_masked = GPIOC->IDR & (GPIO_IDR_ID5_Msk | GPIO_IDR_ID6_Msk | GPIO_IDR_ID7_Msk | GPIO_IDR_ID8_Msk);
		if(temp_inputs_masked){

			//Calculate the row that is currently high
			uint32_t row = i - 10;

			//Calculate the col that is is pressed (or the first set bit of the 4 inputs)
			//builtin ctz takes a nonzero input, which we guarantee is true from the if block
			uint32_t col = first_set_bit_pos(temp_inputs_masked >> 5);

			//sets outputs PC10-13 high for starting
			GPIOC->ODR |= (GPIO_ODR_OD10_Msk | GPIO_ODR_OD11_Msk | GPIO_ODR_OD12_Msk | GPIO_ODR_OD13_Msk);

			return col + 4*row;

		}
	}

	//keypad near-instantaneously unpressed; set outputs and return no press
	GPIOC->ODR |= (GPIO_ODR_OD10_Msk | GPIO_ODR_OD11_Msk | GPIO_ODR_OD12_Msk | GPIO_ODR_OD13_Msk);
	return KEYPAD_NONE;

}


