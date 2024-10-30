/*
 * keypad.h
 *
 *  Created on: Oct 30, 2024
 *      Author: Blake
 */

#ifndef SRC_KEYPAD_H_
#define SRC_KEYPAD_H_
#include "main.h"

#define KEYPAD_NONE 16

extern uint32_t lut[];

void setup_keypad_pins(void);
uint32_t first_set_bit_pos(uint32_t input);
uint32_t get_keypad_press(void);

#endif /* SRC_KEYPAD_H_ */
