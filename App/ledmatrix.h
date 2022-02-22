/*
 * ledmatrix.h
 *
 *  Created on: Feb 13, 2022
 *      Author: tobias
 */

#ifndef LEDMATRIX_H_
#define LEDMATRIX_H_

#include <stm32f0xx.h>

#define LED_MODULE_ROWS 8
#define LED_MODULE_COUNT 1
#define LED_MODULE_RGB 3

#define LEDMATRIX_OE_ON 0
#define LEDMATRIX_OE_OFF 1

enum {
	COLOR_BLACK = 0,
	COLOR_RED = 1,
	COLOR_BLUE = 2,
	COLOR_GREEN = 4
};


void ledmatrix_init(void);
void ledmatrix_oe(uint8_t enable);
void ledmatrix_latch(void);
void ledmatrix_enable_row(uint8_t index);
void ledmatrix_clear(void);
void ledmatrix_pixel(uint8_t x, uint8_t y, uint8_t color);

#endif /* LEDMATRIX_H_ */
