/*
 * ledmatrix.c
 *
 *  Created on: Feb 13, 2022
 *      Author: tobias
 */

#include "ledmatrix.h"

uint8_t dotmatrix_framebuffer[LED_MODULE_ROWS][LED_MODULE_RGB * LED_MODULE_COUNT];

void ledmatrix_init(void) {
	// variables init
	multiplex_index = 0;

	// IO Config
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
	// IO for Multiplexing - output
	GPIOA->MODER |= (0b01 << GPIO_MODER_MODER0_Pos) | (0b01 << GPIO_MODER_MODER1_Pos) | (0b01 << GPIO_MODER_MODER2_Pos);
	// IO for SPI
	// SDI / CLK - alternate function
	GPIOA->MODER |= (0b10 << GPIO_MODER_MODER4_Pos) | (0b10 << GPIO_MODER_MODER5_Pos);
	// alternate function set to AF0 (SPI) after reset
	// OE / LE - output
	GPIOA->MODER |= (0b01 << GPIO_MODER_MODER6_Pos) | (0b01 << GPIO_MODER_MODER7_Pos);

	// SPI Config
	RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
	SPI1->CR1 |= SPI_CR1_MSTR | SPI_CR1_SSI | SPI_CR1_SSM | (0b110 << SPI_CR1_BR_Pos);
	SPI1->CR2 |= (0b0111 << SPI_CR2_DS_Pos) | SPI_CR2_TXDMAEN;
	SPI1->CR1 |= SPI_CR1_SPE;

	// DMA Config
	RCC->AHBENR |= RCC_AHBENR_DMA1EN;
	DMA1_Channel3->CCR |= DMA_CCR_MINC | DMA_CCR_DIR | DMA_CCR_TCIE;
	DMA1_Channel3->CPAR = (uint32_t)&(SPI1->DR);
	DMA1_Channel3->CNDTR = LED_MODULE_RGB * LED_MODULE_COUNT;
	DMA1_Channel3->CMAR = (uint32_t)&dotmatrix_framebuffer[multiplex_index][0];
	NVIC_EnableIRQ(DMA1_Channel1_IRQn);

	// Timer Config for multiplexing
	// TIM CLK = 48 MHz / (47 + 1) / (999 + 1) = 1kHz
	RCC->APB1ENR |= RCC_APB1ENR_TIM14EN;
	TIM14->PSC = 470;
	TIM14->ARR = 9999;
	TIM14->DIER |= TIM_DIER_UIE;	// enable update interrupt
	NVIC_EnableIRQ(TIM14_IRQn);
	//TIM14->CR1 |= TIM_CR1_CEN;
}

void TIM14_IRQHandler(void) {
	TIM14->SR &= ~TIM_SR_UIF;
	ledmatrix_oe(LEDMATRIX_OE_OFF);
	ledmatrix_enable_row(multiplex_index);
	DMA1_Channel3->CNDTR = LED_MODULE_RGB * LED_MODULE_COUNT;
	DMA1_Channel3->CMAR = (uint32_t)&dotmatrix_framebuffer[multiplex_index][0];
	DMA1_Channel3->CPAR = (uint32_t)&(SPI1->DR);
	DMA1_Channel3->CCR |= DMA_CCR_EN;
	multiplex_index++;
	if (multiplex_index >= LED_MODULE_ROWS) {multiplex_index = 0;};
}

void DMA1_Channel3_IRQHandler(void) {
	if (DMA1->ISR & DMA_ISR_TCIF3) {
		DMA1->IFCR |= DMA_IFCR_CTCIF3;
		DMA1_Channel3->CCR &= ~DMA_CCR_EN;
		while(SPI1->SR & SPI_SR_BSY);
		ledmatrix_latch();
		ledmatrix_oe(LEDMATRIX_OE_ON);
	}
}

void ledmatrix_oe(uint8_t enable) {
	GPIOA->BSRR = enable ? GPIO_BSRR_BR_6 : GPIO_BSRR_BS_6;
}

void ledmatrix_latch(void) {
	GPIOA->BSRR = GPIO_BSRR_BR_7;
	GPIOA->BSRR = GPIO_BSRR_BS_7;
}

void ledmatrix_enable_row(uint8_t index) {
	GPIOA->BSRR |= (index & 0x1) ? GPIO_BSRR_BS_0 : GPIO_BSRR_BR_0;
	GPIOA->BSRR |= (index & 0x2) ? GPIO_BSRR_BS_1 : GPIO_BSRR_BR_1;
	GPIOA->BSRR |= (index & 0x4) ? GPIO_BSRR_BS_2 : GPIO_BSRR_BR_2;
}

void ledmatrix_pixel(uint8_t x, uint8_t y, uint8_t color) {
	dotmatrix_framebuffer[y][x / 8] = (1 << (x % 8));
}
