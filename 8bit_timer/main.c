#include <stdint.h>
#include <stdio.h>

#define CLK_DIVR	(*(volatile uint8_t *)0x50c6)
#define CLK_PCKENR1	(*(volatile uint8_t *)0x50c7)
#define PA_ODR      (*(volatile uint8_t *)0x5000)
#define PA_IDR      (*(volatile uint8_t *)0x5001)
#define PA_DDR      (*(volatile uint8_t *)0x5002)
#define PA_CR1      (*(volatile uint8_t *)0x5003)
#define PA_CR2      (*(volatile uint8_t *)0x5004)

#define UART1_SR	(*(volatile uint8_t *)0x5230)
#define UART1_DR	(*(volatile uint8_t *)0x5231)
#define UART1_BRR1	(*(volatile uint8_t *)0x5232)
#define UART1_BRR2	(*(volatile uint8_t *)0x5233)
#define UART1_CR2	(*(volatile uint8_t *)0x5235)
#define UART1_CR3	(*(volatile uint8_t *)0x5236)

#define UART_CR2_TEN    (1 << 3)
#define UART_CR3_STOP2  (1 << 5)
#define UART_CR3_STOP1  (1 << 4)
#define UART_SR_TXE     (1 << 7)

#define TIM4_CR1    (*(volatile uint8_t *)0x5340)
#define TIM4_IER    (*(volatile uint8_t *)0x5343)
#define TIM4_SR     (*(volatile uint8_t *)0x5344)
#define TIM4_EGR    (*(volatile uint8_t *)0x5345)
#define TIM4_CNTR   (*(volatile uint8_t *)0x5346)
#define TIM4_PSCR   (*(volatile uint8_t *)0x5347)
#define TIM4_ARR    (*(volatile uint8_t *)0x5348)

volatile int tim4count = 0;

void putchar(char c) {
	while(!(UART1_SR & UART_SR_TXE));
	UART1_DR = c;
}

void init(void) {
    // Clock
    CLK_DIVR = 0x00;    // Set the frequency to 16 MHz
    CLK_PCKENR1 = 0xFF; // Enable peripherals

    // UART
    UART1_CR2 = UART_CR2_TEN;							// UART TX
    UART1_CR3 &= ~(UART_CR3_STOP1 | UART_CR3_STOP2);	// 1 stop bit
    UART1_BRR2 = 0x03; UART1_BRR1 = 0x68; 				// 9600 baud

    PA_DDR = (1 << 1);  // Output
    PA_CR1 = (1 << 2);  // Push Pull
    PA_ODR = 0x00;		// Low output
}

void main(void) {
    init();

	// 0x07 = /128 8us per tick, 2048ms per overflow
	// 0x06 = /64 4us per tick, 1024ms per overflow
	// 0x05 = /32 2us per tick, 512us per overflow
	// 0x04 = /16 1us per tick, 256us per overflow
	// 0x03 = /8 0.5us per tick, 128us per overflow
	// 0x02 = /4 0.25us per tick, 64us per overflow
	// 0x01 = /2 0.125us per tick, 32us per overflow
	// 0x00 = /0 0.0625us per tick, 16us per overflow

    TIM4_PSCR = 0x7;		// Divide clock
	TIM4_ARR = 64;			// Count to
    TIM4_IER = (1 << 0);	// Interrupt enable
    TIM4_CR1 = (1 << 0);	// Enable timer

    __asm__("rim");			// Global interrupts

    while(1) {}
}

void tim4_isr(void) __interrupt(23) {
	TIM4_SR &= ~(1 << 0);
	PA_ODR ^= 0xFF;
}
