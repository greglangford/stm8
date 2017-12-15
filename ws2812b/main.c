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

volatile uint8_t timer_done;
volatile int timer_count;

void (*timer_func)();

void putchar(char c) {
	while(!(UART1_SR & UART_SR_TXE));
	UART1_DR = c;
}

void init(void) {
    // Clock
    CLK_DIVR = 0x00;    // Set the frequency to 16 MHz
    CLK_PCKENR1 = 0xFF; // Enable peripherals

    // UART
    UART1_CR2 = UART_CR2_TEN;
    UART1_CR3 &= ~(UART_CR3_STOP1 | UART_CR3_STOP2); // 1 stop bit
    UART1_BRR2 = 0x03; UART1_BRR1 = 0x68; // 9600 baud

    PA_DDR = (1 << 1);  // Output
    PA_CR1 = (1 << 1);  // Push Pull
    PA_ODR = 0x00;

	__asm__("rim");
}

void flash_led() {
	PA_ODR ^= 0xFF;
}

void timer_delay(uint8_t prescaler, uint8_t arr, uint16_t count, void *timer_func_cb) {
	if(!(TIM4_CR1 & (1 << 0))) {
		timer_func = timer_func_cb;
		timer_count = count;
		TIM4_PSCR = prescaler;		// Divide clock
		TIM4_ARR = arr;				// Count to
		TIM4_IER = (1 << 0);		// Interrupt enable
		TIM4_CR1 = (1 << 0);		// Enable timer
	}
}

void main(void) {
    init();

    while(1) {
		timer_delay(0x07, 255, 50, &flash_led);
	}
}

void tim4_isr(void) __interrupt(23) {
	TIM4_SR &= ~(1 << 0);

	timer_count--;

	if(timer_count < 1) {
		TIM4_CR1 &= ~(1 << 0);
		timer_func();
	}
}
