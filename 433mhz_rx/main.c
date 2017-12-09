#include <stdint.h>
#include <stdio.h>

#define CLK_DIVR	(*(volatile uint8_t *)0x50c6)
#define CLK_PCKENR1	(*(volatile uint8_t *)0x50c7)
#define PA_ODR      (*(volatile uint8_t *)0x5000)
#define PA_IDR      (*(volatile uint8_t *)0x5001)
#define PA_DDR      (*(volatile uint8_t *)0x5002)
#define PA_CR1      (*(volatile uint8_t *)0x5003)
#define PA_CR2      (*(volatile uint8_t *)0x5004)

#define PA0        (1 << 0)
#define PA1        (1 << 1)

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

volatile uint8_t interrupt;
volatile uint8_t gpio_status;

void putchar(char c) {
	while(!(UART1_SR & UART_SR_TXE));
	UART1_DR = c;
}

void main(void) {
    // Clock
    CLK_DIVR = 0x00;    // Set the frequency to 16 MHz
    CLK_PCKENR1 = 0xFF; // Enable peripherals

    // UART
    UART1_CR2 = UART_CR2_TEN;
    UART1_CR3 &= ~(UART_CR3_STOP1 | UART_CR3_STOP2); // 1 stop bit
    UART1_BRR2 = 0x03; UART1_BRR1 = 0x68; // 9600 baud

    //PA_CR1 |= PA1;  // PA1 Input
    PA_CR2 |= PA1;  // PA1 External interrupt

    interrupt = 0;
    gpio_status = 0;

    __asm__("rim"); // enable interrupts

    while(1) {
        if(interrupt) {
            interrupt = 0;
            if((gpio_status & PA1) == PA1) {
                printf("Interrupt Positive Edge\r\n");
            }

            if((gpio_status & PA1) == 0) {
                printf("Interrupt Negative Edge\r\n");
            }

            PA_CR2 &= ~(PA0);
        }
    }
}

// Port A external interrupts
void PA_external_isr() __interrupt(0x03) {
	PA_CR2 &= ~(PA0);
	gpio_status = PA_IDR;
	interrupt = 1;
}
