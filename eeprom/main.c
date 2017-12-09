#include <stdint.h>
#include <stdio.h>
#include "include/i2c.h"

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

#define I2C_CR1     (*(volatile uint8_t *)0x5210)
#define I2C_CR2     (*(volatile uint8_t *)0x5211)
#define I2C_FREQR   (*(volatile uint8_t *)0x5212)
#define I2C_DR      (*(volatile uint8_t *)0x5216)
#define I2C_SR1     (*(volatile uint8_t *)0x5217)
#define I2C_SR2     (*(volatile uint8_t *)0x5218)
#define I2C_SR3     (*(volatile uint8_t *)0x5219)
#define I2C_SR3     (*(volatile uint8_t *)0x5219)
#define I2C_CCRL    (*(volatile uint8_t *)0x521B)
#define I2C_CCRH    (*(volatile uint8_t *)0x521C)

#define I2C_PE      (1 << 0)

uint8_t buf[32];
uint8_t addr;
uint8_t r;


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
}

void i2c_init() {
	I2C_CR1 &= ~I2C_CR1_PE;		// Disable I2C
	I2C_FREQR = 0x16;
	I2C_CCRH = 0x00;
	I2C_CCRL = 0x50;
	I2C_TRISER = 0x11;
	I2C_OARH |= (1 << 6);
	I2C_OARH &= ~(1 << 7);
	I2C_CR1 |= I2C_CR1_PE;
}

void i2c_cmd(uint8_t byte) {
	I2C_CR2 |= I2C_CR2_START;			// Set start bit
	while(!(I2C_SR1 & I2C_SR1_SB));		// Wait for start bit

	I2C_DR = (0x50 << 1) | 0;			// Write to address 0x3c
	while(!(I2C_SR1 & I2C_SR1_ADDR));	// Wait for ack
	r = I2C_SR3;						// Clear SR3 register

	I2C_DR = byte;						// Send 0xAF;

	while(!(I2C_SR1 & I2C_SR1_TXE));	// Wait for TXE
	I2C_CR2 |= I2C_CR2_STOP;			// Set STOP bit
}

void eeprom_write(uint16_t addr, uint8_t data) {
	I2C_CR2 |= I2C_CR2_START;			// Set start bit
	while(!(I2C_SR1 & I2C_SR1_SB));		// Wait for start bit

	I2C_DR = (0x50 << 1) | 0;			// Write to address 0x3c
	while(!(I2C_SR1 & I2C_SR1_ADDR));	// Wait for ack
	r = I2C_SR3;						// Clear SR3 register
	I2C_DR = (addr >> 8);				// high location byte

	while(!(I2C_SR1 & I2C_SR1_TXE));	// Wait for TXE
	I2C_DR = (addr & 0xff);						// low location byte

	while(!(I2C_SR1 & I2C_SR1_TXE));	// Wait for TXE
	I2C_DR = data;						// data to write to address

	while(!(I2C_SR1 & I2C_SR1_TXE));	// Wait for TXE
	I2C_CR2 |= I2C_CR2_STOP;
}

void eeprom_read(uint16_t addr, uint8_t *data) {
	I2C_CR2 |= I2C_CR2_START;			// Set start bit
	while(!(I2C_SR1 & I2C_SR1_SB));		// Wait for start bit

	I2C_DR = (0x50 << 1) | 0;			// Write to address 0x3c
	while(!(I2C_SR1 & I2C_SR1_ADDR));	// Wait for ack
	r = I2C_SR3;						// Clear SR3 register

	I2C_DR = (addr >> 8);				// high location byte

	while(!(I2C_SR1 & I2C_SR1_TXE));	// Wait for TXE
	I2C_DR = (addr & 0xff);				// low location byte

	while(!(I2C_SR1 & I2C_SR1_TXE));	// Wait for TXE
	I2C_CR2 |= I2C_CR2_START;			// Set start bit

	while(!(I2C_SR1 & I2C_SR1_SB));		// Wait for start bit

	I2C_DR = (0x50 << 1) | 1;			// Write to address 0x3c
	while(!(I2C_SR1 & I2C_SR1_ADDR));	// Wait for ack
	r = I2C_SR3;						// Clear SR3 register

	while(!(I2C_SR1 & I2C_SR1_RXNE));
	*data = I2C_DR;

	I2C_CR2 |= I2C_CR2_STOP;
}


void main(void) {
	uint8_t data;
	uint16_t i;
    init();
	i2c_init();

	/*
	for(i = 0; i < 127; i++) {
		eeprom_write(0x00 + i, i);
		printf("Written: %02x\r\n", i);
	}
	*/

	for(i = 0; i < 0x7FFF; i++) {
		eeprom_read(0x00 + i, &data);
		printf("Read: %02x\r\n", data);
	}

    while(1) {}
}
