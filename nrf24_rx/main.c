#include <stdint.h>
#include <stdio.h>
#include "string.h"
#include "include/spi.h"
#include "include/nrf24.h"

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

uint8_t status;
uint8_t buf[64];

volatile uint8_t interrupt = 0;
volatile uint8_t gpio_status = 0;

void putchar(char c) {
	while(!(UART1_SR & UART_SR_TXE));
	UART1_DR = c;
}

void spi_transfer(uint8_t *byte) {
    while(!(SPI_SR & SPI_SR_TXE));
    SPI_DR = *byte;     // derefrence byte

    while(!(SPI_SR & SPI_SR_RXNE));
    *byte = SPI_DR;     // derefrence byte
}

uint8_t nrf24_send_command(uint8_t command, void *ptrdata, uint8_t len) {
    uint8_t status, i;
    uint8_t *data = ptrdata;

    PA_ODR &= ~(1 << 3); // PA3 Pulldown

    spi_transfer(&command);
    status = command;       // command var set by pointer

    for (i = 0; i < len; i++) {
        spi_transfer(&data[i]);
    }

    PA_ODR |= (1 << 3); // PA3 Pullup

    return status;
}

void nrf24_begin() {
    // clear buffers
    nrf24_send_command(NRF24_FLUSH_TX, 0, 0);
    nrf24_send_command(NRF24_FLUSH_RX, 0, 0);

    // clear interrupts
    buf[0] = NRF24_RX_DR | NRF24_TX_DS | NRF24_MAX_RT;
    status = nrf24_send_command(NRF24_W_REGISTER | NRF24_STATUS, &buf, 1);

    //buf[0] = NRF24_EN_CRC | NRF24_CRCO | NRF24_PWR_UP | NRF24_PRIM_RX;
	buf[0] = NRF24_PWR_UP | NRF24_PRIM_RX;

    status = nrf24_send_command(NRF24_W_REGISTER | NRF24_CONFIG, &buf, 1);

	buf[0] = 0x05;
	nrf24_send_command(NRF24_W_REGISTER | NRF24_RX_PW_P0, &buf, 1);

    buf[0] = 0x00;
    status = nrf24_send_command(NRF24_W_REGISTER | NRF24_EN_AA, &buf, 1);
}

void nrf24_listen() {
    buf[0] = 0x01; buf[1] = 0x01; buf[2] = 0x01; buf[3] = 0x01; buf[4] = 0x01;
    status = nrf24_send_command(NRF24_W_REGISTER | NRF24_RX_ADDR_P0, &buf, 5);

	buf[0] |= (1 << 0) | (1 << 1);
	status = nrf24_send_command(NRF24_W_REGISTER | NRF24_EN_RXADDR, &buf, 1);

    status = nrf24_send_command(NRF24_R_REGISTER | NRF24_EN_RXADDR, &buf, 1);

	PA_ODR |= (1 << 1);
}

void main(void) {
    // Clock
    CLK_DIVR = 0x00;    // Set the frequency to 16 MHz
    CLK_PCKENR1 = 0xFF; // Enable peripherals

    // UART
    UART1_CR2 = UART_CR2_TEN;
    UART1_CR3 &= ~(UART_CR3_STOP1 | UART_CR3_STOP2); // 1 stop bit
    UART1_BRR2 = 0x03; UART1_BRR1 = 0x68; // 9600 baud

    PA_DDR |= (1 << 1) | (1 << 3);  // PA1 OUT, PA2 IN, PA3 OUT
    PA_CR1 |= (1 << 1) | (1 << 2) | (1 << 3);   // PA1 Push-Pull, PA2 Pull-Up , PA3 Push-Pull
    PA_CR2 |= (1 << 2);                         // PA2 Interrupt
    PA_ODR |= (1 << 3); // PA3 Pullup

    // SPI
    SPI_CR1 = SPI_CR1_SPE | SPI_CR1_MSTR;    // SPI Enabled, SPI Master Mode

    __asm__("rim"); // enable interrupt


    nrf24_begin();
    nrf24_listen();

    while(1) {
		if(interrupt) {
			interrupt = 0;
			// check pin low IRQ is active low
			if((gpio_status & (1 << 2)) == 0) {

				PA_ODR &= ~(1 << 1);

				status = nrf24_send_command(NRF24_R_REGISTER | NRF24_STATUS, &buf, 1);
				printf("Status: %02x\r\n", status);

				buf[0] = (1 << 6) | status;
				status = nrf24_send_command(NRF24_W_REGISTER | NRF24_STATUS, &buf, 1);

				status = nrf24_send_command(NRF24_R_REGISTER | NRF24_STATUS, &buf, 1);
				printf("Clear Interrupt Status: %02x\r\n", status);

				status = nrf24_send_command(NRF24_R_REGISTER | NRF24_R_RX_PAYLOAD, &buf, 5);
				printf("Payload: %02x %02x %02x %02x %02x\r\n", buf[0], buf[1], buf[2], buf[3], buf[4]);

				status = nrf24_send_command(NRF24_R_REGISTER | NRF24_STATUS, &buf, 1);
				printf("Clear Interrupt Status: %02x\r\n", status);
			}
		}

		PA_CR2 |= (1 << 2);
		nrf24_listen();
    }
}

// Port A external interrupts
void PA_external_isr() __interrupt(0x03) {
	PA_CR2 &= ~(1 << 2);
	gpio_status = PA_IDR;
	interrupt = 1;
}
