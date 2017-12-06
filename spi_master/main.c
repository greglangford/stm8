#include <stdint.h>
#include <stdio.h>
#include "string.h"
#include "include/spi.h"
#include "include/nrf24.h"

#define CLK_DIVR	(*(volatile uint8_t *)0x50c6)
#define CLK_PCKENR1	(*(volatile uint8_t *)0x50c7)
#define PA_ODR      (*(volatile uint8_t *)0x5000)
#define PA_DDR      (*(volatile uint8_t *)0x5002)
#define PA_CR1      (*(volatile uint8_t *)0x5003)

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

    buf[0] = NRF24_EN_CRC | NRF24_CRCO | NRF24_PWR_UP | NRF24_PRIM_RX;
    status = nrf24_send_command(NRF24_W_REGISTER | NRF24_CONFIG, &buf, 1);

    buf[0] = 0x00;
    status = nrf24_send_command(NRF24_W_REGISTER | NRF24_EN_AA, &buf, 1);
}

void main(void) {
    int x, i;
    // Clock
    CLK_DIVR = 0x00;    // Set the frequency to 16 MHz
    CLK_PCKENR1 = 0xFF; // Enable peripherals

    // UART
    UART1_CR2 = UART_CR2_TEN;
    UART1_CR3 &= ~(UART_CR3_STOP1 | UART_CR3_STOP2); // 1 stop bit
    UART1_BRR2 = 0x03; UART1_BRR1 = 0x68; // 9600 baud

    // GPIO
    PA_DDR |= (1 << 1) | (1 << 3);  // PA3 Output
    PA_CR1 |= (1 << 1) | (1 << 3);  // PA3 Push-Pull
    PA_ODR |= ~(1 << 1) | (1 << 3); // PA3 Pullup

    // SPI
    SPI_CR1 = SPI_CR1_SPE | SPI_CR1_MSTR;    // SPI Enabled, SPI Master Mode

    nrf24_begin();
    while(1) {
        // clear interrupts
        buf[0] = NRF24_TX_DS | NRF24_MAX_RT;
        status = nrf24_send_command(NRF24_W_REGISTER | NRF24_STATUS, &buf, 1);

        printf("Status Start:\t%02x\r\n", status);

        buf[0] = 0x00; buf[1] = 0x00; buf[2] = 0x00; buf[3] = 0x00; buf[4] = 0x00;
        status = nrf24_send_command(NRF24_W_REGISTER | NRF24_TX_ADDR, &buf, 5);
        //printf("Set tx addr:\t\t%02x\r\n", status);

        buf[0] = 0x00; buf[1] = 0x00; buf[2] = 0x00; buf[3] = 0x00; buf[4] = 0x00;
        status = nrf24_send_command(NRF24_W_REGISTER | NRF24_RX_ADDR_P0, &buf, 5);
        //printf("Set rx addr P0:\t\t%02x\r\n", status);

        buf[0] = 0x02; buf[1] = 0x01; buf[2] = 0x02; buf[3] = 0x03; buf[4] = 0x05;
        status = nrf24_send_command(NRF24_W_TX_PAYLOAD, &buf, 5);
        //printf("Set rx addr P0:\t\t%02x\r\n", status);

        status = nrf24_send_command(NRF24_R_REGISTER | NRF24_CONFIG, &buf, 1);

        buf[0] &= ~NRF24_PRIM_RX;
        status = nrf24_send_command(NRF24_W_REGISTER | NRF24_CONFIG, &buf, 1);

        PA_ODR |= (1 << 1);
        for(i = 0; i < 5000; i++) {
            for(x = 0; x < 100; x++) {}
        }
        PA_ODR &= ~(1 << 1);

        status = nrf24_send_command(NRF24_R_REGISTER | NRF24_STATUS, 0, 0);
        printf("Status Packet:\t%02x\r\n", status);

        buf[0] = NRF24_TX_DS | NRF24_MAX_RT;
        status = nrf24_send_command(NRF24_W_REGISTER | NRF24_STATUS, &buf, 1);

        status = nrf24_send_command(NRF24_R_REGISTER | NRF24_STATUS, 0, 0);
        printf("Status Clear:\t%02x\r\n", status);

        printf("\r\n\r\n");
    };
}
