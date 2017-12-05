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

uint8_t *dataptr;   // pointer

void putchar(char c) {
	while(!(UART1_SR & UART_SR_TXE));
	UART1_DR = c;
}

void spi_transfer(uint8_t *byte) {
    //printf("TRANSFER BYTE ADDR: %p\r\n", byte);
    while(!(SPI_SR & SPI_SR_TXE));
    SPI_DR = *byte;     // derefrence byte

    while(!(SPI_SR & SPI_SR_RXNE));
    *byte = SPI_DR;     // derefrence byte
}

void nrf24_send_command(uint8_t command, uint8_t data[], uint8_t len) {
    uint8_t i;

    spi_transfer(&command);
    // status = *command;

    for (i = 0; i < len; i++)
        spi_transfer(&data[i]);
}

void main(void) {
    uint8_t data[5];
    uint8_t data1[5];

    // Clock
    CLK_DIVR = 0x00;    // Set the frequency to 16 MHz
    CLK_PCKENR1 = 0xFF; // Enable peripherals

    // UART
    UART1_CR2 = UART_CR2_TEN;
    UART1_CR3 &= ~(UART_CR3_STOP1 | UART_CR3_STOP2); // 1 stop bit
    UART1_BRR2 = 0x03; UART1_BRR1 = 0x68; // 9600 baud

    // GPIO
    PA_DDR |= (1 << 3); // PA3 Output
    PA_CR1 |= (1 << 3); // PA3 Push-Pull
    PA_ODR |= (1 << 3); // PA3 Pullup

    // SPI
    SPI_CR1 = SPI_CR1_SPE | SPI_CR1_MSTR;    // SPI Enabled, SPI Master Mode

    data[0] = 0x10; data[1] = 0x01; data[2] = 0x02; data[3] = 0xe0; data[4] = 0xe1;
    data1[0] = 0x00; data1[1] = 0x00; data1[2] = 0x00; data1[3] = 0x00; data1[4] = 0x00;

    printf("Data Address: [0] %p, [1] %p, [2] %p\r\n", &data[0], &data[1], &data[2]);

    nrf24_send_command(NRF24_W_REGISTER | NRF24_CONFIG, data, 5);

    printf("Bytes: [0] %02x, [1] %02x, [2] %02x\r\n", data[0], data[1], data[2]);


    while(1) {};

/*

    while(1) {
        uint8_t data[5];
        uint8_t addr[5];
        uint8_t config;
        uint8_t data1[5];

        // Clear transmit interrupts
        data[0] = NRF24_RX_DR | NRF24_TX_DS | NRF24_MAX_RT;      // Clear transmit interrupts
        nrf24_write_register(NRF24_STATUS, &data, 1);

        addr[0] = 0x00; addr[1] = 0x00; addr[2] = 0x00; addr[3] = 0x00; addr[4] = 0x00;
        nrf24_write_register(NRF24_TX_ADDR, &addr, 5);
        nrf24_write_register(NRF24_RX_ADDR_P0, &addr, 5);

        data[0] = 0x01; data[1] = 0x02; data[2] = 0x03; data[3] = 0x04; data[4] = 0x05;
        nrf24_send_command(NRF24_W_TX_PAYLOAD, &data, 5);


        nrf24_read_register(NRF24_CONFIG, &config, 1);
        config &= 0x00;
        nrf24_write_register(NRF24_CONFIG, &config, 1);


        data1[0] = 0x00; data1[1] = 0x00; data1[2] = 0x00; data1[3] = 0x00; data1[4] = 0x00;

        nrf24_send_command(0x00 | NRF24_TX_ADDR, &data1, 5);

        printf("%02x %02x %02x %02x %02x\r\n", data1[0], data1[1], data1[2], data1[3], data1[4]);
    }

*/
}
