// SPI Registers
#define SPI_CR1     (*(volatile uint8_t *)0x5200)
#define SPI_CR2     (*(volatile uint8_t *)0x5201)
#define SPI_ICR     (*(volatile uint8_t *)0x5202)
#define SPI_SR      (*(volatile uint8_t *)0x5203)
#define SPI_DR      (*(volatile uint8_t *)0x5204)
#define SPI_CRCPR   (*(volatile uint8_t *)0x5205)
#define SPI_RXCRCR  (*(volatile uint8_t *)0x5205)
#define SPI_TXCRCR  (*(volatile uint8_t *)0x5205)

// SPI Bits
#define SPI_CR1_LSBFIRST    (1 << 7)
#define SPI_CR1_SPE         (1 << 6)
#define SPI_CR1_BR2         (1 << 5)
#define SPI_CR1_BR1         (1 << 4)
#define SPI_CR1_BR0         (1 << 3)
#define SPI_CR1_MSTR        (1 << 2)
#define SPI_CR1_CPOL        (1 << 1)
#define SPI_CR1_CPHA        (1 << 0)

#define SPI_CR2_BDM         (1 << 7)
#define SPI_CR2_BDOE        (1 << 6)
#define SPI_CR2_CREN        (1 << 5)
#define SPI_CR2_CRCNEXT     (1 << 4)
// SPI_CR2 Bit 3 Reserved
#define SPI_CR2_RXONLY      (1 << 2)
#define SPI_CR2_SSM         (1 << 1)
#define SPI_CR2_SSI         (1 << 0)

#define SPI_ICR_TXIE        (1 << 7)
#define SPI_ICR_RXIE        (1 << 6)
#define SPI_ICR_ERRIE       (1 << 5)
#define SPI_ICR_WKIE        (1 << 4)
// SPI_ICR Bit 3:0 Reserved

#define SPI_SR_BSY          (1 << 7)
#define SPI_SR_OVR          (1 << 6)
#define SPI_SR_MODF         (1 << 5)
#define SPI_SR_CRCERR       (1 << 4)
#define SPI_SR_WKUP         (1 << 3)
// SPI_SR Bit 2 Reserved
#define SPI_SR_TXE          (1 << 1)
#define SPI_SR_RXNE         (1 << 0)
