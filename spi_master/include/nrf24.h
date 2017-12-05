// SPI Commands

#define NRF24_R_REGISTER    0x00
#define NRF24_W_REGISTER    0x20
#define NRF24_W_TX_PAYLOAD  0xA0
#define NRF24_FLUSH_TX      0xE1
#define NRF24_FLUSH_RX      0xE2

// Register Map

#define NRF24_RX_DR         (1 << 6)
#define NRF24_TX_DS         (1 << 5)
#define NRF24_MAX_RT        (1 << 4)

#define NRF24_CONFIG        0x00
#define NRF24_STATUS        0x07


#define NRF24_TX_ADDR       0x10

#define NRF24_RX_ADDR_P0    0x0A
