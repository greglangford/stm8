// SPI Commands

#define NRF24_R_REGISTER    0x00
#define NRF24_W_REGISTER    0x20
#define NRF24_R_RX_PAYLOAD  0x61
#define NRF24_W_TX_PAYLOAD  0xA0
#define NRF24_FLUSH_TX      0xE1
#define NRF24_FLUSH_RX      0xE2

// Register Map

#define NRF24_RX_DR         (1 << 6)
#define NRF24_TX_DS         (1 << 5)
#define NRF24_MAX_RT        (1 << 4)
#define NRF24_EN_CRC        (1 << 3)
#define NRF24_CRCO          (1 << 2)
#define NRF24_PWR_UP        (1 << 1)
#define NRF24_PRIM_RX       (1 << 0)

#define NRF24_EN_AA         0x01
#define NRF24_ENAA_P5       (1 << 5)
#define NRF24_ENAA_P4       (1 << 4)
#define NRF24_ENAA_P3       (1 << 3)
#define NRF24_ENAA_P2       (1 << 2)
#define NRF24_ENAA_P1       (1 << 1)
#define NRF24_ENAA_P0       (1 << 0)

#define NRF24_RPD           0x09

#define NRF24_FIFO_STATUS   0x17

#define NRF24_RX_PW_P0      0x11


#define NRF24_CONFIG        0x00
#define NRF24_STATUS        0x07

#define NRF24_TX_ADDR       0x10

#define NRF24_RX_ADDR_P0    0x0A

#define NRF24_EN_RXADDR     0x02
