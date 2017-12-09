// Registers
#define I2C_CR1     (*(volatile uint8_t *)0x5210)
#define I2C_CR2     (*(volatile uint8_t *)0x5211)
#define I2C_FREQR   (*(volatile uint8_t *)0x5212)
#define I2C_OARL    (*(volatile uint8_t *)0x5213)
#define I2C_OARH    (*(volatile uint8_t *)0x5214)
#define I2C_DR      (*(volatile uint8_t *)0x5216)
#define I2C_SR1     (*(volatile uint8_t *)0x5217)
#define I2C_SR2     (*(volatile uint8_t *)0x5218)
#define I2C_SR3     (*(volatile uint8_t *)0x5219)
#define I2C_ITR     (*(volatile uint8_t *)0x521A)
#define I2C_CCRL    (*(volatile uint8_t *)0x521B)
#define I2C_CCRH    (*(volatile uint8_t *)0x521C)
#define I2C_TRISER  (*(volatile uint8_t *)0x521D)
#define I2C_PECR    (*(volatile uint8_t *)0x521E)

// I2C_CR1 Bits
#define I2C_CR1_PE          (1 << 0)
#define I2C_CR1_ENGC        (1 << 6)
#define I2C_CR1_NOSTRETCH   (1 << 7)

// I2C_CR2 Bits
#define I2C_CR2_START       (1 << 0)
#define I2C_CR2_STOP        (1 << 1)
#define I2C_CR2_ACK         (1 << 2)
#define I2C_CR2_POS         (1 << 3)
#define I2C_CR2_SWRST       (1 << 7)

// I2C_FREQR Bits
#define I2C_FREQR_1MHZ      (1 << 0)

// I2C_SR1 Bits
#define I2C_SR1_SB          (1 << 0)
#define I2C_SR1_ADDR        (1 << 1)
#define I2C_SR1_BTF         (1 << 2)
#define I2C_SR1_ADD10       (1 << 3)
#define I2C_SR1_STOPF       (1 << 4)
#define I2C_SR1_RXNE        (1 << 6)
#define I2C_SR1_TXE         (1 << 7)

// I2C_SR2 Bits
#define I2C_SR2_BERR        (1 << 0)
#define I2C_SR2_ARLO        (1 << 1)
#define I2C_SR2_AF          (1 << 2)
#define I2C_SR2_OVR         (1 << 3)
#define I2C_SR2_WUFH        (1 << 5)

// I2C_SR3 Bits
#define I2C_SR3_MSL         (1 << 0)
#define I2C_SR3_BUSY        (1 << 1)
#define I2C_SR3_TRA         (1 << 2)
#define I2C_SR3_GENCALL     (1 << 4)
