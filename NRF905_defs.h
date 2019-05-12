/*
 * NRF905_defs.h
 *
 *  Created on: Jul 29, 2018
 *      Author: Mourad-Rashwan
 */

#ifndef NRF905_DEFS_H_
#define NRF905_DEFS_H_

#define PORT_CLK_BASE(X)    (SYSCTL_PERIPH_GPIO ## X)
#define PORT_ADDR_BASE(X)   (GPIO_PORT ## X ## _BASE)
#define PORT_DATA_REG(X)    (GPIO_PORT ## X ## _DATA_R)

#define NRF905_CALC_CHANNEL(f, b)   ((((f) / (1 + (b>>1))) - 422400000ULL) / 100000ULL) ///< Workout channel from frequency & band

#if !NRF905_USE_PWR
#define POWERED_UP()    (1)
#define POWER_UP()      (void)(0)
#define POWER_DOWN()    (void)(0)
#endif

#if NRF905_USE_PWR
#define POWERED_UP()    ((NRF905_PWR_MODE_PORT_DATA & (1 << NRF905_PWR_MODE_BIT_NUM)) >> NRF905_PWR_MODE_BIT_NUM)
#define POWER_UP()      (NRF905_PWR_MODE_PORT_DATA |= (1 << NRF905_PWR_MODE_BIT_NUM))
#define POWER_DOWN()    (NRF905_PWR_MODE_PORT_DATA &= ~(1 << NRF905_PWR_MODE_BIT_NUM))
#endif

#define DATA_READY()        ((NRF905_DR_PORT_DATA & (1 << NRF905_DR_BIT_NUM)) >> NRF905_DR_BIT_NUM)
#define ADDRESS_MATCHED()   ((NRF905_AM_PORT_DATA & (1 << NRF905_AM_BIT_NUM)) >> NRF905_AM_BIT_NUM)
#define CARRIER_DETECTED()  ((NRF905_CD_PORT_DATA & (1 << NRF905_CD_BIT_NUM)) >> NRF905_CD_BIT_NUM)

#define STANDBY_LEAVE() (NRF905_TRX_EN_PORT_DATA |= (1 << NRF905_TRX_EN_BIT_NUM))
#define STANDBY_ENTER() (NRF905_TRX_EN_PORT_DATA &= ~(1 << NRF905_TRX_EN_BIT_NUM))
#define MODE_TX()       (NRF905_TX_EN_PORT_DATA |= (1 << NRF905_TX_EN_BIT_NUM))
#define MODE_RX()       (NRF905_TX_EN_PORT_DATA &= ~(1 << NRF905_TX_EN_BIT_NUM))
#define SPI_DESELECT()  (NRF905_CSN_PORT_DATA |= (1 << NRF905_CSN_BIT_NUM))
#define SPI_SELECT()    (NRF905_CSN_PORT_DATA &= ~(1 << NRF905_CSN_BIT_NUM))

// Instructions
#define NRF905_CMD_NOP			0xFF
#define NRF905_CMD_W_CONFIG		0x00   // write config starting from address 0  // 0x0X  // X is the starting address byte 0
#define NRF905_CMD_R_CONFIG		0x10   // read config starting from address 0   // 0x1X  // X is the starting address byte 0
#define NRF905_CMD_W_TX_PAYLOAD	0x20
#define NRF905_CMD_R_TX_PAYLOAD	0x21
#define NRF905_CMD_W_TX_ADDRESS	0x22
#define NRF905_CMD_R_TX_ADDRESS	0x23
#define NRF905_CMD_R_RX_PAYLOAD	0x24
#define NRF905_CMD_CHAN_CONFIG	0x80

// Registers
#define NRF905_REG_CHANNEL			0x00
#define NRF905_REG_CONFIG1			0x01
#define NRF905_REG_ADDR_WIDTH		0x02
#define NRF905_REG_RX_PAYLOAD_SIZE	0x03
#define NRF905_REG_TX_PAYLOAD_SIZE	0x04
#define NRF905_REG_RX_ADDRESS		0x05
#define NRF905_REG_CONFIG2			0x09

// TODO remove
#define NRF905_REG_AUTO_RETRAN	NRF905_REG_CONFIG1
#define NRF905_REG_LOW_RX		NRF905_REG_CONFIG1
#define NRF905_REG_PWR			NRF905_REG_CONFIG1
#define NRF905_REG_BAND			NRF905_REG_CONFIG1
#define NRF905_REG_CRC			NRF905_REG_CONFIG2
#define NRF905_REG_CLK			NRF905_REG_CONFIG2
#define NRF905_REG_OUTCLK		NRF905_REG_CONFIG2
#define NRF905_REG_OUTCLK_FREQ	NRF905_REG_CONFIG2

// Register masks
#define NRF905_MASK_CHANNEL		0xFE
#define NRF905_MASK_AUTO_RETRAN	~(NRF905_AUTO_RETRAN_ENABLE | NRF905_AUTO_RETRAN_DISABLE) //0xDF
#define NRF905_MASK_LOW_RX		~(NRF905_LOW_RX_ENABLE | NRF905_LOW_RX_DISABLE) //0xEF
#define NRF905_MASK_PWR			~(NRF905_PWR_n10 | NRF905_PWR_n2 | NRF905_PWR_6 | NRF905_PWR_10) //0xF3
#define NRF905_MASK_BAND		~(NRF905_BAND_433 | NRF905_BAND_868 | NRF905_BAND_915) //0xFD
#define NRF905_MASK_CRC			(uint8_t)(~(NRF905_CRC_DISABLE | NRF905_CRC_8 | NRF905_CRC_16)) //0x3F // typecast to stop compiler moaning about large integer truncation
#define NRF905_MASK_CLK			~(NRF905_CLK_4MHZ | NRF905_CLK_8MHZ | NRF905_CLK_12MHZ | NRF905_CLK_16MHZ | NRF905_CLK_20MHZ) //0xC7
#define NRF905_MASK_OUTCLK		~(NRF905_OUTCLK_DISABLE | NRF905_OUTCLK_4MHZ | NRF905_OUTCLK_2MHZ | NRF905_OUTCLK_1MHZ | NRF905_OUTCLK_500KHZ) // 0xF8

// Bit positions // don't change them
#define NRF905_STATUS_DR		5
#define NRF905_STATUS_AM		7
#define NRF905_CONFIG_REG_SIZE   10 ///< Configuration register count // 10 bytes
#define NRF905_MAX_CHANNEL      511
#define NRF905_MAX_PAYLOAD      32 ///< Maximum payload size

#if (NRF905_DR_SW && NRF905_INTERRUPTS)
#error "NRF905_INTERRUPTS and NRF905_DR_SW cannot both be enabled"
#elif (NRF905_AM_SW && NRF905_INTERRUPTS_AM)
#error "NRF905_INTERRUPTS_AM and NRF905_AM_SW cannot both be enabled"
#elif (!NRF905_INTERRUPTS && NRF905_INTERRUPTS_AM)
#error "NRF905_INTERRUPTS_AM cannot be enabled without NRF905_INTERRUPTS"
#endif

/**
 * @brief Available modes after transmission complete.
 */
typedef enum
{
    NRF905_NEXTMODE_STANDBY, ///< Standby mode
    NRF905_NEXTMODE_RX, ///< Receive mode
    NRF905_NEXTMODE_TX ///< Transmit mode (will auto-retransmit if ::NRF905_AUTO_RETRAN is ::NRF905_AUTO_RETRAN_ENABLE, otherwise will transmit a carrier wave with no data)
} NRF905_nextmode_t;

/**
 * @brief Frequency bands.
 */
typedef enum
{
// NOTE:
// When using NRF905_BAND_868 and NRF905_BAND_915 for calculating channel (NRF905_CALC_CHANNEL(f, b)) they should be value 0x01,
// but when using them for setting registers their value should be 0x02.
// They're defined as 0x02 here so when used for calculating channel they're right shifted by 1

    NRF905_BAND_433 = 0x00, ///< 433MHz band
    NRF905_BAND_868 = 0x02, ///< 868/915MHz band
    NRF905_BAND_915 = 0x02  ///< 868/915MHz band
} NRF905_band_t;

/**
 * @brief Output power (n means negative, n10 = -10).
 */
typedef enum
{
    NRF905_PWR_n10 = 0x00,  ///< -10dBm = 100uW
    NRF905_PWR_n2 = 0x04,   ///< -2dBm = 631uW
    NRF905_PWR_6 = 0x08,    ///< 6dBm = 4mW
    NRF905_PWR_10 = 0x0C    ///< 10dBm = 10mW
} NRF905_pwr_t;

/**
 * @brief Save a few mA by reducing receive sensitivity.
 */
typedef enum
{
    NRF905_LOW_RX_DISABLE = 0x00,   ///< Disable low power receive
    NRF905_LOW_RX_ENABLE = 0x10     ///< Enable low power receive
} NRF905_low_rx_t;

/**
 * @brief Auto re-transmit options.
 */
typedef enum
{
    NRF905_AUTO_RETRAN_DISABLE = 0x00,  ///< Disable auto re-transmit
    NRF905_AUTO_RETRAN_ENABLE = 0x20    ///< Enable auto re-transmit
} NRF905_auto_retran_t;

/**
 * @brief Output a clock signal on pin 3 of IC.
 */
typedef enum
{
    NRF905_OUTCLK_DISABLE = 0x00,   ///< Disable output clock
    NRF905_OUTCLK_4MHZ = 0x04,      ///< 4MHz clock
    NRF905_OUTCLK_2MHZ = 0x05,      ///< 2MHz clock
    NRF905_OUTCLK_1MHZ = 0x06,      ///< 1MHz clock
    NRF905_OUTCLK_500KHZ = 0x07,    ///< 500KHz clock (default)
} NRF905_outclk_t;

/**
 * @brief CRC Checksum.
 *
 * The CRC is calculated across the address (SYNC word) and payload
 */
typedef enum
{
    NRF905_CRC_DISABLE = 0x00,  ///< Disable CRC
    NRF905_CRC_8 = 0x40, ///< 8bit CRC (Don't know what algorithm is used for this one)
    NRF905_CRC_16 = 0xC0,       ///< 16bit CRC (CRC16-CCITT-FALSE (0xFFFF))
} NRF905_crc_t;

typedef enum
{
    NRF905_CLK_4MHZ = 0x00,
    NRF905_CLK_8MHZ = 0x08,
    NRF905_CLK_12MHZ = 0x10,
    NRF905_CLK_16MHZ = 0x18,
    NRF905_CLK_20MHZ = 0x20,
} NRF905_crystal_t;

/**
 * @brief Address size.
 *
 * This is actually used as the SYNC word
 */
typedef enum
{
    NRF905_ADDR_SIZE_1 = 0x01, ///< 1 byte (not recommended, a lot of false invalid packets will be received)
    NRF905_ADDR_SIZE_4 = 0x04,  ///< 4 bytes
} NRF905_addr_size_t;

#endif /* NRF905_DEFS_H_ */
