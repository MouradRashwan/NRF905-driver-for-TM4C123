/*
 * NRF905_config.h
 *
 *  Created on: Jul 29, 2018
 *      Author: Mourad-Rashwan
 */

#ifndef NRF905_CONFIG_H_
#define NRF905_CONFIG_H_

#define NRF905_DEFAULT_RXADDR     0xA1B1C1D1 ///< Default receive address 0xA1B1C1D1
#define NRF905_DEFAULT_TXADDR     0x1A1B1C1D ///< Default transmit/destination address

#define NRF905_DEFAULT_TXPAYLOAD  0 // data byte in the payload

// Crystal frequency (the one the radio module is using)
// NRF905_CLK_4MHZ
// NRF905_CLK_8MHZ
// NRF905_CLK_12MHZ
// NRF905_CLK_16MHZ
// NRF905_CLK_20MHZ
#define NRF905_CLK_FREQ		NRF905_CLK_16MHZ // dont change it, that's external crystal

// Use software to get data ready state instead of reading pin for high/low state, this means you don't need to connect to DR pin
// This option can only be used if NRF905_INTERRUPTS is 0

#define NRF905_DR_SW		0

// Use software to get address match state instead of reading pin for high/low state, this means you don't need to connect to AM pin
// This option can only be used if NRF905_INTERRUPTS_AM is 0
#define NRF905_AM_SW		0

// Don't transmit if airway is busy (other transmissions are going on)
// This feature uses the CD pin
#define NRF905_COLLISION_AVOID	1

///////////////////
// Default radio settings
///////////////////

// Frequency
// Channel 0 is 422.4MHz for the 433MHz band, each channel increments the frequency by 100KHz, so channel 10 would be 423.4MHz
// Channel 0 is 844.8MHz for the 868/915MHz band, each channel increments the frequency by 200KHz, so channel 10 would be 846.8MHz
// Max channel is 511 (473.5MHz / 947.0MHz)
#define NRF905_CHANNEL			10

// Frequency band
// 868 and 915 are actually the same thing
// NRF905_BAND_433
// NRF905_BAND_868
// NRF905_BAND_915
#define NRF905_BAND			NRF905_BAND_915

// Output power
// n means negative, n10 = -10
// NRF905_PWR_n10 (-10dBm = 100uW)
// NRF905_PWR_n2 (-2dBm = 631uW)
// NRF905_PWR_6 (6dBm = 4mW)
// NRF905_PWR_10 (10dBm = 10mW)
#define NRF905_PWR			NRF905_PWR_10

// Save a few mA by reducing receive sensitivity
// NRF905_LOW_RX_DISABLE (Normal sensitivity)
// NRF905_LOW_RX_ENABLE (Lower sensitivity)
#define NRF905_LOW_RX		NRF905_LOW_RX_DISABLE

// Constantly retransmit payload while in transmit mode
// Can be useful in areas with lots of interference, but you'll need to make sure you can differentiate between re-transmitted packets and new packets (like an ID number).
// It will also block other transmissions if collision avoidance is enabled.
// NRF905_AUTO_RETRAN_DISABLE
// NRF905_AUTO_RETRAN_ENABLE
#define NRF905_AUTO_RETRAN	NRF905_AUTO_RETRAN_DISABLE // ********************************

// Output a clock signal on pin 3 of IC
// NRF905_OUTCLK_DISABLE
// NRF905_OUTCLK_500KHZ
// NRF905_OUTCLK_1MHZ
// NRF905_OUTCLK_2MHZ
// NRF905_OUTCLK_4MHZ
#define NRF905_OUTCLK		NRF905_OUTCLK_DISABLE

// CRC checksum
// NRF905_CRC_DISABLE
// NRF905_CRC_8
// NRF905_CRC_16
#define NRF905_CRC			NRF905_CRC_16

// Address size
// The address is actually the SYNC part of the packet, just after the preamble and before the data
// NRF905_ADDR_SIZE_1 (not recommended, a lot of false invalid packets will be received)
// NRF905_ADDR_SIZE_4
#define NRF905_ADDR_SIZE	NRF905_ADDR_SIZE_4

// Payload size (1 - 32)
#define NRF905_PAYLOAD_SIZE	20 //NRF905_MAX_PAYLOAD

// *#*#*#*#*#*#*#*#*#**#*#*#**#*#*#*#*#*#*#**#*#*#**#
// -------------------------------------------------
// Everything below is for ARM TM4C (Tiva-C) stuff [Hardware connections]
// -------------------------------------------------
// *#*#*#*#*#*#*#*#*#**#*#*#**#*#*#*#*#*#*#**#*#*#**#

#define NRF905_USE_PWR                  0

// Power mode pin
#define NRF905_PWR_MODE_BIT_NUM         6
#define NRF905_PWR_MODE_PORT_CLK        PORT_CLK_BASE(B)
#define NRF905_PWR_MODE_PORT_ADDR       PORT_ADDR_BASE(B)
#define NRF905_PWR_MODE_PORT_DATA       PORT_DATA_REG(B)

// Enable/standby pin
#define NRF905_TRX_EN_BIT_NUM           4                   // 0 : bit number
#define NRF905_TRX_EN_PORT_CLK          PORT_CLK_BASE(B)    // B : port name
#define NRF905_TRX_EN_PORT_ADDR         PORT_ADDR_BASE(B)
#define NRF905_TRX_EN_PORT_DATA         PORT_DATA_REG(B)

// TX / RX mode pin
#define NRF905_TX_EN_BIT_NUM            2
#define NRF905_TX_EN_PORT_CLK           PORT_CLK_BASE(D)
#define NRF905_TX_EN_PORT_ADDR          PORT_ADDR_BASE(D)
#define NRF905_TX_EN_PORT_DATA          PORT_DATA_REG(D)

// Carrier detect pin (Optional, used for collision avoidance if NRF905_COLLISION_AVOID is 1 or if you want to use the NRF905_airwayBusy() function)
#define NRF905_CD_BIT_NUM               5
#define NRF905_CD_PORT_CLK              PORT_CLK_BASE(B)
#define NRF905_CD_PORT_ADDR             PORT_ADDR_BASE(B)
#define NRF905_CD_PORT_DATA             PORT_DATA_REG(B)

// Data ready pin
#define NRF905_DR_BIT_NUM               7
#define NRF905_DR_PORT_CLK              PORT_CLK_BASE(B)
#define NRF905_DR_PORT_ADDR             PORT_ADDR_BASE(B)
#define NRF905_DR_PORT_DATA             PORT_DATA_REG(B)
#define NRF905_DR_PORT_INT_NUM          INT_GPIOB

// Address match pin
#define NRF905_AM_BIT_NUM               6
#define NRF905_AM_PORT_CLK              PORT_CLK_BASE(B)
#define NRF905_AM_PORT_ADDR             PORT_ADDR_BASE(B)
#define NRF905_AM_PORT_DATA             PORT_DATA_REG(B)

// SPI configuration

// SPI slave select pin
#define NRF905_CSN_BIT_NUM              3
#define NRF905_CSN_PORT_CLK             PORT_CLK_BASE(A)
#define NRF905_CSN_PORT_ADDR            PORT_ADDR_BASE(A)
#define NRF905_CSN_PORT_DATA            PORT_DATA_REG(A)

#define SSI_CLK             SPI0_CLK_BASE  // base clock
#define SSI_BASE            SPI0_ADDR_BASE           // base address
#define SSI_PORT_CLK        SPI0_PORTA_CLK_BASE
#define SSI_PORT_BASE       SPI0_PORTA_ADDR_BASE

#define SSI_CLK_PIN         SPI0_CLK_PIN_A2
#define SSI_RX_PIN          SPI0_RX_PIN_A4
#define SSI_TX_PIN          SPI0_TX_PIN_A5

#define SSI_CLK_PIN_AFN     SPI0_CLK_PIN_PCTL_A2
#define SSI_RX_PIN_AFN      SPI0_RX_PIN_PCTL_A4
#define SSI_TX_PIN_AFN      SPI0_TX_PIN_PCTL_A5

#define SSI_MODE            SPI_PROTOCOL_MODE_0
#define SSI_TYPE            SPI_MODE_MASTER
#define DATA_LEN            SPI_DATA_LEN_8          // bits
#define BIT_RATE            1000000     // b/s

#endif /* NRF905_CONFIG_H_ */
