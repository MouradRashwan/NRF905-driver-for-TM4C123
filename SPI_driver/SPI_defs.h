/*
 * SPI_defs.h
 *
 *  Created on: Apr 18, 2019
 *      Author: Administrator
 */

#ifndef SPI_DRIVER_SPI_DEFS_H_
#define SPI_DRIVER_SPI_DEFS_H_

#define SPI_CMD_NOP     0xFFU

typedef enum SPIClkBase
{
    SPI0_CLK_BASE = SYSCTL_PERIPH_SSI0,
    SPI1_CLK_BASE = SYSCTL_PERIPH_SSI1,
    SPI2_CLK_BASE = SYSCTL_PERIPH_SSI2,
    SPI3_CLK_BASE = SYSCTL_PERIPH_SSI3
} SPIClkBase_t;

typedef enum SPIAddrBase
{
    SPI0_ADDR_BASE = SSI0_BASE,
    SPI1_ADDR_BASE = SSI1_BASE,
    SPI2_ADDR_BASE = SSI2_BASE,
    SPI3_ADDR_BASE = SSI3_BASE
} SPIAddrBase_t;

typedef enum SPIPortClkBase
{
    SPI0_PORTA_CLK_BASE = SYSCTL_PERIPH_GPIOA,
    SPI1_PORTD_CLK_BASE = SYSCTL_PERIPH_GPIOD,
    SPI1_PORTF_CLK_BASE = SYSCTL_PERIPH_GPIOF,
    SPI2_PORTB_CLK_BASE = SYSCTL_PERIPH_GPIOB,
    SPI3_PORTD_CLK_BASE = SYSCTL_PERIPH_GPIOD
} SPIPortClkBase_t;

typedef enum SPIPortAddrBase
{
    SPI0_PORTA_ADDR_BASE = GPIO_PORTA_BASE,
    SPI1_PORTD_ADDR_BASE = GPIO_PORTD_BASE,
    SPI1_PORTF_ADDR_BASE = GPIO_PORTF_BASE,
    SPI2_PORTB_ADDR_BASE = GPIO_PORTB_BASE,
    SPI3_PORTD_ADDR_BASE = GPIO_PORTD_BASE
} SPIPortAddrBase_t;

typedef enum SPIClkPinNum
{
    SPI0_CLK_PIN_A2 = GPIO_PIN_2,
    SPI1_CLK_PIN_D0 = GPIO_PIN_0,
    SPI1_CLK_PIN_F2 = GPIO_PIN_2,
    SPI2_CLK_PIN_B4 = GPIO_PIN_4,
    SPI3_CLK_PIN_D0 = GPIO_PIN_0
} SPIClkPinNum_t;

typedef enum SPIRxPinNum
{
    SPI0_RX_PIN_A4 = GPIO_PIN_4,
    SPI1_RX_PIN_D2 = GPIO_PIN_2,
    SPI1_RX_PIN_F0 = GPIO_PIN_0,
    SPI2_RX_PIN_B6 = GPIO_PIN_6,
    SPI3_RX_PIN_D2 = GPIO_PIN_2
} SPIRxPinNum_t;

typedef enum SPITxPinNum
{
    SPI0_TX_PIN_A5 = GPIO_PIN_5,
    SPI1_TX_PIN_D3 = GPIO_PIN_3,
    SPI1_TX_PIN_F1 = GPIO_PIN_1,
    SPI2_TX_PIN_B7 = GPIO_PIN_7,
    SPI3_TX_PIN_D3 = GPIO_PIN_3
} SPITxPinNum_t;

typedef enum SPIClkPinPctl
{
    SPI0_CLK_PIN_PCTL_A2 = GPIO_PA2_SSI0CLK,
    SPI1_CLK_PIN_PCTL_D0 = GPIO_PD0_SSI1CLK,
    SPI1_CLK_PIN_PCTL_F2 = GPIO_PF2_SSI1CLK,
    SPI2_CLK_PIN_PCTL_B4 = GPIO_PB4_SSI2CLK,
    SPI3_CLK_PIN_PCTL_D0 = GPIO_PD0_SSI3CLK
} SPIClkPinPctl_t;

typedef enum SPIRxPinPctl
{
    SPI0_RX_PIN_PCTL_A4 = GPIO_PA4_SSI0RX,
    SPI1_RX_PIN_PCTL_D2 = GPIO_PD2_SSI1RX,
    SPI1_RX_PIN_PCTL_F0 = GPIO_PF0_SSI1RX,
    SPI2_RX_PIN_PCTL_B6 = GPIO_PB6_SSI2RX,
    SPI3_RX_PIN_PCTL_D2 = GPIO_PD2_SSI3RX
} SPIRxPinPctl_t;

typedef enum SPITxPinPctl
{
    SPI0_TX_PIN_PCTL_A5 = GPIO_PA5_SSI0TX,
    SPI1_TX_PIN_PCTL_D3 = GPIO_PD3_SSI1TX,
    SPI1_TX_PIN_PCTL_F1 = GPIO_PF1_SSI1TX,
    SPI2_TX_PIN_PCTL_B7 = GPIO_PB7_SSI2TX,
    SPI3_TX_PIN_PCTL_D3 = GPIO_PD3_SSI3TX
} SPITxPinPctl_t;

typedef enum SPIProtocol
{
    SPI_PROTOCOL_MODE_0 = SSI_FRF_MOTO_MODE_0,  // Moto fmt, polarity 0, phase 0
    SPI_PROTOCOL_MODE_1 = SSI_FRF_MOTO_MODE_1,  // Moto fmt, polarity 0, phase 1
    SPI_PROTOCOL_MODE_2 = SSI_FRF_MOTO_MODE_2,  // Moto fmt, polarity 1, phase 0
    SPI_PROTOCOL_MODE_3 = SSI_FRF_MOTO_MODE_3,  // Moto fmt, polarity 1, phase 1
    SPI_PROTOCOL_MODE_TI = SSI_FRF_TI,          // TI frame format
    SPI_PROTOCOL_MODE_MICROWIRE = SSI_FRF_NMW // National MicroWire frame format
} SPIProtocol_t;

typedef enum SPIMode
{
    SPI_MODE_MASTER = SSI_MODE_MASTER,  // SSI master
    SPI_MODE_SLAVE = SSI_MODE_SLAVE    // SSI slave
} SPIMode_t;

typedef enum SPIDataLen
{
    SPI_DATA_LEN_4 = 4U, SPI_DATA_LEN_8 = 8U, SPI_DATA_LEN_16 = 16U
} SPIDataLen_t;

typedef struct SPIObject
{
    SPIClkBase_t tSPIClkBase;
    SPIAddrBase_t tSPIAddrBase;
    SPIPortClkBase_t tSPIPortClkBase;
    SPIPortAddrBase_t tSPIPortAddrBase;
    SPIClkPinNum_t tSPIClkPinNum;
    SPIRxPinNum_t tSPIRxPinNum;
    SPITxPinNum_t tSPITxPinNum;
    SPIClkPinPctl_t tSPIClkPinPctl;
    SPIRxPinPctl_t tSPIRxPinPctl;
    SPITxPinPctl_t tSPITxPinPctl;
    SPIProtocol_t tSPIProtocol;
    SPIMode_t tSPIMode;
    SPIDataLen_t tSPIDataLen;
    uint32_t ui32SPIBitRate; /* [MASTER mode] MAX data rate of 25 MHz at System Clock of 80 MHz */
} SPIObject_t;

#endif /* SPI_DRIVER_SPI_DEFS_H_ */
