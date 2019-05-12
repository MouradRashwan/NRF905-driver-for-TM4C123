/*
 * NRF905.c
 *
 *  Created on: Jul 29, 2018
 *      Author: Mourad-Rashwan
 */

// If DR && AM = RX new packet
// If DR && !AM = TX finished
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>

#include "NRF905_driver.h"

static SPIObject_t g_tSPIObject = { .tSPIClkBase = SSI_CLK,
                                    .tSPIAddrBase = SSI_BASE,
                                    .tSPIPortClkBase = SSI_PORT_CLK,
                                    .tSPIPortAddrBase = SSI_PORT_BASE,
                                    .tSPIClkPinNum = SSI_CLK_PIN,
                                    .tSPIRxPinNum = SSI_RX_PIN,
                                    .tSPITxPinNum = SSI_TX_PIN,
                                    .tSPIClkPinPctl = SSI_CLK_PIN_AFN,
                                    .tSPIRxPinPctl = SSI_RX_PIN_AFN,
                                    .tSPITxPinPctl = SSI_TX_PIN_AFN,
                                    .tSPIProtocol = SSI_MODE,
                                    .tSPIMode = SSI_TYPE,
                                    .tSPIDataLen = DATA_LEN,
                                    .ui32SPIBitRate = BIT_RATE };

static void (*g_pfnNRFTxISR)(void) = NULL;
static void (*g_pfnNRFRxISR)(void) = NULL;

static uint8_t _cs;
static bool _bx, bLast;

#define CHIP_SELECT()   STANDBY_ENTER(); \
                        for(_cs = SPI_select(); _cs; _cs = SPI_deselect())

#define ATOMIC_BLOCK()  bLast = IntMasterDisable(); \
                        for(_bx = true; _bx; _bx = bLast?false:!IntMasterEnable())

static uint8_t aui8Config[10] = {
NRF905_CHANNEL,
                                  NRF905_AUTO_RETRAN | NRF905_LOW_RX
                                          | NRF905_PWR | NRF905_BAND
                                          | ((NRF905_CHANNEL >> 8) & 0x01),
                                  (NRF905_ADDR_SIZE << 4) | NRF905_ADDR_SIZE,
                                  NRF905_PAYLOAD_SIZE, // RX payload size
        NRF905_PAYLOAD_SIZE, // TX payload size
        NRF905_DEFAULT_RXADDR & 0x000000FF, (NRF905_DEFAULT_RXADDR & 0x0000FF00)
                >> 8,
        (NRF905_DEFAULT_RXADDR & 0x00FF0000) >> 16, (NRF905_DEFAULT_RXADDR
                & 0xFF000000) >> 24, // Default receive address
        NRF905_CRC | NRF905_CLK_FREQ | NRF905_OUTCLK };

uint32_t SPI_select(void)
{
    SPI_SELECT();
    return 1;
}

uint32_t SPI_deselect(void)
{
    SPI_DESELECT();
    return 0;
}

static void NRF905_DataReadyISR(void)
{
    GPIOIntClear(NRF905_DR_PORT_ADDR, (1 << NRF905_DR_BIT_NUM));

    if (ADDRESS_MATCHED() == 0)
    {
        if (g_pfnNRFTxISR != NULL)
        {
            g_pfnNRFTxISR();
        }
    }
    else
    {
        if (g_pfnNRFRxISR != NULL)
        {
            g_pfnNRFRxISR();
        }
    }
}

void DelayMicros(uint32_t ui32Micros)
{
    uint32_t ix;
    uint32_t ui32Period = (SysCtlClockGet() / 1000000U) / 3U;
    for (ix = 0; ix < ui32Micros; ix++)
    {
        SysCtlDelay(ui32Period);
    }
}

static void DelayMillis(uint32_t ui32Millis)
{
    uint32_t ix;
    uint32_t ui32Period = (SysCtlClockGet() / 1000U) / 3U;
    for (ix = 0; ix < ui32Millis; ix++)
    {
        SysCtlDelay(ui32Period);
    }
}

static int32_t readConfigRegister(uint8_t ui8Reg)
{
    int32_t i32Val;
    ATOMIC_BLOCK()
    {
        CHIP_SELECT()
        {
            SPI_command(&g_tSPIObject, NRF905_CMD_R_CONFIG | ui8Reg);
            i32Val = SPI_read(&g_tSPIObject);
        }
    }
    STANDBY_LEAVE();
    DelayMicros(650);
    return i32Val;
}

static void writeConfigRegister(uint8_t ui8Reg, uint8_t ui8Val)
{
    ATOMIC_BLOCK()
    {
        CHIP_SELECT()
        {
            SPI_command(&g_tSPIObject, NRF905_CMD_W_CONFIG | ui8Reg);
            SPI_write(&g_tSPIObject, ui8Val);
        }
    }
    STANDBY_LEAVE();
    DelayMicros(650);
}

static void setConfigReg1(uint8_t ui8Reg, uint8_t ui8Mask, uint8_t ui8Val)
{
    writeConfigRegister(
            ui8Reg,
            (readConfigRegister(NRF905_REG_CONFIG1) & ui8Mask) | ui8Val);
}

static void setConfigReg2(uint8_t ui8Reg, uint8_t ui8Mask, uint8_t ui8Val)
{
    writeConfigRegister(
            ui8Reg,
            (readConfigRegister(NRF905_REG_CONFIG2) & ui8Mask) | ui8Val);
}

static void defaultConfig(void)
{
    uint8_t i;
    ATOMIC_BLOCK()
    // Should be in standby mode
    {
        // Set control registers
        CHIP_SELECT()
        {
            SPI_command(&g_tSPIObject, NRF905_CMD_W_CONFIG);
            for (i = 0; i < sizeof(aui8Config); i++)
            {
                SPI_write(&g_tSPIObject, aui8Config[i]);
            }
        }

        // Default transmit address
        CHIP_SELECT()
        {
            SPI_command(&g_tSPIObject, NRF905_CMD_W_TX_ADDRESS);
            for (i = 0; i < NRF905_ADDR_SIZE; i++)
            {
                SPI_write(
                        &g_tSPIObject,
                        (uint8_t) ((NRF905_DEFAULT_TXADDR >> (8 * i))
                                & 0x000000FF));
            }
        }

        // Write default transmit payload
        CHIP_SELECT()
        {
            SPI_command(&g_tSPIObject, NRF905_CMD_W_TX_PAYLOAD);
            for (i = 0; i < NRF905_PAYLOAD_SIZE; i++)
            {
                SPI_write(&g_tSPIObject, NRF905_DEFAULT_TXPAYLOAD);
            }
        }

        // Clear DR by reading receive payload (clearing Rx payload)
        CHIP_SELECT()
        {
            SPI_command(&g_tSPIObject, NRF905_CMD_R_RX_PAYLOAD);
            for (i = 0; i < NRF905_PAYLOAD_SIZE; i++)
            {
                SPI_read(&g_tSPIObject);
            }
        }
    }
    STANDBY_LEAVE();
    DelayMicros(650);
}

static void setTxAddress(uint32_t ui32TxAddress)
{
    uint8_t i;
    ATOMIC_BLOCK()
    {
        CHIP_SELECT()
        {
            SPI_command(&g_tSPIObject, NRF905_CMD_W_TX_ADDRESS);
            for (i = 0; i < NRF905_ADDR_SIZE; i++)
            {
                SPI_write(&g_tSPIObject,
                          ((ui32TxAddress >> (8 * i)) & 0x000000FF));
            }
        }
    }
    STANDBY_LEAVE();
    DelayMicros(650);
}

static void setTxPayload(void *pvData, uint8_t ui8DataLen)
{
    uint8_t i;
    ATOMIC_BLOCK()
    {
        CHIP_SELECT()
        {
            SPI_command(&g_tSPIObject, NRF905_CMD_W_TX_PAYLOAD);
            for (i = 0; i < NRF905_PAYLOAD_SIZE; i++)
            {
                if (i < ui8DataLen)
                {
                    SPI_write(&g_tSPIObject, *(((uint8_t*) pvData) + i));
                }
                else
                {
                    SPI_write(&g_tSPIObject, 0);
                }
            }
        }
    }
    STANDBY_LEAVE();
    DelayMicros(650);
}

static uint8_t readSwStatus(void)
{
    uint8_t ui8Status;
    ATOMIC_BLOCK()
    {
        CHIP_SELECT()
        {
            ui8Status = SPI_command(&g_tSPIObject, NRF905_CMD_NOP);
        }
    }
    STANDBY_LEAVE();
    DelayMicros(650);
    return ui8Status;
}

// Hardware: Data ready pin
// Software: Data ready status bit
static bool dataReady(void)
{
#if NRF905_DR_SW
    return ((readSwStatus() & (1 << NRF905_STATUS_DR)) >> NRF905_STATUS_DR);
#else
    return DATA_READY();
#endif
}

// Hardware: Address match pin
// Software: Address match status bit
static bool addressMatched(void)
{
#if NRF905_AM_SW
    return ((readSwStatus() & (1<<NRF905_STATUS_AM)) >> NRF905_STATUS_AM);
#else
    return ADDRESS_MATCHED();
#endif
}

// Hardware: Carrier detect pin
static bool carrierDetected(void)
{
    return CARRIER_DETECTED();
}

bool NRF905_dataReady()
{
    return dataReady();
}

bool NRF905_receiveBusy()
{
    return addressMatched();
}

bool NRF905_airwayBusy()
{
    return carrierDetected();
}

void NRF905_init(uint8_t ui8NRFDRPortIntPri, uint32_t ui32RxAddress,
                 void (*pfnNRFRxISR)(void))
{
    uint32_t *pui32RxAddr;

    g_pfnNRFRxISR = pfnNRFRxISR;
    ui8NRFDRPortIntPri = ((ui8NRFDRPortIntPri % 8) << 5) & 0xE0;

    SysCtlPeripheralEnable(NRF905_CSN_PORT_CLK);
    SysCtlPeripheralEnable(NRF905_TRX_EN_PORT_CLK);
    SysCtlPeripheralEnable(NRF905_PWR_MODE_PORT_CLK);
    SysCtlPeripheralEnable(NRF905_TX_EN_PORT_CLK);
    SysCtlPeripheralEnable(NRF905_CD_PORT_CLK);
    SysCtlPeripheralEnable(NRF905_AM_PORT_CLK);
    SysCtlPeripheralEnable(NRF905_DR_PORT_CLK);
    while (!SysCtlPeripheralReady(NRF905_DR_PORT_CLK))
    {

    }

////////// INPUTS/////////////////////// CD, AM, DR input ports from the NRF905

    GPIOPinTypeGPIOInput(NRF905_CD_PORT_ADDR, (1 << NRF905_CD_BIT_NUM),
    GPIO_PIN_TYPE_STD);
    GPIOPinTypeGPIOInput(NRF905_AM_PORT_ADDR, (1 << NRF905_AM_BIT_NUM),
    GPIO_PIN_TYPE_STD);
    GPIOPinTypeGPIOInput(NRF905_DR_PORT_ADDR, (1 << NRF905_DR_BIT_NUM),
    GPIO_PIN_TYPE_STD);

    IntPrioritySet(NRF905_DR_PORT_INT_NUM, ui8NRFDRPortIntPri);

    GPIOIntTypeSet(NRF905_DR_PORT_ADDR, (1 << NRF905_DR_BIT_NUM),
    GPIO_RISING_EDGE);
    GPIOIntRegister(NRF905_DR_PORT_ADDR, NRF905_DataReadyISR);
    GPIOIntEnable(NRF905_DR_PORT_ADDR, (1 << NRF905_DR_BIT_NUM));

////////// OUTPUTS///////////////////////////// PWR, TRX, TX, CSN(SS) output ports to the NRF905

    GPIOPinTypeGPIOOutput(NRF905_TRX_EN_PORT_ADDR, (1 << NRF905_TRX_EN_BIT_NUM),
    GPIO_PIN_TYPE_STD);
    GPIOPinTypeGPIOOutput(NRF905_TX_EN_PORT_ADDR, (1 << NRF905_TX_EN_BIT_NUM),
    GPIO_PIN_TYPE_STD);
    GPIOPinTypeGPIOOutput(NRF905_CSN_PORT_ADDR, (1 << NRF905_CSN_BIT_NUM),
    GPIO_PIN_TYPE_STD);

#if NRF905_USE_PWR
    GPIOPinTypeGPIOOutput(NRF905_PWR_MODE_PORT_ADDR,
            (1 << NRF905_PWR_MODE_BIT_NUM), GPIO_PIN_TYPE_STD);
#endif

    SPI_init(&g_tSPIObject);

    POWER_UP();
    STANDBY_ENTER();
    MODE_RX();

    DelayMillis(3);

    pui32RxAddr = (uint32_t*) (&aui8Config[5]);
    *pui32RxAddr = ui32RxAddress;

    defaultConfig();
}

void NRF905_setChannel(uint16_t ui16Channel) // under testing
{
    uint8_t ui8Val;
    if (ui16Channel > NRF905_MAX_CHANNEL)
    {
        ui16Channel = NRF905_MAX_CHANNEL;
    }

    ATOMIC_BLOCK()
    {
        ui8Val = (readConfigRegister(NRF905_REG_CONFIG1) & NRF905_MASK_CHANNEL)
                | (ui16Channel >> 8);

        CHIP_SELECT()
        {
            SPI_command(&g_tSPIObject,
            NRF905_CMD_W_CONFIG | NRF905_REG_CHANNEL);
            SPI_write(&g_tSPIObject, ui16Channel);
        }

        CHIP_SELECT()
        {
            SPI_command(&g_tSPIObject,
            NRF905_CMD_W_CONFIG | NRF905_REG_CONFIG1);
            SPI_write(&g_tSPIObject, ui8Val);
        }
    }
    STANDBY_LEAVE();
    DelayMicros(650);
}

void NRF905_setBand(NRF905_band_t tBand) // under testing
{
    uint8_t ui8Val;
    ATOMIC_BLOCK()
    {
        ui8Val = (readConfigRegister(NRF905_REG_CONFIG1) & NRF905_MASK_BAND)
                | tBand;

        CHIP_SELECT()
        {
            SPI_command(&g_tSPIObject,
            NRF905_CMD_W_CONFIG | NRF905_REG_CONFIG1);
            SPI_write(&g_tSPIObject, ui8Val);
        }
    }
    STANDBY_LEAVE();
    DelayMicros(650);
}

void NRF905_setAutoRetransmit(NRF905_auto_retran_t tAutoRetransmit)
{
    setConfigReg1(NRF905_REG_AUTO_RETRAN, NRF905_MASK_AUTO_RETRAN,
                  tAutoRetransmit);
}

void NRF905_setLowRxPower(NRF905_low_rx_t tLowRx)
{
    setConfigReg1(NRF905_REG_LOW_RX, NRF905_MASK_LOW_RX, tLowRx);
}

void NRF905_setTransmitPower(NRF905_pwr_t tTxPwr)
{
    setConfigReg1(NRF905_REG_PWR, NRF905_MASK_PWR, tTxPwr);
}

void NRF905_setCRC(NRF905_crc_t tCrc)
{
    setConfigReg2(NRF905_REG_CRC, NRF905_MASK_CRC, tCrc);
}

void NRF905_setClockOut(NRF905_outclk_t tClkOut)
{
    setConfigReg2(NRF905_REG_OUTCLK, NRF905_MASK_OUTCLK, tClkOut);
}

void NRF905_setRxAddress(uint32_t ui32RxAddress)
{
    uint8_t i;
    ATOMIC_BLOCK()
    {
        CHIP_SELECT()
        {
            SPI_command(&g_tSPIObject,
            NRF905_CMD_W_CONFIG | NRF905_REG_RX_ADDRESS);
            for (i = 0; i < NRF905_ADDR_SIZE; i++)
            {
                SPI_write(&g_tSPIObject,
                          ((ui32RxAddress >> (8 * i)) & 0x000000FF));
            }
        }
    }
    STANDBY_LEAVE();
    DelayMicros(650);
}

bool NRF905_writeTxPayload(uint32_t ui32TxAddr, void *pvDataToBeTransmitted,
                           uint8_t ui8DataLen)
{
    if (ui8DataLen > NRF905_MAX_PAYLOAD)
    {
        ui8DataLen = NRF905_MAX_PAYLOAD;
    }

    if (pvDataToBeTransmitted == NULL)
    {
        return false;
    }

    setTxAddress(ui32TxAddr);

    setTxPayload(pvDataToBeTransmitted, ui8DataLen);

    return true;
}

bool NRF905_TX(void (*pfnNRFTxISR)(void))
{
    g_pfnNRFTxISR = pfnNRFTxISR;

    if (!POWERED_UP())
    {
        POWER_UP();
        DelayMillis(3);
    }

#if NRF905_COLLISION_AVOID
    if (carrierDetected())
    {
        return false;
    }
#endif

    // Put into Standby mode
    STANDBY_ENTER();
    // Put into transmit mode
    MODE_TX();
    // Pulse standby pin to start transmission
    STANDBY_LEAVE();
    DelayMicros(15); //minimum pulse 10 us
    STANDBY_ENTER();

    return true;
}

void NRF905_RX()
{
    if (!POWERED_UP())
    {
        POWER_UP();
        DelayMillis(3);
    }
    MODE_RX();
    STANDBY_LEAVE();
    DelayMicros(650);
}

void NRF905_readRxPayload(void *pvDataToBeReceived, uint32_t ui32Size)
{
    uint8_t i;
    int32_t i32Val;
    ATOMIC_BLOCK()
    {
        CHIP_SELECT()
        {
            SPI_command(&g_tSPIObject, NRF905_CMD_R_RX_PAYLOAD);
            for (i = 0; i < NRF905_PAYLOAD_SIZE; i++)
            {
                i32Val = SPI_read(&g_tSPIObject);
                if (i < ui32Size)
                {
                    if (i32Val != -1)
                    {
                        *(((uint8_t*) pvDataToBeReceived) + i) =
                                (uint8_t) i32Val;
                    }
                }
            }
            // Must make sure all of the payload has been read, otherwise DR never goes low
            //uint8_t remaining = NRF905_MAX_PAYLOAD - len;
            //while(remaining--)
            //	SPI_read();
        }
    }
    STANDBY_LEAVE();
    DelayMicros(650);
}

void NRF905_powerDown(void)
{
    POWER_DOWN();
}

void NRF905_powerUp(void)
{
    STANDBY_ENTER();
    if (!POWERED_UP())
    {
        POWER_UP();
        DelayMillis(3);
    }
    DelayMicros(650);
}

void NRF905_standby(void)
{
    STANDBY_ENTER();
    if (!POWERED_UP())
    {
        POWER_UP();
        DelayMillis(3);
    }
    DelayMicros(650);
}

void NRF905_getConfigRegisters(void *pvRegs)
{
    uint8_t i;

    ATOMIC_BLOCK()
    {
        CHIP_SELECT()
        {
            SPI_command(&g_tSPIObject, NRF905_CMD_R_CONFIG);

            for (i = 0; i < NRF905_CONFIG_REG_SIZE; i++)
            {
                int32_t i32Val = SPI_read(&g_tSPIObject);
                if (i32Val != -1)
                {
                    *(((uint8_t*) pvRegs) + i) = i32Val;
                }
                else
                {
                    *(((uint8_t*) pvRegs) + i) = 0;
                }
            }
        }
    }
    STANDBY_LEAVE();
    DelayMicros(650);
}

//void NRF905_setPayloadSize(uint8_t ui8PayloadSize)
//{
//    if (ui8PayloadSize > NRF905_MAX_PAYLOAD)
//    {
//        ui8PayloadSize = NRF905_MAX_PAYLOAD;
//    }
//
//    ATOMIC_BLOCK()
//    {
//        CHIP_SELECT()
//        {
//            SPI_command(NRF905_CMD_W_CONFIG | NRF905_REG_RX_PAYLOAD_SIZE);
//            SPI_write(ui8PayloadSize);
//        }
//
//        CHIP_SELECT()
//        {
//            SPI_command(NRF905_CMD_W_CONFIG | NRF905_REG_TX_PAYLOAD_SIZE);
//            SPI_write(ui8PayloadSize);
//        }
//    }
//}
//
//void NRF905_setAddressSize(NRF905_addr_size_t tAddrSize)
//{
//    ATOMIC_BLOCK()
//    {
//        CHIP_SELECT()
//        {
//            SPI_command(NRF905_CMD_W_CONFIG | NRF905_REG_ADDR_WIDTH);
//            SPI_write(tAddrSize);
//        }
//    }
//}

