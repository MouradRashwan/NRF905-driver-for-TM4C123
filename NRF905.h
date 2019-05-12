/*
 * NRF905.h
 *
 *  Created on: Jul 29, 2018
 *      Author: Mourad-Rashwan
 */

#ifndef NRF905_H_
#define NRF905_H_

bool NRF905_dataReady(void);

/**
 * @brief See if the address match is asserted
 *
 * @return 1 if currently receiving payload or payload is ready to be read, otherwise 0
 */
bool NRF905_receiveBusy(void);

/**
 * @brief See if airway is busy (carrier detect pin asserted).
 *
 * @return 1 if other transmissions detected, otherwise 0
 */
bool NRF905_airwayBusy(void);

/**
 * @brief Initialise, must be called before anything else!
 *
 * @return (none)
 */
void NRF905_init(uint8_t ui8NRFDRPortIntPri, uint32_t ui32RxAddress,
                 void (*pfnNRFRxISR)(void));

/**
 * @brief Channel to listen and transmit on
 *
 * 433MHz band: Channel 0 is 422.4MHz up to 511 which is 473.5MHz (Each channel is 100KHz apart)
 *
 * 868/915MHz band: Channel 0 is 844.8MHz up to 511 which is 947MHz (Each channel is 200KHz apart)
 *
 * @param [channel] The channel (0 - 511)
 * @return (none)
 *
 * @see ::NRF905_setBand()
 */
void NRF905_setChannel(uint16_t tChannel);

/**
 * @brief Frequency band
 *
 * @param [band] Frequency band, see ::NRF905_band_t
 * @return (none)
 *
 * @see ::NRF905_setChannel() ::NRF905_band_t
 */
void NRF905_setBand(NRF905_band_t tBand);

/**
 * @brief Set auto retransmit
 *
 * If next mode is set to ::NRF905_NEXTMODE_TX when calling ::NRF905_TX() and auto-retransmit is enabled then it will constantly retransmit the payload, otherwise a carrier wave with no data will be transmitted instead (kinda useless).\n
 * Transmission will continue until the radio is put into standby, power down or RX mode.
 *
 * Can be useful in areas with lots of interference, but you'll need to make sure you can differentiate between re-transmitted packets and new packets (like an ID number).
 *
 * Other transmissions will be blocked if collision avoidance is enabled.
 *
 * @param [val] Enable/disable auto retransmit, see ::NRF905_auto_retran_t
 * @return (none)
 */
void NRF905_setAutoRetransmit(NRF905_auto_retran_t tAutoRetransmit);

/**
 * @brief Set low power receive
 *
 * @param [val] Enable/disable low power receive, see ::NRF905_low_rx_t
 * @return (none)
 */
void NRF905_setLowRxPower(NRF905_low_rx_t tLowRx);

/**
 * @brief Set output power
 *
 * @param [val] Output power level, see ::NRF905_pwr_t
 * @return (none)
 */
void NRF905_setTransmitPower(NRF905_pwr_t tTxPwr);

/**
 * @brief Set CRC
 *
 * @param [val] CRC Type, see ::NRF905_crc_t
 * @return (none)
 */
void NRF905_setCRC(NRF905_crc_t tCrc);

/**
 * @brief Set clock output
 *
 * @param [val] Clock out frequency, see ::NRF905_outclk_t
 * @return (none)
 */
void NRF905_setClockOut(NRF905_outclk_t tClkOut);

/**
 * @brief Set address to listen to
 *
 * @note From the datasheet: Each byte within the address should be unique.
 * Repeating bytes within the address reduces the effectiveness of the address and increases its susceptibility to noise which increases the packet error rate.
 * The address should also have several level shifts (that is, 10101100) reducing the statistical effect of noise and the packet error rate.
 * @param [address] The address, a 32 bit integer (default address is 0xE7E7E7E7)
 * @return (none)
 */
void NRF905_setRxAddress(uint32_t ui32RxAddress);

bool NRF905_writeTxPayload(uint32_t ui32TxAddr, void *pvDataToBeTransmitted,
                           uint8_t ui8DataLen);

bool NRF905_TX(void (*pfnNRFTxISR)(void));

void NRF905_readRxPayload(void *pvDataToBeReceived, uint32_t ui32Size);

void NRF905_RX(void);

/**
 * @brief Sleep.
 *
 * This also clears the RX payload.
 *
 * @return (none)
 */
void NRF905_powerDown(void);

/**
 * @brief Enter standby mode.
 *
 * Will take 3ms to complete if the radio was in power down mode.
 * ::NRF905_standby() does the same thing, but without the delay.
 * There must be a 3ms delay between powering up and beginning a transmission.
 *
 * @return (none)
 */
void NRF905_powerUp(void);

/**
 * @brief Enter standby mode.
 *
 * Similar to ::NRF905_powerUp() but without any delays.
 * There must be a 3ms delay between powering up and beginning a transmission.
 *
 * @return (none)
 */
void NRF905_standby(void);

/**
 * @brief Read configuration registers into byte array of ::NRF905_REGISTER_COUNT elements, mainly for debugging.
 *
 * @return (none)
 */
void NRF905_getConfigRegisters(void *pvRegs);

///**
// * @brief Payload size
// *
// * @param [size] Payload size (1 - 32)
// * @return (none)
// *
// * @see ::NRF905_MAX_PAYLOAD
// */
//void NRF905_setPayloadSize(uint8_t tPayloadSize);
//
///**
// * @brief Address size
// *
// * @param [size] Address size, see ::NRF905_addr_size_t
// * @return (none)
// */
//void NRF905_setAddressSize(NRF905_addr_size_t tAddrSize);

#endif /* NRF905_H_ */
