/*
 * SPI.h
 *
 *  Created on: Apr 18, 2019
 *      Author: Administrator
 */

#ifndef SPI_DRIVER_SPI_H_
#define SPI_DRIVER_SPI_H_

void SPI_init(SPIObject_t *ptSPIObject);

inline void SPI_write(SPIObject_t *ptSPIObject, uint32_t ui32Data)
{
    uint32_t ui32Dummy;
    
    SSIDataPut(ptSPIObject->tSPIAddrBase, ui32Data);
    while (SSIBusy(ptSPIObject->tSPIAddrBase))
    {
        // Loop until Sending Data to be completed.
    }
    SSIDataGet(ptSPIObject->tSPIAddrBase, &ui32Dummy);
}

inline uint8_t SPI_command(SPIObject_t *ptSPIObject, uint32_t ui32Command)
{
    uint32_t ui32Status;

    SSIDataPut(ptSPIObject->tSPIAddrBase, ui32Command);
    while (SSIBusy(ptSPIObject->tSPIAddrBase))
    {
        // Loop until Sending Command to be completed.
    }
    SSIDataGet(ptSPIObject->tSPIAddrBase, &ui32Status);

    return (uint8_t) ui32Status;
}

inline int32_t SPI_read(SPIObject_t *ptSPIObject)
{
    uint32_t ui32Data;

    SSIDataPut(ptSPIObject->tSPIAddrBase, SPI_CMD_NOP);
    while (SSIBusy(ptSPIObject->tSPIAddrBase))
    {
        // Loop until Sending Command to be completed.
    }

    if (SSIDataGetNonBlocking(ptSPIObject->tSPIAddrBase, &ui32Data)) // check if Rx FIFO is not empty
    {
        return ui32Data;
    }
    else
    {
        return -1;
    }
}

#endif /* SPI_DRIVER_SPI_H_ */
