/*
 * SPI.c
 *
 *  Created on: Apr 18, 2019
 *      Author: Administrator
 */

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>

#include "SPI_driver.h"

void SPI_init(SPIObject_t *ptSPIObject)
{
    SysCtlPeripheralEnable(ptSPIObject->tSPIPortClkBase);
    while (!SysCtlPeripheralReady(ptSPIObject->tSPIPortClkBase))
    {

    }
    GPIOPinTypeSSI(
            ptSPIObject->tSPIPortAddrBase,
            ptSPIObject->tSPIClkPinNum | ptSPIObject->tSPIRxPinNum
                    | ptSPIObject->tSPITxPinNum);
    GPIOPinConfigure(ptSPIObject->tSPIClkPinPctl);
    GPIOPinConfigure(ptSPIObject->tSPIRxPinPctl);
    GPIOPinConfigure(ptSPIObject->tSPITxPinPctl);

    SysCtlPeripheralEnable(ptSPIObject->tSPIClkBase);
    while (!SysCtlPeripheralReady(ptSPIObject->tSPIClkBase))
    {

    }
    SSIConfigSetExpClk(ptSPIObject->tSPIAddrBase, SysCtlClockGet(),
                       ptSPIObject->tSPIProtocol, ptSPIObject->tSPIMode,
                       ptSPIObject->ui32SPIBitRate, ptSPIObject->tSPIDataLen);
    SSIEnable(ptSPIObject->tSPIAddrBase);
}
