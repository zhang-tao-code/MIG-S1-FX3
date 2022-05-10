#include <cyu3system.h>
#include <cyu3os.h>
#include <cyu3error.h>
#include "cyu3i2c.h"
#include "hal_i2c.h"

uint16_t glI2cPageSize = 0x100;   /* I2C Page size to be used for transfers. */
CyU3PDmaChannel glI2cTxHandle;   /* I2C Tx channel handle */
CyU3PDmaChannel glI2cRxHandle;   /* I2C Rx channel handle */

/* I2c initialization for EEPROM programming. */
CyU3PReturnStatus_t HAL_I2cInit (uint16_t pageLen)
{
    CyU3PI2cConfig_t i2cConfig;
    CyU3PDmaChannelConfig_t dmaConfig;
    CyU3PReturnStatus_t status = CY_U3P_SUCCESS;

    /* Initialize and configure the I2C master module. */
    status = CyU3PI2cInit ();
    if (status != CY_U3P_SUCCESS)
    {
        return status;
    }

    /* Start the I2C master block. The bit rate is set at 100KHz.
     * The data transfer is done via DMA. */
    CyU3PMemSet ((uint8_t *)&i2cConfig, 0, sizeof(i2cConfig));
    i2cConfig.bitRate    = EEPROM_I2C_BITRATE;
    i2cConfig.busTimeout = 0xFFFFFFFF;
    i2cConfig.dmaTimeout = 0xFFFF;
    i2cConfig.isDma      = CyFalse;

    status = CyU3PI2cSetConfig (&i2cConfig, NULL);
    if (status != CY_U3P_SUCCESS)
    {
        return status;
    }
#if 0
    /* Now create the DMA channels required for read and write. */
    CyU3PMemSet ((uint8_t *)&dmaConfig, 0, sizeof(dmaConfig));
    dmaConfig.size           = pageLen;
    /* No buffers need to be allocated as this will be used
     * only in override mode. */
    dmaConfig.count          = 0;
    dmaConfig.prodAvailCount = 0;
    dmaConfig.dmaMode        = CY_U3P_DMA_MODE_BYTE;
    dmaConfig.prodHeader     = 0;
    dmaConfig.prodFooter     = 0;
    dmaConfig.consHeader     = 0;
    dmaConfig.notification   = 0;
    dmaConfig.cb             = NULL;

    /* Create a channel to write to the EEPROM. */
    dmaConfig.prodSckId = CY_U3P_CPU_SOCKET_PROD;
    dmaConfig.consSckId = CY_U3P_LPP_SOCKET_I2C_CONS;
    status = CyU3PDmaChannelCreate (&glI2cTxHandle,
            CY_U3P_DMA_TYPE_MANUAL_OUT, &dmaConfig);
    if (status != CY_U3P_SUCCESS)
    {
        return status;
    }

    /* Create a channel to read from the EEPROM. */
    dmaConfig.prodSckId = CY_U3P_LPP_SOCKET_I2C_PROD;
    dmaConfig.consSckId = CY_U3P_CPU_SOCKET_CONS;
    status = CyU3PDmaChannelCreate (&glI2cRxHandle,
            CY_U3P_DMA_TYPE_MANUAL_IN, &dmaConfig);

    if (status == CY_U3P_SUCCESS)
    {
        glI2cPageSize = pageLen;
    }
#endif
    return status;
}



/* I2C read / write for programmer application. */
CyU3PReturnStatus_t HAL_I2cTransfer ( uint16_t  byteAddress,
									  uint8_t   devAddr,
									  uint16_t  byteCount,
									  uint8_t  *buffer,
									  CyBool_t  isRead )
{
    CyU3PDmaBuffer_t buf_p;
    CyU3PI2cPreamble_t preamble;
    uint16_t pageCount = (byteCount / glI2cPageSize);
    CyU3PReturnStatus_t status = CY_U3P_SUCCESS;

    if (byteCount == 0)
    {
        return CY_U3P_SUCCESS;
    }

    if ((byteCount % glI2cPageSize) != 0)
    {
        pageCount ++;
    }


    /* Update the buffer address. */
    buf_p.buffer = buffer;
    buf_p.status = 0;

    while (pageCount != 0)
    {
        if (isRead)
        {
            /* Update the preamble information. */
            preamble.length    = 4;
            preamble.buffer[0] = devAddr;
            preamble.buffer[1] = (uint8_t)(byteAddress >> 8);
            preamble.buffer[2] = (uint8_t)(byteAddress & 0xFF);
            preamble.buffer[3] = (devAddr | 0x01);
            preamble.ctrlMask  = 0x0004;

            buf_p.size = glI2cPageSize;
            buf_p.count = glI2cPageSize;

            status = CyU3PI2cSendCommand (&preamble, glI2cPageSize, isRead);
            if (status != CY_U3P_SUCCESS)
            {
                return status;
            }
            status = CyU3PDmaChannelSetupRecvBuffer (&glI2cRxHandle, &buf_p);
            if (status != CY_U3P_SUCCESS)
            {
                return status;
            }
            status = CyU3PDmaChannelWaitForCompletion(&glI2cRxHandle,EEPROM_PROG_TIMEOUT);
            if (status != CY_U3P_SUCCESS)
            {
                return status;
            }
        }
        else /* Write */
        {
            /* Update the preamble information. */
            preamble.length    = 3;
            preamble.buffer[0] = devAddr;
            preamble.buffer[1] = (uint8_t)(byteAddress >> 8);
            preamble.buffer[2] = (uint8_t)(byteAddress & 0xFF);
            preamble.ctrlMask  = 0x0000;

            buf_p.size = glI2cPageSize;
            buf_p.count = glI2cPageSize;

            status = CyU3PDmaChannelSetupSendBuffer (&glI2cTxHandle,
                    &buf_p);
            if (status != CY_U3P_SUCCESS)
            {
                return status;
            }
            status = CyU3PI2cSendCommand (&preamble, glI2cPageSize, isRead);
            if (status != CY_U3P_SUCCESS)
            {
                return status;
            }
            status = CyU3PDmaChannelWaitForCompletion(&glI2cTxHandle,EEPROM_PROG_TIMEOUT);
            if (status != CY_U3P_SUCCESS)
            {
                return status;
            }
        }

        /* Update the parameters */
        byteAddress  += glI2cPageSize;
        buf_p.buffer += glI2cPageSize;
        pageCount --;

        /* Need a delay between write operations. */
        CyU3PThreadSleep (10);
    }

    return CY_U3P_SUCCESS;
}

CyU3PReturnStatus_t HAL_I2CWrite (uint8_t slaveAddr,uint8_t Addr,uint8_t count,uint8_t *buf)
{
    CyU3PReturnStatus_t apiRetStatus = CY_U3P_SUCCESS;
    CyU3PI2cPreamble_t preamble;


    /* Set up the I2C control parameters and invoke the write API. */
    preamble.buffer[0] = (slaveAddr<<1);
    preamble.buffer[1] = Addr;
    preamble.length    = 2;
    preamble.ctrlMask  = 0x0000;

    apiRetStatus = CyU3PI2cTransmitBytes (&preamble, buf, count, 0);

    return apiRetStatus;
}
CyU3PReturnStatus_t HAL_I2CRead(uint8_t slaveAddr,uint8_t Addr,uint8_t count,uint8_t *buf)
{
    CyU3PReturnStatus_t apiRetStatus = CY_U3P_SUCCESS;
    CyU3PI2cPreamble_t preamble;


    preamble.buffer[0] = (slaveAddr<<1);   
    preamble.buffer[1] = Addr;
    preamble.buffer[2] = (slaveAddr<<1)|0x01;
    preamble.length    = 3;
    preamble.ctrlMask  = 0x0002;                                /*  Send start bit after third byte of preamble. */

    apiRetStatus = CyU3PI2cReceiveBytes (&preamble, buf, count, 0);

    return apiRetStatus;
}


CyU3PReturnStatus_t HAL_I2CWriteEEPROM (uint8_t slaveAddr,uint16_t Addr,uint32_t count,uint8_t *buf)
{
    CyU3PReturnStatus_t apiRetStatus = CY_U3P_SUCCESS;
    CyU3PI2cPreamble_t preamble;


    /* Set up the I2C control parameters and invoke the write API. */
	preamble.length    = 3;
	preamble.buffer[0] = slaveAddr;
	preamble.buffer[1] = (uint8_t)(Addr >> 8);
	preamble.buffer[2] = (uint8_t)(Addr & 0xFF);
	preamble.ctrlMask  = 0x0000;

    apiRetStatus = CyU3PI2cTransmitBytes (&preamble, buf, count, 0);

    return apiRetStatus;
}

CyU3PReturnStatus_t HAL_I2CReadEEPROM(uint8_t slaveAddr,uint16_t Addr,uint32_t count,uint8_t *buf)
{
    CyU3PReturnStatus_t apiRetStatus = CY_U3P_SUCCESS;
    CyU3PI2cPreamble_t preamble;

	preamble.length    = 4;
	preamble.buffer[0] = slaveAddr;
	preamble.buffer[1] = (uint8_t)(Addr >> 8);
	preamble.buffer[2] = (uint8_t)(Addr & 0xFF);
	preamble.buffer[3] = (slaveAddr | 0x01);
	preamble.ctrlMask  = 0x0004;                            /*  Send start bit after third byte of preamble. */

    apiRetStatus = CyU3PI2cReceiveBytes (&preamble, buf, count, 0);

    return apiRetStatus;
}


