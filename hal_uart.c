#include <cyu3uart.h>
#include <cyu3error.h>
#include <cyu3dma.h>
#include <cyu3lpp.h>
#include <cyu3system.h>
#include <cyu3utils.h>
#include <uart_regs.h>
#include <cyu3os.h>
#include "uart.h"

/* @@UART
 * Summary
 * UART driver and convenience API for the EZ-USB FX3 device.
 */

/* Default baud rate used by the UART driver. */
#define CY_U3P_UART_DEFAULT_BAUD_RATE           (9600)

/* Default timeout value in waiting for a response from the UART registers. */
#define CY_U3P_UART_TIMEOUT                     (0xFFFFF)

static CyBool_t          glIsUartActive = CyFalse;                      /* Whether UART module is enabled. */
static CyBool_t          glIsUartDma = CyFalse;                         /* Whether UART is in DMA mode. */
static CyBool_t          glIsUartConfigured = CyFalse;                  /* Whether the UART has been configured. */
static CyU3PUartIntrCb_t glUartIntrCb = NULL;                           /* Callback for UART event notifications. */
static uint32_t          glUartReadTimeout = CY_U3P_UART_TIMEOUT;       /* Timeout used for UART read operation. */
static uint32_t          glUartWriteTimeout = CY_U3P_UART_TIMEOUT;      /* Timeout used for UART write operation. */
static CyU3PTimer frmaeTimer;

extern UARTControl * uartController;
extern CyU3PEvent  UartEvent; 
/* Register Call back function for UART */
void HAL_RegisterUartCallBack ( CyU3PUartIntrCb_t uartIntrCb)
{
    glUartIntrCb = uartIntrCb;
}

void HAL_UartInt_Handler (void)
{
    uint32_t mask, status;

    /* Read out the interrupts and clear them. */
    mask = UART->lpp_uart_intr & UART->lpp_uart_intr_mask;
    UART->lpp_uart_intr = mask;

    if (mask & CY_U3P_LPP_UART_RX_DATA)
    {
        while ((UART->lpp_uart_status & CY_U3P_LPP_UART_RX_DATA) != 0)
        {
			uartController->rx_buffer[uartController->rx_index++] = ((uint8_t)(UART->lpp_uart_ingress_data));
        }
		if(uartController->rx_index>=MIN_FRAME_LENGTH && !uartController->ackEventSent)
		{
			uartController->ackEventSent = CyTrue;
			CyU3PEventSet (&UartEvent, UART_EVT_ACK_FRAME, CYU3P_EVENT_OR);				
		}
		else if( uartController->rx_index>=MIN_ACK_RSP_FRAMES_LENGTH && 
			     uartController->rx_buffer[uartController->rx_index-2]==FPGA_FX3_TAIL1 &&
			     uartController->rx_buffer[uartController->rx_index-1]==FPGA_FX3_TAIL2
			    )
		{
			CyU3PEventSet (&UartEvent, UART_EVT_RESPOND_FRAME, CYU3P_EVENT_OR);
		}
	}

    if (mask & CY_U3P_LPP_UART_RX_DONE)
    {
        glUartIntrCb (CY_U3P_UART_EVENT_RX_DONE, (CyU3PUartError_t)0);
    }

    if (mask & CY_U3P_LPP_UART_TX_DONE)
    {
        glUartIntrCb (CY_U3P_UART_EVENT_TX_DONE, (CyU3PUartError_t)0);
    }

    if (mask & CY_U3P_LPP_UART_ERROR)
    {
        glUartIntrCb (CY_U3P_UART_EVENT_ERROR, (CyU3PUartError_t)((UART->lpp_uart_status &
            CY_U3P_LPP_UART_ERROR_CODE_MASK) >> CY_U3P_LPP_UART_ERROR_CODE_POS));
    }
}

/*
 * This function initializes the UART
 */
CyU3PReturnStatus_t HAL_UartInit (void)
{
    CyU3PReturnStatus_t status;
    /* Check the IO matrix */
    if (!CyU3PIsLppIOConfigured(CY_U3P_LPP_UART))
    {
        return CY_U3P_ERROR_NOT_CONFIGURED;
    }
    if (glIsUartActive)
    {
        return CY_U3P_ERROR_ALREADY_STARTED;
    }

    /* Set the clock to a default value.
    * This should prcede the UART power up*/
    status = CyU3PUartSetClock (CY_U3P_UART_DEFAULT_BAUD_RATE);
    if(status != CY_U3P_SUCCESS)
    {
        return status;
    }

    /* Identify if the LPP block has been initialized. */
    status = CyU3PLppInit (CY_U3P_LPP_UART,HAL_UartInt_Handler);
    if(status != CY_U3P_SUCCESS)
    {
        return status;
    }

    /* Hold the UART block in reset. */
    UART->lpp_uart_power &= ~(CY_U3P_LPP_UART_RESETN);
    CyU3PBusyWait (10);
    UART->lpp_uart_power |= (CY_U3P_LPP_UART_RESETN);

    /* Wait for the active bit to be asserted by the hardware */
    while (!(UART->lpp_uart_power & CY_U3P_LPP_UART_ACTIVE));

    /* Mark the module as active. */
    glIsUartActive = CyTrue;

	//CyU3PTimerCreate (&frmaeTimer, TimerCb, 0, 100, 100, CYU3P_NO_ACTIVATE);

    return CY_U3P_SUCCESS;
}

/*
 * This function de-initializes the UART
 */
CyU3PReturnStatus_t HAL_UartDeInit(void)
{
    CyU3PReturnStatus_t status=CY_U3P_SUCCESS;

    if (!glIsUartActive)
    {
        return CY_U3P_ERROR_NOT_STARTED;
    }

    /* Power off UART block. */
    UART->lpp_uart_power &= ~(CY_U3P_LPP_UART_RESETN);
    CyU3PBusyWait (10);

    /* Mark the block as disabled. */
    glIsUartConfigured = CyFalse;
    glIsUartDma        = CyFalse;
    glUartReadTimeout  = CY_U3P_UART_TIMEOUT;
    glUartWriteTimeout = CY_U3P_UART_TIMEOUT;
    glIsUartActive     = CyFalse;

    /* Identify if the LPP block has to be disabled. */
    status = CyU3PLppDeInit (CY_U3P_LPP_UART);

    /* Disable the UART clock */
    CyU3PUartStopClock();

    return status;
}

/*
 * configures and opens the UART
 */
CyU3PReturnStatus_t HAL_UartSetConfig ( CyU3PUartConfig_t *config,CyU3PUartIntrCb_t cb)
{
    uint32_t regVal = 0, status;

    /* Check if the UART is initiaized */
    if (!glIsUartActive)
    {
        return CY_U3P_ERROR_NOT_STARTED;
    }

    /* Check for parameter validity. */
    if (config == NULL)
    {
        return CY_U3P_ERROR_NULL_POINTER;
    }
    if (!(config->rxEnable | config->txEnable))
    {
        return CY_U3P_ERROR_BAD_ARGUMENT;
    }
    if (config->parity >= CY_U3P_UART_NUM_PARITY)
    {
        return CY_U3P_ERROR_BAD_ARGUMENT;
    }
    if ((config->stopBit != CY_U3P_UART_ONE_STOP_BIT) && (config->stopBit != CY_U3P_UART_TWO_STOP_BIT))
    {
        return CY_U3P_ERROR_BAD_ARGUMENT;
    }


    /* Setup clock for the UART block according
    * to the baud rate. */
    status = CyU3PUartSetClock (config->baudRate);
    if (status != CY_U3P_SUCCESS)
    {
        return status;
    }

    /* Disable the UART before changing any register */
    UART->lpp_uart_config &= ~(CY_U3P_LPP_UART_ENABLE);

    if (config->rxEnable)
    {
        regVal |= (CY_U3P_LPP_UART_RTS | CY_U3P_LPP_UART_RX_ENABLE);
    }
    if (config->txEnable)
    {
        regVal |= CY_U3P_LPP_UART_TX_ENABLE;
    }
    if (config->flowCtrl)
    {
        regVal |= (CY_U3P_LPP_UART_TX_FLOW_CTRL_ENBL |
            CY_U3P_LPP_UART_RX_FLOW_CTRL_ENBL);
    }
    if (config->isDma)
    {
        regVal |= CY_U3P_LPP_UART_DMA_MODE;
        glIsUartDma = CyTrue;
    }
    else
    {
        glIsUartDma = CyFalse;
    }

    regVal |= (config->stopBit << CY_U3P_LPP_UART_STOP_BITS_POS) &
        CY_U3P_LPP_UART_STOP_BITS_MASK;

    switch (config->parity)
    {
    case CY_U3P_UART_EVEN_PARITY:
        regVal |= CY_U3P_LPP_UART_PARITY;
        break;
    case CY_U3P_UART_ODD_PARITY:
        regVal |= (CY_U3P_LPP_UART_PARITY | CY_U3P_LPP_UART_PARITY_ODD);
        break;
    default:
        break;
    }

    /* Set timing when to sample for RX input */
    regVal |= 7 << CY_U3P_LPP_UART_RX_POLL_POS;
    /* Update the configuration. */
    UART->lpp_uart_config = regVal;

    /* Update the UART DMA sockets. */
    UART->lpp_uart_socket = ((CY_U3P_LPP_SOCKET_UART_CONS &
        CY_U3P_LPP_UART_EGRESS_SOCKET_MASK) |
        ((CY_U3P_LPP_SOCKET_UART_PROD & CY_U3P_LPP_UART_EGRESS_SOCKET_MASK)
        << CY_U3P_LPP_UART_INGRESS_SOCKET_POS));

    HAL_RegisterUartCallBack (cb);
    if (cb != NULL)
    {
        /* Enable the RX_DATA interrupt if DMA mode is not selected. */
        if (config->isDma)
            regVal = CY_U3P_LPP_UART_RX_DONE | CY_U3P_LPP_UART_TX_DONE | CY_U3P_LPP_UART_BREAK | CY_U3P_LPP_UART_ERROR;
        else
            regVal = CY_U3P_LPP_UART_RX_DATA | CY_U3P_LPP_UART_ERROR;

        /* Enable the interrupts. */
        UART->lpp_uart_intr_mask = regVal;
    }
    else
    {
        /* Disable the interrupt */
        UART->lpp_uart_intr_mask = 0;
    }

    glIsUartConfigured = CyTrue;

    /* Enable the UART only at the end. */
    UART->lpp_uart_config |= CY_U3P_LPP_UART_ENABLE;


    return CY_U3P_SUCCESS;
}

/*
 * Sets registers for dma egress transfer
 */
CyU3PReturnStatus_t HAL_UartTxSetBlockXfer (uint32_t txSize)
{
    CyU3PReturnStatus_t status;

    /* Lock is not acquired as this is only a read. */
    if (!glIsUartDma)
    {
        return CY_U3P_ERROR_NOT_CONFIGURED;
    }


    if (status != CY_U3P_SUCCESS)
    {
        return status;
    }

    UART->lpp_uart_tx_byte_count = txSize;


    return CY_U3P_SUCCESS;
}

/*
 * Sets registers for dma ingress transfer
 */
CyU3PReturnStatus_t HAL_UartRxSetBlockXfer (uint32_t rxSize)
{
    CyU3PReturnStatus_t status;

    /* Lock is not acquired as this is only a read. */
    if (!glIsUartDma)
    {
        return CY_U3P_ERROR_NOT_CONFIGURED;
    }

    if (status != CY_U3P_SUCCESS)
    {
        return status;
    }

    UART->lpp_uart_rx_byte_count = rxSize;

    return CY_U3P_SUCCESS;
}

/* Set the timeout for UART transmit/receive operations. */
CyU3PReturnStatus_t HAL_UartSetTimeout (uint32_t readLoopCnt, uint32_t writeLoopCnt)
{
    /* If the UART has been initialized, update the timeout value. */
    if (!glIsUartActive)
        return CY_U3P_ERROR_NOT_STARTED;

    glUartReadTimeout  = readLoopCnt;
    glUartWriteTimeout = writeLoopCnt;

    return CY_U3P_SUCCESS;
}

/*
 * Transmits data byte by byte over the UART interface
 */
uint32_t HAL_UartTransmitBytes (uint8_t *data_p,uint32_t count,CyU3PReturnStatus_t *status)
{
    uint32_t timeout;
    int32_t i;
    CyU3PReturnStatus_t temp = CY_U3P_SUCCESS;

    if (!glIsUartConfigured)
    {
        temp= CY_U3P_ERROR_NOT_CONFIGURED;
    }
    /* Lock is not acquired as this is only a read. */
    if (glIsUartDma)
    {
        temp = CY_U3P_ERROR_NOT_CONFIGURED;
    }
    if (data_p == NULL)
    {
        temp = CY_U3P_ERROR_NULL_POINTER;
    }
    if (count == 0)
    {
        temp = CY_U3P_ERROR_BAD_ARGUMENT;
    }

    if (status != NULL)
    {
        *status = temp;
    }
	else
	{
        return 0;
	}


    if (status != NULL)
    {
        *status = temp;
    }
	else
	{
		return 0;
	}


    temp    = CY_U3P_SUCCESS;
    timeout = glUartWriteTimeout;

    /* While we have data to transmit, wait for space in the FIFO and add data to the FIFO. */
    for (i = 0; i < count; i++)
    {
        while ((UART->lpp_uart_status & CY_U3P_LPP_UART_TX_SPACE) == 0)
        {
            if (timeout-- == 0)
            {
                temp = CY_U3P_ERROR_TIMEOUT;
                break;
            }
        }

        if (temp != CY_U3P_SUCCESS)
        {
            break;
        }

        UART->lpp_uart_egress_data = data_p[i];
    }

    /* Wait for the transfer to be done */
    while (((UART->lpp_uart_status & CY_U3P_LPP_UART_TX_DONE) == 0) && (timeout > 0))
    {
        /* Not returning an error because data has already been queued. */
        timeout--;
    }

    /* Clear the tx interrupts. */
    UART->lpp_uart_intr = CY_U3P_LPP_UART_TX_DONE | CY_U3P_LPP_UART_TX_SPACE;

    if (status != NULL)
    {
        *status = temp;
    }

    return i;
}

/*
 * Receives data byte by byte over the UART interface
 */
uint32_t HAL_UartReceiveBytes ( uint8_t *data_p,uint32_t count,CyU3PReturnStatus_t *status)
{
    uint32_t timeout;
    int32_t i;
    CyU3PReturnStatus_t temp = CY_U3P_SUCCESS;

    /* No need to acquire lock during status checks. */
    if (!glIsUartConfigured)
    {
        temp= CY_U3P_ERROR_NOT_CONFIGURED;
    }
    if (glIsUartDma)
    {
        temp = CY_U3P_ERROR_NOT_CONFIGURED;
    }
    if (data_p == NULL)
    {
        temp = CY_U3P_ERROR_NULL_POINTER;
    }
    if (count == 0)
    {
        temp = CY_U3P_ERROR_BAD_ARGUMENT;
    }

    if (temp != CY_U3P_SUCCESS)
    {
        if (status != NULL)
        {
            *status = temp;
        }
        return 0;
    }

    if (status != NULL)
    {
        *status = temp;
    }
	else
	{
		return 0;
	}

    temp    = CY_U3P_SUCCESS;
    timeout = glUartReadTimeout;

    /* While all the data is not received, wait for data in the FIFO and read it. */
    for (i = 0; i < count; i++)
    {
        while ((UART->lpp_uart_status & CY_U3P_LPP_UART_RX_DATA) == 0)
        {
            if (timeout-- == 0)
            {
                temp = CY_U3P_ERROR_TIMEOUT;
                break;
            }
        }

        /* Break from loop in case of timeout. */
        if (temp == CY_U3P_ERROR_TIMEOUT)
        {
            break;
        }

        data_p[i] = ((uint8_t)(UART->lpp_uart_ingress_data));
    }

    if (status != NULL)
    {
        *status = temp;
    }

    return i;
}
