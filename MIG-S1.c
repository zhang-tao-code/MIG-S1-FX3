/*
 ## Cypress USB 3.0 Platform source file (cyfxslfifosync.c)
 ## ===========================
 ##
 ##  Copyright Cypress Semiconductor Corporation, 2010-2011,
 ##  All Rights Reserved
 ##  UNPUBLISHED, LICENSED SOFTWARE.
 ##
 ##  CONFIDENTIAL AND PROPRIETARY INFORMATION
 ##  WHICH IS THE PROPERTY OF CYPRESS.
 ##
 ##  Use of this file is governed
 ##  by the license agreement included in the file
 ##
 ##     <install>/license/license.txt
 ##
 ##  where <install> is the Cypress software
 ##  installation root directory path.
 ##
 ## ===========================
*/

/* This file illustrates the Slave FIFO Synchronous mode example */

/*
   This example comprises of two USB bulk endpoints. A bulk OUT endpoint acts as the
   producer of data from the host. A bulk IN endpoint acts as the consumer of data to
   the host. Appropriate vendor class USB enumeration descriptors with these two bulk
   endpoints are implemented.

   The GPIF configuration data for the Synchronous Slave FIFO operation is loaded onto
   the appropriate GPIF registers. The p-port data transfers are done via the producer
   p-port socket and the consumer p-port socket.

   This example implements two DMA Channels in MANUAL mode one for P to U data transfer
   and one for U to P data transfer.

   The U to P DMA channel connects the USB producer (OUT) endpoint to the consumer p-port
   socket. And the P to U DMA channel connects the producer p-port socket to the USB 
   consumer (IN) endpoint.

   Upon every reception of data in the DMA buffer from the host or from the p-port, the
   CPU is signalled using DMA callbacks. There are two DMA callback functions implemented
   each for U to P and P to U data paths. The CPU then commits the DMA buffer received so
   that the data is transferred to the consumer.

   The DMA buffer size for each channel is defined based on the USB speed. 64 for full
   speed, 512 for high speed and 1024 for super speed. CY_FX_SLFIFO_DMA_BUF_COUNT in the
   header file defines the number of DMA buffers per channel.

   The constant CY_FX_SLFIFO_GPIF_16_32BIT_CONF_SELECT in the header file is used to
   select 16bit or 32bit GPIF data bus configuration.
 */
#include "math.h"
#include "cyu3system.h"
#include "cyu3os.h"
#include "cyu3dma.h"
#include "cyu3error.h"
#include "cyu3usb.h"
#include "cyu3uart.h"
#include "MIG-S1.h"
#include "cyu3gpif.h"
#include "cyu3pib.h"
#include "pib_regs.h"
#include "cyu3i2c.h"
#include "cyu3gpio.h"
#include "cyu3utils.h"

/* This file should be included only once as it contains
 * structure definitions. Including it in multiple places
 * can result in linker error. */
#include "cyfxgpif_syncsf.h"

#include "i2c_fpga.h"
#include "dps.h"
#include "epcs.h"
#include "err_code.h"
#include "autoskew.h"
#include "uart.h"
#include "hal_i2c.h"
#include "pca6416a.h"

//define by Atlas
#define TEST

#ifdef TEST
#define REG_SET_STDBY(data)
#define REG_GET_STDBY(data) CY_U3P_SUCCESS
#define REG_SET_RESET(data)
#define REG_GET_RESET(data) CY_U3P_SUCCESS
#define REG_GET_GPIODIR(data) CY_U3P_SUCCESS
#define REG_SET_GPIODIR(data)
#define REG_SET_GPIOOUT(data)
#define REG_SET_MCLK(data)

#define DMM_BYPASS(data)
#define DMM_SELECT(data)

#define GET_VERSION_FPGA(data) CY_U3P_SUCCESS
#define GET_VERSION_PCB(data) CY_U3P_SUCCESS

#define REG_SET(data1,data2)
#define REG_GET(data1,data2) CY_U3P_SUCCESS

#define POWER_READ(data1,data2)

#define WRITE_WORD(data1,data2)
#define READ_WORD(data1,data2)

#define LOCAL_READ(data1,data2,data3)
#define LOCAL_WRITE(data1,data2,data3)

#define Init_FrameMonitor()
#define GetFramePeriod(data)

#define ProtectPower(data)
#define SelectInputBits(data)
uint32_t tick;
uint32_t cur_tick;

CyU3PReturnStatus_t
I2CWrite (
        uint8_t slaveAddr,
        uint8_t Addr,
        uint8_t count,
        uint8_t *buf)
{
    CyU3PReturnStatus_t apiRetStatus = CY_U3P_SUCCESS;
    CyU3PI2cPreamble_t preamble;


    /* Set up the I2C control parameters and invoke the write API. */
    preamble.buffer[0] = slaveAddr;
    preamble.buffer[1] = Addr;
    preamble.length    = 2;
    preamble.ctrlMask  = 0x0000;

    apiRetStatus = CyU3PI2cTransmitBytes (&preamble, buf, count, 0);

    return apiRetStatus;
}

CyU3PReturnStatus_t
I2CRead (
        uint8_t slaveAddr,
        uint8_t Addr,
        uint8_t count,
        uint8_t *buf)
{
#define I2C_SLAVEADDR_MASK 0xFE
    CyU3PReturnStatus_t apiRetStatus = CY_U3P_SUCCESS;
    CyU3PI2cPreamble_t preamble;


    preamble.buffer[0] = slaveAddr & I2C_SLAVEADDR_MASK;        /*  Mask out the transfer type bit. */
    preamble.buffer[1] = Addr;
    preamble.buffer[2] = slaveAddr;
    preamble.length    = 3;
    preamble.ctrlMask  = 0x0002;                                /*  Send start bit after third byte of preamble. */

    apiRetStatus = CyU3PI2cReceiveBytes (&preamble, buf, count, 0);

    return apiRetStatus;
}

#endif



#define REF_CLK_FREQ		(40*1000000)	//	40 MHz
#define GET_PRESCALE_VAL(F)	((REF_CLK_FREQ/5)/(F))

CyU3PThread slFifoAppThread;	        /* Slave FIFO application thread structure */
//CyU3PDmaChannel glChHandleSlFifoUtoP;   /* DMA Channel handle for U2P transfer. */
CyU3PDmaChannel glChHandleSlFifoPtoU;   /* DMA Channel handle for P2U transfer. */

//uint32_t glDMARxCount = 0;               /* Counter to track the number of buffers received from USB. */
//uint32_t glDMATxCount = 0;               /* Counter to track the number of buffers sent to USB. */
CyBool_t glIsApplnActive = CyFalse;      /* Whether the loopback application is active or not. */
CyBool_t glDataTransStarted = CyFalse;   /* Whether DMA transfer has been started after enumeration. */

uint32_t glTimer = 0;
uint32_t glPTimer = 0;
uint32_t glDTimer = 0;

uint8_t glEp0Buffer[4096] __attribute__ ((aligned (32))); /* Local buffer used for vendor command handling. */
//uint8_t glEp0Buffer[1024] __attribute__ ((aligned (32))); /* Local buffer used for vendor command handling. */


/* Control request related variables. */
CyU3PEvent glBulkLpEvent;       /* Event group used to signal the thread that there is a pending request. */
uint32_t   gl_setupdat0;        /* Variable that holds the setupdat0 value (bmRequestType, bRequest and wValue). */
uint32_t   gl_setupdat1;        /* Variable that holds the setupdat1 value (wIndex and wLength). */
#define CYFX_USB_CTRL_TASK      (1 << 0)        /* Event that indicates that there is a pending USB control request. */
#define CYFX_USB_HOSTWAKE_TASK  (1 << 1)        /* Event that indicates the a Remote Wake should be attempted. */
#define APP_USB_EVT_VENDOR_RQT  (1 << 2) 

#define FPGA_I2C_REG_ADDR_MASK  (1 << 0)        /*If Bit 0 set,16bit address*/
#define FPGA_I2C_REG_DARA_MASK  (1 << 1)        /*If Bit 1 set,16bit data   */

#pragma pack(push,1)
typedef union
{
	uint8_t data[MAX_PAYLOAD_SIZE];
	FPGA_IIC_A8D8_REG_t  regA8D8[MAX_REGS_PER_ACCESS];
	FPGA_IIC_A8D16_REG_t regA8D16[MAX_REGS_PER_ACCESS];
	FPGA_IIC_A16D8_REG_t regA16D8[MAX_REGS_PER_ACCESS];
	FPGA_IIC_A16D16_REG_t regA16D16[MAX_REGS_PER_ACCESS];
}USB_UART_MAPPER;

typedef struct
{
	uint8_t  type;
	uint8_t  address_mode;
	uint16_t regs_number;
	uint32_t long_cmd_len;
	USB_UART_MAPPER mapper;
}USBVENDOR_RQT;

typedef struct
{
	uint8_t  slave_addr;
	uint8_t  reg_addr;
	uint8_t  length;
	uint8_t  data[256];
}MCU_RW_RQT;

#pragma pack(pop)

static USBVENDOR_RQT usbvendor_request;
static MCU_RW_RQT    mcuRW_request; 


uint8_t m_retValue[5];  //(J.J 2011.07.12) [0] = return code, [1] = I2C Write Count
//CyBool_t glRegWriteActive = CyFalse;

/* Application Error Handler */
void
CyFxAppErrorHandler (
        CyU3PReturnStatus_t apiRetStatus    /* API return status */
        )
{
    /* Application failed with the error code apiRetStatus */

    /* Add custom debug or recovery actions here */

    /* Loop Indefinitely */
    for (;;)
    {
        /* Thread sleep : 100 ms */
        CyU3PThreadSleep (100);
    }
}

/* This function starts the slave FIFO loop application. This is called
 * when a SET_CONF event is received from the USB host. The endpoints
 * are configured and the DMA pipe is setup in this function. */
void
CyFxSlFifoApplnStart (
        void)
{
    uint16_t size = 0;
    CyU3PEpConfig_t epCfg;
    CyU3PDmaChannelConfig_t dmaCfg;
    CyU3PReturnStatus_t apiRetStatus = CY_U3P_SUCCESS;
    CyU3PUSBSpeed_t usbSpeed = CyU3PUsbGetSpeed();

    /* First identify the usb speed. Once that is identified,
     * create a DMA channel and start the transfer on this. */

    /* Based on the Bus Speed configure the endpoint packet size */
    switch (usbSpeed)
    {
        case CY_U3P_FULL_SPEED:
            size = 64;
            break;

        case CY_U3P_HIGH_SPEED:
            size = 512;
            break;

        case  CY_U3P_SUPER_SPEED:
            size = 1024;
            break;

        default:
            CyFxAppErrorHandler (CY_U3P_ERROR_FAILURE);
            break;
    }

    CyU3PMemSet ((uint8_t *)&epCfg, 0, sizeof (epCfg));
    epCfg.enable = CyTrue;
    epCfg.epType = CY_U3P_USB_EP_BULK;
    epCfg.burstLen = (usbSpeed == CY_U3P_SUPER_SPEED) ?
        (CY_FX_EP_BURST_LENGTH) : 1;
    epCfg.streams = 0;
    epCfg.pcktSize = size;

    /* Producer endpoint configuration */
/*    apiRetStatus = CyU3PSetEpConfig(CY_FX_EP_PRODUCER, &epCfg);
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
        CyFxAppErrorHandler (apiRetStatus);
    }
*/
    /* Consumer endpoint configuration */
    apiRetStatus = CyU3PSetEpConfig(CY_FX_EP_CONSUMER, &epCfg);
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
        CyFxAppErrorHandler (apiRetStatus);
    }

    /* Create a DMA AUTO channel for U2P transfer.
     * DMA size is set based on the USB speed. */
//    dmaCfg.size  = size;
    dmaCfg.size  = (usbSpeed == CY_U3P_SUPER_SPEED) ?
//            (size * CY_FX_EP_BURST_LENGTH) : (size);
    	    (2 * size * CY_FX_EP_BURST_LENGTH) : (size);
//    (7 * size * CY_FX_EP_BURST_LENGTH) : (size);

    dmaCfg.count = CY_FX_SLFIFO_DMA_BUF_COUNT;

    dmaCfg.dmaMode = CY_U3P_DMA_MODE_BYTE;
    dmaCfg.notification = 0;
    dmaCfg.cb = NULL;
    dmaCfg.prodHeader = 0;
    dmaCfg.prodFooter = 0;
    dmaCfg.consHeader = 0;
    dmaCfg.prodAvailCount = 0;

    /* Create a DMA MANUAL channel for P2U transfer. */
    dmaCfg.prodSckId = CY_FX_PRODUCER_PPORT_SOCKET;
    dmaCfg.consSckId = CY_FX_CONSUMER_USB_SOCKET;
    dmaCfg.cb = NULL;
    apiRetStatus = CyU3PDmaChannelCreate (&glChHandleSlFifoPtoU,
//            CY_U3P_DMA_TYPE_MANUAL, &dmaCfg);
			CY_U3P_DMA_TYPE_AUTO, &dmaCfg);
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
        CyFxAppErrorHandler(apiRetStatus);
    }

    /* Flush the Endpoint memory */
//    CyU3PUsbFlushEp(CY_FX_EP_PRODUCER);
    CyU3PUsbFlushEp(CY_FX_EP_CONSUMER);

    /* Set DMA channel transfer size. */

    apiRetStatus = CyU3PDmaChannelSetXfer (&glChHandleSlFifoPtoU, CY_FX_SLFIFO_DMA_TX_SIZE);
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
        CyFxAppErrorHandler(apiRetStatus);
    }

    /* Update the status flag. */
    glIsApplnActive = CyTrue;
}


/* This function stops the slave FIFO loop application. This shall be called
 * whenever a RESET or DISCONNECT event is received from the USB host. The
 * endpoints are disabled and the DMA pipe is destroyed by this function. */
void
CyFxSlFifoApplnStop (
        void)
{
    CyU3PEpConfig_t epCfg;
    CyU3PReturnStatus_t apiRetStatus = CY_U3P_SUCCESS;

    /* Update the flag. */
    glIsApplnActive = CyFalse;

    /* Destroy the channel */
    CyU3PDmaChannelDestroy (&glChHandleSlFifoPtoU);

    /* Flush the endpoint memory */
    CyU3PUsbFlushEp(CY_FX_EP_CONSUMER);

    /* Disable endpoints. */
    CyU3PMemSet ((uint8_t *)&epCfg, 0, sizeof (epCfg));
    epCfg.enable = CyFalse;

    /* Consumer endpoint configuration. */
    apiRetStatus = CyU3PSetEpConfig(CY_FX_EP_CONSUMER, &epCfg);
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
        CyFxAppErrorHandler (apiRetStatus);
    }
}

uint8_t lookupOct(float f)
{
	if(f<2.076)  		return 0;
	else if(f<4.152)	return 1;
	else if(f<8.304)	return 2;
	else if(f<16.61)	return 3;
	else if(f<33.22)	return 4;
	else if(f<66.43)	return 5;
	else if(f<132.9)	return 6;
	else if(f<265.7)	return 7;
	else if(f<531.4)	return 8;
	else if(f<1063)	    return 9;
	else if(f<2126)	    return 10;
	else if(f<4252)	    return 11;
	else if(f<8503)	    return 12;
	else if(f<17010)	return 13;
	else if(f<34010)	return 14;
	else if(f<68030)	return 15;

}


/* Callback to handle the USB setup requests. */
CyBool_t
CyFxSlFifoApplnUSBSetupCB (
        uint32_t setupdat0, /* SETUP Data 0 */
        uint32_t setupdat1  /* SETUP Data 1 */
    )
{
    /* Fast enumeration is used. Only requests addressed to the interface, class,
     * vendor and unknown control requests are received by this function.
     * This application does not support any class or vendor requests. */

    uint8_t  bRequest, bReqType;
    uint8_t  bType, bTarget;
    uint16_t wValue, wIndex, wLength;
    CyBool_t isHandled = CyFalse;

    uint8_t param[4];
    //CyBool_t txEnable = CyFalse;
    uint8_t bytRet, i, j, read_byte;
    uint8_t dev_id;
    uint8_t i2cAddr = 0;
    uint32_t *addr = NULL;
    int32_t offset = 0;
    
    uint16_t	RegCount = 0, RegLoop = 0, SetSize = 0;
    unsigned char byte_Value[16] = {0,};

    CyBool_t isDevIDChange = CyFalse;  //(J.J 2012.11.06) "DEVADR" Add
    uint8_t bytDevID = 0x00;
    uint32_t retCount = 0;             //(J.J 2012.11.06) "PollingRegBit" Add

    CyU3PReturnStatus_t status = CY_U3P_SUCCESS;
	int uart_status = 0;
	uint16_t eepromAddr;
	uint32_t CMCLK_f;
	uint32_t pin_0_31,pin_32_60;
	uint16_t duration;
	

    /* Decode the fields from the setup request. */
    bReqType = (setupdat0 & CY_U3P_USB_REQUEST_TYPE_MASK);
    bType    = (bReqType & CY_U3P_USB_TYPE_MASK);
    bTarget  = (bReqType & CY_U3P_USB_TARGET_MASK);
    bRequest = ((setupdat0 & CY_U3P_USB_REQUEST_MASK) >> CY_U3P_USB_REQUEST_POS);
    wValue   = ((setupdat0 & CY_U3P_USB_VALUE_MASK)   >> CY_U3P_USB_VALUE_POS);
    wIndex   = ((setupdat1 & CY_U3P_USB_INDEX_MASK)   >> CY_U3P_USB_INDEX_POS);
    wLength  = ((setupdat1 & CY_U3P_USB_LENGTH_MASK)  >> CY_U3P_USB_LENGTH_POS);

    param[0] = (uint8_t) (wValue & 0x00FF);
    param[1] = (uint8_t) ((wValue & 0xFF00) >> 8);
    param[2] = (uint8_t) (wIndex & 0x00FF);
    param[3] = (uint8_t) ((wIndex & 0xFF00) >> 8);

    if (bType == CY_U3P_USB_STANDARD_RQT)
    {
        /* Handle SET_FEATURE(FUNCTION_SUSPEND) and CLEAR_FEATURE(FUNCTION_SUSPEND)
         * requests here. It should be allowed to pass if the device is in configured
         * state and failed otherwise. */
        if ((bTarget == CY_U3P_USB_TARGET_INTF) && ((bRequest == CY_U3P_USB_SC_SET_FEATURE)
                    || (bRequest == CY_U3P_USB_SC_CLEAR_FEATURE)) && (wValue == 0))
        {
            if (glIsApplnActive)
                CyU3PUsbAckSetup ();
            else
                CyU3PUsbStall (0, CyTrue, CyFalse);

            isHandled = CyTrue;
        }

        /* Handle Microsoft OS String Descriptor request. */
        if ((bTarget == CY_U3P_USB_TARGET_DEVICE) && (bRequest == CY_U3P_USB_SC_GET_DESCRIPTOR) &&
                (wValue == ((CY_U3P_USB_STRING_DESCR << 8) | 0xEE)))
        {
            /* Make sure we do not send more data than requested. */
            if (wLength > CyFxUsbOSDscr[0])
                wLength = CyFxUsbOSDscr[0];

            CyU3PUsbSendEP0Data (wLength, (uint8_t *)CyFxUsbOSDscr);
            isHandled = CyTrue;
        }

        /* CLEAR_FEATURE request for endpoint is always passed to the setup callback
         * regardless of the enumeration model used. When a clear feature is received,
         * the previous transfer has to be flushed and cleaned up. This is done at the
         * protocol level. Since this is just a loopback operation, there is no higher
         * level protocol. So flush the EP memory and reset the DMA channel associated
         * with it. If there are more than one EP associated with the channel reset both
         * the EPs. The endpoint stall and toggle / sequence number is also expected to be
         * reset. Return CyFalse to make the library clear the stall and reset the endpoint
         * toggle. Or invoke the CyU3PUsbStall (ep, CyFalse, CyTrue) and return CyTrue.
         * Here we are clearing the stall. */
        if ((bTarget == CY_U3P_USB_TARGET_ENDPT) && (bRequest == CY_U3P_USB_SC_CLEAR_FEATURE)
                && (wValue == CY_U3P_USBX_FS_EP_HALT))
        {
            if (glIsApplnActive)
            {
                if (wIndex == CY_FX_EP_CONSUMER)
                {
                    CyU3PDmaChannelReset (&glChHandleSlFifoPtoU);
                    CyU3PUsbFlushEp(CY_FX_EP_CONSUMER);
                    CyU3PUsbResetEp (CY_FX_EP_CONSUMER);
                    CyU3PDmaChannelSetXfer (&glChHandleSlFifoPtoU, CY_FX_SLFIFO_DMA_TX_SIZE);
                    CyU3PUsbStall (wIndex, CyFalse, CyTrue);
                    CyU3PUsbAckSetup ();
                    isHandled = CyTrue;
                }
            }
        }
    }

    if ((bType == CY_U3P_USB_VENDOR_RQT) && (bTarget == CY_U3P_USB_TARGET_DEVICE))
    {
        /* We set an event here and let the application thread below handle these requests.
         * isHandled needs to be set to True, so that the driver does not stall EP0. */
        isHandled = CyTrue;
        
        uint16_t unSize = 0;

        switch (bRequest)
        {
        	case 0xB5:
        		break;
            case 0x77:      /* Trigger remote wakeup. */
            case 0x84:
            case 0x90:
                /* Request to switch control back to the boot firmware. */
                gl_setupdat0 = setupdat0;
                gl_setupdat1 = setupdat1;
                CyU3PEventSet (&glBulkLpEvent, CYFX_USB_CTRL_TASK, CYU3P_EVENT_OR);
                break;

// MA6 --------------------------------------------------------------------------

            case erVER:
				usbvendor_request.type = erVER;
				CyU3PEventSet (&glBulkLpEvent, APP_USB_EVT_VENDOR_RQT, CYU3P_EVENT_OR);
				break;

			case ewREGWR:
				status = CyU3PUsbGetEP0Data (wLength, glEp0Buffer, NULL);
				usbvendor_request.type = ewREGWR;				
				usbvendor_request.mapper.data[0] = param[0];					
				usbvendor_request.mapper.data[1] = param[1];
				CyU3PEventSet (&glBulkLpEvent, APP_USB_EVT_VENDOR_RQT, CYU3P_EVENT_OR);
				break;

			case erREGRD:
				usbvendor_request.type = erREGRD;				
				usbvendor_request.mapper.data[0] = param[0];
				CyU3PEventSet (&glBulkLpEvent, APP_USB_EVT_VENDOR_RQT, CYU3P_EVENT_OR);				
				break;

			case ewCI2COPTION:
				usbvendor_request.address_mode=param[0]&FPGA_I2C_MODE_MASK;
				usbvendor_request.mapper.data[0]=param[2];
				usbvendor_request.type = ewCI2COPTION;
				CyU3PEventSet (&glBulkLpEvent, APP_USB_EVT_VENDOR_RQT, CYU3P_EVENT_OR);	
				//wLength = 4;				
				break;

			case ewCI2CW:
				status = CyU3PUsbGetEP0Data (wLength, glEp0Buffer, NULL);
				usbvendor_request.type = ewCI2CW;				
				switch(usbvendor_request.address_mode)
				{
					case FPGA_I2C_8A8D:
					usbvendor_request.mapper.regA8D8[0].deviceAddress = param[0];
					usbvendor_request.mapper.regA8D8[0].regAddress    = param[1];
					usbvendor_request.mapper.regA8D8[0].data          = glEp0Buffer[0];
					break;
					
					case FPGA_I2C_8A16D:
					usbvendor_request.mapper.regA8D16[0].deviceAddress = param[0];
					usbvendor_request.mapper.regA8D16[0].regAddress    = param[1];
					usbvendor_request.mapper.regA8D16[0].data.raw[1]   = glEp0Buffer[0];
					usbvendor_request.mapper.regA8D16[0].data.raw[0]   = glEp0Buffer[1];						
					break;
					
					case FPGA_I2C_16A8D:
					usbvendor_request.mapper.regA16D8[0].deviceAddress    = param[0];
					usbvendor_request.mapper.regA16D8[0].regAddress.raw[1]= param[1];
					usbvendor_request.mapper.regA16D8[0].regAddress.raw[0]= param[2];					
					usbvendor_request.mapper.regA16D8[0].data             = glEp0Buffer[0];						
					break;
					
					case FPGA_I2C_16A16D:
					usbvendor_request.mapper.regA16D16[0].deviceAddress    = param[0];
					usbvendor_request.mapper.regA16D16[0].regAddress.raw[1]= param[1];
					usbvendor_request.mapper.regA16D16[0].regAddress.raw[0]= param[2];					
					usbvendor_request.mapper.regA16D16[0].data.raw[1]      = glEp0Buffer[0];							
					usbvendor_request.mapper.regA16D16[0].data.raw[0]      = glEp0Buffer[1];
					break;					
				}
				
				CyU3PEventSet (&glBulkLpEvent, APP_USB_EVT_VENDOR_RQT, CYU3P_EVENT_OR);				
				break;

			case erCI2CR:
				usbvendor_request.type = erCI2CR;				
				switch(usbvendor_request.address_mode)
				{
					case FPGA_I2C_8A8D:
					usbvendor_request.mapper.regA8D8[0].deviceAddress = param[0];
					usbvendor_request.mapper.regA8D8[0].regAddress    = param[1];
					break;
					
					case FPGA_I2C_8A16D:
					usbvendor_request.mapper.regA8D16[0].deviceAddress = param[0];
					usbvendor_request.mapper.regA8D16[0].regAddress    = param[1];						
					break;
					
					case FPGA_I2C_16A8D:
					usbvendor_request.mapper.regA16D8[0].deviceAddress    = param[0];
					usbvendor_request.mapper.regA16D8[0].regAddress.raw[1]= param[1];
					usbvendor_request.mapper.regA16D8[0].regAddress.raw[0]= param[2];											
					break;
					
					case FPGA_I2C_16A16D:
					usbvendor_request.mapper.regA16D16[0].deviceAddress    = param[0];
					usbvendor_request.mapper.regA16D16[0].regAddress.raw[1]= param[1];
					usbvendor_request.mapper.regA16D16[0].regAddress.raw[0]= param[2];											
					break;					
				}				
				CyU3PEventSet (&glBulkLpEvent, APP_USB_EVT_VENDOR_RQT, CYU3P_EVENT_OR);	

				/*
				bytRet = i2c_ReadReg(I2C_TYPE_CCM, param[0], param[1], param[2], glEp0Buffer, wLength);

				m_retValue[0] = bytRet & 0xf;  //(J.J 2011.07.11) return value save
				m_retValue[1] = glEp0Buffer[wLength];

				if (bytRet == err_OK)  //(J.J 2013.01.02) Add
					status = CyU3PUsbSendEP0Data(wLength, glEp0Buffer);
				else
					status = bytRet;
				*/
				break;

			case erCI2CRB:	//(o)
				i2c_SelectType(I2C_TYPE_CCM);
				bytRet = i2c_ReadBytes(param[0], glEp0Buffer, wLength);

				m_retValue[0] = bytRet & 0xf;  //(J.J 2011.07.11) return value save
				if (bytRet == err_OK)
					status = CyU3PUsbSendEP0Data(wLength, glEp0Buffer);
				else
					status = bytRet;
				m_retValue[1] = glEp0Buffer[wLength];

				if (bytRet == err_OK)
					status = CyU3PUsbSendEP0Data(wLength, glEp0Buffer);
				else
					status = bytRet;
				break;

			//-----------------------------------------------------
			case erCustomI2C:
				i2c_SelectType(I2C_TYPE_CCM);

				switch(param[0])
				{
					case I2C_CUSTOM_START:
						/*bytRet = i2c_Start_sendbyte(param[1]);
						wLength = 0;
						if (bytRet == err_OK)
							status = CyU3PUsbSendEP0Data(wLength, glEp0Buffer);
						else
							status = bytRet;
						break;*/

						status  = CyU3PUsbGetEP0Data(wLength, glEp0Buffer, NULL);  //(J.J 2013.09.03) Read => Write
						if (status == CY_U3P_SUCCESS)
						{
							//bytRet = i2c_Start_sendbyte(param[1]);
							m_retValue[0] = bytRet;
							m_retValue[2] = status;
							status = bytRet;
						}
						break;
					case I2C_CUSTOM_WRITE:
						status  = CyU3PUsbGetEP0Data(wLength, glEp0Buffer, NULL);
						if (status == CY_U3P_SUCCESS)
						{
							//bytRet = i2c_SendBytes(glEp0Buffer, wLength);
							m_retValue[0] = bytRet;
							m_retValue[2] = status;
							status = bytRet;
						}
						break;
					case I2C_CUSTOM_READ:
						//bytRet = i2c_ReceiveBytes(glEp0Buffer, wLength);
						if (bytRet == err_OK)
						{
							status = CyU3PUsbSendEP0Data(wLength, glEp0Buffer);
							m_retValue[0] = bytRet;
							m_retValue[2] = status;
						}
						else
						{
							m_retValue[0] = bytRet;
							m_retValue[2] = status;
							status = bytRet;
						}

						break;
					case I2C_CUSTOM_STOP:
						/*bytRet = i2c_Stop();
						wLength = 0;
						if (bytRet == err_OK)
							status = CyU3PUsbSendEP0Data(wLength, glEp0Buffer);
						else
							status = bytRet;
						break;*/
						status  = CyU3PUsbGetEP0Data(wLength, glEp0Buffer, NULL);  //(J.J 2013.09.03) Read => Write
						if (status == CY_U3P_SUCCESS)
						{
							bytRet = i2c_Stop();
							m_retValue[0] = bytRet;
							m_retValue[2] = status;
							status = bytRet;
						}
						break;
					default:
							status = err_WRONG_PARAM;
						break;
				}
				break;
				//-----------------------------------------------------

			case ewClearTransfer:
				status = CyU3PUsbGetEP0Data(wLength, glEp0Buffer, NULL);
				//ClearTransfer();
				// Not supported this version
				break;

			case ewII2CW:
				status = CyU3PUsbGetEP0Data(wLength, glEp0Buffer, NULL);
				// Not supported this version
				break;

			case ewREGPAGEW:  //(o)
				usbvendor_request.type = ewREGPAGEW;
				usbvendor_request.long_cmd_len = wLength;
				status = CyU3PUsbGetEP0Data(wLength, glEp0Buffer, NULL);
				if (status == CY_U3P_SUCCESS)
					CyU3PEventSet (&glBulkLpEvent, APP_USB_EVT_VENDOR_RQT, CYU3P_EVENT_OR);
				break;

			case erEEPROM:
				param[0] = 0x03;
				i2cAddr = EEPROM_ADDR | ((param[0] & 0x07) << 1);
				CyU3PMemSet (glEp0Buffer, 0, sizeof (glEp0Buffer));
				//eepromAddr = (uint16_t)param[1]<<8+param[2];
				eepromAddr = 0xFF00;
				status = HAL_I2CReadEEPROM(i2cAddr,eepromAddr,wLength,glEp0Buffer);
				if (status == CY_U3P_SUCCESS)
				{
					status = CyU3PUsbSendEP0Data(wLength, glEp0Buffer);
				}
				break;

			case ewEEPROM://0x000000- 0x03FFFF    最后一页是3 FF00
				param[0] = 0x03;
				i2cAddr = EEPROM_ADDR | ((param[0] & 0x07) << 1);
				eepromAddr = 0xFF00;//(uint16_t)param[1]<<8+param[2];

				status = CyU3PUsbGetEP0Data(wLength, glEp0Buffer, NULL);
				if (status == CY_U3P_SUCCESS)
				{
					HAL_I2CWriteEEPROM(i2cAddr,eepromAddr,wLength,glEp0Buffer);
				}

				break;

			case ewDmmCh:
				status = CyU3PUsbGetEP0Data(wLength, glEp0Buffer, NULL);
				ProtectPower(CyFalse);

				if(param[0] == BIT_DMM_OFF)
				{
					// 1. Read current selected channel
					// 2. Enable all selected power channel
					// 3. Disable DMM => All power source is bypass
//					local_write(DEV_CTRL, eIO_DmmBypass, param[0]);
					DMM_BYPASS(param[0]);

					CyU3PThreadSleep (20); // Dealy 20mSec
					// Disable Dmm channel
//					local_write(DEV_CTRL, eIO_DmmSelect, param[0]);
					DMM_SELECT(param[0]);
					// Disable DMM & Disable PowerOff
				}
				else
				{
					// Select Dmm channel
//					local_write(DEV_CTRL, eIO_DmmSelect, param[0]);
					DMM_SELECT(param[0]);
					CyU3PThreadSleep (20); // Dealy 20mSec
					// Select (off/on) power source
//					local_write(DEV_CTRL, eIO_DmmBypass, param[0]);
					DMM_BYPASS(param[0]);
				}

				ProtectPower(CyTrue);
				break;

			case ewPLevel:	// set power level in milivolt step
				status = CyU3PUsbGetEP0Data(wLength, glEp0Buffer, NULL);
				// Not supported this version
				break;

			case ewFrameSize:	// Set frame size (wLength)
				status = CyU3PUsbGetEP0Data(wLength, glEp0Buffer, NULL);
				// Not supported this version
				break;

			// This version deos not support USB Capture
			case ewFrameRead:
				status = CyU3PUsbGetEP0Data(wLength, glEp0Buffer, NULL);
				// Not supported this version
				break;

			case ewDELAY:
				status = CyU3PUsbGetEP0Data(wLength, glEp0Buffer, NULL);
				// Not supported this version
				break;

			case ewInputMask:		// Input Mask
				status = CyU3PUsbGetEP0Data(wLength, glEp0Buffer, NULL);
				SelectInputBits(param[0]);
				break;

			case ewCMCLK:	
				tick = CyU3PGetTime();
				status = CyU3PUsbGetEP0Data(wLength, glEp0Buffer, NULL);
				CMCLK_f = param[0];
				CMCLK_f = (CMCLK_f<<8)+param[1];
				CMCLK_f = (CMCLK_f<<8)+param[2];
				uint8_t  oct = lookupOct((float)CMCLK_f);
				float factor = pow(2,10+oct);
				uint16_t dac = 2048-2.078*factor/(float)CMCLK_f;
				dac = dac&0x3FF; //10bits
				usbvendor_request.type = ewCMCLK;
				usbvendor_request.address_mode = FPGA_I2C_8A8D;
				usbvendor_request.mapper.regA8D8[0].deviceAddress = 0x2E;
				usbvendor_request.mapper.regA8D8[0].regAddress    = (oct<<4)|(dac>>6);				
				usbvendor_request.mapper.regA8D8[0].data          = (dac<<2)|0x02;
				CyU3PEventSet (&glBulkLpEvent, APP_USB_EVT_VENDOR_RQT, CYU3P_EVENT_OR);	
				break;

			case ewI2COPTION:
				usbvendor_request.address_mode=param[0]&FPGA_I2C_MODE_MASK;
				usbvendor_request.mapper.data[0]=param[2];
				usbvendor_request.type = ewI2COPTION;
				CyU3PEventSet (&glBulkLpEvent, APP_USB_EVT_VENDOR_RQT, CYU3P_EVENT_OR);	
				//wLength = 4;
				/*To be update when confirmed*/

				break;

			case erGETERRCODE:
				wLength = 1;
				glEp0Buffer[0] = GET_ERRORCODE();
				status = CyU3PUsbSendEP0Data(wLength, glEp0Buffer);
				break;

			case eiKEY:		// Read Key Code
				// Not supported this version
				break;

			case ewMV9315_BIN:
				// Not supported this version
				break;

			case ewEnableFrameMonitor:
				status = CyU3PUsbGetEP0Data(wLength, glEp0Buffer, NULL);
				Init_FrameMonitor();
				break;

			case erGetFrameCount:
				wLength = 4;
				GetFramePeriod(glEp0Buffer);
				status = CyU3PUsbSendEP0Data(wLength, glEp0Buffer);
				break;

			case ewRESETMCU:
				duration = param[0];
				duration = duration<<8 + param[1];
				reset_dps(duration);
				break;

			case ewDEBUGMCU: //0x31				
				set_pca6416a(BOOT_DPS,(CyBool_t)param[0]);
				break;	

			case erSENSEMCU:
				CyU3PGpioGetIOValues(&pin_0_31,&pin_32_60);
				glEp0Buffer[0] = 0;
				if(pin_0_31&0x4000000)
				{
					glEp0Buffer[0] = 1;
				}
				status = CyU3PUsbSendEP0Data(1, glEp0Buffer);
				break;
				
			case erMCUR:
				usbvendor_request.type = erMCUR;
				mcuRW_request.slave_addr = param[0];
				mcuRW_request.reg_addr   = param[1];
				mcuRW_request.length     = wLength;
				CyU3PEventSet (&glBulkLpEvent, APP_USB_EVT_VENDOR_RQT, CYU3P_EVENT_OR);
				break;
				
			case ewMCUW:
				if(wLength<=256)
				{
					status = CyU3PUsbGetEP0Data (wLength, glEp0Buffer, NULL);
					usbvendor_request.type = ewMCUW;
					mcuRW_request.slave_addr = param[0];
					mcuRW_request.reg_addr	 = param[1];
					mcuRW_request.length	 = wLength;
					
					memcpy(mcuRW_request.data,glEp0Buffer,wLength); 			
					CyU3PEventSet (&glBulkLpEvent, APP_USB_EVT_VENDOR_RQT, CYU3P_EVENT_OR); 
				}			
				break;


			case erGetRetCode:  //0x41  历厘等 I2C Write value return
				glEp0Buffer[0] = m_retValue[0];
				glEp0Buffer[1] = m_retValue[1];
				glEp0Buffer[2] = m_retValue[2];
				glEp0Buffer[3] = m_retValue[3];
				glEp0Buffer[4] = m_retValue[4];

				wLength = 5;
				status = CyU3PUsbSendEP0Data(wLength, glEp0Buffer);
				break;

//--------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------

//--------------------------------------------------------------------------------------------------

			case CY_FX_RQT_I2C_EEPROM_WRITE:
				i2cAddr = 0xA0 | ((wValue & 0x0007) << 1);
				status = CyU3PUsbGetEP0Data(wLength, glEp0Buffer, NULL);
				if (status == CY_U3P_SUCCESS)
				{
					//CyFxFlashProgI2cTransfer (wIndex, i2cAddr, wLength, glEp0Buffer, CyFalse);
				}
				break;

			case CY_FX_RQT_I2C_EEPROM_READ:
				i2cAddr = 0xA0 | ((wValue & 0x0007) << 1);
				CyU3PMemSet (glEp0Buffer, 0, sizeof (glEp0Buffer));
				//status = CyFxFlashProgI2cTransfer (wIndex, i2cAddr, wLength, glEp0Buffer, CyTrue);
				//if (status == CY_U3P_SUCCESS)
				{
					status = CyU3PUsbSendEP0Data(wLength, glEp0Buffer);
				}
				break;

			case CY_FX_RQT_SYS_MEM_READ:
				addr = (uint32_t *)((wIndex << 16) | wValue);
				offset = 0;
				if (wLength)
				{
					while (offset < (wLength / 4))
					{
						((uint32_t *)glEp0Buffer)[offset++] = *addr;
						addr++;
					}

					status = CyU3PUsbSendEP0Data (wLength, glEp0Buffer);
				}
				else
				{
					// Zero length read. Just ACK.
					CyU3PUsbAckSetup ();
				}
				break;

			default:
				// This is unknown request.
				isHandled = CyFalse;
				break;
			}
    }

    /* If there was any error, return not handled so that the library will
     * stall the request. Alternatively EP0 can be stalled here and return
     * CyTrue. */
    if (status != CY_U3P_SUCCESS)
    {
        isHandled = CyFalse;
    }

    return isHandled;
}

/* This is the callback function to handle the USB events. */
void
CyFxSlFifoApplnUSBEventCB (
    CyU3PUsbEventType_t evtype,
    uint16_t            evdata
    )
{
    switch (evtype)
    {
        case CY_U3P_USB_EVENT_SETCONF:
            /* Stop the application before re-starting. */
            if (glIsApplnActive)
            {
                CyFxSlFifoApplnStop ();
            }
            /* Start the loop back function. */
            CyFxSlFifoApplnStart ();
            break;

        case CY_U3P_USB_EVENT_RESET:
        case CY_U3P_USB_EVENT_DISCONNECT:
            /* Reset the I2C and SPI DMA channels. */
            CyU3PDmaChannelReset (&glI2cTxHandle);
            CyU3PDmaChannelReset (&glI2cRxHandle);
            /* Stop the loop back function. */
            if (glIsApplnActive)
            {
                CyFxSlFifoApplnStop ();
            }
            break;

        default:
            break;
    }
}

/* Callback function to handle LPM requests from the USB 3.0 host. This function is invoked by the API
   whenever a state change from U0 -> U1 or U0 -> U2 happens. If we return CyTrue from this function, the
   FX3 device is retained in the low power state. If we return CyFalse, the FX3 device immediately tries
   to trigger an exit back to U0.

   This application does not have any state in which we should not allow U1/U2 transitions; and therefore
   the function always return CyTrue.
 */
CyBool_t
CyFxApplnLPMRqtCB (
        CyU3PUsbLinkPowerMode link_mode)
{
    return CyTrue;
}

/* This function initializes the GPIF interface and initializes
 * the USB interface. */
void
CyFxSlFifoApplnInit (void)
{
    CyU3PPibClock_t pibClock;
    CyU3PReturnStatus_t apiRetStatus = CY_U3P_SUCCESS;

    /* Initialize the I2C interface for the EEPROM of page size 64 bytes. */
	
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
        CyFxAppErrorHandler(apiRetStatus);
    }

    /* Initialize the p-port block. */
    pibClock.clkDiv = 2;
    pibClock.clkSrc = CY_U3P_SYS_CLK;
    pibClock.isHalfDiv = CyFalse;
    /* Disable DLL for sync GPIF */
    pibClock.isDllEnable = CyFalse;
    apiRetStatus = CyU3PPibInit(CyTrue, &pibClock);
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
        CyFxAppErrorHandler(apiRetStatus);
    }

    /* Load the GPIF configuration for Slave FIFO sync mode. */
    apiRetStatus = CyU3PGpifLoad (&Sync_Slave_Fifo_2Bit_CyFxGpifConfig);
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
        CyFxAppErrorHandler(apiRetStatus);
    }

    /* Start the state machine. */
    apiRetStatus = CyU3PGpifSMStart (SYNC_SLAVE_FIFO_2BIT_RESET, SYNC_SLAVE_FIFO_2BIT_ALPHA_RESET);
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
        CyFxAppErrorHandler(apiRetStatus);
    }

    /* Start the USB functionality. */
    apiRetStatus = CyU3PUsbStart();
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
        CyFxAppErrorHandler(apiRetStatus);
    }

    /* The fast enumeration is the easiest way to setup a USB connection,
     * where all enumeration phase is handled by the library. Only the
     * class / vendor requests need to be handled by the application. */
    CyU3PUsbRegisterSetupCallback(CyFxSlFifoApplnUSBSetupCB, CyTrue);

    /* Setup the callback to handle the USB events. */
    CyU3PUsbRegisterEventCallback(CyFxSlFifoApplnUSBEventCB);

    /* Register a callback to handle LPM requests from the USB 3.0 host. */
//    CyU3PUsbRegisterLPMRequestCallback(CyFxApplnLPMRqtCB);

    /* Set the USB Enumeration descriptors */

    /* Super speed device descriptor. */
    apiRetStatus = CyU3PUsbSetDesc(CY_U3P_USB_SET_SS_DEVICE_DESCR, 0, (uint8_t *)CyFxUSB30DeviceDscr);
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
        CyFxAppErrorHandler(apiRetStatus);
    }

    /* High speed device descriptor. */
    apiRetStatus = CyU3PUsbSetDesc(CY_U3P_USB_SET_HS_DEVICE_DESCR, 0, (uint8_t *)CyFxUSB20DeviceDscr);
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
        CyFxAppErrorHandler(apiRetStatus);
    }

    /* BOS descriptor */
    apiRetStatus = CyU3PUsbSetDesc(CY_U3P_USB_SET_SS_BOS_DESCR, 0, (uint8_t *)CyFxUSBBOSDscr);
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
        CyFxAppErrorHandler(apiRetStatus);
    }

    /* Device qualifier descriptor */
    apiRetStatus = CyU3PUsbSetDesc(CY_U3P_USB_SET_DEVQUAL_DESCR, 0, (uint8_t *)CyFxUSBDeviceQualDscr);
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
        CyFxAppErrorHandler(apiRetStatus);
    }

    /* Super speed configuration descriptor */
    apiRetStatus = CyU3PUsbSetDesc(CY_U3P_USB_SET_SS_CONFIG_DESCR, 0, (uint8_t *)CyFxUSBSSConfigDscr);
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
        CyFxAppErrorHandler(apiRetStatus);
    }

    /* High speed configuration descriptor */
    apiRetStatus = CyU3PUsbSetDesc(CY_U3P_USB_SET_HS_CONFIG_DESCR, 0, (uint8_t *)CyFxUSBHSConfigDscr);
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
        CyFxAppErrorHandler(apiRetStatus);
    }

    /* Full speed configuration descriptor */
    apiRetStatus = CyU3PUsbSetDesc(CY_U3P_USB_SET_FS_CONFIG_DESCR, 0, (uint8_t *)CyFxUSBFSConfigDscr);
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
        CyFxAppErrorHandler(apiRetStatus);
    }

    /* String descriptor 0 */
    apiRetStatus = CyU3PUsbSetDesc(CY_U3P_USB_SET_STRING_DESCR, 0, (uint8_t *)CyFxUSBStringLangIDDscr);
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
        CyFxAppErrorHandler(apiRetStatus);
    }

    /* String descriptor 1 */
    apiRetStatus = CyU3PUsbSetDesc(CY_U3P_USB_SET_STRING_DESCR, 1, (uint8_t *)CyFxUSBManufactureDscr);
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
        CyFxAppErrorHandler(apiRetStatus);
    }

    /* String descriptor 2 */
    apiRetStatus = CyU3PUsbSetDesc(CY_U3P_USB_SET_STRING_DESCR, 2, (uint8_t *)CyFxUSBProductDscr);
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
        CyFxAppErrorHandler(apiRetStatus);
    }

    /* Connect the USB Pins with super speed operation enabled. */
    apiRetStatus = CyU3PConnectState(CyTrue, CyTrue);
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
        CyFxAppErrorHandler(apiRetStatus);
    }
}



void
CyFxGpioInit (void)
{
    CyU3PGpioClock_t gpioClock;
    CyU3PGpioSimpleConfig_t gpioConfig;
    CyU3PReturnStatus_t apiRetStatus = CY_U3P_SUCCESS;

    /* Init the GPIO module */
    gpioClock.fastClkDiv = 2;
    gpioClock.slowClkDiv = 0;
    gpioClock.simpleDiv = CY_U3P_GPIO_SIMPLE_DIV_BY_2;
    gpioClock.clkSrc = CY_U3P_SYS_CLK;
    gpioClock.halfDiv = 0;

    apiRetStatus = CyU3PGpioInit(&gpioClock, NULL);
    if (apiRetStatus != 0)
    {
        /* Error Handling */
        CyFxAppErrorHandler(apiRetStatus);
    }

    /* Override GPIO 21 as this pin is associated with GPIF Control signal.
     * The IO cannot be selected as GPIO by CyU3PDeviceConfigureIOMatrix call
     * as it is part of the GPIF IOs. Override API call must be made with
     * caution as this will change the functionality of the pin. If the IO
     * line is used as part of GPIF and is connected to some external device,
     * then the line will no longer behave as a GPIF IO.. Here CTL4 line is
     * not used and so it is safe to override.  */

    // FPGA PROM enable
    apiRetStatus = CyU3PDeviceGpioOverride (23, CyTrue);
    if (apiRetStatus != 0)
    {
        CyFxAppErrorHandler(apiRetStatus);
    }

    // O[60] (LED)
    apiRetStatus = CyU3PDeviceGpioOverride (60, CyTrue);
    if (apiRetStatus != 0)
    {
        CyFxAppErrorHandler(apiRetStatus);
    }

    // FXGPIO[0]-GPIO[50]
    apiRetStatus = CyU3PDeviceGpioOverride (50, CyTrue);
    if (apiRetStatus != 0)
    {
        CyFxAppErrorHandler(apiRetStatus);
    }
    // FXGPIO[1]-GPIO[51]
    apiRetStatus = CyU3PDeviceGpioOverride (51, CyTrue);
    if (apiRetStatus != 0)
    {
        CyFxAppErrorHandler(apiRetStatus);
    }
    // FXGPIO[2]-GPIO[52]
    apiRetStatus = CyU3PDeviceGpioOverride (52, CyTrue);
    if (apiRetStatus != 0)
    {
        CyFxAppErrorHandler(apiRetStatus);
    }
    // FXGPIO[3]-GPIO[53]
    apiRetStatus = CyU3PDeviceGpioOverride (53, CyTrue);
    if (apiRetStatus != 0)
    {
        CyFxAppErrorHandler(apiRetStatus);
    }
    // FXGPIO[4]-GPIO[54]
    apiRetStatus = CyU3PDeviceGpioOverride (54, CyTrue);
    if (apiRetStatus != 0)
    {
        CyFxAppErrorHandler(apiRetStatus);
    }
    // FXGPIO[5]-GPIO[57]
    apiRetStatus = CyU3PDeviceGpioOverride (57, CyTrue);
    if (apiRetStatus != 0)
    {
        CyFxAppErrorHandler(apiRetStatus);
    }
    // FXGPIO[6]-GPIO[27]
    apiRetStatus = CyU3PDeviceGpioOverride (27, CyTrue);
    if (apiRetStatus != 0)
    {
       CyFxAppErrorHandler(apiRetStatus);
    }
    // FXGPIO[7]-GPIO[26]
    apiRetStatus = CyU3PDeviceGpioOverride (26, CyTrue);
    if (apiRetStatus != 0)
    {
        CyFxAppErrorHandler(apiRetStatus);
    }
    // FXGPIO[8]-GPIO[45]
    apiRetStatus = CyU3PDeviceGpioOverride (45, CyTrue);
    if (apiRetStatus != 0)
    {
        CyFxAppErrorHandler(apiRetStatus);
    }
    // FXGPIO[9]-LBUS CS EN (LED)
    apiRetStatus = CyU3PDeviceGpioOverride (25, CyTrue);
    if (apiRetStatus != 0)
    {
        CyFxAppErrorHandler(apiRetStatus);
    }

    /* Configure output */
    //gpioConfig.outValue = CyTrue;
    gpioConfig.driveLowEn = CyTrue;
    gpioConfig.driveHighEn = CyTrue;
    gpioConfig.inputEn = CyFalse;
    gpioConfig.intrMode = CY_U3P_GPIO_NO_INTR;

    // FPGA PROM enable
    gpioConfig.outValue = CyTrue;
    apiRetStatus = CyU3PGpioSetSimpleConfig(23, &gpioConfig);
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
        CyFxAppErrorHandler(apiRetStatus);
    }

    // O[60] (LED)
    gpioConfig.outValue = CyFalse;
    apiRetStatus = CyU3PGpioSetSimpleConfig(60, &gpioConfig);
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
        CyFxAppErrorHandler(apiRetStatus);
    }


    // FXGPIO[0]-GPIO[50]
    gpioConfig.outValue = CyTrue;
    apiRetStatus = CyU3PGpioSetSimpleConfig(50, &gpioConfig);
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
        CyFxAppErrorHandler(apiRetStatus);
    }
    // FXGPIO[1]-GPIO[51]
    gpioConfig.outValue = CyFalse;
    apiRetStatus = CyU3PGpioSetSimpleConfig(51, &gpioConfig);
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
        CyFxAppErrorHandler(apiRetStatus);
    }
    // FXGPIO[2]-GPIO[52]
    gpioConfig.outValue = CyTrue;
    apiRetStatus = CyU3PGpioSetSimpleConfig(52, &gpioConfig);
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
        CyFxAppErrorHandler(apiRetStatus);
    }
    // FXGPIO[3]-GPIO[53]
    gpioConfig.outValue = CyFalse;
    apiRetStatus = CyU3PGpioSetSimpleConfig(53, &gpioConfig);
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
        CyFxAppErrorHandler(apiRetStatus);
    }
	
    gpioConfig.outValue = CyTrue;
    apiRetStatus = CyU3PGpioSetSimpleConfig(27, &gpioConfig);
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
        CyFxAppErrorHandler(apiRetStatus);
    }
	
    // FXGPIO[4]-GPIO[54]
    gpioConfig.outValue = CyFalse;
    apiRetStatus = CyU3PGpioSetSimpleConfig(54, &gpioConfig);
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
        CyFxAppErrorHandler(apiRetStatus);
    }

    /* Configure input */
    gpioConfig.outValue = CyTrue;
    gpioConfig.driveLowEn = CyFalse;
    gpioConfig.driveHighEn = CyFalse;
    gpioConfig.inputEn = CyTrue;
    gpioConfig.intrMode = CY_U3P_GPIO_NO_INTR;
    // FXGPIO[5]-GPIO[57]
    apiRetStatus = CyU3PGpioSetSimpleConfig(57, &gpioConfig);
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
        CyFxAppErrorHandler(apiRetStatus);
    }

    // FXGPIO[8]-GPIO[45]
    apiRetStatus = CyU3PGpioSetSimpleConfig(45, &gpioConfig);
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
        CyFxAppErrorHandler(apiRetStatus);
    }
    // FXGPIO[9]-LBUS CS EN (LED)
    apiRetStatus = CyU3PGpioSetSimpleConfig(25, &gpioConfig);
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
        CyFxAppErrorHandler(apiRetStatus);
    }
	
    // FXGPIO[0]-GPIO[50]
    //Atlas test GPIO26
    gpioConfig.outValue = CyFalse;
    gpioConfig.driveLowEn = CyFalse;
    gpioConfig.driveHighEn = CyFalse;
    gpioConfig.inputEn = CyTrue;
    gpioConfig.intrMode = CY_U3P_GPIO_NO_INTR;	
	apiRetStatus = CyU3PGpioSetSimpleConfig(26, &gpioConfig);
	if (apiRetStatus != CY_U3P_SUCCESS)
	{
		CyFxAppErrorHandler(apiRetStatus);
	}
}

static void PageWCmd_process(void) 
{
	int i,ret;
	int mult_wr_idx = 0;
	uint8_t  op_result;
	uint16_t delay;
	for(i=0;i<usbvendor_request.long_cmd_len;i+=8)
	{
		switch(glEp0Buffer[i])
		{
		case ewCI2COPTION:
		usbvendor_request.address_mode = glEp0Buffer[i+1]&FPGA_I2C_MODE_MASK;
		if( glEp0Buffer[i+3] == IIC_SPEED_50KBPS  || 
			glEp0Buffer[i+3] == IIC_SPEED_100KBPS || 
			glEp0Buffer[i+3] == IIC_SPEED_200KBPS ||
			glEp0Buffer[i+3] == IIC_SPEED_400KBPS ||
			glEp0Buffer[i+3] == IIC_SPEED_1000KBPS )
		{
			ret= init_setup_FPGAI2C(FPGA_IIC0,glEp0Buffer[i+3]);
		}			
		break;
		
		case ewCI2CW:
		switch(usbvendor_request.address_mode)
		{
			case FPGA_I2C_8A8D:
			usbvendor_request.mapper.regA8D8[mult_wr_idx].deviceAddress = glEp0Buffer[i+1];
			usbvendor_request.mapper.regA8D8[mult_wr_idx].regAddress	= glEp0Buffer[i+2];
			usbvendor_request.mapper.regA8D8[mult_wr_idx++].data        = glEp0Buffer[i+7];
			break;
			
			case FPGA_I2C_8A16D:
			usbvendor_request.mapper.regA8D16[mult_wr_idx].deviceAddress = glEp0Buffer[i+1];
			usbvendor_request.mapper.regA8D16[mult_wr_idx].regAddress    = glEp0Buffer[i+2];						
			usbvendor_request.mapper.regA8D16[mult_wr_idx].data.raw[1]   = glEp0Buffer[i+6];
			usbvendor_request.mapper.regA8D16[mult_wr_idx++].data.raw[0] = glEp0Buffer[i+7];
			break;
			
			case FPGA_I2C_16A8D:
			usbvendor_request.mapper.regA16D8[mult_wr_idx].deviceAddress	 = glEp0Buffer[i+1];
			usbvendor_request.mapper.regA16D8[mult_wr_idx].regAddress.raw[1] = glEp0Buffer[i+2];
			usbvendor_request.mapper.regA16D8[mult_wr_idx].regAddress.raw[0] = glEp0Buffer[i+3];	
			usbvendor_request.mapper.regA16D8[mult_wr_idx++].data            = glEp0Buffer[i+7];			
			break;
			
			case FPGA_I2C_16A16D:
			usbvendor_request.mapper.regA16D16[mult_wr_idx].deviceAddress    = glEp0Buffer[i+1];
			usbvendor_request.mapper.regA16D16[mult_wr_idx].regAddress.raw[1]= glEp0Buffer[i+2];
			usbvendor_request.mapper.regA16D16[mult_wr_idx].regAddress.raw[0]= glEp0Buffer[i+3];										
			usbvendor_request.mapper.regA16D16[mult_wr_idx].data.raw[1]      = glEp0Buffer[i+6];
			usbvendor_request.mapper.regA16D16[mult_wr_idx++].data.raw[0]    = glEp0Buffer[i+7];
			break;					
		}
		break;
		
		case ewDELAY:
		if(	mult_wr_idx>0 )
		{
			switch(usbvendor_request.address_mode)
			{
				case FPGA_I2C_8A8D: 						
				ret = random_A8D8_write_FPGAI2C(FPGA_IIC0,&usbvendor_request.mapper.regA8D8[0],mult_wr_idx,&op_result);
				break;
				
				case FPGA_I2C_8A16D:
				ret = random_A8D16_write_FPGAI2C(FPGA_IIC0,&usbvendor_request.mapper.regA8D16[0],mult_wr_idx,&op_result); 				
				break;
				
				case FPGA_I2C_16A8D:
				ret = random_A16D8_write_FPGAI2C(FPGA_IIC0,&usbvendor_request.mapper.regA16D8[0],mult_wr_idx,&op_result); 					
				break;
				
				case FPGA_I2C_16A16D:
				ret = random_A16D16_write_FPGAI2C(FPGA_IIC0,&usbvendor_request.mapper.regA16D16[0],mult_wr_idx,&op_result);							
				break;					
			}
			mult_wr_idx = 0;
			delay = glEp0Buffer[i+1];
			delay = (delay<<8)+glEp0Buffer[i+2];
			CyU3PThreadSleep(delay);
		}
		break;
		}		
	}

	if(	mult_wr_idx>0 )
	{
		switch(usbvendor_request.address_mode)
		{
			case FPGA_I2C_8A8D: 						
			ret = random_A8D8_write_FPGAI2C(FPGA_IIC0,&usbvendor_request.mapper.regA8D8[0],mult_wr_idx,&op_result);
			break;
			
			case FPGA_I2C_8A16D:
			ret = random_A8D16_write_FPGAI2C(FPGA_IIC0,&usbvendor_request.mapper.regA8D16[0],mult_wr_idx,&op_result); 				
			break;
			
			case FPGA_I2C_16A8D:
			ret = random_A16D8_write_FPGAI2C(FPGA_IIC0,&usbvendor_request.mapper.regA16D8[0],mult_wr_idx,&op_result); 					
			break;
			
			case FPGA_I2C_16A16D:
			ret = random_A16D16_write_FPGAI2C(FPGA_IIC0,&usbvendor_request.mapper.regA16D16[0],mult_wr_idx,&op_result);							
			break;					
		}
	}	
}

/* Entry function for the slFifoAppThread. */
void
SlFifoAppThread_Entry (
        uint32_t input)
{
    CyU3PReturnStatus_t status;
	uint8_t op_result;
    uint32_t eventMask = CYFX_USB_CTRL_TASK | CYFX_USB_HOSTWAKE_TASK | APP_USB_EVT_VENDOR_RQT;   /* Mask representing events that we are interested in. */
    uint32_t eventStat;                                                 /* Variable to hold current status of the events. */
	int      uart_status,i;
	CyBool_t iic1_inited=CyFalse;
	CyU3PReturnStatus_t apiRetStatus;

	i2c_Init();
	apiRetStatus = HAL_I2cInit(EEPROM_I2C_PAGE_SIZE);	
    /* Initialize GPIO module. */
    CyFxGpioInit();    

    Init_FrameMonitor();

	uint8_t my_id[10];	
	my_id[0]=0x82;
	I2CWrite(0xC0,0x10,1,my_id);
	
	my_id[0]=0x22;
	I2CWrite(0xC0,0x11,1,my_id);
	
	my_id[0]=0x08;
	I2CWrite(0xC0,0x12,1,my_id);
	
	my_id[0]=0xA4;
	I2CWrite(0xC0,0x20,1,my_id);
	
	my_id[0]=0xA8;
	I2CWrite(0xC0,0x24,1,my_id);
	
	my_id[0]=0xD0;
	I2CWrite(0xC0,0x26,1,my_id);
	
	my_id[0]=0xD0;
	I2CWrite(0xC0,0x29,1,my_id);
	apiRetStatus =I2CRead(0xC1,0x10,1,my_id);

	CyU3PGpioSimpleSetValue (27, CyFalse);
	CyU3PThreadSleep (200);
	CyU3PGpioSimpleSetValue (27, CyTrue);


	if(apiRetStatus==CY_U3P_SUCCESS)
		CyU3PThreadSleep (100);
	else
		CyU3PThreadSleep (10);

    /* Initialize the slave FIFO application */
    CyFxSlFifoApplnInit();

	init_pca6416a();

    for (;;)
    {
//        CyU3PUsbLPMDisable();
        /* The following call will block until at least one of the events enabled in eventMask is received.
           The eventStat variable will hold the events that were active at the time of returning from this API.
           The CLEAR flag means that all events will be atomically cleared before this function returns.
          
           We cause this event wait to time out every two seconds, so that we can periodically get the FX3
           device out of low power modes.
         */
        status = CyU3PEventGet (&glBulkLpEvent, eventMask, CYU3P_EVENT_OR_CLEAR, &eventStat, 10);

        if (status == CY_U3P_SUCCESS)
        {
        	if(eventStat&APP_USB_EVT_VENDOR_RQT)
        	{
        		switch(usbvendor_request.type)
        		{
					case erVER:
					glEp0Buffer[0] = UCI_MA6_FW_MIN_VERSION;
					glEp0Buffer[1] = UCI_MA6_FW_MAJ_VERSION;
					uart_status = reg_map_read(DEV_CTRL, &glEp0Buffer[2]);//FPGA
					glEp0Buffer[3] = UCI_MA6_PCB_VERSION;//PCB
					glEp0Buffer[4] = uart_status;
					status = CyU3PUsbSendEP0Data(5, glEp0Buffer);					
					break;

					case ewREGWR:
					uart_status = reg_map_write(usbvendor_request.mapper.data[0], usbvendor_request.mapper.data[1]);	
					break;

					case erREGRD:
					uart_status = reg_map_read(usbvendor_request.mapper.data[0], &glEp0Buffer[0]);
					glEp0Buffer[1] = uart_status;
					status = CyU3PUsbSendEP0Data(2, glEp0Buffer);						
					break;

					case ewI2COPTION:
					uart_status = init_setup_FPGAI2C(FPGA_IIC1,usbvendor_request.mapper.data[0]);
					glEp0Buffer[0] = uart_status;
					glEp0Buffer[1] = 1;
					glEp0Buffer[2] = 2;
					glEp0Buffer[3] = 3;
					status = CyU3PUsbSendEP0Data(4, glEp0Buffer);
					break;

					case ewCI2COPTION:
					uart_status = init_setup_FPGAI2C(FPGA_IIC0,usbvendor_request.mapper.data[0]);
					glEp0Buffer[0] = uart_status;
					glEp0Buffer[1] = 1;
					glEp0Buffer[2] = 2;
					glEp0Buffer[3] = 3;
					status = CyU3PUsbSendEP0Data(4, glEp0Buffer);						
					break;

					case ewCI2CW:
					switch(usbvendor_request.address_mode)
					{
						case FPGA_I2C_8A8D:							
    					uart_status = random_A8D8_write_FPGAI2C(FPGA_IIC0,&usbvendor_request.mapper.regA8D8[0],1,&op_result);
    					g_ErrCode = op_result;
						break;
						
						case FPGA_I2C_8A16D:
    					uart_status = random_A8D16_write_FPGAI2C(FPGA_IIC0,&usbvendor_request.mapper.regA8D16[0],1,&op_result);
    					g_ErrCode = op_result;
						break;
						
						case FPGA_I2C_16A8D:
    					uart_status = random_A16D8_write_FPGAI2C(FPGA_IIC0,&usbvendor_request.mapper.regA16D8[0],1,&op_result);
    					g_ErrCode = op_result;
						break;
						
						case FPGA_I2C_16A16D:
    					uart_status = random_A16D16_write_FPGAI2C(FPGA_IIC0,&usbvendor_request.mapper.regA16D16[0],1,&op_result);
    					g_ErrCode = op_result;
						break;					
					}
					break;

					case erCI2CR:
					switch(usbvendor_request.address_mode)
					{
						case FPGA_I2C_8A8D:							
    					uart_status = random_A8D8_read_FPGAI2C(FPGA_IIC0,&usbvendor_request.mapper.regA8D8[0],1,&op_result);
    					g_ErrCode = op_result;
						if(FPGA_ACTION_OK==uart_status) {
							glEp0Buffer[0] = usbvendor_request.mapper.regA8D8[0].data;
							CyU3PUsbSendEP0Data(1, glEp0Buffer);
						}
						break;
						
						case FPGA_I2C_8A16D:
    					uart_status = random_A8D16_read_FPGAI2C(FPGA_IIC0,&usbvendor_request.mapper.regA8D16[0],1,&op_result);
    					g_ErrCode = op_result;
						if(FPGA_ACTION_OK==uart_status) {
							glEp0Buffer[0] = usbvendor_request.mapper.regA8D16[0].data.raw[1];
							glEp0Buffer[1] = usbvendor_request.mapper.regA8D16[0].data.raw[0];
							CyU3PUsbSendEP0Data(2, glEp0Buffer);
						}							
						break;
						
						case FPGA_I2C_16A8D:
    					uart_status = random_A16D8_read_FPGAI2C(FPGA_IIC0,&usbvendor_request.mapper.regA16D8[0],1,&op_result);
    					g_ErrCode = op_result;
						if(FPGA_ACTION_OK==uart_status) {
							glEp0Buffer[0] = usbvendor_request.mapper.regA16D8[0].data;
							CyU3PUsbSendEP0Data(2, glEp0Buffer);
						}						
						break;
						
						case FPGA_I2C_16A16D:
    					uart_status = random_A16D16_read_FPGAI2C(FPGA_IIC0,&usbvendor_request.mapper.regA16D16[0],1,&op_result);
    					g_ErrCode = op_result;
						if(FPGA_ACTION_OK==uart_status) {
							glEp0Buffer[0] = usbvendor_request.mapper.regA8D16[0].data.raw[1];
							glEp0Buffer[1] = usbvendor_request.mapper.regA8D16[0].data.raw[0];
							CyU3PUsbSendEP0Data(2, glEp0Buffer);
						}						
						break;					
					}						
					break;

					case ewCMCLK:					
					if(!iic1_inited) {
						uart_status = init_setup_FPGAI2C(FPGA_IIC1,0x22);
						iic1_inited =CyTrue;
					}
					tick = CyU3PGetTime();
					uart_status = random_A8D8_write_FPGAI2C(FPGA_IIC1,&usbvendor_request.mapper.regA8D8[0],1,&op_result);
					uart_status = 1;
					cur_tick = CyU3PGetTime();
					tick ++;
					break;

					case ewREGPAGEW:
					PageWCmd_process();
					break;

					case erMCUR:
					status = HAL_I2CRead( mcuRW_request.slave_addr,
										  mcuRW_request.reg_addr,
										  mcuRW_request.length,
										  glEp0Buffer);
					if(status==CY_U3P_SUCCESS)
					{
						CyU3PUsbSendEP0Data(mcuRW_request.length, glEp0Buffer);
					}
					break;

					case ewMCUW:
					status = HAL_I2CWrite( mcuRW_request.slave_addr,
										  mcuRW_request.reg_addr,
										  mcuRW_request.length,
										  mcuRW_request.data);						
					break;
				}
        	}



//            /* If the HOSTWAKE task is set, send a DEV_NOTIFICATION (FUNCTION_WAKE) or remote wakeup signalling
//               based on the USB connection speed. */
//            if (eventStat & CYFX_USB_HOSTWAKE_TASK)
//            {
//                CyU3PThreadSleep (1000);
//                if (CyU3PUsbGetSpeed () == CY_U3P_SUPER_SPEED)
//                    stat = CyU3PUsbSendDevNotification (1, 0, 0);
//                else
//                    stat = CyU3PUsbDoRemoteWakeup ();
//
//            }
//
//            /* If there is a pending control request, handle it here. */
//            if (eventStat & CYFX_USB_CTRL_TASK)
//            {
//                uint8_t  bRequest, bReqType;
//                uint16_t wLength;//, temp;
//                uint16_t wValue, wIndex;
//
//                /* Decode the fields from the setup request. */
//                bReqType = (gl_setupdat0 & CY_U3P_USB_REQUEST_TYPE_MASK);
//                bRequest = ((gl_setupdat0 & CY_U3P_USB_REQUEST_MASK) >> CY_U3P_USB_REQUEST_POS);
//                wLength  = ((gl_setupdat1 & CY_U3P_USB_LENGTH_MASK)  >> CY_U3P_USB_LENGTH_POS);
//                wValue   = ((gl_setupdat0 & CY_U3P_USB_VALUE_MASK) >> CY_U3P_USB_VALUE_POS);
//                wIndex   = ((gl_setupdat1 & CY_U3P_USB_INDEX_MASK) >> CY_U3P_USB_INDEX_POS);
//
//                if ((bReqType & CY_U3P_USB_TYPE_MASK) == CY_U3P_USB_VENDOR_RQT)
//                {
//                    switch (bRequest)
//                    {
//                    case 0x77:      /* Trigger remote wakeup. */
//                        CyU3PUsbAckSetup ();
//                        CyU3PEventSet (&glBulkLpEvent, CYFX_USB_HOSTWAKE_TASK, CYU3P_EVENT_OR);
//                        break;
//
//                    case 0x84:
//                        {
//                            uint8_t major, minor, patch;
//
//                            if (CyU3PUsbGetBooterVersion (&major, &minor, &patch) == CY_U3P_SUCCESS)
//                            {
//                                glEp0Buffer[0] = major;
//                                glEp0Buffer[1] = minor;
//                                glEp0Buffer[2] = patch;
//                                CyU3PUsbSendEP0Data (3, glEp0Buffer);
//                            }
//                            else
//                                CyU3PUsbStall (0, CyTrue, CyFalse);
//                        }
//                        break;
//
//                    case 0x90:
//                        /* Request to switch control back to the boot firmware. */
//
//                        /* Complete the control request. */
//                        CyU3PUsbAckSetup ();
//                        CyU3PThreadSleep (10);
//
//                        /* Get rid of the DMA channels and EP configuration. */
//                        CyFxSlFifoApplnStop ();
//
//                        /* De-initialize the Debug and UART modules. */
//
//                        /* Now jump back to the boot firmware image. */
//                        CyU3PUsbSetBooterSwitch (CyTrue);
//                        CyU3PUsbJumpBackToBooter (0x40078000);
//                        while (1)
//                            CyU3PThreadSleep (100);
//                        break;
//
//                    default:
//                        // This is unknown request.
//                        CyU3PUsbStall (0, CyTrue, CyFalse);
//                        break;
//                    }
//                }
//                else
//                {
//                    // Only vendor requests are to be handled here.
//                    CyU3PUsbStall (0, CyTrue, CyFalse);
//                }
//            }
        }
    }
}

/* Application define function which creates the threads. */
void
CyFxApplicationDefine (
        void)
{
    void *ptr = NULL;
    uint32_t ret = CY_U3P_SUCCESS;

    /* Create an event flag group that will be used for signalling the application thread. */
    ret = CyU3PEventCreate (&glBulkLpEvent);
    if (ret != 0)
    {
        /* Loop indefinitely */
        while (1);
    }
    
    /* Allocate the memory for the thread */

    ptr  = CyU3PMemAlloc (CY_FX_SLFIFO_THREAD_STACK);

    /* Create the thread for the application */
    ret |= CyU3PThreadCreate (&slFifoAppThread,           /* Slave FIFO app thread structure */
                          "21:Slave_FIFO_sync",                    /* Thread ID and thread name */
                          SlFifoAppThread_Entry,                   /* Slave FIFO app thread entry function */
                          0,                                       /* No input parameter to thread */
                          ptr,                                     /* Pointer to the allocated thread stack */
                          CY_FX_SLFIFO_THREAD_STACK,               /* App Thread stack size */
                          CY_FX_SLFIFO_THREAD_PRIORITY,            /* App Thread priority */
                          CY_FX_SLFIFO_THREAD_PRIORITY,            /* App Thread pre-emption threshold */
                          CYU3P_NO_TIME_SLICE,                     /* No time slice for the application thread */
                          CYU3P_AUTO_START                         /* Start the thread immediately */
                          );

#if 1
    ptr  = CyU3PMemAlloc (UART_THREAD_STACK);
    ret |= CyU3PThreadCreate (&UartThread,                         /* UART thread structure */
                          "22:uart_thread_interpreter",            /* Thread ID and thread name */
                          UartThread_Entry,                        /* Slave FIFO app thread entry function */
                          0,                                       /* No input parameter to thread */
                          ptr,                                     /* Pointer to the allocated thread stack */
                          UART_THREAD_STACK,                       /* UART Thread stack size */
                          UART_THREAD_PRIORITY,                    /* UART Thread priority   */
                          UART_THREAD_PRIORITY,                    /* UART Thread pre-emption threshold */
                          CYU3P_NO_TIME_SLICE,                     /* No time slice for the application thread */
                          CYU3P_AUTO_START                         /* Start the thread immediately */
                          );
	#endif

    /* Check the return code */
    if (ret != 0)
    {
        /* Thread Creation failed with the error code retThrdCreate */

        /* Add custom recovery or debug actions here */

        /* Application cannot continue */
        /* Loop indefinitely */
        while(1);
    }
}

/*
 * Main function
 */
int
main (void)
{
    CyU3PIoMatrixConfig_t io_cfg;
    CyU3PReturnStatus_t status = CY_U3P_SUCCESS;

    /* Initialize the device */
    CyU3PSysClockConfig_t clockConfig;
    clockConfig.setSysClk400  = CyTrue;
    clockConfig.cpuClkDiv     = 2;
    clockConfig.dmaClkDiv     = 2;
    clockConfig.mmioClkDiv    = 2;
    clockConfig.useStandbyClk = CyFalse;
    clockConfig.clkSrc         = CY_U3P_SYS_CLK;
    status = CyU3PDeviceInit (&clockConfig);
    if (status != CY_U3P_SUCCESS)
    {
        goto handle_fatal_error;
    }

    /* Initialize the caches. Enable both Instruction and Data caches. */
    status = CyU3PDeviceCacheControl (CyTrue, CyTrue, CyTrue);
    if (status != CY_U3P_SUCCESS)
    {
        goto handle_fatal_error;
    }

    /* Configure the IO matrix for the device. On the FX3 DVK board, the COM port 
     * is connected to the IO(53:56). This means that either DQ32 mode should be
     * selected or lppMode should be set to UART_ONLY. Here we are choosing
     * UART_ONLY configuration for 16 bit slave FIFO configuration and setting
     * isDQ32Bit for 32-bit slave FIFO configuration. */
    /*io_cfg.s0Mode = CY_U3P_SPORT_INACTIVE;
    io_cfg.s1Mode = CY_U3P_SPORT_INACTIVE;*/
    io_cfg.useUart   = CyTrue;
    io_cfg.useI2C    = CyTrue;
    io_cfg.useI2S    = CyFalse;
    io_cfg.useSpi    = CyFalse;
#if (CY_FX_SLFIFO_GPIF_16_32BIT_CONF_SELECT == 0)
    io_cfg.isDQ32Bit = CyFalse;
    io_cfg.lppMode   = CY_U3P_IO_MATRIX_LPP_UART_ONLY;
#else
    io_cfg.isDQ32Bit = CyTrue;
    io_cfg.lppMode   = CY_U3P_IO_MATRIX_LPP_DEFAULT;
#endif
    /* GPIOs are enabled. */
    io_cfg.gpioSimpleEn[0]  = 0x0e000000;
    io_cfg.gpioSimpleEn[1]  = 0x02002000;
    io_cfg.gpioComplexEn[0] = 0;
    io_cfg.gpioComplexEn[1] = 0;
    status = CyU3PDeviceConfigureIOMatrix (&io_cfg);
    if (status != CY_U3P_SUCCESS)
    {
        goto handle_fatal_error;
    }

    /* This is a non returnable call for initializing the RTOS kernel */
    CyU3PKernelEntry ();

    /* Dummy return to make the compiler happy */
    return 0;

handle_fatal_error:

    /* Cannot recover from this error. */
    while (1);
}

/* [ ] */

