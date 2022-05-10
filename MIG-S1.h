/*
 ## Cypress USB 3.0 Platform header file (cyfxslfifosync.h)
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

/* This file contains the constants and definitions used by the Slave FIFO application example */

#ifndef _INCLUDED_CYFXSLFIFOASYNC_H_
#define _INCLUDED_CYFXSLFIFOASYNC_H_

#include "cyu3externcstart.h"
#include "cyu3types.h"
#include "cyu3usbconst.h"

#define UCI_MA6_FW_MAJ_VERSION (0x00)
//#define UCI_MA6_FW_MIN_VERSION (0x02)  //2012.11.13 0x02 Update
//#define UCI_MA6_FW_MIN_VERSION (0x04)  //2012.11.15 disable LPM  //2013.01.02 0x04 Update
//#define UCI_MA6_FW_MIN_VERSION (0x05)  //2013.03.21 SDK V1.2.3
//#define UCI_MA6_FW_MIN_VERSION (0x06)  //2013.03.27 ASMI
//#define UCI_MA6_FW_MIN_VERSION (0x07)  //2013.08.13 MD4L W/R Add & GPIO Write
//#define UCI_MA6_FW_MIN_VERSION (0x08)  //2013.09.09 CustomI2C Start/Stop Read => Write
//#define UCI_MA6_FW_MIN_VERSION   (0x09)  //2013.10.25 I2C Write length uint8_t => uint16_t
                                         //2014.05.13 modify MD4L command, SDK V1.3.1, add delay fuction for PageWrite command
//#define UCI_MA6_FW_MIN_VERSION   (0x0A)  //2014.07.22 MD4DSI FPGA Update용 Cmd 추가
//#define UCI_MA6_FW_MIN_VERSION   (0x0B)  //2014.12.23 I2C ack/nack 강제로 uncheck 하는 버전 (디버깅 테스트 버전)
//#define UCI_MA6_FW_MIN_VERSION   (0x0C)  //2014.12.23 I2C ack/nack uncheck 옵션 오류 수정 (I2C start 등)
//#define UCI_MA6_FW_MIN_VERSION   (0x0D)  //2015.02.06 AutoSkew function
//#define UCI_MA6_FW_MIN_VERSION   (0x0E)  //2015.04.07 ber_en 동기화
//#define UCI_MA6_FW_MIN_VERSION   (0x0F)  //2015.04.14 MAX table에서 가운데 값 추출 추가
#define UCI_MA6_FW_MIN_VERSION   (0x11)  //2015.11.11 AutoSettletime 오프셋 보정
#define UCI_MA6_PCB_VERSION      (0x00) 


/* 16/32 bit GPIF Configuration select */
/* Set CY_FX_SLFIFO_GPIF_16_32BIT_CONF_SELECT = 0 for 16 bit GPIF data bus.
 * Set CY_FX_SLFIFO_GPIF_16_32BIT_CONF_SELECT = 1 for 32 bit GPIF data bus.
 */
#define CY_FX_SLFIFO_GPIF_16_32BIT_CONF_SELECT (1)

//#define CY_FX_SLFIFO_DMA_BUF_COUNT      (2)                       /* Slave FIFO channel buffer count */
//#define CY_FX_SLFIFO_DMA_TX_SIZE	    (0)	                  /* DMA transfer size is set to infinite */
//#define CY_FX_SLFIFO_THREAD_STACK       (0x0400)                  /* Slave FIFO application thread stack size */
//#define CY_FX_SLFIFO_THREAD_PRIORITY    (8)                       /* Slave FIFO application thread priority */

#define CY_FX_SLFIFO_DMA_BUF_COUNT      (2)                       /* Slave FIFO channel buffer count */
#define CY_FX_SLFIFO_DMA_TX_SIZE	    (0)	                  /* DMA transfer size is set to infinite */
#define CY_FX_SLFIFO_THREAD_STACK       (0x4000)                  /* Slave FIFO application thread stack size */
#define CY_FX_SLFIFO_THREAD_PRIORITY    (8)                       /* Slave FIFO application thread priority */


/* Endpoint and socket definitions for the Slave FIFO application */

/* To change the Producer and Consumer EP enter the appropriate EP numbers for the #defines.
 * In the case of IN endpoints enter EP number along with the direction bit.
 * For eg. EP 6 IN endpoint is 0x86
 *     and EP 6 OUT endpoint is 0x06.
 * To change sockets mention the appropriate socket number in the #defines. */

/* Note: For USB 2.0 the endpoints and corresponding sockets are one-to-one mapped
         i.e. EP 1 is mapped to UIB socket 1 and EP 2 to socket 2 so on */

#define CY_FX_EP_PRODUCER               0x01    /* EP 1 OUT */
#define CY_FX_EP_CONSUMER               0x81    /* EP 1 IN */

#define CY_FX_PRODUCER_USB_SOCKET    CY_U3P_UIB_SOCKET_PROD_1    /* USB Socket 1 is producer */
#define CY_FX_CONSUMER_USB_SOCKET    CY_U3P_UIB_SOCKET_CONS_1    /* USB Socket 1 is consumer */


/* Used with FX3 Silicon. */
#define CY_FX_PRODUCER_PPORT_SOCKET    CY_U3P_PIB_SOCKET_0    /* P-port Socket 0 is producer */
#define CY_FX_CONSUMER_PPORT_SOCKET    CY_U3P_PIB_SOCKET_3    /* P-port Socket 3 is consumer */



#define CY_FX_VREQ_RD        (1 << 0)                                  /* rd event flag */
#define CY_FX_VREQ_WR        (1 << 1)                                 /* wr event flag */
#define CY_FX_USB_VND_SET_REQ_TYPE    (uint8_t)(0x40) /* Download */
#define CY_FX_USB_VND_GET_REQ_TYPE  (uint8_t)(0xC0) /* Upload */





/* Burst mode definitions: Only for super speed operation. The maximum burst mode
 * supported is limited by the USB hosts available. The maximum value for this is 16
 * and the minimum (no-burst) is 1. */

#define CY_FX_EP_BURST_LENGTH          (16)     /* Super speed burst length in packets. */
//#define CY_FX_EP_BURST_LENGTH          (1)     /* Super speed burst length in packets. */

/* This application uses EEPROM as the slave I2C device. The I2C EEPROM
 * part number used is 24LC256. The capacity of the EEPROM is 256K bits */
#define CY_FX_USBI2C_I2C_MAX_CAPACITY   (32 * 1024) /* Capacity in bytes */







/* USB vendor requests supported by the application. */

/* USB vendor request to read the 8 byte firmware ID. This will return content
 * of glFirmwareID array. */
#define CY_FX_RQT_ID_CHECK                      (0xB0)

/* USB vendor request to write to I2C EEPROM connected. The EEPROM page size is
 * fixed to 64 bytes. The I2C EEPROM address is provided in the value field. The
 * memory address to start writing is provided in the index field of the request.
 * The maximum allowed request length is 4KB. */
#define CY_FX_RQT_I2C_EEPROM_WRITE              (0xBA)

/* USB vendor request to read from I2C EEPROM connected. The EEPROM page size is
 * fixed to 64 bytes. The I2C EEPROM address is provided in the value field. The
 * memory address to start reading from is provided in the index field of the
 * request. The maximum allowed request length is 4KB. */
#define CY_FX_RQT_I2C_EEPROM_READ               (0xBB)

/* USB vendor request to read data from SYS_MEM starting at 32-bit aligned given
 * address. The MS 16-bit of start address is provided in the index field and the
 * LS 16-bit is provided in the value field of the request. */
#define CY_FX_RQT_SYS_MEM_READ                  (0xC0)


#define DEV_CTRL	0x00		// Main controller
#define DEV_DPS		0x02		// Main controller

#define DEV_OST0	0x04
#define DEV_OST1	0x05
#define DEV_CCP2	0x05

// Communication Command Index
typedef enum {
	erVER = 0x10,			//0x10
	erREGRD,				//0x11
	ewREGWR,				//0x12	
	ewEXTW,					//0x13
	ewCI2COPTION,			//0x14	
	erCI2CR,				//0x15
	ewCI2CW,				//0x16
	ewII2CW,				//0x17
	ewREGPAGEW,				//0x18
	erEEPROM,				//0x19
	ewEEPROM,				//0x1A
	ewDmmCh,				//0x1B
	ewMFreq,				//0x1C
	ewPLevel,				//0x1D
	ewPolEn,				//0x1E
	ewFrameSize,			//0x1F
	ewFrameRead,			//0x20
	ewInputMask,			//0x21
	ewCMCLK,				//0x22
	ewDELAY,		     	//0x23
	ewMV9315_BIN,			//0x24
	ewI2COPTION,			//0x25
	erGETERRCODE,			//0x26
	erCustomI2C,			//0x27
	ewLedLevel,				//0x28
	ewClearTransfer,		//0x29
	eiKEY,					//0x2A
	ewPI2CW,				//0x2B
	erPI2CR,				//0x2C
	erCI2CRB	,			//0x2D
	//	F/W 0x040B
	ewEnableFrameMonitor,	//0x2E
	erGetFrameCount,		//0x2F
	//	F/W 0x040F
	ewRESETMCU,				//0x30
	ewDEBUGMCU,	    		//0x31
	erSENSEMCU,	            //0x32
	erMCUR,                 //0x33
	ewMCUW,	                //0x34
	//erDPS_ADCReadDataValue,//0x35
	//ewDI2CW,				//0x36
	//erDI2CR,				//0x37
	//ewEPCSwr,				//0x38
	//erEPCSrd,				//0x39
/////////////////////////////////////////////////////////////////////////////////////
//(J.J 2011.06.10) Add
	//ewREGWC,				//0x3A
	//ePowerMon,			//0x3B
//(J.J 2011.06.14) Add
	//EchoTest,				//0x3C  UsbTest => EchoTest 로 변경

/////////////////////////////////////////////////////////////////////////////////////
// HyBus Version Use
	//ewPowerCon,           //0x3D
	//erPowerRead,          //0x3E
	//ewMemWrite,           //0x3F
	//ewPageWrite,          //0x40
/////////////////////////////////////////////////////////////////////////////////////
	erGetRetCode = 0x41,	//0x41  i2c Write Return value save
/////////////////////////////////////////////////////////////////////////////////////
	//ewMd4lWrite,			//0x42
	//erMd4lRead,			//0x43
	//ewWriteWord,   		//(J.J 2011.10.17) 0x44
	//erReadWord,   		//(J.J 2011.10.17) 0x45

	//ewMd4dsiWrite,		//0x46
	//erMd4dsiRead,			//0x47
	//DebugTest,			//0x48
	//erMD4L_AutoSkew,   	//0x49
	//erMD4L_SearchSettleTime,   //0x4A
	ecmdUnknown
}typUciCmdIndex;

#define NUM_COMMAND_INDEX	(ecmdUnknown+erVER+1)

// for erCustomI2C : Custom Command
#define I2C_CUSTOM_START	0
#define I2C_CUSTOM_WRITE	1
#define I2C_CUSTOM_READ		2
#define I2C_CUSTOM_STOP		3


#define BIT_DMM_OFF			0x00

// AutoSkew & AutoSettletime
#define Settletime_OFFSET	(4 * 8)

#define Settletime_safe_OFFSET	(8 - 1) // 2015.11.9

void
CyFxAppErrorHandler (
        CyU3PReturnStatus_t apiRetStatus    /* API return status */
        );

/* Extern definitions for the USB Descriptors */
extern const uint8_t CyFxUSB20DeviceDscr[];
extern const uint8_t CyFxUSB30DeviceDscr[];
extern const uint8_t CyFxUSBDeviceQualDscr[];
extern const uint8_t CyFxUSBFSConfigDscr[];
extern const uint8_t CyFxUSBHSConfigDscr[];
extern const uint8_t CyFxUSBBOSDscr[];
extern const uint8_t CyFxUSBSSConfigDscr[];
extern const uint8_t CyFxUSBStringLangIDDscr[];
extern const uint8_t CyFxUSBManufactureDscr[];
extern const uint8_t CyFxUSBProductDscr[];
extern const uint8_t CyFxUsbOSDscr[];

#include "cyu3externcend.h"

#endif /* _INCLUDED_CYFXSLFIFOASYNC_H_ */

/*[]*/
