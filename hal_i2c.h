#ifndef _HAL_I2C_H
#define _HAL_I2C_H

#define EEPROM_ADDR  0xA0

/* Give a timeout value of 5s for any flash programming. */
#define EEPROM_PROG_TIMEOUT                (5000)
/* I2C Data rate */
#define EEPROM_I2C_BITRATE        (100000)


extern CyU3PDmaChannel glI2cTxHandle;   /* I2C Tx channel handle */
extern CyU3PDmaChannel glI2cRxHandle;   /* I2C Rx channel handle */


/* The following constant is defined based on the page size that the I2C
 * device support. 24LC256 support 64 byte page write access. */
#define EEPROM_I2C_PAGE_SIZE        (256)

CyU3PReturnStatus_t HAL_I2cInit (uint16_t pageLen);
CyU3PReturnStatus_t HAL_I2cTransfer ( uint16_t  byteAddress,
									  uint8_t   devAddr,
									  uint16_t  byteCount,
									  uint8_t  *buffer,
									  CyBool_t  isRead );
CyU3PReturnStatus_t HAL_I2CWrite(uint8_t slaveAddr,uint8_t Addr,uint8_t count,uint8_t *buf);
CyU3PReturnStatus_t HAL_I2CRead(uint8_t slaveAddr,uint8_t Addr,uint8_t count,uint8_t *buf);
CyU3PReturnStatus_t HAL_I2CWriteEEPROM (uint8_t slaveAddr,uint16_t Addr,uint32_t count,uint8_t *buf);
CyU3PReturnStatus_t HAL_I2CReadEEPROM(uint8_t slaveAddr,uint16_t Addr,uint32_t count,uint8_t *buf);

#endif
