/*
 * i2c_fpga.h
 *
 *  Created on: 2012. 3. 8.
 *      Author: soyim
 */

#ifndef I2C_FPGA_H_
#define I2C_FPGA_H_


#define I2C_TYPE_CCM							0
#define I2C_TYPE_EXP							1
#define I2C_TYPE_ZILKER							2

#define MAX_I2C_TYPE							3


#define I2C_BASE_CCM							0x40
#define I2C_BASE_EXP							0x30

#define DEV_CTRL								0x00		// Main controller
#define DEV_DPS									0x02		// Main controller

#define DEV_OST0								0x04
#define DEV_OST1								0x05
#define DEV_CCP2								0x05

// i2c option command
#define BIT_I2C_CHECKACK						0x01		// Check Acknowledge (normal true)
#define BIT_I2C_REPSTART						0x02		// Repeated Start Mode (normal true)
#define BIT_I2C_WORDINDEX						0x04		// 16bit Register Index
#define BIT_I2C_WORDVALUE						0x08		// if '1', Select option for exPI2C, else select option for I2C(CCM)

#define BIT_DATA_16BIT_MASK                     0x08
#define BIT_ADDR_16BIT_MASK                     0x04
#define FPGA_I2C_MODE_MASK                      (BIT_DATA_16BIT_MASK|BIT_ADDR_16BIT_MASK)

#define FPGA_I2C_8A8D                           0x00
#define FPGA_I2C_16A8D                          0x04
#define FPGA_I2C_8A16D                          0x08
#define FPGA_I2C_16A16D                         0x0C


#define DEF_I2C_OPTION							(BIT_I2C_CHECKACK | BIT_I2C_REPSTART)

#define DPS_ADR_MISC_VERSION					0x00
#define DPS_ADR_MISC_CHIP_ID					0x01
#define DPS_ADR_MISC_CONTROL					0x02
#define DPS_ADR_MISC_PWR_CHK					0x03

#define DPS_ADR_DPS								0x10
#define DPS_ADR_DAC								0x20
#define DPS_ADR_ADC								0x30

#define DPS_IDX_WRITE_START						0x00		//	#define DPS_ADR_DPS_WRITE_START		0x10	//	#define DPS_ADR_DAC_WRITE_START		0x20
#define DPS_IDX_WRITE_LO						0x01		//	#define DPS_ADR_DPS_WRITE_LO		0x11	//	#define DPS_ADR_DAC_WRITE_LO		0x21
#define DPS_IDX_WRITE_MID						0x02		//	#define DPS_ADR_DPS_WRITE_MID		0x12    //  #define DPS_ADR_DAC_WRITE_MID		0x22
#define DPS_IDX_WRITE_HI						0x03		//	#define DPS_ADR_DPS_WRITE_HI		0x13    //  #define DPS_ADR_DAC_WRITE_HI		0x23
#define DPS_IDX_READ_LO							0x04		//	#define DPS_ADR_DPS_READ_LO			0x14    //  #define DPS_ADR_DAC_READ_LO			0x24
#define DPS_IDX_READ_MID						0x05		//	#define DPS_ADR_DPS_READ_MID		0x15    //  #define DPS_ADR_DAC_READ_MID		0x25
#define DPS_IDX_READ_HI							0x06		//	#define DPS_ADR_DPS_READ_HI			0x16    //  #define DPS_ADR_DAC_READ_HI			0x26
#define DPS_IDX_CLR								0x07		//	                                            //  #define DPS_ADR_DAC_CLR				0x27
#define DPS_IDX_READ_START						0x08		//	#define DPS_ADR_DPS_READ_START		0x18    //  #define DPS_ADR_DAC_READ_START		0x28
#define DPS_IDX_STATE							0x09		//	#define DPS_ADR_DPS_STATE			0x19    //  #define DPS_ADR_DAC_STATE			0x29

#define DPS20_IDX_DAC_ADC_CONTROL				0x0A
#define DPS20_IDX_ADC_NUM_OF_MEASURE			0x0B
#define DPS20_IDX_ADC_INTERVAL_TIME_LSB			0x0C
#define DPS20_IDX_ADC_INTERVAL_TIME_MID			0x0D
#define DPS20_IDX_ADC_INTERVAL_TIME_MSB			0x0E
#define DPS20_IDX_ADC_READ_MEASURED				0x0F

#define DPS20_BIT_BUFFER_WRITE_DONE     		0x10
#define DPS20_BIT_BUFFER_READ_DONE      		0x20
#define DPS20_IDX_ADC_NUM_OF_MEASURE    		0x0B
#define DPS20_IDX_ADC_INTERVAL_TIME_LSB 		0x0C
#define DPS20_IDX_ADC_INTERVAL_TIME_MID 		0x0D
#define DPS20_IDX_ADC_INTERVAL_TIME_MSB 		0x0E
#define DPS20_IDX_ADC_READ_MEASURED     		0x0F


#define DPS_ADR_ADC_READ_START					0x30
#define DPS_ADR_ADC_READ_LO						0x34
#define DPS_ADR_ADC_READ_MID					0x35
#define DPS_ADR_ADC_STATE						0x39
#define DPS_ADR_ADC_READ_RUN					0x3A


#define DEVID_DPS								0x02
#define DPS_CHIP_ID								0x02
#define DPS_BIT_STATE_DONE						0x01
#define DPS_BIT_STATE_READY						0x02

#define DPS_CHANNEL_PER_MODULE					2
#define DPS_MAX_MODULE							3

#define DPS_ADC7328_RANGE_10V					0	//! - 10 ~ +  10V
#define DPS_ADC7328_RANGE_5V					1	//! -  5 ~ +   5V
#define DPS_ADC7328_RANGE_2DOT5V				2	//! -2.5 ~ + 2.5V
#define DPS_ADC7328_RANGE_P10V					3	//!    0 ~ +  10V

#define DEVID_ZILKER							0x40
#define DEVID_CH5_ZILKER						0x40
#define DEVID_CH6_ZILKER						0x42

#define DPS_ZILKER_OPERATION					0x01
#define DPS_ZILKER_ON_OFF_CONFIG				0x02
#define DPS_ZILKER_CLEAR_FAULTS					0x03
#define DPS_ZILKER_VOUT_COMMAND					0x21
#define DPS_ZILKER_VOUT_MAX						0x24
#define DPS_ZILKER_VOUT_MARGIN_HIGH				0x25
#define DPS_ZILKER_VOUT_MARGIN_LOW				0x26
#define DPS_ZILKER_VOUT_SCALE_LOOP				0x29
#define DPS_ZILKER_VOUT_SCALE_MONITOR			0x2A
#define DPS_ZILKER_IOUT_CAL_GAIN				0x38
#define DPS_ZILKER_IOUT_CAL_OFFSET				0x39
#define DPS_ZILKER_VOUT_OV_FAULT_LIMT			0x40
#define DPS_ZILKER_VOUT_OV_FAULT_RESPONSE		0x41	//! Data Format: Custom (PMBus spec part II - section 10.5.1)
#define DPS_ZILKER_VOUT_UV_FAULT_LIMT			0x44
#define DPS_ZILKER_VOUT_UV_FAULT_RESPONSE		0x45	//! Data Format: Custom (PMBus spec part II - section 10.5.1)
#define DPS_ZILKER_OT_FAULT_RESPONSE			0x50	//! Data Format: Custom (PMBus spec part II - section 10.5.1)
#define DPS_ZILKER_UT_FAULT_RESPONSE			0x54	//! Data Format: Custom (PMBus spec part II - section 10.5.1)
#define DPS_ZILKER_VIN_OV_FAULT_RESPONSE		0x56	//! Data Format: Custom (PMBus spec part II - section 10.5.1)
#define DPS_ZILKER_VIN_UV_FAULT_RESPONSE		0x5A	//! Data Format: Custom (PMBus spec part II - section 10.5.1)
#define DPS_ZILKER_POWER_GOOD_ON				0x5E
#define DPS_ZILKER_STATUS_BYTE					0x78
#define DPS_ZILKER_STATUS_WORD					0x79
#define DPS_ZILKER_STATUS_VOUT					0x7A
#define DPS_ZILKER_STATUS_IOUT					0x7B
#define DPS_ZILKER_STATUS_CML					0x7E
#define DPS_ZILKER_READ_VIN						0x88
#define DPS_ZILKER_READ_VOUT					0x8B
#define DPS_ZILKER_READ_IOUT					0x8C
#define DPS_ZILKER_PMBUS_REVISION				0x98
#define DPS_ZILKER_IOUT_AVG_OC_FAULT_LIMIT		0xE7
#define DPS_ZILKER_IOUT_AVG_UC_FAULT_LIMIT		0xE8

// MD4L
#define MD4L_SLAVE_ADDRESS						0x7C


uint8_t i2c_SendByte(uint8_t bytData);
uint8_t i2c_SendByteA(uint8_t bytData);
uint8_t SendByteFC(uint8_t bytData);
void i2c_SelectType(uint8_t byI2cType);
void i2c_SetOption(uint8_t byI2cType, uint8_t bytOption);
void i2c_Init();
uint8_t i2c_Start(void);
uint8_t i2c_Start_sendbyte(uint8_t slave_addr);
uint8_t i2c_Stop();
int i2c_ReceiveByte(uint8_t nack);
uint8_t i2c_WriteReg(uint8_t byI2cType, uint8_t slave_addr, uint8_t start_reg, uint8_t start_reg_H, uint8_t *pbytData, uint16_t length);  //(J.J 2013.10.25) length uint8_t => uint16_t
uint8_t i2c_ReadBytes(uint8_t slave_addr, uint8_t *pbytData, uint16_t length);  //(J.J 2013.10.25) length uint8_t => uint16_t
uint8_t i2c_ReadReg(uint8_t byI2cType, uint8_t slave_addr, uint8_t start_reg, uint8_t start_reg_H, uint8_t *pbytData, uint16_t length);  //(J.J 2013.10.25) length uint8_t => uint16_t
uint8_t i2c_SendBytes(uint8_t *pbytData, uint8_t length);
uint8_t i2c_ReceiveBytes(uint8_t *pbytData, uint8_t length);
uint8_t i2c_ReadPower(uint8_t nNumMeasure, uint8_t *pbytData);

uint8_t i2c_getType(void);
uint8_t i2c_getBase(void);
uint8_t i2c_getOption(uint8_t channel);


#endif /* I2C_FPGA_H_ */
