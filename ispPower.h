/*
 * ispPower.h
 *
 *  Created on: 2012. 8. 28.
 *      Author: J.J
 */

#ifndef ISPPOWER_H_
#define ISPPOWER_H_

#define MAX_INTERNAL_POWER		0x01
#define MAX_CAMERA_POWER		0x05
#define MAX_POWER_SOURCE		(MAX_CAMERA_POWER+MAX_INTERNAL_POWER)

#define PWR_CHANNEL_SELECT  	0x04

#define DPS_WRITE_COMMAND   	0x10
#define DPS_WRITE_LSB	    	0x11
#define DPS_WRITE_MIDDLE    	0x12
#define DPS_WRITE_MSB	    	0x13
#define DPS_MACHINE_STATUS  	0x19

#define DAC_WRITE_COMMAND   	0x20
#define DAC_WRITE_LSB	    	0x21
#define DAC_WRITE_MIDDLE    	0x22
#define DAC_WRITE_MSB	    	0x23
#define DAC_MACHINE_STATUS  	0x29

#define DPS_ADR_DPS				0x10
#define DPS_ADR_DAC				0x20
#define DPS_ADR_ADC				0x30

#define DEVID_DPS				0x02

#define DPS_BIT_STATE_DONE		0x01
#define DPS_BIT_STATE_READY		0x02
#define DPS_IDX_READ_LO			0x04
#define DPS_IDX_READ_MID		0x05
#define DPS_IDX_READ_HI			0x06
#define DPS_IDX_CLR				0x07
#define DPS_IDX_READ_START		0x08
#define DPS_IDX_STATE			0x09
#define DPS_ADR_ADC_READ_START	0x30
#define MAX_RETRY_CHECK_STATE	10

#define DPS20_BIT_BUFFER_WRITE_DONE	 	0x10
#define DPS20_BIT_BUFFER_READ_DONE	 	0x20
#define DPS20_IDX_ADC_NUM_OF_MEASURE	0x0B
#define DPS20_IDX_ADC_INTERVAL_TIME_LSB	0x0C
#define DPS20_IDX_ADC_INTERVAL_TIME_MID	0x0D
#define DPS20_IDX_ADC_INTERVAL_TIME_MSB	0x0E
#define DPS20_IDX_ADC_READ_MEASURED		0x0F

#define SINGLE_READ				0x1
#define BURST_READ				0x2

int SetPower(unsigned char *SetValue, unsigned char DevID);

#endif /* ISPPOWER_H_ */
