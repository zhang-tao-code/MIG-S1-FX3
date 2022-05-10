/*
 * epcs.c
 *
 *  Created on: 2012. 3. 8.
 *      Author: soyim
 */
#include "MIG-S1.h"
#include "cyu3error.h"
#include <cyu3gpio.h>

#include "err_code.h"
#include "epcs.h"
#include "i2c_fpga.h"

#define MAX_WAIT_EPCS		0x2000

uint8_t write_wait(uint8_t dev_id)
{
	uint8_t bytRet = err_OK;
//	uint16_t wCnt = 0;
//
//	uint8_t read_byte;
//
//	do
//	{
//		local_read(dev_id, WRITE_BUSY_REG_B1, &read_byte);
//		if(wCnt++ > MAX_WAIT_EPCS)
//		{
//			bytRet = err_EPCS_WAIT;
//			break;
//		}
//
//	} while((read_byte & 0x01) == 0x01);
//
//	if (bytRet==err_OK)
//	{
//		local_read(dev_id, WRITE_BUSY_REG_B1, &read_byte);
//		if ((read_byte & 0x02) !=  err_OK)
//			bytRet = err_ILL_WRITE;
//	}

	return bytRet;
}

uint8_t epcs_write(uint8_t dev_id, uint8_t *pbytData, uint16_t length)
{
	uint8_t bytRet = err_OK;
//	uint16_t data_cnt=0;
//
//	for(data_cnt=0; data_cnt<length; data_cnt++)
//	{
//		local_write(dev_id, WRITE_OPER_REG_B0, pbytData[data_cnt]);
//
//		if ( (bytRet=write_wait(dev_id)) !=  err_OK)
//			break;
//		}

	return bytRet;
}

uint8_t read_wait(uint8_t dev_id)
{
	uint8_t bytRet = err_OK;
//	uint16_t wCnt = 0;
//	uint8_t read_byte;
//
//	do
//	{
//		local_read(dev_id, READ_OPER_REG_B2, &read_byte);
//		if(wCnt++ > MAX_WAIT_EPCS)
//		{
//			bytRet = err_EPCS_WAIT;
//			break;
//		}
//	} while((read_byte & 0x01) == 0x01);

	return bytRet;
}

uint8_t epcs_read(uint8_t dev_id, uint8_t *pbytData, uint16_t length)
{
	uint8_t bytRet = err_OK;
//	uint16_t data_cnt=0;
//
//	for(data_cnt=0; data_cnt<length; data_cnt++)
//	{
//		local_write(dev_id, READ_OPER_REG_B2, 0x01);
//
//		if ( (bytRet=read_wait(dev_id)) !=  err_OK) break;
//
//		local_read(dev_id, READ_DATA_REG_B3, &pbytData[data_cnt]);
//
//	}

	return bytRet;
}

//---------------------------------------------------------------------------------------------------------------
// MD4L
//---------------------------------------------------------------------------------------------------------------

uint8_t md4l_write_wait()
{
	uint8_t bytRet = err_OK;
//	uint16_t wCnt = 0;
//
//	uint8_t read_byte[5];
//
//	do
//	{
//		bytRet = i2c_ReadReg(I2C_TYPE_EXP, MD4L_SLAVE_ADDRESS, MD4L_WRITE_BUSY_REG_49, 0x00, read_byte, 4);
//
//		if (bytRet !=  err_OK)
//		{

//			break;
//		}
//
//		if(wCnt++ > MAX_WAIT_EPCS)
//		{
//			bytRet = err_EPCS_WAIT;
//			break;
//		}
//
//	} while((read_byte[0] & 0x80) == 0x80); // Busy
//
//	if (bytRet == err_OK)
//	{
//		bytRet = i2c_ReadReg(I2C_TYPE_EXP, MD4L_SLAVE_ADDRESS, MD4L_WRITE_BUSY_REG_49, 0x00, read_byte, 4);
//
//		if ((bytRet ==  err_OK) & ((read_byte[0] & 0x40) !=  err_OK)) // illegal write
//		{
//			bytRet = err_ILL_WRITE;

//		}
//	}

	return bytRet;
}

uint8_t md4l_write(uint8_t *pbytData, uint16_t length)
{
	uint8_t bytRet = err_OK;
//	uint16_t data_cnt = 0;
//	uint8_t md4l_dat[5];
//
//	for(data_cnt=0; data_cnt<length; data_cnt=data_cnt+4)
//	{
//		md4l_dat[0] = pbytData[data_cnt];
//		md4l_dat[1] = pbytData[data_cnt+1];
//		md4l_dat[2] = pbytData[data_cnt+2];
//		md4l_dat[3] = pbytData[data_cnt+3];
//
//
//		bytRet = i2c_WriteReg(I2C_TYPE_EXP, MD4L_SLAVE_ADDRESS, MD4L_WRITE_OPER_REG_49, 0x00, md4l_dat, 4);
//
//		if (bytRet !=  err_OK)
//		{
//			break;
//		}
//
//	}

	return bytRet;
}

uint8_t md4l_read_wait()
{
	uint8_t bytRet = err_OK;
//	uint16_t wCnt = 0;
//	uint8_t read_byte[5];
//
//	do
//	{
//		bytRet = i2c_ReadReg(I2C_TYPE_EXP, MD4L_SLAVE_ADDRESS, MD4L_READ_OPER_REG_4A, 0x00, read_byte, 4);
//
//		if (bytRet != err_OK)
//		{
//			break;
//		}
//
//		if(wCnt++ > MAX_WAIT_EPCS)
//		{
//			bytRet = err_EPCS_WAIT;
//			break;
//		}
//	} while((read_byte[0] & 0x80) == 0x80); // Busy

	return bytRet;
}

uint8_t md4l_read(uint8_t *pbytData, uint16_t length)
{
	uint8_t bytRet = err_OK;
//	uint16_t data_cnt=0;
//	uint8_t md4l_dat[5];
//	uint32_t md4l_address = 0;
//
//	bytRet = i2c_ReadReg(I2C_TYPE_EXP, MD4L_SLAVE_ADDRESS, MD4L_READ_OPER_REG_4A, 0x00, md4l_dat, 4);
//	if (bytRet !=  err_OK)
//	{

//		return bytRet;
//	}
//
//	md4l_address = ((md4l_dat[1] & 0xff)<<16) | ((md4l_dat[2] & 0xff)<<8) | (md4l_dat[3] & 0xff);
//
//	for(data_cnt=0; data_cnt<length; data_cnt=data_cnt+4)
//	{
//		md4l_dat[0] = 0x80;
//		md4l_dat[1] = (md4l_address & 0xff0000)>>16;
//		md4l_dat[2] = (md4l_address & 0x00ff00)>>8;
//		md4l_dat[3] = (md4l_address & 0x0000ff);
//
//		bytRet = i2c_WriteReg(I2C_TYPE_EXP, MD4L_SLAVE_ADDRESS, MD4L_READ_OPER_REG_4A, 0x00, md4l_dat, 4);
//
//		md4l_address = md4l_address + 4;
//
//		if (bytRet !=  err_OK)
//		{

//			break;
//		}
//
//		if ( (bytRet=md4l_read_wait()) !=  err_OK)
//		{
//			break;
//		}
//
//		bytRet = i2c_ReadReg(I2C_TYPE_EXP, MD4L_SLAVE_ADDRESS, MD4L_READ_DATA_REG_4B, 0x00, md4l_dat, 4);
//		pbytData[data_cnt] = md4l_dat[0];
//		pbytData[data_cnt+1] = md4l_dat[1];
//		pbytData[data_cnt+2] = md4l_dat[2];
//		pbytData[data_cnt+3] = md4l_dat[3];
//
//		if (bytRet !=  err_OK)
//		{

//			break;
//		}
//	}

	return bytRet;
}

uint8_t md4dsi_write(uint8_t *pbytData , uint16_t length)
{
	uint8_t bytRet = err_OK;
//	uint16_t data_cnt,Cnt = 0;
//	uint8_t md4dsi_wDat[1] = {0,};
//	uint8_t md4dsi_rDat[1] = {0,};
//
//
//	for(data_cnt=0; data_cnt<length; data_cnt++)
//	{
//		md4dsi_wDat[0] = pbytData[data_cnt];
//
//		bytRet = i2c_WriteReg(I2C_TYPE_EXP, MD4DSI_SLAVE_ADD, MD4DSI_WRIT_OPER_REG_B0, 0x00, md4dsi_wDat, 1);
//
//		Cnt = 0;
//		do{
//			bytRet = i2c_ReadReg(I2C_TYPE_EXP, MD4DSI_SLAVE_ADD, MD4DSI_WRITE_BUSY_REG_B1, 0x00, md4dsi_rDat, 1);
//
//			if(md4dsi_rDat[0] ==  err_OK)
//			{
//				bytRet = err_I2C_WAITDONE;
//				break;
//			}
//			else if(Cnt ==  TimeOut)
//			{
//				bytRet = err_I2C_WAITDONE;
//				break;
//			}
//
//		}while(Cnt++ < TimeOut);
//	}

	return bytRet;
}

uint8_t md4dsi_read(uint8_t *pbytData, uint16_t length)
{
	uint8_t bytRet = err_OK;
//	uint16_t data_cnt,Cnt=0;
//	uint8_t md4dsi_rDat[1] = {0,};
//	uint8_t md4dsi_wDat[1] = {0x01};
//
//	for(data_cnt=0; data_cnt<length; data_cnt++)
//	{
//
//		bytRet = i2c_WriteReg(I2C_TYPE_EXP, MD4DSI_SLAVE_ADD, MD4DSI_READ_OPER_REG_B2, 0x00, md4dsi_wDat, 1);
//
//		if (bytRet !=  err_OK)
//		{
//			bytRet = err_I2C_WAITDONE;
//			break;
//		}
//
//		Cnt = 0;
//		do{
//			bytRet = i2c_ReadReg(I2C_TYPE_EXP, MD4DSI_SLAVE_ADD, MD4DSI_READ_OPER_REG_B2, 0x00, md4dsi_rDat, 1);
//
//			if( md4dsi_rDat[0] ==  err_OK)
//			{
//				bytRet = err_I2C_WAITDONE;
//				break;
//			}
//			else if(Cnt ==  TimeOut)
//			{
//				bytRet = err_I2C_WAITDONE;
//				break;
//			}
//
//		}while(Cnt++ < TimeOut);
//
//
//		bytRet = i2c_ReadReg(I2C_TYPE_EXP, MD4DSI_SLAVE_ADD, MD4DSI_READ_DATA_REG_B3, 0x00, md4dsi_rDat, 1);
//
//		pbytData[data_cnt] = md4dsi_rDat[0];
//
//		if (bytRet !=  err_OK)
//		{
//			break;
//		}
//	}

	return bytRet;
}
