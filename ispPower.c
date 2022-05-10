/*
 * ispPower.c
 *
 *  Created on: 2012. 8. 28.
 *      Author: J.J
 */

#include "ispPower.h"

#include "MIG-S1.h"
#include "cyu3error.h"
#include <cyu3gpio.h>

#include "i2c_fpga.h"
#include "err_code.h"

#define AD5360_DAC_RES	((0x0001<<15)-1)
#define AD5360_GET_DAC_CODE(V, VERF)	((V * AD5360_DAC_RES) / (2*VERF))

int m_nNumMeasureAverage;  //Average Value Save

unsigned char MsbCh[6] = {0xC8, 0xD0, 0xC8, 0xD0, 0xC8, 0xD0};
unsigned char MsbCh2[6] = {0xCB, 0xD3, 0xCB, 0xD3, 0xCB, 0xD3};
unsigned char pBuf[257] = {0,};

int SetPower(unsigned char *SetValue, unsigned char DevID)
{
	int i = 0;
	//int Meas_Cnt = 0;
	uint8_t read_data = 0;
	uint8_t cBuffer[2] = {0,};

	if(SetValue[0] == '1')
	{
		for(i=0; i<4; i++)
		{
			if((SetValue[1] & (1<<i)) != 0)
			{
				///////////////////////////////////////////////////////////////////////////////////////
				// Power On Sequence
				local_write(DevID, PWR_CHANNEL_SELECT, 1<<i);
				local_write(DevID, DAC_MACHINE_STATUS, 0x00);
				local_write(DevID, DAC_WRITE_LSB, SetValue[(i*2)+2]);
				local_write(DevID, DAC_WRITE_MIDDLE, SetValue[(i*2)+3]);
				local_write(DevID, DAC_WRITE_MSB, MsbCh[i]);
				local_write(DevID, DAC_WRITE_COMMAND, 0x00);

				local_write(DevID, DAC_MACHINE_STATUS, 0x00);
				local_write(DevID, DAC_WRITE_LSB, 0x32);
				local_write(DevID, DAC_WRITE_MIDDLE, 0x33);
				local_write(DevID, DAC_WRITE_MSB, MsbCh2[i]);
				local_write(DevID, DAC_WRITE_COMMAND, 0x00);

				local_write(DevID, DAC_MACHINE_STATUS, 0x00);
				local_write(DevID, DAC_WRITE_LSB, 0x00);
				local_write(DevID, DAC_WRITE_MIDDLE, 0x00);
				local_write(DevID, DAC_WRITE_MSB, MsbCh2[i]+1);
				local_write(DevID, DAC_WRITE_COMMAND, 0x00);

				local_write(DevID, DPS_MACHINE_STATUS, 0x00);
				local_write(DevID, DPS_WRITE_LSB, 0xdf);
				local_write(DevID, DPS_WRITE_MIDDLE, 0x19);
				local_write(DevID, DPS_WRITE_MSB, 0x00);
				local_write(DevID, DPS_WRITE_COMMAND, 0x00);
				///////////////////////////////////////////////////////////////////////////////////////

				int j=0;
				while(1)
				{
					local_read (DevID, DPS_MACHINE_STATUS, &read_data);
					if((read_data & 0x03) == 0x03)
					{
						break;
					}
					else if(j > 1000)
					{
						return 0;
					}

					j++;
					//udelay(100);
					CyU3PThreadSleep(1);
				}

				local_write(DevID, 0x05, 1<<i);
				//udelay(500000); //500ms sleep
				CyU3PThreadSleep(5);

				local_write(DevID, 0x39, 0x00);
				local_write(DevID, 0x31, 0x10);
				local_write(DevID, 0x32, 0x84);
				local_write(DevID, 0x30, 0x00);

				j = 0;
				while(1)
				{
					local_read (DevID, 0x39, &read_data);
					if((read_data & 0x13) == 0x13)
					{
						break;
					}
					else if(j > 1000)
					{
						return 0;
					}

					j++;
					//udelay(100);
					CyU3PThreadSleep(1);
				}

				local_write(DevID, 0x39, 0x00);
				local_write(DevID, 0x31, 0xA0);
				local_write(DevID, 0x32, 0x92);
				local_write(DevID, 0x30, 0x00);

				j = 0;
				while(1)
				{
					local_read (DevID, 0x39, &read_data);
					if((read_data & 0x13) == 0x13)
					{
						break;
					}
					else if(j > 5000)
					{
						return 0;
					}

					j++;
					//udelay(100);
					CyU3PThreadSleep(1);
				}

				local_write(DevID, 0x39, 0x00);
				local_write(DevID, 0x31, 0x10);
				local_write(DevID, 0x32, 0x84);
				local_write(DevID, 0x30, 0x00);

				j = 0;
				while(1)
				{
					local_read (DevID, 0x39, &read_data);
					if((read_data & 0x13) == 0x13)
					{

						break;
					}
					else if(j > 5000)
					{
						return 0;
					}

					j++;
					//udelay(100);
					CyU3PThreadSleep(1);
				}

				//Write_Reg(DevID, 0x02, 0x08);
				local_write(DevID, 0x3b, 0x1E);
				local_write(DevID, 0x3c, 0x0a);
				local_write(DevID, 0x3d, 0x00);
				local_write(DevID, 0x3e, 0x00);
				local_write(DevID, 0x39, 0x00);
				local_write(DevID, 0x30, 0x00);

				j = 0;
				while(1)
				{
					local_read (DevID, 0x39, &read_data);
					if((read_data & 0x13) == 0x13)
					{
						break;
					}
					else if(j > 5000)
					{
						break;
					}
					j++;
					//udelay(100);
					CyU3PThreadSleep(1);
				}
			}
		}

		local_write(DevID, 0x5, 0x30);

		cBuffer[0] = SetValue[10];
		cBuffer[1] = SetValue[11];
		i2c_WriteReg(I2C_TYPE_ZILKER, 0x40, 0x21, 0, cBuffer, 2);

		cBuffer[0] = SetValue[12];
		cBuffer[1] = SetValue[13];
		i2c_WriteReg(I2C_TYPE_ZILKER, 0x42, 0x21, 0, cBuffer, 2);

		CyU3PThreadSleep(5);

		//local_write(DevID, 0x5, 0x30);

		CyU3PThreadSleep(100);

	}
	else
	{
		for(i=0; i<MAX_POWER_SOURCE; i++)
		{
			local_write(DevID, PWR_CHANNEL_SELECT, 1<<i);
			local_write(DevID, DAC_WRITE_LSB, 0x00);
			local_write(DevID, DAC_WRITE_MIDDLE, 0x00);
			local_write(DevID, DAC_WRITE_MSB, MsbCh[i]);
			local_write(DevID, DAC_WRITE_COMMAND, 0x00);
		}

/*		cBuffer[0] = 0;
		cBuffer[1] = 0;
		i2c_WriteReg(I2C_TYPE_ZILKER, 0x40, 0x21, 0, cBuffer, 2);

		cBuffer[0] = 0;
		cBuffer[1] = 0;
		i2c_WriteReg(I2C_TYPE_ZILKER, 0x42, 0x21, 0, cBuffer, 2);

		local_write(DevID, 0x5, 0x30);*/
	}

	//Read_Reg(DEV_CTRL, eCCM_Rev_Minor);*
	return 1;
}
