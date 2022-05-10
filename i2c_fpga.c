/*
 * i2c_fpga.c
 *
 *  Created on: 2012. 3. 8.
 *      Author: soyim
 */


#include "MIG-S1.h"
#include "cyu3error.h"
#include <cyu3gpio.h>

#include "i2c_fpga.h"
#include "err_code.h"

//define by Atlas
#define TEST

#ifdef TEST
#define UART_READ_I2C_STATUS(data)
#define UART_SEND_I2C_DATA(data)
#define UART_READ_I2C_DATA(data)
#define UART_GET_I2C_ACK(data)
#define UART_SEND_I2C_START()
#define UART_SEND_I2C_DATA(data)
#define UART_SEND_I2C_SLAVEADDR(data)
#define UART_SEND_I2C_STOP(data)
#define UART_SEND_I2C_READ()

#define DPS_WRITE(data1,data2)
#define DPS_READ(data1,data2)
#endif

// Declare Local constant...
#define ACK         0x00
#define NACK        0x01

#define REF_CLK_FREQ		(40*1000000)	//	40 MHz
#define GET_PRESCALE_VAL(F)	((REF_CLK_FREQ/5)/(F))

#define DEF_I2C_FREQ		(400*1000)		// 400 KHz
#define DEF_I2C_PRESCALE	GET_PRESCALE_VAL(DEF_I2C_FREQ)

//#define MAX_WAIT_I2C		0x100
#define MAX_WAIT_I2C		0x1000  //1Cycle = 7.1us
#define I2C_CTRL_TIP		0x01
#define I2C_CTRL_BUSY		0x02

#define I2C_CTRL_READY		0x01
#define I2C_CTRL_DONE		0x02
#define I2C_CTRL_SCLK		0x80
#define I2C_CTRL_SDATA		0x40

#define MAX_RETRY_CHECK_STATE 1000  //(J.J 2011.06.14)


uint8_t g_bytI2cDev = DEV_CTRL;
uint8_t	m_byI2cType = I2C_TYPE_CCM;
uint8_t	m_byI2cOption[MAX_I2C_TYPE] = { DEF_I2C_OPTION, DEF_I2C_OPTION, DEF_I2C_OPTION };
uint8_t m_byI2cBase;

#define IS_REPSTART()	(m_byI2cOption[m_byI2cType] & BIT_I2C_REPSTART)
#define IS_WORDINDEX()	(m_byI2cOption[m_byI2cType] & BIT_I2C_WORDINDEX)

uint8_t WaitForTIP(void)
{
	uint8_t bytRet = err_OK;
	uint16_t wCnt = 0;
	uint8_t read_data = 0;
#if 0
	do
	{
//		local_read(g_bytI2cDev, eI2c_Status|m_byI2cBase, &read_data);
		UART_READ_I2C_STATUS(&read_data);
		read_data &= I2C_CTRL_TIP;
		if (wCnt++ > MAX_WAIT_I2C)
		{
			bytRet = err_I2C_WAITREADY;
			break;
		}
	} while(read_data==0x01);
#endif
	return bytRet;
}

uint8_t WaitForBusy(void)
{
	uint8_t bytRet = err_OK;
	uint16_t wCnt = 0;
	uint8_t read_data = 0;
#if 0
	do
	{
//	local_read(g_bytI2cDev, eI2c_Status|m_byI2cBase, &read_data);
	UART_READ_I2C_STATUS(&read_data);
	read_data &= I2C_CTRL_BUSY;
	if (wCnt++ > MAX_WAIT_I2C)
		{
		bytRet = err_I2C_WAITREADY;
		break;
		}
	} while(read_data==0x01);
#endif
	return bytRet;
}

uint8_t i2c_SendByte(uint8_t bytData)
{
//	local_write(g_bytI2cDev, eI2c_SendByte|m_byI2cBase, bytData);
	//UART_SEND_I2C_DATA(bytData);
	//return WaitForTIP();
	return 0;
}

uint8_t i2c_SendByteA(uint8_t bytData)
{
	uint8_t bytRet;
	uint8_t read_data = 0;
#if 0
	if( (bytRet=i2c_SendByte(bytData)) == err_OK )
	{
//		local_read(g_bytI2cDev, eI2c_SendByte|m_byI2cBase, &read_data);
		UART_GET_I2C_ACK(&read_data);
		if ((read_data & 0x01) != 0)	// Check Acknowledge
				bytRet = err_I2C_NOACK;
	}
	#endif
	return bytRet;
}

uint8_t SendByteFC(uint8_t bytData)
{
	uint8_t bytRet;
#if 0
	if (m_byI2cOption[m_byI2cType] & BIT_I2C_CHECKACK)
		bytRet = i2c_SendByteA(bytData);
	else
		bytRet = i2c_SendByte(bytData);
#endif
	return bytRet;
}

//	Select I2C Bus CCM or EXP(Internal Use) ------------------------------------
void i2c_SelectType(uint8_t byI2cType)
{
#if 0
	m_byI2cType = byI2cType;

	switch(byI2cType)
	{
		case I2C_TYPE_EXP:
			g_bytI2cDev = DEV_CTRL;
			m_byI2cBase = I2C_BASE_EXP;
			break;
		case I2C_TYPE_ZILKER:
			g_bytI2cDev = DEV_DPS;
			m_byI2cBase = I2C_BASE_CCM;
			break;
		case I2C_TYPE_CCM:
		default: //I2C_TYPE_CCM
			g_bytI2cDev = DEV_CTRL;
		    m_byI2cBase = I2C_BASE_CCM;
			break;
	}
#endif
}

uint8_t i2c_getType(void)
{
	return m_byI2cType;
}

uint8_t i2c_getBase(void)
{
	return m_byI2cBase;
}

uint8_t i2c_getOption(uint8_t channel)
{
	return m_byI2cOption[channel];
}

void i2c_SetOption(uint8_t byI2cType, uint8_t bytOption)
{
	//i2c_SelectType(byI2cType);
	//m_byI2cOption[m_byI2cType] = bytOption;
}

void i2c_Init()
{
	//Atlas

	//To do set FPGA register

//	i2c_SetOption(I2C_TYPE_CCM, 	DEF_I2C_OPTION);
//	local_write(g_bytI2cDev, eI2c_Prescale|m_byI2cBase, GET_PRESCALE_VAL(400000));	// Set 400KHz
//
//    i2c_SetOption(I2C_TYPE_EXP, 	DEF_I2C_OPTION);
//	local_write(g_bytI2cDev, eI2c_Prescale|m_byI2cBase, GET_PRESCALE_VAL(400000));	// Set 400KHz
//
//	i2c_SetOption(I2C_TYPE_ZILKER, 	DEF_I2C_OPTION);
//	local_write(g_bytI2cDev, eI2c_Prescale|m_byI2cBase, GET_PRESCALE_VAL(99000));	// Set 99KHz(under 100KHz)
}

uint8_t i2c_Start(void)
{
#if 0
	uint8_t bytRet;
	uint8_t read_data = 0;

//	local_write(g_bytI2cDev, eI2c_Start|m_byI2cBase, 0);
	UART_SEND_I2C_START();

    if( (bytRet=WaitForTIP()) == err_OK )
	{
    	if (m_byI2cOption[m_byI2cType] & BIT_I2C_CHECKACK)
    	{
//		local_read(g_bytI2cDev, eI2c_SendByte|m_byI2cBase, &read_data);
    		UART_GET_I2C_ACK(&read_data);
			if((read_data & 0x01) != 0)	// Check Acknowledge
				bytRet = err_I2C_NOACK;
    	}
	}
#endif
	return 0;
}

uint8_t i2c_Start_sendbyte(uint8_t slave_addr)
{
#if 0
	uint8_t bytRet = err_OK;
	uint8_t read_data = 0;

//	local_write(g_bytI2cDev, 0x09|m_byI2cBase, slave_addr);
	UART_SEND_I2C_SLAVEADDR(slave_addr);

//	local_write(g_bytI2cDev, eI2c_Start|m_byI2cBase, 0);
	UART_SEND_I2C_START();

    if( (bytRet=WaitForTIP()) == err_OK )
	{
    	if (m_byI2cOption[m_byI2cType] & BIT_I2C_CHECKACK)
    	{
//		local_read(g_bytI2cDev, eI2c_SendByte|m_byI2cBase, &read_data);
    		UART_GET_I2C_ACK();

		if((read_data & 0x01) != 0)	// Check Acknowledge
			bytRet = err_I2C_NOACK;
    	}
	}
#endif
	return 0;
}

uint8_t i2c_Stop()
{
//	local_write(g_bytI2cDev, eI2c_Stop|m_byI2cBase, 0);
//	UART_SEND_I2C_STOP();
//	return WaitForBusy();
return 0 ;

}

int i2c_ReceiveByte(uint8_t nack)
{
#if 0
	uint8_t bytRet;
	uint8_t read_data = 0;

//	local_write(g_bytI2cDev, eI2c_RecByte|m_byI2cBase, nack);
	UART_SEND_I2C_READ();
	//

	if( (bytRet=WaitForTIP()) != err_OK)
		return -1;  //(J.J 2011.07.29) bytRet => -1
	else
	{
//		local_read(g_bytI2cDev, eI2c_RecByte|m_byI2cBase, &read_data);
		UART_READ_I2C_DATA(&read_data);
		bytRet = read_data;
		return bytRet;
	}
	#endif
	return 0;
}

uint8_t i2c_WriteReg(uint8_t byI2cType, uint8_t slave_addr, uint8_t start_reg, uint8_t start_reg_H, uint8_t *pbytData, uint16_t length)  //(J.J 2013.10.25) length uint8_t => uint16_t
{
	uint16_t data_cnt=0;
	uint8_t bytRet;

//
//	pbytData[length] = 0;  //(J.J 2011.07.07)
//
//	i2c_SelectType(byI2cType);
//
//	local_write(g_bytI2cDev, 0x09|m_byI2cBase, slave_addr & 0xfe);
//
//	if( (bytRet=i2c_Start()) == err_OK )
//	{
//		if( ( IS_WORDINDEX() && (bytRet=SendByteFC(start_reg_H) ==  err_OK) )	// Send start register number
//			|| (!IS_WORDINDEX()) )
//		{
//			pbytData[length]++;  //(J.J 2011.07.07)
//		    if( (bytRet=SendByteFC(start_reg)) ==  err_OK)	// Send start register number
//			{
//				pbytData[length]++;  //(J.J 2011.07.07)
//			    for(data_cnt=0; data_cnt<length; data_cnt++)
//			    {
//					bytRet=SendByteFC(pbytData[data_cnt]);
//
//					if(bytRet != err_OK )
//						break;
//					else    //(J.J 2011.07.07)
//						pbytData[length]++;
//			    }
//			}
//		}
//	}
//
//    i2c_Stop();
	return bytRet;
}

uint8_t i2c_ReadBytes(uint8_t slave_addr, uint8_t *pbytData, uint16_t length)  //(J.J 2013.10.25) length uint8_t => uint16_t
{
	uint16_t data_cnt=0;
	uint8_t bytRet=err_OK;
//	int bytDat = 0;
//
//	local_write(g_bytI2cDev, 0x09|m_byI2cBase, slave_addr | 0x01);
//
//	if (m_byI2cOption[m_byI2cType] & BIT_I2C_CHECKACK)
//		bytRet=i2c_Start();
//	else
//		i2c_Start();
//
//	if(bytRet==err_OK)
//	{
//
//		local_read(g_bytI2cDev, 0x06|m_byI2cBase, &bytRet);
//		if( (bytRet & 0x01) == 0x00)
//		{
//			for(data_cnt=0; data_cnt<length-1; data_cnt++)
//		    {
//				//pbytData[data_cnt] = i2c_ReceiveByte2(ACK);  //(J.J 2011.07.07)
//				bytDat = i2c_ReceiveByte(ACK);
//				if(bytDat < 0)
//				{
//					bytRet = err_I2C_WAITREADY;
//				}
//				else
//				{
//					pbytData[data_cnt] = bytDat;
//					pbytData[length]++;
//				}
//		    }
//
//			//pbytData[length-1] = i2c_ReceiveByte2(NACK);
//			bytDat =  i2c_ReceiveByte(NACK);
//			if(bytDat < 0)
//			{
//				bytRet = err_I2C_WAITREADY;
//			}
//			else
//			{
//				pbytData[length-1] = bytDat;
//				pbytData[length]++;
//			}
//		}
//		else if(bytRet == err_I2C_NOACK)
//		{
//			bytRet = err_I2C_NOACK_DEVADR;
//		}
//	}
//
//    i2c_Stop();
	return bytRet;
}

uint8_t i2c_ReadReg(uint8_t byI2cType, uint8_t slave_addr, uint8_t start_reg, uint8_t start_reg_H, uint8_t *pbytData, uint16_t length)  //(J.J 2013.10.25) length uint8_t => uint16_t
{
	uint8_t bytRet;

//	pbytData[length] = 0x0;  //(J.J 2011.07.07) Init
//
//	i2c_SelectType(byI2cType);
//
//	local_write(g_bytI2cDev, 0x09|m_byI2cBase, slave_addr & 0xfe);
//
//	if( (bytRet=i2c_Start()) == err_OK )
//	{
//
//		if( ( IS_WORDINDEX() && (bytRet=SendByteFC(start_reg_H) ==  err_OK) )	// Send start register number
//			|| (!IS_WORDINDEX()) )
//		{
//			pbytData[length]++;  //(J.J 2011.07.07)
//		    if( (bytRet=SendByteFC(start_reg)) ==  err_OK)	// Send start register number
//			{
//				pbytData[length]++;  //(J.J 2011.07.07)
//				if(!IS_REPSTART())
//					i2c_Stop();
//
//				return(i2c_ReadBytes(slave_addr, pbytData, length));
//			}
//		}
//
//	}
//
//	i2c_Stop();
	return bytRet;
}

uint8_t i2c_SendBytes(uint8_t *pbytData, uint8_t length)
{
	uint8_t data_cnt=0;
	uint8_t bytRet;
#if 0
	if( (bytRet=WaitForTIP()) == err_OK )
	{
	    for(data_cnt=0; data_cnt<length; data_cnt++)
	    {
			if( (bytRet=SendByteFC(pbytData[data_cnt])) != err_OK )
				break;
	    }
	}
	#endif
	return bytRet;
}

uint8_t i2c_ReceiveBytes(uint8_t *pbytData, uint8_t length)
{
	uint8_t data_cnt=0;
	uint8_t bytRet;
	int bytDat = 0;
#if 0
	for(data_cnt=0; data_cnt<length-1; data_cnt++)
    {
		bytDat = i2c_ReceiveByte(ACK);
		if(bytDat < 0)
			bytRet = err_I2C_WAITREADY;
		else
			pbytData[data_cnt] = bytDat;
    }

	bytDat = i2c_ReceiveByte(NACK);
	if(bytDat < 0)
		bytRet = err_I2C_WAITREADY;
	else
		pbytData[length-1]= bytDat;
#endif
	return bytRet;
}


uint8_t i2c_ReadPower(uint8_t nNumMeasure, uint8_t *pbytData)
{
	uint8_t bytRet = 0;
	int ReadCnt = 0, ReadNo = 0, LoopCnt = 0;
	uint8_t bTemp = 0;
	uint8_t bytTemp[2] = {0,};
#if 0
	uint8_t ADC_Set_Value[4] = {0x84, 0x8C, 0x94, 0x9C}; //, 0x80, 0x88, 0x90, 0x98};  //1~4 = Voltage, 5~8 = Current

	for(ReadCnt = 0; ReadCnt < 20; ReadCnt++)  //Read Memory Init
    {
        pbytData[ReadCnt] = NULL;
    }

	nNumMeasure = 0x01;

/////////////////////////////////////////////////////////////////////////////////////////////////////////
// Voltage Read
	for(LoopCnt = 0; LoopCnt < 4; LoopCnt++)
	{
//		local_write(DEVID_DPS, 0x04, (1<<LoopCnt));
		DPS_WRITE(0x04, (1<<LoopCnt));//Channel Select

//		local_write(DEVID_DPS, 0x31, 0x10);
		DPS_WRITE(0x31, 0x10);


//		local_write(DEVID_DPS, 0x32, ADC_Set_Value[LoopCnt]);
		DPS_WRITE(0x32, ADC_Set_Value[LoopCnt]);

//		local_write(DEVID_DPS, DPS_ADR_ADC + DPS20_IDX_ADC_NUM_OF_MEASURE, 1); //Measure Count
//		local_write(DEVID_DPS, DPS_ADR_ADC + DPS20_IDX_ADC_INTERVAL_TIME_LSB, 10); //Interval Time LSB
//		local_write(DEVID_DPS, DPS_ADR_ADC + DPS20_IDX_ADC_INTERVAL_TIME_MID, 0); //Interval Time MID
//		local_write(DEVID_DPS, DPS_ADR_ADC + DPS20_IDX_ADC_INTERVAL_TIME_MSB, 0); //Interval Time MSB
//
//		local_write(DEVID_DPS, DPS_ADR_ADC + DPS_IDX_STATE, 0); //Clear State
//		local_write(DEVID_DPS, DPS_ADR_ADC_READ_START, 0);

		DPS_WRITE(DPS_ADR_ADC + DPS20_IDX_ADC_NUM_OF_MEASURE, 1); //Measure Count
		DPS_WRITE(DPS_ADR_ADC + DPS20_IDX_ADC_INTERVAL_TIME_LSB, 10); //Interval Time LSB
		DPS_WRITE(DPS_ADR_ADC + DPS20_IDX_ADC_INTERVAL_TIME_MID, 0); //Interval Time MID
		DPS_WRITE(DPS_ADR_ADC + DPS20_IDX_ADC_INTERVAL_TIME_MSB, 0); //Interval Time MSB

		DPS_WRITE(DPS_ADR_ADC + DPS_IDX_STATE, 0); //Clear State
		DPS_WRITE(DPS_ADR_ADC_READ_START, 0);

        CyU3PThreadSleep (1);
        //DelayMSec(1);
		//DelayMSec(50);

		ReadCnt = 0;
	    while(CyTrue)
	    {
//			local_read(DEVID_DPS, DPS_ADR_ADC + DPS_IDX_STATE, &bTemp); //State Read
	    	DPS_READ(DPS_ADR_ADC + DPS_IDX_STATE, &bTemp); //State Read
			if(bTemp & DPS20_BIT_BUFFER_WRITE_DONE)
	        {
	            break;
	        }
	        else if(ReadCnt > MAX_RETRY_CHECK_STATE)
	        {
	            break;
	            //return FALSE;
	        }

	        ReadCnt++;
	        CyU3PThreadSleep (1);
//			DelayMSec(1);
	    }

//		local_write(DEVID_DPS, DPS_ADR_ADC + DPS_IDX_STATE, 0);
//		local_write(DEVID_DPS, DPS_ADR_ADC + DPS20_IDX_ADC_READ_MEASURED, 0);

	    DPS_WRITE(DPS_ADR_ADC + DPS_IDX_STATE, 0);
	    DPS_WRITE(DPS_ADR_ADC + DPS20_IDX_ADC_READ_MEASURED, 0);

        ReadCnt = 0;
        while(CyTrue)
        {
//   		    local_read(DEVID_DPS, DPS_ADR_ADC + DPS_IDX_STATE, &bTemp); //State Read
        	DPS_READ(DPS_ADR_ADC + DPS_IDX_STATE, &bTemp); //State Read
            //if(bTemp & (DPS20_BIT_BUFFER_READ_DONE | DPS_BIT_STATE_READY))
			if(bTemp & DPS20_BIT_BUFFER_READ_DONE)
            {
                break;
            }
            else if(ReadCnt > MAX_RETRY_CHECK_STATE)
            {
				break;
                //return FALSE;
            }

            ReadCnt++;
	        CyU3PThreadSleep (1);
        }

//		local_read(DEVID_DPS, DPS_ADR_ADC_READ_LO, &pbytData[ReadNo++]); //Data Read
//		local_read(DEVID_DPS, DPS_ADR_ADC_READ_MID, &pbytData[ReadNo++]);  //Data Read
        DPS_READ(DPS_ADR_ADC_READ_LO, &pbytData[ReadNo++]); //Data Read
        DPS_READ(DPS_ADR_ADC_READ_MID, &pbytData[ReadNo++]);  //Data Read

	}
/////////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////////////////////////
// Zilker Power Read

	i2c_ReadReg(I2C_TYPE_ZILKER, 0x40, 0x8b, 0x00, bytTemp, 2);
	pbytData[ReadNo++] = bytTemp[0];
	pbytData[ReadNo++] = bytTemp[1];
	i2c_ReadReg(I2C_TYPE_ZILKER, 0x42, 0x8b, 0x00, bytTemp, 2);
	pbytData[ReadNo++] = bytTemp[0];
	pbytData[ReadNo++] = bytTemp[1];

/////////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////////////////////////
// Current
	for(LoopCnt = 0; LoopCnt < 4; LoopCnt++)
	{
//		local_write(DEVID_DPS, 0x04, (1<<LoopCnt)); //Channel Select
//		local_write(DEVID_DPS, 0x31, 0x10);
//		local_write(DEVID_DPS, 0x32, (ADC_Set_Value[LoopCnt]-4));
//
//		local_write(DEVID_DPS, DPS_ADR_ADC + DPS20_IDX_ADC_NUM_OF_MEASURE, 1); //Measure Count
//		local_write(DEVID_DPS, DPS_ADR_ADC + DPS20_IDX_ADC_INTERVAL_TIME_LSB, 10); //Interval Time LSB
//		local_write(DEVID_DPS, DPS_ADR_ADC + DPS20_IDX_ADC_INTERVAL_TIME_MID, 0); //Interval Time MID
//		local_write(DEVID_DPS, DPS_ADR_ADC + DPS20_IDX_ADC_INTERVAL_TIME_MSB, 0); //Interval Time MSB
//
//		local_write(DEVID_DPS, DPS_ADR_ADC + DPS_IDX_STATE, 0); //Clear State
//		local_write(DEVID_DPS, DPS_ADR_ADC_READ_START, 0);

		DPS_WRITE(0x04, (1<<LoopCnt)); //Channel Select
		DPS_WRITE(0x31, 0x10);
		DPS_WRITE(0x32, (ADC_Set_Value[LoopCnt]-4));

		DPS_WRITE(DPS_ADR_ADC + DPS20_IDX_ADC_NUM_OF_MEASURE, 1); //Measure Count
		DPS_WRITE(DPS_ADR_ADC + DPS20_IDX_ADC_INTERVAL_TIME_MID, 0); //Interval Time MID
		DPS_WRITE(DPS_ADR_ADC + DPS20_IDX_ADC_INTERVAL_TIME_LSB, 10); //Interval Time LSB
		DPS_WRITE(DPS_ADR_ADC + DPS20_IDX_ADC_INTERVAL_TIME_MSB, 0); //Interval Time MSB

		DPS_WRITE(DPS_ADR_ADC + DPS_IDX_STATE, 0); //Clear State
		DPS_WRITE(DPS_ADR_ADC_READ_START, 0);

        CyU3PThreadSleep (1);
//		DelayMSec(1);
		//DelayMSec(50);

		ReadCnt = 0;
	    while(CyTrue)
	    {
//			local_read(DEVID_DPS, DPS_ADR_ADC + DPS_IDX_STATE, &bTemp); //State Read
			DPS_READ(DPS_ADR_ADC + DPS_IDX_STATE, &bTemp); //State Read
			if(bTemp & DPS20_BIT_BUFFER_WRITE_DONE)
	        {
	            break;
	        }
	        else if(ReadCnt > MAX_RETRY_CHECK_STATE)
	        {
	            break;
	            //return FALSE;
	        }

	        ReadCnt++;
	        CyU3PThreadSleep (1);
//			DelayMSec(1);
	    }

//		local_write(DEVID_DPS, DPS_ADR_ADC + DPS_IDX_STATE, 0);
//		local_write(DEVID_DPS, DPS_ADR_ADC + DPS20_IDX_ADC_READ_MEASURED, 0);
	    DPS_WRITE(DPS_ADR_ADC + DPS_IDX_STATE, 0);
	    DPS_WRITE(DPS_ADR_ADC + DPS20_IDX_ADC_READ_MEASURED, 0);

        ReadCnt = 0;
        while(CyTrue)
        {
//			local_read(DEVID_DPS, DPS_ADR_ADC + DPS_IDX_STATE, &bTemp); //State Read
        	DPS_READ(DPS_ADR_ADC + DPS_IDX_STATE, &bTemp); //State Read
            //if(bTemp & (DPS20_BIT_BUFFER_READ_DONE | DPS_BIT_STATE_READY))
			if(bTemp & DPS20_BIT_BUFFER_READ_DONE)
            {
                break;
            }
            else if(ReadCnt > MAX_RETRY_CHECK_STATE)
            {
				break;
                //return FALSE;
            }

            ReadCnt++;
	        CyU3PThreadSleep (1);
//			DelayMSec(1);
        }

        //DelayMSec(1);
//		local_read(DEVID_DPS, DPS_ADR_ADC_READ_LO, &pbytData[ReadNo++]); //Data Read
//		local_read(DEVID_DPS, DPS_ADR_ADC_READ_MID, &pbytData[ReadNo++]); //Data Read
        DPS_READ(DPS_ADR_ADC_READ_LO, &pbytData[ReadNo++]); //Data Read
        DPS_READ(DPS_ADR_ADC_READ_MID, &pbytData[ReadNo++]); //Data Read

	}
////////////////////////////////////////////////////////////////////////////////////////////////////
#endif
	return bytRet;
}

