/*
 * dps.c
 *
 *  Created on: 2012. 3. 8.
 *      Author: soyim
 */

#include "MIG-S1.h"
#include "cyu3error.h"
#include <cyu3gpio.h>
#include "dps.h"
#include "err_code.h"

//	Local constants
#define MAX_WAIT_FOR_PMU_REG	20
#define MAX_WAIT_FOR_DPS_REG	10




//	Global (Export) Functions	-----------------------------------------------------------------------
CyBool_t PMU_ReadWord(uint8_t DevId, uint8_t bytBlock, uint16_t *pwValue)
{
	CyBool_t bRet = CyFalse;
//	int nCnt = 0;
//	uint8_t read_data = 0;
//
//	local_write(DevId, bytBlock | PMU_RIDX_STATE, 0); // Clear State
//	//	DelayMSec(1);
//	local_write(DevId, bytBlock | PMU_RIDX_READ, 0); // Proceed Read
//
//	do{
//		local_read(DevId, bytBlock | PMU_RIDX_STATE, &read_data);
//		if( read_data & PMU_BIT_STATE_READY )		// Check State Machine is Ready
//			bRet = CyTrue;
//	}while( (bRet != CyTrue) && (nCnt++ < MAX_WAIT_FOR_PMU_REG) );
//
//	if(bRet)
//	{
//		local_read(DevId, bytBlock | PMU_RIDX_RBYTE_L, &read_data);
//		((CyU3P_word_t *)pwValue)->byte[0] = read_data;		// Lo Byte
//		local_read(DevId, bytBlock | PMU_RIDX_RBYTE_H, &read_data);
//		((CyU3P_word_t *)pwValue)->byte[1] = read_data;		// Hi Byte
//	}

	return bRet;
}

CyBool_t PMU_WriteWord(uint8_t DevId, uint8_t bytBlock, uint16_t wValue)
{
	CyBool_t bRet = CyFalse;
//	int nCnt = 0;
//	uint8_t read_data = 0;
//
//	local_write(DevId, bytBlock | PMU_RIDX_STATE, 0); // Clear State
//	local_write(DevId, bytBlock | PMU_RIDX_WBYTE_L, ((CyU3P_word_t *)&wValue)->byte[0] ); // Lo Byte
//	local_write(DevId, bytBlock | PMU_RIDX_WBYTE_H, ((CyU3P_word_t *)&wValue)->byte[1] ); // Hi Byte
//	local_write(DevId, bytBlock | PMU_RIDX_WRITE, 0 ); // Proceed Write
//
//	do
//	{
//		local_read(DevId, bytBlock | PMU_RIDX_STATE, &read_data);
//		if( read_data & PMU_BIT_STATE_READY )		// Check State Machine is Ready
//			bRet = CyTrue;
//	}while( (bRet != CyTrue) && (nCnt++ < MAX_WAIT_FOR_PMU_REG) );

	return bRet;
}

CyBool_t DPS_ReadDWord(uint8_t bytBlock, uint32_t *pdwValue)
{
	CyBool_t bRet = CyFalse;
//	int nCnt = 0;
//	uint8_t read_data = 0;
//
//	if(bytBlock != DPS_ADR_ADC)		// for DPS_ADR_DPS , DPS_ADR_DAC
//	{
//		local_write(DEVID_DPS, bytBlock + DPS_IDX_STATE, 0); // Clear State
//		local_write(DEVID_DPS, bytBlock + DPS_IDX_READ_START, 0); //	Proceed Read
//		do{
//			local_read(DEVID_DPS, bytBlock + DPS_IDX_STATE, &read_data);
//			if( read_data & DPS_BIT_STATE_DONE )
//				bRet = CyTrue;
//		}while( (bRet!=CyTrue) && (nCnt++ < MAX_WAIT_FOR_DPS_REG) );
//
//		if(bRet)
//		{
//			local_read(DEVID_DPS, bytBlock | DPS_IDX_READ_LO, &read_data);
//			((CyU3P_dword_t *)pdwValue)->byte[0]	=	read_data;
//			local_read(DEVID_DPS, bytBlock | DPS_IDX_READ_MID, &read_data);
//			((CyU3P_dword_t *)pdwValue)->byte[1]	=	read_data;
//			local_read(DEVID_DPS, bytBlock | DPS_IDX_READ_HI, &read_data);
//			((CyU3P_dword_t *)pdwValue)->byte[2]	=	read_data;
//			((CyU3P_dword_t *)pdwValue)->byte[3]	=	0;
//		}
//	}
//	else							// for DPS_ADR_ADC
//	{
//		local_write(DEVID_DPS, bytBlock + DPS_IDX_STATE, 0); //	Clear State
//		local_write(DEVID_DPS, DPS_ADR_ADC_READ_START, 0); //	Proceed Read
//		do{
//			local_read(DEVID_DPS, bytBlock + DPS_IDX_STATE, &read_data);
//			if( read_data & DPS_BIT_STATE_DONE )
//				bRet = CyTrue;
//		}while( (bRet!=CyTrue) && (nCnt++ < MAX_WAIT_FOR_DPS_REG) );
//
//		if(bRet)
//		{
//			local_read(DEVID_DPS, bytBlock + DPS_IDX_READ_LO, &read_data);
//			((CyU3P_dword_t *)pdwValue)->byte[0]	=	read_data;
//			local_read(DEVID_DPS, bytBlock + DPS_IDX_READ_MID, &read_data);
//			((CyU3P_dword_t *)pdwValue)->byte[1]	=	read_data;
//			((CyU3P_dword_t *)pdwValue)->byte[2]	=	0;
//			((CyU3P_dword_t *)pdwValue)->byte[3]	=	0;
//		}
//	}

	return bRet;
}

CyBool_t DPS_WriteDWord(uint8_t bytBlock, uint32_t dwValue)
{
	CyBool_t bRet = CyFalse;
//	int nCnt = 0;
//	uint8_t read_data = 0;
//
//	local_write(DEVID_DPS, bytBlock + DPS_IDX_STATE, 0); //	Clear State
//	local_write(DEVID_DPS, bytBlock + DPS_IDX_WRITE_LO, ((CyU3P_dword_t *)&dwValue)->byte[0]); //	Lo Byte
//	local_write(DEVID_DPS, bytBlock + DPS_IDX_WRITE_MID, ((CyU3P_dword_t *)&dwValue)->byte[1]); //	Mid Byte
//	local_write(DEVID_DPS, bytBlock + DPS_IDX_WRITE_HI, ((CyU3P_dword_t *)&dwValue)->byte[2]); // 	Hi Byte
//	local_write(DEVID_DPS, bytBlock + DPS_IDX_WRITE_START, 0); //	Proceed Write
//
//	do{
//		local_read(DEVID_DPS, bytBlock + DPS_IDX_STATE, &read_data);
//		if( read_data & DPS_BIT_STATE_DONE )
//			bRet = CyTrue;
//	}while( (bRet!=CyTrue) && (nCnt++ < MAX_WAIT_FOR_DPS_REG) );

	return bRet;
}
//	Local Functions				----------------------------------------------------------------- to here

