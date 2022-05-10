/*
 * dps.h
 *
 *  Created on: 2012. 3. 8.
 *      Author: soyim
 */

#ifndef DPS_H_
#define DPS_H_

#define PMU_DEF_DEVID		0x06
#define PMU_DEF_DEVID2		0x07
#define PMU_ID_VALUE		0xaa

// Register Map
#define PMU_REG_ID			0x00
#define PMU_REG_REV			0x01
#define PMU_REG_IO			0x02
#define PMU_REG_SEL_DAC		0x03
#define PMU_REG_STATE		0x04

#define PMU_BLOCK_PMU		0x10
#define PMU_BLOCK_DAC		0x20
#define PMU_BLOCK_ADC		0x30

#define PMU_RIDX_STATE		0x00
#define PMU_RIDX_CLEAR		0x01
#define PMU_RIDX_WRITE		0x02
#define PMU_RIDX_READ		0x03
#define PMU_RIDX_WBYTE_L	0x04
#define PMU_RIDX_WBYTE_H	0x05
#define PMU_RIDX_RBYTE_L	0x06
#define PMU_RIDX_RBYTE_H	0x07


// for PMU_REG_IO
#define PMU_BIT_LED4		0x08
#define PMU_BIT_LED3		0x04
#define PMU_BIT_SEL_INA		0x02
#define PMU_BIT_SEL_CUR		0x01

// for PMU_REG_SEL_DAC
#define PMU_BIT_SEL_THRESHOLD	0x00
#define PMU_BIT_SEL_INVOLT		0x01
#define PMU_BIT_SEL_CLAMP		0x02

// for PMU_REG_STATE
#define PMU_BIT_DUT_HA			0x02
#define PMU_BIT_DUT_LA			0x01

// for PMU_RIDX_STATE
#define PMU_BIT_STATE_DONE	0x01
#define PMU_BIT_STATE_READY	0x02



#define DPS_ADR_MISC_VERSION		0x00
#define DPS_ADR_MISC_CHIP_ID		0x01
#define DPS_ADR_MISC_CONTROL		0x02
#define DPS_ADR_MISC_PWR_CHK		0x03

#define DPS_ADR_DPS				0x10
#define DPS_ADR_DAC				0x20
#define DPS_ADR_ADC				0x30

#define DPS_IDX_WRITE_START		0x00		//	#define DPS_ADR_DPS_WRITE_START		0x10	//	#define DPS_ADR_DAC_WRITE_START		0x20
#define DPS_IDX_WRITE_LO		0x01		//	#define DPS_ADR_DPS_WRITE_LO		0x11	//	#define DPS_ADR_DAC_WRITE_LO		0x21
#define DPS_IDX_WRITE_MID		0x02		//	#define DPS_ADR_DPS_WRITE_MID		0x12    //  #define DPS_ADR_DAC_WRITE_MID		0x22
#define DPS_IDX_WRITE_HI		0x03		//	#define DPS_ADR_DPS_WRITE_HI		0x13    //  #define DPS_ADR_DAC_WRITE_HI		0x23
#define DPS_IDX_READ_LO			0x04		//	#define DPS_ADR_DPS_READ_LO			0x14    //  #define DPS_ADR_DAC_READ_LO			0x24
#define DPS_IDX_READ_MID		0x05		//	#define DPS_ADR_DPS_READ_MID		0x15    //  #define DPS_ADR_DAC_READ_MID		0x25
#define DPS_IDX_READ_HI			0x06		//	#define DPS_ADR_DPS_READ_HI			0x16    //  #define DPS_ADR_DAC_READ_HI			0x26
#define DPS_IDX_CLR				0x07		//	                                            //  #define DPS_ADR_DAC_CLR				0x27
#define DPS_IDX_READ_START		0x08		//	#define DPS_ADR_DPS_READ_START		0x18    //  #define DPS_ADR_DAC_READ_START		0x28
#define DPS_IDX_STATE			0x09		//	#define DPS_ADR_DPS_STATE			0x19    //  #define DPS_ADR_DAC_STATE			0x29

#define DPS20_IDX_DAC_ADC_CONTROL		0x0A
#define DPS20_IDX_ADC_NUM_OF_MEASURE	0x0B
#define DPS20_IDX_ADC_INTERVAL_TIME_LSB	0x0C
#define DPS20_IDX_ADC_INTERVAL_TIME_MID	0x0D
#define DPS20_IDX_ADC_INTERVAL_TIME_MSB	0x0E
#define DPS20_IDX_ADC_READ_MEASURED		0x0F

#define DPS20_BIT_BUFFER_WRITE_DONE     0x10
#define DPS20_BIT_BUFFER_READ_DONE      0x20
#define DPS20_IDX_ADC_NUM_OF_MEASURE    0x0B
#define DPS20_IDX_ADC_INTERVAL_TIME_LSB 0x0C
#define DPS20_IDX_ADC_INTERVAL_TIME_MID 0x0D
#define DPS20_IDX_ADC_INTERVAL_TIME_MSB 0x0E
#define DPS20_IDX_ADC_READ_MEASURED     0x0F


#define DPS_ADR_ADC_READ_START		0x30
#define DPS_ADR_ADC_READ_LO			0x34
#define DPS_ADR_ADC_READ_MID		0x35
#define DPS_ADR_ADC_STATE			0x39
#define DPS_ADR_ADC_READ_RUN		0x3A


#define DEVID_DPS					0x02
#define DPS_CHIP_ID					0x02
#define DPS_BIT_STATE_DONE			0x01
#define DPS_BIT_STATE_READY			0x02

#define DPS_CHANNEL_PER_MODULE		2
#define DPS_MAX_MODULE				3

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



// Bit Field Register Structure
// LSB first

/*
typedef union{
	BYTE byValue;
	struct{
		BYTE	DPS_CS_Sel		:	1;	// bit 0
		BYTE	DPS_DIN_Sel		:	1;	// bit 1
		BYTE	ADC_DIN_Sel		:	1;	// bit 2
		BYTE	ADC_VOLT_Sel	:	1;	// bit 3
	}bitmap;
}DPS_MISC_CONTROL_REG;

#define DPS_MEASURE_MODE_VOLT		TRUE
#define DPS_MEASURE_MODE_CURRENT	FALSE

typedef union{
	BYTE byValue;
	struct{
		BYTE	nCH0_LO_Limit	:	1;	// bit 0
		BYTE	nCH1_LO_Limit	:	1;	// bit 1
		BYTE	NC				:	2;	// bit 2 ~ 3
		BYTE	nCH0_HI_Limit	:	1;	// bit 4
		BYTE	nCH1_HI_Limit	:	1;	// bit 5
	}bitmap;
}DPS_MISC_PWR_CHECK_REG;

#define DPS_PWR_LIMIT_LO_0	0x01
#define DPS_PWR_LIMIT_LO_1	0x02
#define DPS_PWR_LIMIT_HI_0	0x10
#define DPS_PWR_LIMIT_HI_1	0x20
*/

/*
typedef union{
	unsigned long dwValue;
	unsigned char byValue[4];
	struct{
		unsigned long	LoadControl			:	2;		// bit 1, 0
		unsigned long	BypassCapSel		:	2;		// bit 3, 2
		unsigned long	LeadCompCapSel		:	2;		// bit 5, 4
		unsigned long	notComparatorHighZ	:	1;		// bit 6
		unsigned long	notMeasureHighZ		:	1;		// bit 7
		unsigned long	notForceHighZ		:	1;		// bit 8
		unsigned long	RESERV_ZERO			:	1;		// bit 9
		unsigned long	ClampEnable			:	1;		// bit 10
		unsigned long	RangeSelect			:	3;		// bit 13, 12, 11
		unsigned long	GainAndPolarity		:	3;		// bit 16, 15, 14
		unsigned long	FMODE				:	1;		// bit 17
	}bitmap;
}DPS_DPS_CONTROL_REG;
*/

#define DPS_DPS_LOAD_NOP		0x00
#define DPS_DPS_LOAD_DPS_REG	0x01
#define DPS_DPS_LOAD_INPUT_REG	0x02
#define DPS_DPS_LOAD_BOTH_REG	0x03

#define DPS_DPS_BCOMP_NONE		0x00
#define DPS_DPS_BCOMP_CB1		0x01
#define DPS_DPS_BCOMP_CB2		0x02
#define DPS_DPS_BCOMP_CB3		0x03

#define DPS_DPS_LCOMP_NONE		0x00
#define DPS_DPS_LCOMP_CCOMP1	0x01	// Min 27pF ~ DPS_DPS 330pF
#define DPS_DPS_LCOMP_CCOMP2	0x02	// Min 27pF ~ DPS_DPS 330pF
#define DPS_DPS_LCOMP_BOTH		0x03	// Min 27pF ~ DPS_DPS 330pF each

#define DPS_DPS_RANGE_D			0x00
#define DPS_DPS_RANGE_C			0x01
#define DPS_DPS_RANGE_B			0x02
#define DPS_DPS_RANGE_A			0x03
#define DPS_DPS_RANGE_EXT		0x04
#define DPS_NUM_CUR_RANGE		4		// (Range_D to Range_A)
#define DPS_VOLT_ON_MAX_CUR		4.0


#define DPS_DPS_VGA_P1			0x00	// +1
#define DPS_DPS_VGA_P2			0x01	// +2
#define DPS_DPS_VGA_P6			0x02	// +6
#define DPS_DPS_VGA_N1			0x04	// -1
#define DPS_DPS_VGA_N2			0x05	// -2
#define DPS_DPS_VGA_N6			0x06	// -6

#define DPS_DPS_FV_MODE			0x00
#define DPS_DPS_FI_MODE			0x01


#define DPS_DAC_BIT_READCOM		0x40
#define DPS_DAC_BIT_WRITCOM		0x00

/*
typedef union{
	unsigned long dwValue;
	unsigned char byValue[4];
	struct{
		unsigned long InputDataWord	:	14;		// bit 13 ~ 0

		unsigned long SelectReg		:	2;		// bit 15, 14
		unsigned long InputChannel	:	4;		// bit 19, 18, 17, 16
		unsigned long DummyZero		:	2;		// bit 21, 20
		unsigned long ReadNotWrite	:	1;		// bit 22
		unsigned long SelNotAorB	:	1;		// bit 23
	}DataReg;
	struct{
		unsigned long DontCare				:	2;	// CR 1 ~ 0
		unsigned long Gropu0FunctionEnable	:	1;	// CR 2
		unsigned long Group1FunctionEnable	:	1;	// CR 3
		unsigned long DontCare2				:	4;	// CR 7 ~ 4
		unsigned long ThermalMonitorEnable	:	1;	// CR 8
		unsigned long ChannelMonitorEnable	:	1;	// CR 9
		unsigned long InternalReference		:	1;	// CR 10 (if 1, InternalRef used / if 0, ExternalRef used)
		unsigned long CurrentBoostOn		:	1;	// CR 11
		unsigned long InternalRefrence2_5V	:	1;	// CR 12 (if 1, Ref=2.5V / if 0, Ref=1.25V)
		unsigned long PowerDownState		:	1;	// CR 13 (if 0, Output is 100KOhm to Gnd / if 1, Output is High-Z)

		unsigned long SelectReg		:	2;		// bit 15, 14
		unsigned long InputChannel	:	4;		// bit 19, 18, 17, 16
		unsigned long DummyZero		:	2;		// bit 21, 20
		unsigned long ReadNotWrite	:	1;		// bit 22
		unsigned long SelNotAorB	:	1;		// bit 23
	}ControlReg;
}DPS_DAC_CONTROL_REG;
*/

// for DAC_AD5392_CONTROL_REG.InputChannel
#define DPS_DAC_MAX_CHANNEL			8
#define DPS_DAC_CH_DPS_OUTPUT_0			0
#define DPS_DAC_CH_VOLT_CLAMP_ON_FI_0	1
#define DPS_DAC_CH_CURRENT_LIMIT_HI_0	2
#define DPS_DAC_CH_CURRENT_LIMIT_LO_0	3
#define DPS_DAC_CH_DPS_OUTPUT_1			4
#define DPS_DAC_CH_CURRENT_LIMIT_HI_1	5
#define DPS_DAC_CH_VOLT_CLAMP_ON_FI_1	6
#define DPS_DAC_CH_CURRENT_LIMIT_LO_1	7

// for DAC_AD5392_CONTROL_REG.SelectReg
#define	AD5392_SEL_SFR				0x00
#define	AD5392_SEL_GAIN_REG			0x01
#define	AD5392_SEL_OFFSET_REG		0x02
#define	AD5392_SEL_INPUT_DATA_REG	0x03

#define	AD5392_SFR_NOP				0x00
#define	AD5392_SFR_WRITE_CLR_CODE	0x01
#define	AD5392_SFR_SOFT_CLR			0x02
#define	AD5392_SFR_SOFT_POWER_DOWN	0x08
#define	AD5392_SFR_SOFT_POWER_UP	0x09
#define	AD5392_SFR_CONTROL_REG		0x0c
#define	AD5392_SFR_MONITOR_CHANNEL	0x0a
#define	AD5392_SFR_SOFT_RESET		0x0f



#define DPS_DAC_RES		(0x0001<<14)
#define DPS_DAC_VREF	2.5
#define DPS_GET_DAC_CODE(V)	( ( V * DPS_DAC_RES ) / (2 * DPS_DAC_VREF))






// ADC AD7367 (2channelx2 14bit)
/*
typedef union{
	unsigned char byValue[2];
	unsigned short wValue;
	struct{
		short int		Value	:	14;
		unsigned short	Zero	:	2;
	}Bipolar;
	struct{
		unsigned short	Value	:	14;
		unsigned short	Zero	:	2;
	}Unipolar;
}DPS_ADC_CODE_REG;
*/

#define DPS_ADC_RES	(0x0001<<14)
#define DPS_ADC_FSR	(20.0)		// +/-10V
#define DPS_ADC_LSB	(DPS_ADC_FSR / DPS_ADC_RES)

#define DPS_GET_ADC_VOLT(C)		(C * DPS_ADC_LSB)


#define UCI40_MIN_POWER_LEVEL 0.0
#define UCI40_MAX_POWER_LEVEL 5.0

#define UCI40_MIN_LIMIT_CURRENT	0.0			//0
#define UCI40_MAX_LIMIT_CURRENT 0.2			//200mA

#define POWER_OFF_STATE		0
#define POWER_ON_STATE		1

//////////////////////////////////////////////////////////////////////////////////////
#define DPS_SETTING_HIGH_D17	0x02		//Mode Select	0
#define DPS_SETTING_HIGH_D16	0x01		//G2			0

#define DPS_SETTING_MID_D15		0x80		//G1			0
#define DPS_SETTING_MID_D14		0x40		//G0			0
#define DPS_SETTING_MID_D13		0x20		//RS2
#define DPS_SETTING_MID_D12		0x10		//RS1

#define DPS_SETTING_MID_D11		0x08		//RS0
#define DPS_SETTING_MID_D10		0x04		//Clamp Enable
#define DPS_SETTING_MID_D9		0x02		//Reserved.
#define DPS_SETTING_MID_D8		0x01		//Force High-Impedance Select

#define DPS_SETTING_LOW_D7		0x80		//Measure High-Impedance Select
#define DPS_SETTING_LOW_D6		0x40		//Comparator High-Impedance Select
#define DPS_SETTING_LOW_D5		0x20		//Compensation Select
#define DPS_SETTING_LOW_D4		0x10		//Compensation Select

#define DPS_SETTING_LOW_D3		0x08		//Compensation Select
#define DPS_SETTING_LOW_D2		0x04		//Compensation Select
#define DPS_SETTING_LOW_D1		0x02		//Serial Interface Data Flow Control
#define DPS_SETTING_LOW_D0		0x01
//////////////////////////////////////////////////////////////////////////////////////
#define DPS_SETTING_MODESELECT	0x02		//Mode Select	0
#define DPS_SETTING_G2			0x01		//G2			0

#define DPS_SETTING_G1			0x80		//G1			0
#define DPS_SETTING_G0			0x40		//G0			0
#define DPS_SETTING_RS2			0x20		//RS2
#define DPS_SETTING_RS1			0x10		//RS1

#define DPS_SETTING_RS0			0x08		//RS0
#define DPS_SETTING_CLAMPENABLE	0x04		//Clamp Enable
#define DPS_SETTING_RESV		0x02		//Reserved.
#define DPS_SETTING_FHIS		0x01		//Force High-Impedance Select

#define DPS_SETTING_MHIS		0x80		//Measure High-Impedance Select
#define DPS_SETTING_COMPHIS		0x40		//Comparator High-Impedance Select
#define DPS_SETTING_COMP_SEL4	0x20		//Compensation Select
#define DPS_SETTING_COMP_SEL3	0x10		//Compensation Select

#define DPS_SETTING_COMP_SEL2	0x08		//Compensation Select
#define DPS_SETTING_COMP_SEL1	0x04		//Compensation Select
#define DPS_SETTING_SIDFC1		0x02		//Serial Interface Data Flow Control
#define DPS_SETTING_SIDFC0		0x01
///////////////////////////////////////////////////////////////////////////////////////

#define DPS_PWR_CHK_REG_D0		0x01		//PWR Fault LowByte Ch1(0dd)
#define DPS_PWR_CHK_REG_D1		0x02		//PWR Fault LowByte Ch2(0dd)
#define DPS_PWR_CHK_REG_D4		0x10		//PWR Fault HighByte Ch1(0dd)
#define DPS_PWR_CHK_REG_D5		0x20		//PWR Fault HighByte Ch2(0dd)

#define DPS_PWR_CHK_SHORT		0x00
#define DPS_PWR_CHK_PASS		0x01

#define DPS_PWR_RESULT_CH1_PASS	0x01
#define DPS_PWR_RESULT_CH2_PASS	0x02
#define DPS_PWR_RESULT_CH3_PASS	0x04
#define DPS_PWR_RESULT_CH4_PASS	0x08
#define DPS_PWR_RESULT_CH5_PASS	0x10
#define DPS_PWR_RESULT_CH6_PASS	0x20


#define DPS_ADC_MEAS_SEL_VOLT	0x08
#define DPS_ADC_MEAS_SEL_CURR	0x00


#define DPS_BIT_STATE_READY		0x02


typedef struct CyU3P_dword_t
{
    uint8_t  byte[4];
} CyU3P_dword_t;

typedef struct CyU3P_word_t
{
    uint8_t  byte[2];
} CyU3P_word_t;


CyBool_t PMU_ReadWord(uint8_t DevId, uint8_t bytBlock, uint16_t *pwValue);
CyBool_t PMU_WriteWord(uint8_t DevId, uint8_t bytBlock, uint16_t wValue);
CyBool_t DPS_ReadDWord(uint8_t bytBlock, uint32_t *pdwValue);
CyBool_t DPS_WriteDWord(uint8_t bytBlock, uint32_t dwValue);

#endif /* DPS_H_ */
