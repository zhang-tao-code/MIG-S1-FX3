/*
 * epcs.h
 *
 *  Created on: 2012. 3. 8.
 *      Author: soyim
 */

#ifndef EPCS_H_
#define EPCS_H_

// MA6
#define WRITE_OPER_REG_B0		0xb0
#define WRITE_BUSY_REG_B1		0xb1
#define READ_OPER_REG_B2		0xb2
#define READ_DATA_REG_B3		0xb3

// MD4L6
#define AutoSkew_Weight_1		10

#define MD4L_SVR_EN_REG_0		0x00
#define MD4L_SVR_TimersCFG_REG_02	0x02
#define MD4L_SVR_TIMER_REG_10		0x10
#define MD4L_CRC_ERR_CNT_REG_15		0x15

#define MD4L_L1_RX_PKT_CNT_REG_34	0x34
#define MD4L_L2_RX_PKT_CNT_REG_35	0x35
#define MD4L_L3_RX_PKT_CNT_REG_36	0x36
#define MD4L_L4_RX_PKT_CNT_REG_37	0x37

#define MD4L_L1_RX_SOT_CNT_REG_38	0x38
#define MD4L_L2_RX_SOT_CNT_REG_39	0x39
#define MD4L_L3_RX_SOT_CNT_REG_3A	0x3A
#define MD4L_L4_RX_SOT_CNT_REG_3B	0x3B

#define MD4L_L12_SETTLE_CNT_REG_3C	0x3C
#define MD4L_L34_SETTLE_CNT_REG_3D	0x3D
#define MD4L_PCLK_CNT_REG_3E	    0x3E

#define MD4L_WRITE_OPER_REG_49		0x49
#define MD4L_WRITE_BUSY_REG_49		0x49
#define MD4L_READ_OPER_REG_4A		0x4a
#define MD4L_READ_DATA_REG_4B		0x4b

#define MD4L_SKEW_DELAY_REG_51		0x51

//MD4DSI
#define MD4DSI_SLAVE_ADD 			0x8c
#define MD4DSI_WRIT_OPER_REG_B0		0xb0
#define MD4DSI_WRITE_BUSY_REG_B1 	0xb1
#define MD4DSI_READ_OPER_REG_B2 	0xb2
#define MD4DSI_READ_DATA_REG_B3 	0xb3

#define TimeOut  2000

uint8_t epcs_write(uint8_t dev_id, uint8_t *pbytData, uint16_t length);
uint8_t epcs_read(uint8_t dev_id, uint8_t *pbytData, uint16_t length);

uint8_t md4l_write(uint8_t *pbytData, uint16_t length);
uint8_t md4l_read(uint8_t *pbytData, uint16_t length);

uint8_t md4dsi_write(uint8_t *pbytData , uint16_t length);
uint8_t md4dsi_read(uint8_t *pbytData, uint16_t length);

#endif /* EPCS_H_ */
