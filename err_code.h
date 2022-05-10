/*
 * err_code.h
 *
 *  Created on: 2012. 3. 8.
 *      Author: soyim
 */

#ifndef ERR_CODE_H_
#define ERR_CODE_H_

// Error Code
//
#define err_OK					0

#define err_MD4L_AutoSkew_Timeout_0		0xE0
#define err_MD4L_AutoSkewfail			0xE1
#define err_MD4L_SettleTime_Timeout_0	0xE2
#define err_MD4L_SettleTime_fail		0xE3

#define err_I2C_INIT			0xF1
#define err_I2C_NOACK			0xF2
#define err_I2C_TRANSMIT		0xF3
#define err_I2C_RECEIVE			0xF4

#define err_I2C_WAITREADY		0xF5
#define err_I2C_WAITDONE		0xF6
#define err_I2C_NOACK_DEVADR	0xF7
#define err_I2C_NOACK_STARTREG	0xF8
#define err_I2C_CANTSTART		0xF9

#define err_I2C_NOTMATCH		0xFA  //(J.J 2011.06.10) Add

#define err_WRONG_PARAM			0xFB

#define err_EPCS_WAIT			0xFC
#define err_ILL_WRITE			0xFD

#define err_USB_COMMUNICATION	0xFE  //(J.J 2011.06.13) Add

/*
#define err_WRONG_PARAM			0xE0

#define err_EPCS_WAIT			0xD0
#define err_ILL_WRITE			0xD1
*/
uint8_t g_ErrCode;

#define SET_ERRCODE(E)		( (E==err_OK) ? CyTrue : !(g_ErrCode=E) )
#define GET_ERRORCODE()		(g_ErrCode)
#define CLEAR_ERRORCODE()	(g_ErrCode=err_OK)


#endif /* ERR_CODE_H_ */
