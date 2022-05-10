#include <cyu3system.h>
#include <cyu3os.h>
#include <cyu3error.h>
#include "pca6416a.h"
#include "cyu3i2c.h"
#include "Hal_i2c.h"

PCA6416A_t pca6416;
void timerCb()
{
	set_pca6416a(ESET_DPS,CyTrue);
	CyU3PTimerStop (&pca6416.resetTimer);
}

CyU3PReturnStatus_t init_pca6416a(void)
{
	uint8_t value = 0;
	CyU3PReturnStatus_t apiRetStatus = CY_U3P_SUCCESS;
	/*PORT0 as output     */
	apiRetStatus = HAL_I2CWrite(PCA6416_SLAVE_ADDR, PCA6416_REG_CFG_PORT0,1,&value);
	value = 0xFF;
	/*PORT0 output as 0xFF*/
	apiRetStatus |= HAL_I2CWrite(PCA6416_SLAVE_ADDR, PCA6416_REG_OUT_PORT0,1,&value);
	pca6416.port0 = 0xFF;
	pca6416.state = DPSRST_ST_NORMAL;
	pca6416.duration = 0;

    /* Create a timer with 100 ms expiry to enable/disable LPM transitions */ 
    CyU3PTimerCreate (&pca6416.resetTimer, timerCb, 0, 100, 0, CYU3P_NO_ACTIVATE);
	
	return apiRetStatus;
}

CyU3PReturnStatus_t set_pca6416a(uint8_t pinMask, CyBool_t state)
{
	CyU3PReturnStatus_t apiRetStatus = CY_U3P_SUCCESS;

	pca6416.port0 = state ? (pca6416.port0|pinMask):
								(pca6416.port0&(~pinMask));
	apiRetStatus = HAL_I2CWrite(PCA6416_SLAVE_ADDR, PCA6416_REG_OUT_PORT0,1,&pca6416.port0);
	return apiRetStatus;
}

CyU3PReturnStatus_t reset_dps(uint16_t duration)
{
	CyU3PReturnStatus_t apiRetStatus = set_pca6416a(ESET_DPS,CyFalse);
	pca6416.state = DPSRST_ST_RESET;
	pca6416.duration = duration;
	CyU3PTimerStop (&pca6416.resetTimer);
	CyU3PTimerModify(&pca6416.resetTimer, pca6416.duration, 0);
	CyU3PTimerStart(&pca6416.resetTimer);
	return apiRetStatus;
}
