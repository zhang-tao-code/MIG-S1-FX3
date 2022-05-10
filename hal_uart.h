#ifndef _HAL_UART_H
#define _HAL_UART_H


void HAL_UartInt_Handler (void);
CyU3PReturnStatus_t HAL_UartInit (void);
CyU3PReturnStatus_t HAL_UartDeInit(void);
CyU3PReturnStatus_t HAL_UartSetConfig ( CyU3PUartConfig_t *config,CyU3PUartIntrCb_t cb);
CyU3PReturnStatus_t HAL_UartTxSetBlockXfer (uint32_t txSize);
CyU3PReturnStatus_t HAL_UartRxSetBlockXfer (uint32_t rxSize);
CyU3PReturnStatus_t HAL_UartSetTimeout (uint32_t readLoopCnt, uint32_t writeLoopCnt);
uint32_t HAL_UartTransmitBytes (uint8_t *data_p,uint32_t count,CyU3PReturnStatus_t *status);
uint32_t HAL_UartReceiveBytes ( uint8_t *data_p,uint32_t count,CyU3PReturnStatus_t *status);

#endif
