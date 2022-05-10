#include <cyu3system.h>
#include <cyu3os.h>
#include <cyu3error.h>
#include <cyu3uart.h>
#include "uart.h"
#include <uart_regs.h>
#include "hal_uart.h"

CyU3PThread UartThread;	                       /* UART Example application thread structure */
CyU3PEvent  UartEvent;                         /* Event flags group used by the application. */

UARTControl * uartController;
static uint32_t errorCounter = 0;
static uint32_t times=0;

#define LOW_LEVEL_FIFO_SIZE 16
#define WAIT_ACK_TIME_OUT   3     /*1ms*/
#define WAIT_FPGA_OPERATION 10    /*10ms*/
#define WAIT_ACK_EVENTS    (UART_EVT_DATA_RX_ACK_ERR|UART_EVT_DATA_RX_ACK_OK)
#define MAX_RETRY_TIMES     3

static void send_request_fpga(uint8_t *buf,uint32_t len, CyU3PReturnStatus_t * status);
static void send_fpga_reset_cmd();
static void prepare_frame_head(uint8_t cmd,uint16_t len);
static uint8_t calcaulet_frame_checksum(uint8_t *buffer,uint32_t len);



static void uart_Callback (CyU3PUartEvt_t   evType,CyU3PUartError_t evStatus)
{
    uint8_t  rxData[LOW_LEVEL_FIFO_SIZE];
    uint32_t count;
	uint32_t intMask;
    CyU3PReturnStatus_t status;
    static uint32_t j;
	if (evType == CY_U3P_UART_EVENT_TX_DONE)
	{
		j=1;
	}
	if (evType == CY_U3P_UART_EVENT_ERROR)
	{
		j=UART->lpp_uart_status;
	}
	if (evType == CY_U3P_UART_EVENT_RX_DONE)
	{
		j=3;
	}	
    if (evType == CY_U3P_UART_EVENT_RX_DATA)
    {
        do
        {
            /* Keep reading the incoming data into the buffer, as long as we have space. */
            count = HAL_UartReceiveBytes (rxData, LOW_LEVEL_FIFO_SIZE, &status);

            /* head   counter type length DATA0 ... DATAn checksum	tail  */
			/*0xeb90  0xYY    0xYY 0xYYYY 0xYY      0xYY  0xYY 	   0x131b */
			//intMask = CyU3PVicDisableAllInterrupts();

			//if((uartController->rx_index+count) < MAX_BUFER_SIZE && count>0)
			for(j=0;j<count;j++)
			{
				/*eg. EB 90 AB 71 00 01 20 B9 13 1B */
				uartController->rx_buffer[uartController->rx_index++]=rxData[j];
				//CyU3PMemCopy(&uartController->rx_buffer[uartController->rx_index],rxData,count);
				//uartController->rx_index+=count;

			}//end of if(uartController->rx_index < MAX_UFFER_SIZE)

			//CyU3PVicEnableInterrupts(intMask);
        } while (count > 0 );

		/*
		if( uartController->rx_index>=6 &&
			uartController->head_received==CyFalse &&
			uartController->rx_buffer[uartController->rx_index-6]==FPGA_FX3_HEAD1 &&
			uartController->rx_buffer[uartController->rx_index-5]==FPGA_FX3_HEAD2 )
		{
			uartController->length_expected  = ((uint32_t)uartController->rx_buffer[uartController->rx_index-2])<<8;
			uartController->length_expected += uartController->rx_buffer[uartController->rx_index-1]+9;//checksum and tail
			uartController->head_index = uartController->rx_index-6;
			uartController->head_received = CyTrue;
		}

		if( (uartController->rx_index-uartController->head_index)>=uartController->length_expected )
		{
			if( uartController->head_received && 
				uartController->rx_buffer[uartController->rx_index-2] == FPGA_FX3_TAIL1 &&
				uartController->rx_buffer[uartController->rx_index-1] == FPGA_FX3_TAIL2 )
			{
			    if(uartController->rx_buffer[uartController->head_index+3]==DATA_RX_ACK)
			    {
					uartController->ack_msg.checksum = uartController->rx_buffer[uartController->rx_index-3];
					uartController->ack_msg.type 	= uartController->rx_buffer[uartController->head_index+3];
					uartController->ack_msg.length	= uartController->length_expected-9;
					uartController->ack_msg.counter	= uartController->rx_buffer[uartController->head_index+2];
					uartController->ack_msg.data    = uartController->rx_buffer[uartController->head_index+6];
					CyU3PEventSet (&UartEvent, UART_EVT_ACK_FRAME, CYU3P_EVENT_OR);
				}
				else
				{
					uartController->rx_msg.checksum = uartController->rx_buffer[uartController->rx_index-3];
					uartController->rx_msg.type 	= uartController->rx_buffer[uartController->head_index+3];
					uartController->rx_msg.length	= uartController->length_expected-9;
					uartController->rx_msg.counter	= uartController->rx_buffer[uartController->head_index+2];
					CyU3PMemCopy(uartController->rx_msg.data,&uartController->rx_buffer[uartController->head_index+6],uartController->length_expected-9);
					CyU3PEventSet (&UartEvent, UART_EVT_RESPOND_FRAME, CYU3P_EVENT_OR);
				}						
			}//end of if(uartController->head_received)
			times++;						
			uartController->rx_index = 0;
			uartController->head_index = 0;
			uartController->length_expected = MIN_FRAME_LENGTH;
			uartController->head_received = CyFalse;						
		}//end of if( (uartController->rx_index-uartController->head_index)>=	
		*/
    }
}

void UartAppErrorHandler (CyU3PReturnStatus_t apiRetStatus )
{
    /* Application failed with the error code apiRetStatus */

    /* Add custom debug or recovery actions here */

    /* Loop Indefinitely */
    for (;;)
    {
        /* Thread sleep : 100 ms */
        CyU3PThreadSleep (100);
    }
}

int Test_reg_map_write(uint8_t address ,uint8_t value)
{
	CyU3PReturnStatus_t status = CY_U3P_SUCCESS;
    uint32_t j;
    uint16_t length = 2;
	uint32_t retry = 0;
	uint32_t evStat;
	CyBool_t acked=CyFalse;
		
	prepare_frame_head(REG_MAP_WR_REQ,length);
	
	j=6;
	uartController->tx_buffer[j++]=address;
	uartController->tx_buffer[j++]=value;

	uartController->tx_buffer[j]  = calcaulet_frame_checksum(uartController->tx_buffer,j);
	j++;
	uartController->tx_buffer[j++]= FX3_FPGA_TAIL1;
	uartController->tx_buffer[j++]= FX3_FPGA_TAIL2;

	send_request_fpga(uartController->tx_buffer, j, &status);

	
	if(!acked && retry>=MAX_RETRY_TIMES)
	{
		send_fpga_reset_cmd();
	}
	CyU3PThreadSleep(100);

	return acked?FPGA_ACTION_OK:FPGA_ACTION_FAIL;
}


void init_uart(void)
{
    CyU3PUartConfig_t uartConfig;
    CyU3PReturnStatus_t apiRetStatus = CY_U3P_SUCCESS;
	uartController = CyU3PDmaBufferAlloc (sizeof(UARTControl));
	if(uartController==NULL)
	{
		return;
	}

    /* Initialize the UART module */
    apiRetStatus = HAL_UartInit();
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
        /* Error handling */
        UartAppErrorHandler(apiRetStatus);
    }

    CyU3PMemSet ((uint8_t *)&uartConfig, 0, sizeof(uartConfig));
    uartConfig.baudRate = CY_U3P_UART_BAUDRATE_921600;
    uartConfig.stopBit  = CY_U3P_UART_ONE_STOP_BIT;
    uartConfig.parity   = CY_U3P_UART_NO_PARITY;
    uartConfig.flowCtrl = CyFalse;
    uartConfig.txEnable = CyTrue;
    uartConfig.rxEnable = CyTrue;
    uartConfig.isDma = CyFalse; /* Register mode */

    /* Set the UART configuration */
    apiRetStatus = HAL_UartSetConfig (&uartConfig, uart_Callback);
    if (apiRetStatus != CY_U3P_SUCCESS )
    {
        /* Error handling */
        UartAppErrorHandler(apiRetStatus);
    }
    apiRetStatus = CyU3PEventCreate (&UartEvent);
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
        /* Error handling */
        UartAppErrorHandler(apiRetStatus);
    }

	CyU3PMemSet((uint8_t*)uartController,0,sizeof(UARTControl));
	uartController->length_expected = MIN_FRAME_LENGTH;
	uartController->head_received   = CyFalse;
	uartController->rx_index        = 0;
	uartController->head_index      = 0;
	uartController->rx_msg.head1    = FPGA_FX3_HEAD1;
	uartController->rx_msg.head2    = FPGA_FX3_HEAD2;
	uartController->rx_msg.tail1    = FPGA_FX3_TAIL1;
	uartController->rx_msg.tail2    = FPGA_FX3_TAIL2;
	
	uartController->ack_msg.head1   = FPGA_FX3_HEAD1;
	uartController->ack_msg.head2   = FPGA_FX3_HEAD2;
}

static CyBool_t isChecksumOk()
{
	int i=0;
	uint16_t length;
	uint8_t checksumCalculated = 0;
	if(uartController->rx_index>=MIN_ACK_RSP_FRAMES_LENGTH)
	{ 
		for(i=ACK_FRAME_LENGTH;i<uartController->rx_index-3;i++)
		{
			checksumCalculated += uartController->rx_buffer[i]; 	
		}
	}

	if(checksumCalculated==uartController->rx_buffer[uartController->rx_index-3])
	{
		uartController->rx_msg.type = uartController->rx_buffer[ACK_FRAME_LENGTH+3];
		length = uartController->rx_buffer[ACK_FRAME_LENGTH+4];
		length = length<<8;
		length = uartController->rx_buffer[ACK_FRAME_LENGTH+5];
		CyU3PMemCopy(uartController->rx_msg.data,&uartController->rx_buffer[ACK_FRAME_LENGTH+6],length);
		return CyTrue;
	}
	else
		return CyFalse;
}

static CyBool_t isAckChecksumOk()
{
#if 0
	int i=0;
	uint8_t *pData=(uint8_t *)&uartController->ack_msg;
	uint8_t checksumCalculated = 0;
	for(i=0;i<uartController->ack_msg.length+6;i++)
	{
		checksumCalculated += pData[i];
	}
	if(checksumCalculated==uartController->ack_msg.checksum)
		return CyTrue;
	else
		return CyFalse;
#endif
	if( uartController->rx_buffer[0]==FPGA_FX3_HEAD1 &&
		uartController->rx_buffer[1]==FPGA_FX3_HEAD2 &&
		uartController->rx_buffer[8]==FPGA_FX3_TAIL1 &&
		uartController->rx_buffer[9]==FPGA_FX3_TAIL2 &&
		uartController->rx_buffer[3]==DATA_RX_ACK)
		return CyTrue;
	else
		return CyFalse;
}


#define THREAD_WAIT_EVENTS (UART_EVT_ACK_FRAME|UART_EVT_RESPOND_FRAME)
void UartThread_Entry ( uint32_t input)
{
    CyU3PReturnStatus_t status = CY_U3P_SUCCESS;
    uint32_t            evStat;
	CyBool_t isFrameOk = CyFalse;


    /* We set a timeout of 1 for receive data and 50000 for transmit data.
     * Then wait until the callback notifies us of incoming data.
     * Once all of the data has been fetched, we loop the data back on the transmit side.
     */
    init_uart();
    HAL_UartSetTimeout(1, 10000);
	
    for (;;)
    {
        status = CyU3PEventGet (&UartEvent, THREAD_WAIT_EVENTS, CYU3P_EVENT_OR_CLEAR, &evStat,
                5);//CYU3P_WAIT_FOREVER

        if (status == CY_U3P_SUCCESS)
        {
			/*Interpreting UART protocol here*/

			if(evStat&UART_EVT_ACK_FRAME )
			{
				if(isAckChecksumOk())
				{
					if(uartController->rx_buffer[6]==0)
						CyU3PEventSet (&UartEvent, UART_EVT_DATA_RX_ACK_OK, CYU3P_EVENT_OR);
					else
						CyU3PEventSet (&UartEvent, UART_EVT_DATA_RX_ACK_ERR, CYU3P_EVENT_OR);
				}
			}
			if(evStat&UART_EVT_RESPOND_FRAME )
			{
				isFrameOk = isChecksumOk();
				if(isFrameOk)
				{
					switch(uartController->rx_msg.type)
					{	
						case IIC0_WR_RSP:
						CyU3PEventSet (&UartEvent, UART_EVT_IIC0_WR_RSP, CYU3P_EVENT_OR);	
						break;	
						
						case IIC1_WR_RSP:
						CyU3PEventSet (&UartEvent, UART_EVT_IIC1_WR_RSP, CYU3P_EVENT_OR);	
						break;	
						
						case IIC0_RD_RSP:
						CyU3PEventSet (&UartEvent, UART_EVT_IIC0_RD_RSP, CYU3P_EVENT_OR);	
						break;
						
						case IIC1_RD_RSP:
						CyU3PEventSet (&UartEvent, UART_EVT_IIC1_RD_RSP, CYU3P_EVENT_OR);	
						break;

						case REG_MAP_RD_RSP:
						CyU3PEventSet (&UartEvent, UART_EVT_REG_MAP_RD_RSP, CYU3P_EVENT_OR);	
						break;

						case IIC0_RA8D8_ONE_BYTE_RSP:
						CyU3PEventSet (&UartEvent, UART_EVT_IIC0_RA8D8_ONE_BYTE, CYU3P_EVENT_OR);	
						break;

						case IIC0_RA8D8_CNT_BYTES_RSP:
						CyU3PEventSet (&UartEvent, UART_EVT_IIC0_RA8D8_CNT_BYTES, CYU3P_EVENT_OR);	
						break;
						
						case IIC0_RA8D16_ONE_WORD_RSP:
						CyU3PEventSet (&UartEvent, UART_EVT_IIC0_RA8D16_ONE_WORD, CYU3P_EVENT_OR);	
						break;

						case IIC0_RA8D16_CNT_WORDS_RSP:
						CyU3PEventSet (&UartEvent, UART_EVT_IIC0_RA8D16_CNT_WORDS, CYU3P_EVENT_OR);	
						break;
						
						case IIC0_RA16D8_ONE_BYTE_RSP:
						CyU3PEventSet (&UartEvent, UART_EVT_IIC0_RA16D8_ONE_BYTE, CYU3P_EVENT_OR);	
						break;
						
						case IIC0_RA16D8_CNT_BYTES_RSP:
						CyU3PEventSet (&UartEvent, UART_EVT_IIC0_RA16D8_CNT_BYTES, CYU3P_EVENT_OR);	
						break;
						
						case IIC0_RA16D16_ONE_WORD_RSP:
						CyU3PEventSet (&UartEvent, UART_EVT_IIC0_RA16D16_ONE_WORD, CYU3P_EVENT_OR);	
						break;
						
						case IIC0_RA16D16_CNT_WORDS_RSP:
						CyU3PEventSet (&UartEvent, UART_EVT_IIC0_RA16D16_CNT_WORDS, CYU3P_EVENT_OR);	
						break;
						
						case IIC1_RA8D8_ONE_BYTE_RSP:
						CyU3PEventSet (&UartEvent, UART_EVT_IIC1_RA8D8_ONE_BYTE, CYU3P_EVENT_OR);	
						break;

						case IIC1_RA8D8_CNT_BYTES_RSP:
						CyU3PEventSet (&UartEvent, UART_EVT_IIC1_RA8D8_CNT_BYTES, CYU3P_EVENT_OR);	
						break;
						
						case IIC1_RA8D16_ONE_WORD_RSP:
						CyU3PEventSet (&UartEvent, UART_EVT_IIC1_RA8D16_ONE_WORD, CYU3P_EVENT_OR);	
						break;

						case IIC1_RA8D16_CNT_WORDS_RSP:
						CyU3PEventSet (&UartEvent, UART_EVT_IIC1_RA8D16_CNT_WORDS, CYU3P_EVENT_OR);	
						break;
						
						case IIC1_RA16D8_ONE_BYTE_RSP:
						CyU3PEventSet (&UartEvent, UART_EVT_IIC1_RA16D8_ONE_BYTE, CYU3P_EVENT_OR);	
						break;
						
						case IIC1_RA16D8_CNT_BYTES_RSP:
						CyU3PEventSet (&UartEvent, UART_EVT_IIC1_RA16D8_CNT_BYTES, CYU3P_EVENT_OR);	
						break;
						
						case IIC1_RA16D16_ONE_WORD_RSP:
						CyU3PEventSet (&UartEvent, UART_EVT_IIC1_RA16D16_ONE_WORD, CYU3P_EVENT_OR);	
						break;
						
						case IIC1_RA16D16_CNT_WORDS_RSP:
						CyU3PEventSet (&UartEvent, UART_EVT_IIC1_RA16D16_CNT_WORDS, CYU3P_EVENT_OR);	
						break;

					    case DATA_RX_ACK:
						default:
						break;					
					}
				}
			}
        }
    }
}

static void prepare_frame_head(uint8_t cmd,uint16_t len)
{
    U16_CNV length;
	length.value = len;
	uartController->tx_buffer[0]=FX3_FPGA_HEAD1;
	uartController->tx_buffer[1]=FX3_FPGA_HEAD2;
	
	//uartController->tx_buffer[2]++;
	uartController->tx_buffer[3]=cmd;
	uartController->tx_buffer[4]=length.raw[1];
	uartController->tx_buffer[5]=length.raw[0];
}

static uint8_t calcaulet_frame_checksum(uint8_t *buffer,uint32_t len)
{
    uint8_t checksum = 0;
	uint32_t i;
	for(i=0;i<len;i++)
		checksum += buffer[i];

	return checksum;
}

static void send_fpga_reset_cmd()
{
	//return 0;
}

static void send_request_fpga(uint8_t *buf,uint32_t len, CyU3PReturnStatus_t * status)
{
	uint32_t intMask;
	uartController->rx_index = 0;
	uartController->head_index = 0;
	uartController->length_expected = MIN_FRAME_LENGTH;
	uartController->head_received = CyFalse;
	uartController->ackEventSent  = CyFalse;
	
	CyU3PMemSet (uartController->rx_buffer, 0, MAX_BUFER_SIZE);

	buf[2]++;
	buf[len-3] = calcaulet_frame_checksum(buf,len-3);
	HAL_UartTransmitBytes (buf, len, &status);
}

int random_A8D8_write_FPGAI2C(E_FPGA_IIC ch,FPGA_IIC_A8D8_REG_t *pRegTable, uint32_t number,uint8_t *op_result) 
{
	CyU3PReturnStatus_t status = CY_U3P_SUCCESS;
    uint32_t i,j;
    uint16_t length = number*sizeof(FPGA_IIC_A8D8_REG_t);
	uint32_t retry = 0;
	uint32_t evStat;
	CyBool_t acked=CyFalse;
	uint32_t timeut=(9*number*3*10+350)/1000+1;//100kbps I2C speed
	uint8_t  cmd;
	uint32_t waitEvent=(ch==FPGA_IIC0)?UART_EVT_IIC0_WR_RSP:UART_EVT_IIC1_WR_RSP;
	uint32_t tick;
	uint32_t cur_tick;
	tick = CyU3PGetTime();
	if(ch>=FPGA_IIC_LAST)
		return FPGA_ACTION_INVALID_IIC;
	cmd = ch==FPGA_IIC0?IIC0_WA8D8_ONE_BYTE:IIC1_WA8D8_ONE_BYTE;
	if(number*sizeof(FPGA_IIC_A8D8_REG_t)>MAX_PAYLOAD_SIZE)
	{
		return FPGA_ACTION_WR_LONG;
	}
	prepare_frame_head(cmd,length);
	j=6;
	for(i=0;i<number;i++)
	{
		uartController->tx_buffer[j++]=pRegTable[i].deviceAddress;
		uartController->tx_buffer[j++]=pRegTable[i].regAddress;
		uartController->tx_buffer[j++]=pRegTable[i].data;
	}
	uartController->tx_buffer[j]  = calcaulet_frame_checksum(uartController->tx_buffer,j);
	j++;
	uartController->tx_buffer[j++]= FX3_FPGA_TAIL1;
	uartController->tx_buffer[j++]= FX3_FPGA_TAIL2;
	do
	{
		send_request_fpga(uartController->tx_buffer, j, &status);
		cur_tick = CyU3PGetTime();
		retry++;
		if(status==CY_U3P_SUCCESS)
		{
	        status = CyU3PEventGet (&UartEvent,
				                    WAIT_ACK_EVENTS, 
				                    CYU3P_EVENT_OR_CLEAR, 
				                    &evStat,
				                    WAIT_ACK_TIME_OUT);	
			if(status == CY_U3P_SUCCESS && (evStat&UART_EVT_DATA_RX_ACK_ERR))
			{
				acked = CyFalse;
			}
			else if(status == CY_U3P_SUCCESS && (evStat&UART_EVT_DATA_RX_ACK_OK))
			{
				acked = CyTrue;
			}
		}
	}
	while(retry<MAX_RETRY_TIMES && !acked);

	
	if(!acked && retry>=MAX_RETRY_TIMES)
	{
		send_fpga_reset_cmd();
		return FPGA_ACTION_FAIL;
	}
	
	status = CyU3PEventGet (&UartEvent,
							waitEvent, 
							CYU3P_EVENT_OR_CLEAR, 
							&evStat,
							timeut);

	if(status==CY_U3P_SUCCESS && (evStat&waitEvent))
	{
		*op_result = uartController->rx_msg.data[0];
		return FPGA_ACTION_OK;
	}

	return FPGA_ACTION_FAIL;
}

int continuous_A8D8_write_FPGAI2C(E_FPGA_IIC ch,FPGA_IIC_A8D8_REG_t *pRegTable, uint32_t number,uint8_t *op_result) 
{
	CyU3PReturnStatus_t status = CY_U3P_SUCCESS;
    uint32_t i,j;
    uint16_t length = number+2+1+1;//2bytes data number,1 byte device address,1 register address
	uint32_t retry = 0;
	uint32_t evStat;
	CyBool_t acked=CyFalse;
	uint32_t timeut=(9*number*3*10+350)/1000+1;//100kbps I2C speed
	U16_CNV  reg_num;	
	uint8_t  cmd;
	uint32_t waitEvent=(ch==FPGA_IIC0)?UART_EVT_IIC0_WR_RSP:UART_EVT_IIC1_WR_RSP;

	if(ch>=FPGA_IIC_LAST)
		return FPGA_ACTION_INVALID_IIC;	

	if(number*sizeof(FPGA_IIC_A8D8_REG_t)>MAX_PAYLOAD_SIZE)
	{
		return FPGA_ACTION_WR_LONG;
	}
	
	cmd = ch==FPGA_IIC0?IIC0_WA8D8_CNT_BYTES:IIC1_WA8D8_CNT_BYTES;
	prepare_frame_head(cmd,length);
	j=6;
	/*
	data0 = data number (high 8bit)
	data1 = data number (low 8bit)
	data2 = device address 
	data3 = register address
	data4 = register data0
	data5 = register data1 
    ......
	*/
	reg_num.value = number;
	uartController->tx_buffer[j++] = reg_num.raw[1];
	uartController->tx_buffer[j++] = reg_num.raw[0];
	uartController->tx_buffer[j++] = pRegTable[0].deviceAddress;
	uartController->tx_buffer[j++] = pRegTable[0].regAddress;
	for(i=0;i<number;i++)
	{
		uartController->tx_buffer[j++]=pRegTable[i].data;	
	}
	uartController->tx_buffer[j]  = calcaulet_frame_checksum(uartController->tx_buffer,j);
	j++;
	uartController->tx_buffer[j++]= FX3_FPGA_TAIL1;
	uartController->tx_buffer[j++]= FX3_FPGA_TAIL2;
	do
	{
		send_request_fpga(uartController->tx_buffer, j, &status);
		retry++;
		if(status==CY_U3P_SUCCESS)
		{
	        status = CyU3PEventGet (&UartEvent,
				                    WAIT_ACK_EVENTS, 
				                    CYU3P_EVENT_OR_CLEAR, 
				                    &evStat,
				                    WAIT_ACK_TIME_OUT);	
			if(status == CY_U3P_SUCCESS && (evStat&UART_EVT_DATA_RX_ACK_ERR))
			{

			}
			else if(status == CY_U3P_SUCCESS && (evStat&UART_EVT_DATA_RX_ACK_OK))
			{
				acked = CyTrue;
			}
		}
	}
	while(retry<MAX_RETRY_TIMES && !acked);

	if(!acked && retry>=MAX_RETRY_TIMES)
	{
		send_fpga_reset_cmd();
		return FPGA_ACTION_FAIL;
	}
	
	status = CyU3PEventGet (&UartEvent,
							waitEvent, 
							CYU3P_EVENT_OR_CLEAR, 
							&evStat,
							timeut);

	if(status==CY_U3P_SUCCESS && (evStat&waitEvent))
	{
		*op_result = uartController->rx_msg.data[0];
		return FPGA_ACTION_OK;
	}

	return FPGA_ACTION_FAIL;
}

int random_A8D16_write_FPGAI2C(E_FPGA_IIC ch,FPGA_IIC_A8D16_REG_t *pRegTable, uint32_t number,uint8_t *op_result) 
{
	CyU3PReturnStatus_t status = CY_U3P_SUCCESS;
    uint32_t i,j;
    uint16_t length = number*sizeof(FPGA_IIC_A8D16_REG_t);
	uint32_t retry = 0;
	uint32_t evStat;
	CyBool_t acked=CyFalse;
	uint32_t timeut=(9*number*4*10+350)/1000+1;//100kbps I2C speed
	uint8_t  cmd;	
	uint32_t waitEvent=(ch==FPGA_IIC0)?UART_EVT_IIC0_WR_RSP:UART_EVT_IIC1_WR_RSP;
	
	if(ch>=FPGA_IIC_LAST)
		return FPGA_ACTION_INVALID_IIC;
	cmd = ch==FPGA_IIC0?IIC0_WA8D16_ONE_WORD:IIC1_WA8D16_ONE_WORD;

	if(number*sizeof(FPGA_IIC_A8D8_REG_t)>MAX_PAYLOAD_SIZE)
	{
		return FPGA_ACTION_WR_LONG;
	}
	prepare_frame_head(cmd,length);
	j=6;
	for(i=0;i<number;i++)
	{
		uartController->tx_buffer[j++]=pRegTable[i].deviceAddress;
		uartController->tx_buffer[j++]=pRegTable[i].regAddress;
		uartController->tx_buffer[j++]=pRegTable[i].data.raw[1];		
		uartController->tx_buffer[j++]=pRegTable[i].data.raw[0];			
	}
	uartController->tx_buffer[j]  = calcaulet_frame_checksum(uartController->tx_buffer,j);
	j++;
	uartController->tx_buffer[j++]= FX3_FPGA_TAIL1;
	uartController->tx_buffer[j++]= FX3_FPGA_TAIL2;
	do
	{
		send_request_fpga(uartController->tx_buffer, j, &status);
		retry++;
		if(status==CY_U3P_SUCCESS)
		{
	        status = CyU3PEventGet (&UartEvent,
				                    WAIT_ACK_EVENTS, 
				                    CYU3P_EVENT_OR_CLEAR, 
				                    &evStat,
				                    WAIT_ACK_TIME_OUT);	
			if(status == CY_U3P_SUCCESS && (evStat&UART_EVT_DATA_RX_ACK_ERR))
			{

			}
			else if(status == CY_U3P_SUCCESS && (evStat&UART_EVT_DATA_RX_ACK_OK))
			{
				acked = CyTrue;
			}
		}
	}
	while(retry<MAX_RETRY_TIMES && !acked);

	if(!acked && retry>=MAX_RETRY_TIMES)
	{
		send_fpga_reset_cmd();
		return FPGA_ACTION_FAIL;
	}
	
	status = CyU3PEventGet (&UartEvent,
							waitEvent, 
							CYU3P_EVENT_OR_CLEAR, 
							&evStat,
							timeut);

	if(status==CY_U3P_SUCCESS && (evStat&waitEvent))
	{
		*op_result = uartController->rx_msg.data[0];
		return FPGA_ACTION_OK;
	}

	return FPGA_ACTION_FAIL;
}

int continuous_A8D16_write_FPGAI2C(E_FPGA_IIC ch,FPGA_IIC_A8D16_REG_t *pRegTable, uint32_t number,uint8_t *op_result) 
{
	CyU3PReturnStatus_t status = CY_U3P_SUCCESS;
    uint32_t i,j;
    uint16_t length = number*2+2+1+1;
	uint32_t retry = 0;
	uint32_t evStat;
	CyBool_t acked=CyFalse;
	uint32_t timeut=(9*number*4*10+350)/1000+1;//100kbps I2C speed
	U16_CNV  reg_num;	
	uint8_t  cmd;
	uint32_t waitEvent=(ch==FPGA_IIC0)?UART_EVT_IIC0_WR_RSP:UART_EVT_IIC1_WR_RSP;
	
	if(ch>=FPGA_IIC_LAST)
		return FPGA_ACTION_INVALID_IIC;
	
	if(number*sizeof(FPGA_IIC_A8D8_REG_t)>MAX_PAYLOAD_SIZE)
	{
		return FPGA_ACTION_WR_LONG;
	}
	cmd = ch==FPGA_IIC0?IIC0_WA8D16_CNT_WORDS:IIC1_WA8D16_CNT_WORDS;
	prepare_frame_head(cmd,length);
	j=6;
	/*
	data0 = data number (high 8bit) 
	data1 = data number (low 8bit) 
	data2 = device address 
	data3 = register address
	data4 = register data0 (high 8bit)
	data5 = register data0 (low 8bit)
	data6 = register data1 (high 8bit)
	data7 = register data1 (low 8bit)
	...........
	*/
	reg_num.value = number;
	uartController->tx_buffer[j++] = reg_num.raw[1];
	uartController->tx_buffer[j++] = reg_num.raw[0];
	uartController->tx_buffer[j++] = pRegTable[1].deviceAddress;
	uartController->tx_buffer[j++] = pRegTable[0].regAddress;	
	for(i=0;i<number;i++)
	{
		uartController->tx_buffer[j++]=pRegTable[i].data.raw[1];		
		uartController->tx_buffer[j++]=pRegTable[i].data.raw[0];			
	}
	uartController->tx_buffer[j]  = calcaulet_frame_checksum(uartController->tx_buffer,j);
	j++;
	uartController->tx_buffer[j++]= FX3_FPGA_TAIL1;
	uartController->tx_buffer[j++]= FX3_FPGA_TAIL2;
	do
	{
		send_request_fpga(uartController->tx_buffer, j, &status);
		retry++;
		if(status==CY_U3P_SUCCESS)
		{
	        status = CyU3PEventGet (&UartEvent,
				                    WAIT_ACK_EVENTS, 
				                    CYU3P_EVENT_OR_CLEAR, 
				                    &evStat,
				                    WAIT_ACK_TIME_OUT);	
			if(status == CY_U3P_SUCCESS && (evStat&UART_EVT_DATA_RX_ACK_ERR))
			{

			}
			else if(status == CY_U3P_SUCCESS && (evStat&UART_EVT_DATA_RX_ACK_OK))
			{
				acked = CyTrue;
			}
		}
	}
	while(retry<MAX_RETRY_TIMES && !acked);

	if(!acked && retry>=MAX_RETRY_TIMES)
	{
		send_fpga_reset_cmd();
		return FPGA_ACTION_FAIL;
	}
	
	status = CyU3PEventGet (&UartEvent,
							waitEvent, 
							CYU3P_EVENT_OR_CLEAR, 
							&evStat,
							timeut);

	if(status==CY_U3P_SUCCESS && (evStat&waitEvent))
	{
		*op_result = uartController->rx_msg.data[0];
		return FPGA_ACTION_OK;
	}

	return FPGA_ACTION_FAIL;
}

int random_A16D8_write_FPGAI2C(E_FPGA_IIC ch,FPGA_IIC_A16D8_REG_t *pRegTable, uint32_t number,uint8_t *op_result) 
{
	CyU3PReturnStatus_t status = CY_U3P_SUCCESS;
    uint32_t i,j;
    uint16_t length = number*sizeof(FPGA_IIC_A8D16_REG_t);
	uint32_t retry = 0;
	uint32_t evStat;
	CyBool_t acked=CyFalse;
	uint32_t timeut=(9*number*4*10+350)/1000+1;//100kbps I2C speed
	uint8_t  cmd;
	uint32_t waitEvent=(ch==FPGA_IIC0)?UART_EVT_IIC0_WR_RSP:UART_EVT_IIC1_WR_RSP;
	
	if(ch>=FPGA_IIC_LAST)
		return FPGA_ACTION_INVALID_IIC;
	cmd = ch==FPGA_IIC0?IIC0_WA16D8_ONE_BYTE:IIC1_WA16D8_ONE_BYTE;

	if(number*sizeof(FPGA_IIC_A8D8_REG_t)>MAX_PAYLOAD_SIZE)
	{
		return FPGA_ACTION_WR_LONG;
	}
	prepare_frame_head(cmd,length);
	j=6;
	for(i=0;i<number;i++)
	{
		uartController->tx_buffer[j++]=pRegTable[i].deviceAddress;
		uartController->tx_buffer[j++]=pRegTable[i].regAddress.raw[1];
		uartController->tx_buffer[j++]=pRegTable[i].regAddress.raw[0];
		uartController->tx_buffer[j++]=pRegTable[i].data;			
	}
	uartController->tx_buffer[j]  = calcaulet_frame_checksum(uartController->tx_buffer,j);
	j++;
	uartController->tx_buffer[j++]= FX3_FPGA_TAIL1;
	uartController->tx_buffer[j++]= FX3_FPGA_TAIL2;
	do
	{
		send_request_fpga(uartController->tx_buffer, j, &status);
		retry++;
		if(status==CY_U3P_SUCCESS)
		{
	        status = CyU3PEventGet (&UartEvent,
				                    WAIT_ACK_EVENTS, 
				                    CYU3P_EVENT_OR_CLEAR, 
				                    &evStat,
				                    WAIT_ACK_TIME_OUT);	
			if(status == CY_U3P_SUCCESS && (evStat&UART_EVT_DATA_RX_ACK_ERR))
			{
				acked = CyFalse;
			}
			else if(status == CY_U3P_SUCCESS && (evStat&UART_EVT_DATA_RX_ACK_OK))
			{
				acked = CyTrue;
			}
		}
	}
	while(retry<MAX_RETRY_TIMES && !acked);

	if(!acked && retry>=MAX_RETRY_TIMES)
	{
		send_fpga_reset_cmd();
		return FPGA_ACTION_FAIL;
	}
	
	status = CyU3PEventGet (&UartEvent,
							waitEvent, 
							CYU3P_EVENT_OR_CLEAR, 
							&evStat,
							timeut);

	if(status==CY_U3P_SUCCESS && (evStat&waitEvent))
	{
	    *op_result = uartController->rx_msg.data[0];
		return FPGA_ACTION_OK;
	}

	return FPGA_ACTION_FAIL;
}

int continuous_A16D8_write_FPGAI2C(E_FPGA_IIC ch,FPGA_IIC_A16D8_REG_t *pRegTable, uint32_t number,uint8_t *op_result) 
{
	CyU3PReturnStatus_t status = CY_U3P_SUCCESS;
    uint32_t i,j;
    uint16_t length = number+2+1+2;
	uint32_t retry = 0;
	uint32_t evStat;
	CyBool_t acked=CyFalse;
	uint32_t timeut=(9*number*2*10+350)/1000+1;//100kbps I2C speed
	U16_CNV  reg_num;
	uint8_t  cmd;
	uint32_t waitEvent=(ch==FPGA_IIC0)?UART_EVT_IIC0_WR_RSP:UART_EVT_IIC1_WR_RSP;
	
	if(ch>=FPGA_IIC_LAST)
		return FPGA_ACTION_INVALID_IIC;
	
	if(number*sizeof(FPGA_IIC_A8D8_REG_t)>MAX_PAYLOAD_SIZE)
	{
		return FPGA_ACTION_WR_LONG;
	}

	cmd = ch==FPGA_IIC0?IIC0_WA8D16_CNT_WORDS:IIC1_WA8D16_CNT_WORDS;
	prepare_frame_head(cmd,length);
	j=6;
	/*
	data0 = data number (high 8bit)
	data1 = data number (low 8bit)	
	data2 = device address 
	data3 = register address (high 8bit)
	data4 = register address (low 8bit)
	data5 = register data0 
	data6 = register data1 
	data7 = register data2 
	...........
	*/
	reg_num.value = number;
	uartController->tx_buffer[j++] = reg_num.raw[1];
	uartController->tx_buffer[j++] = reg_num.raw[0];
	uartController->tx_buffer[j++] = pRegTable[0].deviceAddress;
	uartController->tx_buffer[j++] = pRegTable[0].regAddress.raw[1];
	uartController->tx_buffer[j++] = pRegTable[0].regAddress.raw[0];
	for(i=0;i<number;i++)
	{
		uartController->tx_buffer[j++]=pRegTable[i].data;		 			
	}
	uartController->tx_buffer[j]  = calcaulet_frame_checksum(uartController->tx_buffer,j);
	j++;
	uartController->tx_buffer[j++]= FX3_FPGA_TAIL1;
	uartController->tx_buffer[j++]= FX3_FPGA_TAIL2;
	do
	{
		send_request_fpga(uartController->tx_buffer, j, &status);
		retry++;
		if(status==CY_U3P_SUCCESS)
		{
	        status = CyU3PEventGet (&UartEvent,
				                    WAIT_ACK_EVENTS, 
				                    CYU3P_EVENT_OR_CLEAR, 
				                    &evStat,
				                    WAIT_ACK_TIME_OUT);	
			if(status == CY_U3P_SUCCESS && (evStat&UART_EVT_DATA_RX_ACK_ERR))
			{

			}
			else if(status == CY_U3P_SUCCESS && (evStat&UART_EVT_DATA_RX_ACK_OK))
			{
				acked = CyTrue;
			}
		}
	}
	while(retry<MAX_RETRY_TIMES && !acked);

	if(!acked && retry>=MAX_RETRY_TIMES)
	{
		send_fpga_reset_cmd();
		return FPGA_ACTION_FAIL;
	}
	
	status = CyU3PEventGet (&UartEvent,
							waitEvent, 
							CYU3P_EVENT_OR_CLEAR, 
							&evStat,
							timeut);

	if(status==CY_U3P_SUCCESS && (evStat&waitEvent))
	{
	    *op_result = uartController->rx_msg.data[0];
		return FPGA_ACTION_OK;
	}

	return FPGA_ACTION_FAIL;
}

int random_A16D16_write_FPGAI2C(E_FPGA_IIC ch,FPGA_IIC_A16D16_REG_t *pRegTable, uint32_t number,uint8_t *op_result) 
{
	CyU3PReturnStatus_t status = CY_U3P_SUCCESS;
    uint32_t i,j;
    uint16_t length = number*sizeof(FPGA_IIC_A16D16_REG_t);
	uint32_t retry = 0;
	uint32_t evStat;
	CyBool_t acked=CyFalse;
	uint32_t timeut=(9*number*5*10+350)/1000+1;//100kbps I2C speed
	uint8_t  cmd;
	uint32_t waitEvent=(ch==FPGA_IIC0)?UART_EVT_IIC0_WR_RSP:UART_EVT_IIC1_WR_RSP;
	
	if(ch>=FPGA_IIC_LAST)
		return FPGA_ACTION_INVALID_IIC;
	cmd = ch==FPGA_IIC0?IIC0_WA16D16_ONE_WORD:IIC1_WA16D16_ONE_WORD;

	if(number*sizeof(FPGA_IIC_A8D8_REG_t)>MAX_PAYLOAD_SIZE)
	{
		return FPGA_ACTION_WR_LONG;
	}
	prepare_frame_head(cmd,length);
	j=6;
	for(i=0;i<number;i++)
	{
		uartController->tx_buffer[j++]=pRegTable[i].deviceAddress;
		uartController->tx_buffer[j++]=pRegTable[i].regAddress.raw[1];
		uartController->tx_buffer[j++]=pRegTable[i].regAddress.raw[0];
		uartController->tx_buffer[j++]=pRegTable[i].data.raw[1];
		uartController->tx_buffer[j++]=pRegTable[i].data.raw[0];
	}
	uartController->tx_buffer[j]  = calcaulet_frame_checksum(uartController->tx_buffer,j);
	j++;
	uartController->tx_buffer[j++]= FX3_FPGA_TAIL1;
	uartController->tx_buffer[j++]= FX3_FPGA_TAIL2;
	do
	{
		send_request_fpga(uartController->tx_buffer, j, &status);
		retry++;
		if(status==CY_U3P_SUCCESS)
		{
	        status = CyU3PEventGet (&UartEvent,
				                    WAIT_ACK_EVENTS, 
				                    CYU3P_EVENT_OR_CLEAR, 
				                    &evStat,
				                    WAIT_ACK_TIME_OUT);	
			if(status == CY_U3P_SUCCESS && (evStat&UART_EVT_DATA_RX_ACK_ERR))
			{

			}
			else if(status == CY_U3P_SUCCESS && (evStat&UART_EVT_DATA_RX_ACK_OK))
			{
				acked = CyTrue;
			}
		}
	}
	while(retry<MAX_RETRY_TIMES && !acked);

	if(!acked && retry>=MAX_RETRY_TIMES)
	{
		send_fpga_reset_cmd();
		return FPGA_ACTION_FAIL;
	}
	
	status = CyU3PEventGet (&UartEvent,
							waitEvent, 
							CYU3P_EVENT_OR_CLEAR, 
							&evStat,
							timeut);

	if(status==CY_U3P_SUCCESS && (evStat&waitEvent))
	{
	    *op_result = uartController->rx_msg.data[0];	    
		return FPGA_ACTION_OK;
	}

	return FPGA_ACTION_FAIL;

}

int continuous_A16D16_write_FPGAI2C(E_FPGA_IIC ch,FPGA_IIC_A16D16_REG_t *pRegTable, uint32_t number,uint8_t *op_result) 
{
	CyU3PReturnStatus_t status = CY_U3P_SUCCESS;
    uint32_t i,j;
    uint16_t length = number+2+1+2;
	uint32_t retry = 0;
	uint32_t evStat;
	CyBool_t acked=CyFalse;
	uint32_t timeut=(9*number*2*10+350)/1000+1;//100kbps I2C speed
	U16_CNV  reg_num;
	uint8_t  cmd;
	uint32_t waitEvent=(ch==FPGA_IIC0)?UART_EVT_IIC0_WR_RSP:UART_EVT_IIC1_WR_RSP;
	
	if(ch>=FPGA_IIC_LAST)
		return FPGA_ACTION_INVALID_IIC;
	
	if(number*sizeof(FPGA_IIC_A8D8_REG_t)>MAX_PAYLOAD_SIZE)
	{
		return FPGA_ACTION_WR_LONG;
	}
	
	cmd = ch==FPGA_IIC0?IIC0_WA16D16_CNT_WORDS:IIC1_WA16D16_CNT_WORDS;
	prepare_frame_head(cmd,length);
	j=6;
	/*
	data0 = data number (high 8bit)
	data1 = data number (low 8bit)
	data2 = device address 
	data3 = register address (high 8bit)
	data4 = register address (low 8bit)
	data5 = register data0 (high 8bit)
	data6 = register data0 (low 8bit)
	data7 = register data1 (high 8bit)	
	data8 = register data1 (low 8bit)  
	...........
	*/
	reg_num.value = number;
	uartController->tx_buffer[j++] = reg_num.raw[1];
	uartController->tx_buffer[j++] = reg_num.raw[0];
	uartController->tx_buffer[j++] = pRegTable[0].deviceAddress;
	uartController->tx_buffer[j++] = pRegTable[0].regAddress.raw[1];
	uartController->tx_buffer[j++] = pRegTable[0].regAddress.raw[0];
	for(i=0;i<number;i++)
	{
		uartController->tx_buffer[j++]=pRegTable[i].data.raw[1];
		uartController->tx_buffer[j++]=pRegTable[i].data.raw[0];
	}
	uartController->tx_buffer[j]  = calcaulet_frame_checksum(uartController->tx_buffer,j);
	j++;
	uartController->tx_buffer[j++]= FX3_FPGA_TAIL1;
	uartController->tx_buffer[j++]= FX3_FPGA_TAIL2;
	do
	{
		send_request_fpga(uartController->tx_buffer, j, &status);
		retry++;
		if(status==CY_U3P_SUCCESS)
		{
	        status = CyU3PEventGet (&UartEvent,
				                    WAIT_ACK_EVENTS, 
				                    CYU3P_EVENT_OR_CLEAR, 
				                    &evStat,
				                    WAIT_ACK_TIME_OUT);	
			if(status == CY_U3P_SUCCESS && (evStat&UART_EVT_DATA_RX_ACK_ERR))
			{

			}
			else if(status == CY_U3P_SUCCESS && (evStat&UART_EVT_DATA_RX_ACK_OK))
			{
				acked = CyTrue;
			}
		}
	}
	while(retry<MAX_RETRY_TIMES && !acked);

	if(!acked && retry>=MAX_RETRY_TIMES)
	{
		send_fpga_reset_cmd();
		return FPGA_ACTION_FAIL;
	}
	
	status = CyU3PEventGet (&UartEvent,
							waitEvent, 
							CYU3P_EVENT_OR_CLEAR, 
							&evStat,
							timeut);

	if(status==CY_U3P_SUCCESS && (evStat&waitEvent))
	{
	    *op_result = uartController->rx_msg.data[0];	
		return FPGA_ACTION_OK;
	}

	return FPGA_ACTION_FAIL;
}

int random_A8D8_read_FPGAI2C(E_FPGA_IIC ch,FPGA_IIC_A8D8_REG_t *pRegTable, uint32_t number,uint8_t *op_result) 
{
	CyU3PReturnStatus_t status = CY_U3P_SUCCESS;
    uint32_t i,j;
    uint16_t length = number*(sizeof(FPGA_IIC_A8D8_REG_t)-1);//
	uint32_t retry = 0;
	uint32_t evStat;
	CyBool_t acked=CyFalse;
	uint32_t timeut=(9*number*3*10+350)/1000+1;//100kbps I2C speed
	uint8_t  cmd;
	uint32_t waitEvents=(ch==FPGA_IIC0)?
		               (UART_EVT_IIC0_RD_RSP|UART_EVT_IIC0_RA8D8_ONE_BYTE):
		               (UART_EVT_IIC1_RD_RSP|UART_EVT_IIC1_RA8D8_ONE_BYTE);
	uint32_t failEevnt=(ch==FPGA_IIC0)?
		               (UART_EVT_IIC0_RD_RSP):
		               (UART_EVT_IIC1_RD_RSP); 
	uint32_t succssEvent=(ch==FPGA_IIC0)?
		               (UART_EVT_IIC0_RA8D8_ONE_BYTE):
		               (UART_EVT_IIC1_RA8D8_ONE_BYTE); 
					   
	FPGA_IIC_A8D8_REG_t *pRegRead;
	
	if(ch>=FPGA_IIC_LAST)
		return FPGA_ACTION_INVALID_IIC;
	
	if(length>MAX_PAYLOAD_SIZE)
	{
		return FPGA_ACTION_WR_LONG;
	}

	cmd = ch==FPGA_IIC0?IIC0_RA8D8_ONE_BYTE:IIC1_RA8D8_ONE_BYTE;
	prepare_frame_head(cmd,length);
	j=6;
	for(i=0;i<number;i++)
	{
		uartController->tx_buffer[j++]=pRegTable[i].deviceAddress;
		uartController->tx_buffer[j++]=pRegTable[i].regAddress;
	}
	uartController->tx_buffer[j]  = calcaulet_frame_checksum(uartController->tx_buffer,j);
	j++;
	uartController->tx_buffer[j++]= FX3_FPGA_TAIL1;
	uartController->tx_buffer[j++]= FX3_FPGA_TAIL2;
	do
	{
		send_request_fpga(uartController->tx_buffer, j, &status);
		retry++;
		if(status==CY_U3P_SUCCESS)
		{
	        status = CyU3PEventGet (&UartEvent,
				                    WAIT_ACK_EVENTS, 
				                    CYU3P_EVENT_OR_CLEAR, 
				                    &evStat,
				                    WAIT_ACK_TIME_OUT);	
			if(status == CY_U3P_SUCCESS && (evStat&UART_EVT_DATA_RX_ACK_ERR))
			{
				acked=CyFalse;
			}
			else if(status == CY_U3P_SUCCESS && (evStat&UART_EVT_DATA_RX_ACK_OK))
			{
				acked = CyTrue;
			}
		}
	}
	while(retry<MAX_RETRY_TIMES && !acked);

	if(!acked && retry>=MAX_RETRY_TIMES)
	{
		send_fpga_reset_cmd();
		return FPGA_ACTION_FAIL;
	}
	
	status = CyU3PEventGet (&UartEvent,
							waitEvents, 
							CYU3P_EVENT_OR_CLEAR, 
							&evStat,
							timeut);

	if(status==CY_U3P_SUCCESS)// && (evStat&waitEvent))
	{
		//read fail
		if(evStat&failEevnt)
			*op_result = uartController->rx_msg.data[0];

		//read succed
		if(evStat&succssEvent)
		{
			*op_result = I2C_READ_OK;
			pRegRead = (FPGA_IIC_A8D8_REG_t *)uartController->rx_msg.data;
			for(j=0;j<number;j++)
			{//to be checked
				*pRegTable = *pRegRead;
				pRegRead++;
				pRegTable++;
			}
		}			
		return FPGA_ACTION_OK;
	}

	return FPGA_ACTION_FAIL;
}

int continuous_A8D8_read_FPGAI2C(E_FPGA_IIC ch,FPGA_IIC_A8D8_REG_t *pRegTable, uint32_t number,uint8_t *op_result) 
{
	CyU3PReturnStatus_t status = CY_U3P_SUCCESS;
    uint32_t j;
    uint16_t length = 4;//
	uint32_t retry = 0;
	uint32_t evStat;
	CyBool_t acked=CyFalse;
	uint32_t timeut=(9*number*3*10+350)/1000+1;//100kbps I2C speed
	uint8_t  cmd;
	U16_CNV  reg_num;
	uint32_t waitEvents=(ch==FPGA_IIC0)?
		               (UART_EVT_IIC0_RD_RSP|UART_EVT_IIC0_RA8D8_CNT_BYTES):
		               (UART_EVT_IIC1_RD_RSP|UART_EVT_IIC1_RA8D8_CNT_BYTES);
	uint32_t failEvent=(ch==FPGA_IIC0)?
		               (UART_EVT_IIC0_RD_RSP):
		               (UART_EVT_IIC1_RD_RSP); 
	uint32_t succssEvent=(ch==FPGA_IIC0)?
		               (UART_EVT_IIC0_RA8D8_CNT_BYTES):
		               (UART_EVT_IIC1_RA8D8_CNT_BYTES); 

	if(ch>=FPGA_IIC_LAST)
		return FPGA_ACTION_INVALID_IIC;
	cmd = ch==FPGA_IIC0?IIC0_RA8D8_CNT_BYTES:IIC1_RA8D8_CNT_BYTES;
	if(number*(sizeof(FPGA_IIC_A8D8_REG_t)-1)>MAX_PAYLOAD_SIZE)
	{
		return FPGA_ACTION_WR_LONG;
	}
	prepare_frame_head(cmd,length);
	j=6;
	reg_num.value = number;
	uartController->tx_buffer[j++] = reg_num.raw[1];
	uartController->tx_buffer[j++] = reg_num.raw[0];
	uartController->tx_buffer[j++] = pRegTable[0].deviceAddress;
	uartController->tx_buffer[j++] = pRegTable[0].regAddress;
	uartController->tx_buffer[j]  = calcaulet_frame_checksum(uartController->tx_buffer,j);
	j++;
	uartController->tx_buffer[j++]= FX3_FPGA_TAIL1;
	uartController->tx_buffer[j++]= FX3_FPGA_TAIL2;
	do
	{
		send_request_fpga(uartController->tx_buffer, j, &status);
		retry++;
		if(status==CY_U3P_SUCCESS)
		{
	        status = CyU3PEventGet (&UartEvent,
				                    WAIT_ACK_EVENTS, 
				                    CYU3P_EVENT_OR_CLEAR, 
				                    &evStat,
				                    WAIT_ACK_TIME_OUT);	
			if(status == CY_U3P_SUCCESS && (evStat&UART_EVT_DATA_RX_ACK_ERR))
			{
				acked=CyFalse;
			}
			else if(status == CY_U3P_SUCCESS && (evStat&UART_EVT_DATA_RX_ACK_OK))
			{
				acked = CyTrue;
			}
		}
	}
	while(retry<MAX_RETRY_TIMES && !acked);

	if(!acked && retry>=MAX_RETRY_TIMES)
	{
		send_fpga_reset_cmd();
		return FPGA_ACTION_FAIL;
	}
	
	status = CyU3PEventGet (&UartEvent,
							waitEvents, 
							CYU3P_EVENT_OR_CLEAR, 
							&evStat,
							timeut);

	if(status==CY_U3P_SUCCESS)// && (evStat&waitEvent))
	{
		//read fail
		if(evStat&failEvent)
			*op_result = uartController->rx_msg.data[0];

		//read succed
		if(evStat&succssEvent)
		{
			*op_result = I2C_READ_OK;
			for(j=0;j<number;j++)
			{//to be checked
				pRegTable[j].deviceAddress = uartController->rx_msg.data[0];
				pRegTable[j].regAddress    = uartController->rx_msg.data[1];
				pRegTable[j].data = uartController->rx_msg.data[2+j];
			}
		}			
		return FPGA_ACTION_OK;
	}

	return FPGA_ACTION_FAIL;
}


int random_A8D16_read_FPGAI2C(E_FPGA_IIC ch,FPGA_IIC_A8D16_REG_t *pRegTable, uint32_t number,uint8_t *op_result) 
{
	CyU3PReturnStatus_t status = CY_U3P_SUCCESS;
    uint32_t i,j;
    uint16_t length = number*(sizeof(FPGA_IIC_A8D16_REG_t)-2);//
	uint32_t retry = 0;
	uint32_t evStat;
	CyBool_t acked=CyFalse;
	uint32_t timeut=(9*number*3*10+350)/1000+1;//100kbps I2C speed
	uint8_t  cmd;
	uint32_t waitEvents=(ch==FPGA_IIC0)?
		               (UART_EVT_IIC0_RD_RSP|UART_EVT_IIC0_RA8D16_ONE_WORD):
		               (UART_EVT_IIC1_RD_RSP|UART_EVT_IIC1_RA8D16_ONE_WORD);
	uint32_t failEvent=(ch==FPGA_IIC0)?
		               (UART_EVT_IIC0_RD_RSP):
		               (UART_EVT_IIC1_RD_RSP); 
	uint32_t succssEvent=(ch==FPGA_IIC0)?
		               (UART_EVT_IIC0_RA8D16_ONE_WORD):
		               (UART_EVT_IIC1_RA8D16_ONE_WORD); 
					   
	FPGA_IIC_A8D16_REG_t *pRegRead;
	
	if(ch>=FPGA_IIC_LAST)
		return FPGA_ACTION_INVALID_IIC;
	
	if(length>MAX_PAYLOAD_SIZE)
	{
		return FPGA_ACTION_WR_LONG;
	}

	cmd = ch==FPGA_IIC0?IIC0_RA8D16_ONE_WORD:IIC1_RA8D16_ONE_WORD;
	prepare_frame_head(cmd,length);
	j=6;
	/*
	data0 = device address0 
	data1 = register address0
	data2 = device address1 
	data3 = register address1
    ........
	*/
	for(i=0;i<number;i++)
	{
		uartController->tx_buffer[j++]=pRegTable[i].deviceAddress;
		uartController->tx_buffer[j++]=pRegTable[i].regAddress;
	}
	uartController->tx_buffer[j]  = calcaulet_frame_checksum(uartController->tx_buffer,j);
	j++;
	uartController->tx_buffer[j++]= FX3_FPGA_TAIL1;
	uartController->tx_buffer[j++]= FX3_FPGA_TAIL2;
	do
	{
		send_request_fpga(uartController->tx_buffer, j, &status);
		retry++;
		if(status==CY_U3P_SUCCESS)
		{
	        status = CyU3PEventGet (&UartEvent,
				                    WAIT_ACK_EVENTS, 
				                    CYU3P_EVENT_OR_CLEAR, 
				                    &evStat,
				                    WAIT_ACK_TIME_OUT);	
			if(status == CY_U3P_SUCCESS && (evStat&UART_EVT_DATA_RX_ACK_ERR))
			{
				acked=CyFalse;
			}
			else if(status == CY_U3P_SUCCESS && (evStat&UART_EVT_DATA_RX_ACK_OK))
			{
				acked = CyTrue;
			}
		}
	}
	while(retry<MAX_RETRY_TIMES && !acked);

	if(!acked && retry>=MAX_RETRY_TIMES)
	{
		send_fpga_reset_cmd();
		return FPGA_ACTION_FAIL;
	}
	
	status = CyU3PEventGet (&UartEvent,
							waitEvents, 
							CYU3P_EVENT_OR_CLEAR, 
							&evStat,
							timeut);

	if(status==CY_U3P_SUCCESS)// && (evStat&waitEvent))
	{
		//read fail
		if(evStat&failEvent)
			*op_result = uartController->rx_msg.data[0];

		//read succed
		if(evStat&succssEvent)
		{
			*op_result = I2C_READ_OK;
			pRegRead = (FPGA_IIC_A8D16_REG_t *)uartController->rx_msg.data;
			/*
			data1 = device address0 
			data2 = register address0
			data3 = register data0 (high 8bit)
			data4 = register data0 (low 8bit)
			data5 = device address1
			data6 = register address1
			data7 = register data1 (high 8bit)
			data8 = register data1 (low 8bit)
			*/
			for(j=0;j<number;j++)
			{//to be checked
				*pRegTable = *pRegRead;
				pRegRead++;
				pRegTable++;
			}
		}			
		return FPGA_ACTION_OK;
	}

	return FPGA_ACTION_FAIL;
}

int continuous_A8D16_read_FPGAI2C(E_FPGA_IIC ch,FPGA_IIC_A8D16_REG_t *pRegTable, uint32_t number,uint8_t *op_result) 
{
	CyU3PReturnStatus_t status = CY_U3P_SUCCESS;
    uint32_t i,j;
    uint16_t length = 4;//
	uint32_t retry = 0;
	uint32_t evStat;
	CyBool_t acked=CyFalse;
	uint32_t timeut=(9*number*3*10+350)/1000+1;//100kbps I2C speed
	uint8_t  cmd;
	U16_CNV  reg_num;
	uint32_t waitEvents=(ch==FPGA_IIC0)?
		               (UART_EVT_IIC0_RD_RSP|UART_EVT_IIC0_RA8D16_CNT_WORDS):
		               (UART_EVT_IIC1_RD_RSP|UART_EVT_IIC1_RA8D16_CNT_WORDS);
	uint32_t failMask=(ch==FPGA_IIC0)?
		               (UART_EVT_IIC0_RD_RSP):
		               (UART_EVT_IIC1_RD_RSP); 
	uint32_t succssMask=(ch==FPGA_IIC0)?
		               (UART_EVT_IIC0_RA8D16_CNT_WORDS):
		               (UART_EVT_IIC1_RA8D16_CNT_WORDS); 
					   
	
	if(ch>=FPGA_IIC_LAST)
		return FPGA_ACTION_INVALID_IIC;
	
	if(length>MAX_PAYLOAD_SIZE)
	{
		return FPGA_ACTION_WR_LONG;
	}
	
	cmd = ch==FPGA_IIC0?IIC0_RA8D16_CNT_WORDS:IIC1_RA8D16_CNT_WORDS;
	prepare_frame_head(cmd,length);
	j=6;
	reg_num.value = number;
	/*
	data0 = data number (high 8bit)
	data1 = data number (low 8bit)
	data2 = device address 
	data3 = register address
	*/
	uartController->tx_buffer[j++] = reg_num.raw[1];
	uartController->tx_buffer[j++] = reg_num.raw[0];
	uartController->tx_buffer[j++] = pRegTable[0].deviceAddress;
	uartController->tx_buffer[j++] = pRegTable[0].regAddress;
	uartController->tx_buffer[j]  = calcaulet_frame_checksum(uartController->tx_buffer,j);
	j++;
	uartController->tx_buffer[j++]= FX3_FPGA_TAIL1;
	uartController->tx_buffer[j++]= FX3_FPGA_TAIL2;
	do
	{
		send_request_fpga(uartController->tx_buffer, j, &status);
		retry++;
		if(status==CY_U3P_SUCCESS)
		{
	        status = CyU3PEventGet (&UartEvent,
				                    WAIT_ACK_EVENTS, 
				                    CYU3P_EVENT_OR_CLEAR, 
				                    &evStat,
				                    WAIT_ACK_TIME_OUT);	
			if(status == CY_U3P_SUCCESS && (evStat&UART_EVT_DATA_RX_ACK_ERR))
			{
				acked=CyFalse;
			}
			else if(status == CY_U3P_SUCCESS && (evStat&UART_EVT_DATA_RX_ACK_OK))
			{
				acked = CyTrue;
			}
		}
	}
	while(retry<MAX_RETRY_TIMES && !acked);

	if(!acked && retry>=MAX_RETRY_TIMES)
	{
		send_fpga_reset_cmd();
		return FPGA_ACTION_FAIL;
	}
	
	status = CyU3PEventGet (&UartEvent,
							waitEvents, 
							CYU3P_EVENT_OR_CLEAR, 
							&evStat,
							timeut);

	if(status==CY_U3P_SUCCESS)// && (evStat&waitEvent))
	{
		//read fail
		if(evStat&failMask)
			*op_result = uartController->rx_msg.data[0];

		//read succed
		/*
		data0 = device address 
		data1 = register address
		data2 = register data0 (high 8bit)
		data3 = register data0 (low 8bit)
		data4 = register data1 (high 8bit)
		data5 = register data1 (low 8bit)
		*/
		if(evStat&succssMask)
		{
			*op_result = I2C_READ_OK;
			for(j=0;j<number;j=j+2)
			{//to be checked
				pRegTable[j].deviceAddress = uartController->rx_msg.data[0];
				pRegTable[j].regAddress    = uartController->rx_msg.data[1];
				pRegTable[j].data.raw[1]   = uartController->rx_msg.data[2+j];
				pRegTable[j].data.raw[0]   = uartController->rx_msg.data[3+j];
			}
		}			
		return FPGA_ACTION_OK;
	}

	return FPGA_ACTION_FAIL;
}

int random_A16D8_read_FPGAI2C(E_FPGA_IIC ch,FPGA_IIC_A16D8_REG_t *pRegTable, uint32_t number,uint8_t *op_result) 
{
	CyU3PReturnStatus_t status = CY_U3P_SUCCESS;
    uint32_t i,j;
    uint16_t length = number*(sizeof(FPGA_IIC_A16D8_REG_t)-1);//
	uint32_t retry = 0;
	uint32_t evStat;
	CyBool_t acked=CyFalse;
	uint32_t timeut=(9*number*3*10+350)/1000+1;//100kbps I2C speed
	uint8_t  cmd;
	uint32_t waitEvents=(ch==FPGA_IIC0)?
		               (UART_EVT_IIC0_RD_RSP|UART_EVT_IIC0_RA16D8_ONE_BYTE):
		               (UART_EVT_IIC1_RD_RSP|UART_EVT_IIC1_RA16D8_ONE_BYTE);
	uint32_t failEvent=(ch==FPGA_IIC0)?
		               (UART_EVT_IIC0_RD_RSP):
		               (UART_EVT_IIC1_RD_RSP); 
	uint32_t succssEvent=(ch==FPGA_IIC0)?
		               (UART_EVT_IIC0_RA16D8_ONE_BYTE):
		               (UART_EVT_IIC1_RA16D8_ONE_BYTE); 
					   
	FPGA_IIC_A16D8_REG_t *pRegRead;
	
	if(ch>=FPGA_IIC_LAST)
		return FPGA_ACTION_INVALID_IIC;
	
	if(length>MAX_PAYLOAD_SIZE)
	{
		return FPGA_ACTION_WR_LONG;
	}

	cmd = ch==FPGA_IIC0?IIC0_RA16D8_ONE_BYTE:IIC0_RA16D8_ONE_BYTE;
	prepare_frame_head(cmd,length);
	j=6;
	/*
	data0 = device address0 
	data1 = register address0 (high 8bit)
	data2 = register address0 (low 8bit)
	data3 = device address1 
	data4 = register address1 (high 8bit)
	data5 = register address1 (low 8bit)
	.....
	*/
	for(i=0;i<number;i++)
	{
		uartController->tx_buffer[j++]=pRegTable[i].deviceAddress;
		uartController->tx_buffer[j++]=pRegTable[i].regAddress.raw[1];
		uartController->tx_buffer[j++]=pRegTable[i].regAddress.raw[0];
	}
	uartController->tx_buffer[j]  = calcaulet_frame_checksum(uartController->tx_buffer,j);
	j++;
	uartController->tx_buffer[j++]= FX3_FPGA_TAIL1;
	uartController->tx_buffer[j++]= FX3_FPGA_TAIL2;
	do
	{
		send_request_fpga(uartController->tx_buffer, j, &status);
		retry++;
		if(status==CY_U3P_SUCCESS)
		{
	        status = CyU3PEventGet (&UartEvent,
				                    WAIT_ACK_EVENTS, 
				                    CYU3P_EVENT_OR_CLEAR, 
				                    &evStat,
				                    WAIT_ACK_TIME_OUT);	
			if(status == CY_U3P_SUCCESS && (evStat&UART_EVT_DATA_RX_ACK_ERR))
			{
				acked=CyFalse;
			}
			else if(status == CY_U3P_SUCCESS && (evStat&UART_EVT_DATA_RX_ACK_OK))
			{
				acked = CyTrue;
			}
		}
	}
	while(retry<MAX_RETRY_TIMES && !acked);

	if(!acked && retry>=MAX_RETRY_TIMES)
	{
		send_fpga_reset_cmd();
		return FPGA_ACTION_FAIL;
	}
	
	status = CyU3PEventGet (&UartEvent,
							waitEvents, 
							CYU3P_EVENT_OR_CLEAR, 
							&evStat,
							timeut);

	if(status==CY_U3P_SUCCESS)// && (evStat&waitEvent))
	{
		//read fail
		if(evStat&failEvent)
			*op_result = uartController->rx_msg.data[0];

		//read succed
		if(evStat&succssEvent)
		{
			*op_result = I2C_READ_OK;
			pRegRead = (FPGA_IIC_A16D8_REG_t *)uartController->rx_msg.data;
			/*
			data0 = device address0 
			data1 = register address0 (high 8bit) 
			data2 = register address0 (low 8bit)
			data3 = register data0
			data4 = device address1
			data5 = register address1 (high 8bit)
			data6 = register address1 (low 8bit)
			data7 = register data1
			.....
			*/
			for(j=0;j<number;j++)
			{//to be checked
				*pRegTable = *pRegRead;
				pRegRead++;
				pRegTable++;
			}
		}			
		return FPGA_ACTION_OK;
	}

	return FPGA_ACTION_FAIL;
}


int continuous_A16D8_read_FPGAI2C(E_FPGA_IIC ch,FPGA_IIC_A16D8_REG_t *pRegTable, uint32_t number,uint8_t *op_result) 
{
	CyU3PReturnStatus_t status = CY_U3P_SUCCESS;
    uint32_t j;
    uint16_t length = 5;//
	uint32_t retry = 0;
	uint32_t evStat;
	CyBool_t acked=CyFalse;
	uint32_t timeut=(9*number*3*10+350)/1000+1;//100kbps I2C speed
	uint8_t  cmd;
	U16_CNV  reg_num;
	uint32_t waitEvent=(ch==FPGA_IIC0)?
		               (UART_EVT_IIC0_RD_RSP|UART_EVT_IIC0_RA16D8_CNT_BYTES):
		               (UART_EVT_IIC1_RD_RSP|UART_EVT_IIC1_RA16D8_CNT_BYTES);
	uint32_t failMask=(ch==FPGA_IIC0)?
		               (UART_EVT_IIC0_RD_RSP):
		               (UART_EVT_IIC1_RD_RSP); 
	uint32_t succssMask=(ch==FPGA_IIC0)?
		               (UART_EVT_IIC0_RA16D8_CNT_BYTES):
		               (UART_EVT_IIC1_RA16D8_CNT_BYTES); 
					   
	FPGA_IIC_A8D8_REG_t *pRegRead;
	
	if(ch>=FPGA_IIC_LAST)
		return FPGA_ACTION_INVALID_IIC;
	
	if(length>MAX_PAYLOAD_SIZE)
	{
		return FPGA_ACTION_WR_LONG;
	}
	
	cmd = ch==FPGA_IIC0?IIC0_RA16D8_CNT_BYTES:IIC1_RA16D8_CNT_BYTES;
	prepare_frame_head(cmd,length);
	j=6;
	reg_num.value = number;
	/*
	data0 = data number (high 8bit)
	data1 = data number (low 8bit)
	data2 = device address 
	data3 = register address (high 8bit)
	data4 = register address (low 8bit)
	*/
	uartController->tx_buffer[j++] = reg_num.raw[1];
	uartController->tx_buffer[j++] = reg_num.raw[0];
	uartController->tx_buffer[j++] = pRegTable[0].deviceAddress;
	uartController->tx_buffer[j++] = pRegTable[0].regAddress.raw[1];
	uartController->tx_buffer[j++] = pRegTable[0].regAddress.raw[0];
	uartController->tx_buffer[j]  = calcaulet_frame_checksum(uartController->tx_buffer,j);
	j++;
	uartController->tx_buffer[j++]= FX3_FPGA_TAIL1;
	uartController->tx_buffer[j++]= FX3_FPGA_TAIL2;
	do
	{
		send_request_fpga(uartController->tx_buffer, j, &status);
		retry++;
		if(status==CY_U3P_SUCCESS)
		{
	        status = CyU3PEventGet (&UartEvent,
				                    WAIT_ACK_EVENTS, 
				                    CYU3P_EVENT_OR_CLEAR, 
				                    &evStat,
				                    WAIT_ACK_TIME_OUT);	
			if(status == CY_U3P_SUCCESS && (evStat&UART_EVT_DATA_RX_ACK_ERR))
			{
				acked=CyFalse;
			}
			else if(status == CY_U3P_SUCCESS && (evStat&UART_EVT_DATA_RX_ACK_OK))
			{
				acked = CyTrue;
			}
		}
	}
	while(retry<MAX_RETRY_TIMES && !acked);

	if(!acked && retry>=MAX_RETRY_TIMES)
	{
		send_fpga_reset_cmd();
		return FPGA_ACTION_FAIL;
	}
	
	status = CyU3PEventGet (&UartEvent,
							waitEvent, 
							CYU3P_EVENT_OR_CLEAR, 
							&evStat,
							timeut);

	if(status==CY_U3P_SUCCESS)// && (evStat&waitEvent))
	{
		//read fail
		if(evStat&failMask)
			*op_result = uartController->rx_msg.data[0];

		//read succed
		/*
		data0 = device address 
		data1 = register address (high 8bit)
		data2 = register address (low 8bit)
		data3 = register data0
		data4 = register data1
		*/
		if(evStat&succssMask)
		{
			*op_result = I2C_READ_OK;
			for(j=0;j<number;j++)
			{//to be checked
				pRegTable[j].deviceAddress = uartController->rx_msg.data[0];
				pRegTable[j].regAddress.raw[1]= uartController->rx_msg.data[1];
				pRegTable[j].regAddress.raw[0]= uartController->rx_msg.data[2];
				pRegTable[j].data   = uartController->rx_msg.data[3+j];
			}
		}			
		return FPGA_ACTION_OK;
	}

	return FPGA_ACTION_FAIL;
}


int random_A16D16_read_FPGAI2C(E_FPGA_IIC ch,FPGA_IIC_A16D16_REG_t *pRegTable, uint32_t number,uint8_t *op_result) 
{
	CyU3PReturnStatus_t status = CY_U3P_SUCCESS;
    uint32_t i,j;
    uint16_t length = number*(sizeof(FPGA_IIC_A16D16_REG_t)-2);//
	uint32_t retry = 0;
	uint32_t evStat;
	CyBool_t acked=CyFalse;
	uint32_t timeut=(9*number*3*10+350)/1000+1;//100kbps I2C speed
	uint8_t  cmd;
	uint32_t waitEvents=(ch==FPGA_IIC0)?
		               (UART_EVT_IIC0_RD_RSP|UART_EVT_IIC0_RA16D16_ONE_WORD):
		               (UART_EVT_IIC1_RD_RSP|UART_EVT_IIC1_RA16D16_ONE_WORD);
	uint32_t failEvent=(ch==FPGA_IIC0)?
		               (UART_EVT_IIC0_RD_RSP):
		               (UART_EVT_IIC1_RD_RSP); 
	uint32_t succssEvent=(ch==FPGA_IIC0)?
		               (UART_EVT_IIC0_RA16D16_ONE_WORD):
		               (UART_EVT_IIC1_RA16D16_ONE_WORD); 
					   
	FPGA_IIC_A16D16_REG_t *pRegRead;
	
	if(ch>=FPGA_IIC_LAST)
		return FPGA_ACTION_INVALID_IIC;
	
	if(length>MAX_PAYLOAD_SIZE)
	{
		return FPGA_ACTION_WR_LONG;
	}

	cmd = ch==FPGA_IIC0?IIC0_RA16D16_ONE_WORD:IIC1_RA16D16_ONE_WORD;
	prepare_frame_head(cmd,length);
	j=6;
	/*
	data0 = device address0 
	data1 = register address0 (high 8bit)
	data2 = register address0 (low 8bit)
	data3 = device address1 
	data4 = register address1 (high 8bit)
	data5 = register address1 (low 8bit)
	.....
	*/
	for(i=0;i<number;i++)
	{
		uartController->tx_buffer[j++]=pRegTable[i].deviceAddress;
		uartController->tx_buffer[j++]=pRegTable[i].regAddress.raw[1];
		uartController->tx_buffer[j++]=pRegTable[i].regAddress.raw[0];
	}
	uartController->tx_buffer[j]  = calcaulet_frame_checksum(uartController->tx_buffer,j);
	j++;
	uartController->tx_buffer[j++]= FX3_FPGA_TAIL1;
	uartController->tx_buffer[j++]= FX3_FPGA_TAIL2;
	do
	{
		send_request_fpga(uartController->tx_buffer, j, &status);
		retry++;
		if(status==CY_U3P_SUCCESS)
		{
	        status = CyU3PEventGet (&UartEvent,
				                    WAIT_ACK_EVENTS, 
				                    CYU3P_EVENT_OR_CLEAR, 
				                    &evStat,
				                    WAIT_ACK_TIME_OUT);	
			if(status == CY_U3P_SUCCESS && (evStat&UART_EVT_DATA_RX_ACK_ERR))
			{
				acked=CyFalse;
			}
			else if(status == CY_U3P_SUCCESS && (evStat&UART_EVT_DATA_RX_ACK_OK))
			{
				acked = CyTrue;
			}
		}
	}
	while(retry<MAX_RETRY_TIMES && !acked);

	if(!acked && retry>=MAX_RETRY_TIMES)
	{
		send_fpga_reset_cmd();
		return FPGA_ACTION_FAIL;
	}
	
	status = CyU3PEventGet (&UartEvent,
							waitEvents, 
							CYU3P_EVENT_OR_CLEAR, 
							&evStat,
							timeut);

	if(status==CY_U3P_SUCCESS)// && (evStat&waitEvent))
	{
		//read fail
		if(evStat&failEvent)
			*op_result = uartController->rx_msg.data[0];

		//read succed
		if(evStat&succssEvent)
		{
			*op_result = I2C_READ_OK;
			pRegRead = (FPGA_IIC_A16D16_REG_t *)uartController->rx_msg.data;
			/*
			data0 = device address0 
			data1 = register address0 (high 8bit) 
			data2 = register address0 (low 8bit)
			data3 = register data0
			data4 = device address1
			data5 = register address1 (high 8bit)
			data6 = register address1 (low 8bit)
			data7 = register data1
			.....
			*/
			for(j=0;j<number;j++)
			{//to be checked
				*pRegTable = *pRegRead;
				pRegRead++;
				pRegTable++;
			}
		}			
		return FPGA_ACTION_OK;
	}

	return FPGA_ACTION_FAIL;
}


int continuous_A16D16_read_FPGAI2C(E_FPGA_IIC ch,FPGA_IIC_A16D16_REG_t *pRegTable, uint32_t number,uint8_t *op_result) 
{
	CyU3PReturnStatus_t status = CY_U3P_SUCCESS;
    uint32_t j;
    uint16_t length = 5;//
	uint32_t retry = 0;
	uint32_t evStat;
	CyBool_t acked=CyFalse;
	uint32_t timeut=(9*number*3*10+350)/1000+1;//100kbps I2C speed
	uint8_t  cmd;
	U16_CNV  reg_num;
	uint32_t waitEvents=(ch==FPGA_IIC0)?
		               (UART_EVT_IIC0_RD_RSP|UART_EVT_IIC0_RA16D16_CNT_WORDS):
		               (UART_EVT_IIC1_RD_RSP|UART_EVT_IIC1_RA16D16_CNT_WORDS);
	uint32_t failMask=(ch==FPGA_IIC0)?
		               (UART_EVT_IIC0_RD_RSP):
		               (UART_EVT_IIC1_RD_RSP); 
	uint32_t succssMask=(ch==FPGA_IIC0)?
		               (UART_EVT_IIC0_RA16D16_CNT_WORDS):
		               (UART_EVT_IIC1_RA16D16_CNT_WORDS); 
					   
	if(ch>=FPGA_IIC_LAST)
		return FPGA_ACTION_INVALID_IIC;
	
	if(length>MAX_PAYLOAD_SIZE)
	{
		return FPGA_ACTION_WR_LONG;
	}
	
	cmd = ch==FPGA_IIC0?IIC0_RA16D16_CNT_WORDS:IIC1_RA16D16_CNT_WORDS;
	prepare_frame_head(cmd,length);
	j=6;
	reg_num.value = number;
	/*
	data0 = data number (high 8bit)
	data1 = data number (low 8bit)
	data2 = device address 
	data3 = register address (high 8bit)
	data4 = register address (low 8bit)
	*/
	uartController->tx_buffer[j++] = reg_num.raw[1];
	uartController->tx_buffer[j++] = reg_num.raw[0];
	uartController->tx_buffer[j++] = pRegTable[0].deviceAddress;
	uartController->tx_buffer[j++] = pRegTable[0].regAddress.raw[1];
	uartController->tx_buffer[j++] = pRegTable[0].regAddress.raw[0];
	uartController->tx_buffer[j]  = calcaulet_frame_checksum(uartController->tx_buffer,j);
	j++;
	uartController->tx_buffer[j++]= FX3_FPGA_TAIL1;
	uartController->tx_buffer[j++]= FX3_FPGA_TAIL2;
	do
	{
		send_request_fpga(uartController->tx_buffer, j, &status);
		retry++;
		if(status==CY_U3P_SUCCESS)
		{
	        status = CyU3PEventGet (&UartEvent,
				                    WAIT_ACK_EVENTS, 
				                    CYU3P_EVENT_OR_CLEAR, 
				                    &evStat,
				                    WAIT_ACK_TIME_OUT);	
			if(status == CY_U3P_SUCCESS && (evStat&UART_EVT_DATA_RX_ACK_ERR))
			{
				acked=CyFalse;
			}
			else if(status == CY_U3P_SUCCESS && (evStat&UART_EVT_DATA_RX_ACK_OK))
			{
				acked = CyTrue;
			}
		}
	}
	while(retry<MAX_RETRY_TIMES && !acked);

	if(!acked && retry>=MAX_RETRY_TIMES)
	{
		send_fpga_reset_cmd();
		return FPGA_ACTION_FAIL;
	}
	
	status = CyU3PEventGet (&UartEvent,
							waitEvents, 
							CYU3P_EVENT_OR_CLEAR, 
							&evStat,
							timeut);

	if(status==CY_U3P_SUCCESS)// && (evStat&waitEvent))
	{
		//read fail
		if(evStat&failMask)
			*op_result = uartController->rx_msg.data[0];

		//read succed
		/*
		data0 = device address 
		data1 = register address (high 8bit)
		data2 = register address (low 8bit)
		data3 = register0 data (high 8bit)
		data4 = register0 data (low 8bit) 
		data5 = register1 data (high 8bit)
		data6 = register1 data (low 8bit) 		
		*/
		if(evStat&succssMask)
		{
			*op_result = I2C_READ_OK;
			for(j=0;j<number;j=j+2)
			{//to be checked
				pRegTable[j].deviceAddress = uartController->rx_msg.data[0];
				pRegTable[j].regAddress.raw[1]= uartController->rx_msg.data[1];
				pRegTable[j].regAddress.raw[0]= uartController->rx_msg.data[2];
				pRegTable[j].data.raw[1]      = uartController->rx_msg.data[3+j];
				pRegTable[j].data.raw[0]      = uartController->rx_msg.data[4+j];
			}
		}			
		return FPGA_ACTION_OK;
	}

	return FPGA_ACTION_FAIL;
}

delay(int i)
{
	int j=0;
	int n;
	for(j=0;j<i;j++)
		for(n=0;n<600000;n++);
}
int reg_map_write(uint8_t address ,uint8_t value)
{
	CyU3PReturnStatus_t status = CY_U3P_SUCCESS;
    uint32_t j;
    uint16_t length = 2;
	uint32_t retry = 0;
	uint32_t evStat;
	CyBool_t acked=CyFalse;
		
	prepare_frame_head(REG_MAP_WR_REQ,length);
	
	j=6;
	uartController->tx_buffer[j++]=address;
	uartController->tx_buffer[j++]=value;

	uartController->tx_buffer[j]  = calcaulet_frame_checksum(uartController->tx_buffer,j);
	j++;
	uartController->tx_buffer[j++]= FX3_FPGA_TAIL1;
	uartController->tx_buffer[j++]= FX3_FPGA_TAIL2;

	do
	{
		send_request_fpga(uartController->tx_buffer, j, &status);
		retry++;

		if(status==CY_U3P_SUCCESS)
		{
	        status = CyU3PEventGet (&UartEvent,
				                    WAIT_ACK_EVENTS, 
				                    CYU3P_EVENT_OR_CLEAR, 
				                    &evStat,
				                    WAIT_ACK_TIME_OUT);	
			if(status == CY_U3P_SUCCESS && (evStat&UART_EVT_DATA_RX_ACK_ERR))
			{
				acked=CyFalse;
			}
			else if(status == CY_U3P_SUCCESS && (evStat&UART_EVT_DATA_RX_ACK_OK))
			{
				acked = CyTrue;
			}
		}
	}
	while(retry<MAX_RETRY_TIMES && !acked);

	
	if(!acked && retry>=MAX_RETRY_TIMES)
	{
		send_fpga_reset_cmd();
	}

	return acked?FPGA_ACTION_OK:FPGA_ACTION_FAIL;
}

int reg_map_read(uint8_t address,uint8_t *value) 
{
	CyU3PReturnStatus_t status = CY_U3P_SUCCESS;
    uint32_t j;
    uint16_t length = 1;
	uint32_t retry = 0;
	uint32_t evStat;
	CyBool_t acked=CyFalse;
	uint32_t timeut=2;
	uint32_t intMask;
	int ret=FPGA_ACTION_FAIL;

	prepare_frame_head(REG_MAP_RD_REQ,length);
	
	j=6;
	uartController->tx_buffer[j++]=address;
	uartController->tx_buffer[j]  = calcaulet_frame_checksum(uartController->tx_buffer,j);
	j++;
	uartController->tx_buffer[j++]= FX3_FPGA_TAIL1;
	uartController->tx_buffer[j++]= FX3_FPGA_TAIL2;

	//intMask = CyU3PVicDisableAllInterrupts();
	//CyU3PVicEnableInterrupts(intMask&0x140010);
	do
	{
		send_request_fpga(uartController->tx_buffer, j, &status);
		retry++;
		if(status==CY_U3P_SUCCESS)
		{
	        status = CyU3PEventGet (&UartEvent,
				                    WAIT_ACK_EVENTS, 
				                    CYU3P_EVENT_OR_CLEAR, 
				                    &evStat,
				                    WAIT_ACK_TIME_OUT);	
			if(status == CY_U3P_SUCCESS && (evStat&UART_EVT_DATA_RX_ACK_ERR))
			{
				acked=CyFalse;
			}
			else if(status == CY_U3P_SUCCESS && (evStat&UART_EVT_DATA_RX_ACK_OK))
			{
				acked = CyTrue;
			}
		}
	}
	while(retry<MAX_RETRY_TIMES && !acked);

	if(!acked && retry>=MAX_RETRY_TIMES)
	{
		send_fpga_reset_cmd();
		//return FPGA_ACTION_FAIL;
	}
	
	status = CyU3PEventGet (&UartEvent,
							UART_EVT_REG_MAP_RD_RSP, 
							CYU3P_EVENT_OR_CLEAR, 
							&evStat,
							timeut);

	if(status==CY_U3P_SUCCESS && (evStat&UART_EVT_REG_MAP_RD_RSP))
	{
		*value = uartController->rx_msg.data[0];
		ret = FPGA_ACTION_OK;
	}

	//CyU3PVicEnableInterrupts(intMask);
	return ret;
}

int init_setup_FPGAI2C(E_FPGA_IIC ch,uint8_t speed) 
{
	CyU3PReturnStatus_t status = CY_U3P_SUCCESS;
    uint32_t j;
    uint16_t length = 1;
	uint32_t retry = 0;
	uint32_t evStat;
	CyBool_t acked=CyFalse;
	uint8_t  cmd;
	
	if(ch>=FPGA_IIC_LAST)
		return FPGA_ACTION_INVALID_IIC;
	
	cmd = ch==FPGA_IIC0?IIC0_INIT_SETUP:IIC1_INIT_SETUP;

	prepare_frame_head(cmd,length);
	j=6;

	uartController->tx_buffer[j++]=speed;
	uartController->tx_buffer[j]  = calcaulet_frame_checksum(uartController->tx_buffer,j);
	j++;
	uartController->tx_buffer[j++]= FX3_FPGA_TAIL1;
	uartController->tx_buffer[j++]= FX3_FPGA_TAIL2;
	do
	{
		send_request_fpga(uartController->tx_buffer, j, &status);
		retry++;
		if(status==CY_U3P_SUCCESS)
		{
	        status = CyU3PEventGet (&UartEvent,
				                    WAIT_ACK_EVENTS, 
				                    CYU3P_EVENT_OR_CLEAR, 
				                    &evStat,
				                    WAIT_ACK_TIME_OUT);	
			if(status == CY_U3P_SUCCESS && (evStat&UART_EVT_DATA_RX_ACK_ERR))
			{
				acked=CyFalse;
			}
			else if(status == CY_U3P_SUCCESS && (evStat&UART_EVT_DATA_RX_ACK_OK))
			{
				acked = CyTrue;
			}
		}
	}
	while(retry<MAX_RETRY_TIMES && !acked);

	if(!acked && retry>=MAX_RETRY_TIMES)
	{
		send_fpga_reset_cmd();
		return FPGA_ACTION_FAIL;
	}

	return FPGA_ACTION_OK;
}

