#ifndef _UART_H_
#define _UART_H_

#define FX3_FPGA_HEAD1    0xB1
#define FX3_FPGA_HEAD2    0xC8
#define FX3_FPGA_TAIL1    0xC2
#define FX3_FPGA_TAIL2    0xF9
#define FPGA_FX3_HEAD1    0xEB
#define FPGA_FX3_HEAD2    0x90
#define FPGA_FX3_TAIL1    0x13
#define FPGA_FX3_TAIL2    0x1B

/*data type FX3->FPGA*/
#define IIC0_WA8D8_ONE_BYTE     0x11 /*IIC0 interface one by one write data (8bit addr+8bit data)*/
#define IIC0_WA8D8_CNT_BYTES    0x12 /*IIC0 interface continuous write data (8bit addr+8bit data)*/
#define IIC0_WA8D16_ONE_WORD    0x13 /*IIC0 interface one by one write data (8bit addr+16bit data)*/
#define IIC0_WA8D16_CNT_WORDS   0x14 /*IIC0 interface continuous write data (8bit addr+16bit data)*/
#define IIC0_WA16D8_ONE_BYTE    0x15 /*IIC0 interface one by one write data (16bit addr+8bit data)*/
#define IIC0_WA16D8_CNT_BYTES   0x16 /*IIC0 interface continuous write data (16bit addr+8bit data)*/
#define IIC0_WA16D16_ONE_WORD   0x17 /*IIC0 interface one by one write data (16bit addr+16bit data)*/
#define IIC0_WA16D16_CNT_WORDS  0x18 /*IIC0 interface continuous write data (16bit addr+16bit data)*/

#define IIC0_RA8D8_ONE_BYTE     0x21 /*IIC0 interface one by one read data (8bit addr+8bit data)*/
#define IIC0_RA8D8_CNT_BYTES    0x22 /*IIC0 interface continuous read data (8bit addr+8bit data)*/
#define IIC0_RA8D16_ONE_WORD    0x23 /*IIC0 interface one by one read data (8bit addr+16bit data)*/
#define IIC0_RA8D16_CNT_WORDS   0x24 /*IIC0 interface continuous read data (8bit addr+16bit data)*/
#define IIC0_RA16D8_ONE_BYTE    0x25 /*IIC0 interface one by one read data (16bit addr+8bit data)*/
#define IIC0_RA16D8_CNT_BYTES   0x26 /*IIC0 interface continuous read data (16bit addr+8bit data)*/
#define IIC0_RA16D16_ONE_WORD   0x27 /*IIC0 interface one by one read data (16bit addr+16bit data)*/
#define IIC0_RA16D16_CNT_WORDS  0x28 /*IIC0 interface continuous read data (16bit addr+16bit data)*/

#define IIC1_WA8D8_ONE_BYTE     0x31 /*IIC1 interface one by one write data (8bit addr+8bit data)*/
#define IIC1_WA8D8_CNT_BYTES    0x32 /*IIC1 interface continuous write data (8bit addr+8bit data)*/
#define IIC1_WA8D16_ONE_WORD    0x33 /*IIC1 interface one by one write data (8bit addr+16bit data)*/
#define IIC1_WA8D16_CNT_WORDS   0x34 /*IIC1 interface continuous write data (8bit addr+16bit data)*/
#define IIC1_WA16D8_ONE_BYTE    0x35 /*IIC1 interface one by one write data (16bit addr+8bit data)*/
#define IIC1_WA16D8_CNT_BYTES   0x36 /*IIC1 interface continuous write data (16bit addr+8bit data)*/
#define IIC1_WA16D16_ONE_WORD   0x37 /*IIC1 interface one by one write data (16bit addr+16bit data)*/
#define IIC1_WA16D16_CNT_WORDS  0x38 /*IIC1 interface continuous write data (16bit addr+16bit data)*/

#define IIC1_RA8D8_ONE_BYTE     0x41 /*IIC1 interface one by one read data (8bit addr+8bit data)*/
#define IIC1_RA8D8_CNT_BYTES    0x42 /*IIC1 interface continuous read data (8bit addr+8bit data)*/
#define IIC1_RA8D16_ONE_WORD    0x43 /*IIC1 interface one by one read data (8bit addr+16bit data)*/
#define IIC1_RA8D16_CNT_WORDS   0x44 /*IIC1 interface continuous read data (8bit addr+16bit data)*/
#define IIC1_RA16D8_ONE_BYTE    0x45 /*IIC1 interface one by one read data (16bit addr+8bit data)*/
#define IIC1_RA16D8_CNT_BYTES   0x46 /*IIC1 interface continuous read data (16bit addr+8bit data)*/
#define IIC1_RA16D16_ONE_WORD   0x47 /*IIC1 interface one by one read data (16bit addr+16bit data)*/
#define IIC1_RA16D16_CNT_WORDS  0x48 /*IIC1 interface continuous read data (16bit addr+16bit data)*/

#define REG_MAP_WR_REQ          0X51 /*0x51: register map write request    */
#define REG_MAP_RD_REQ          0X52 /*0x52: register map read request     */
#define IIC0_INIT_SETUP         0X53 /*0x53: IIC0 initial setup instruction*/
#define IIC1_INIT_SETUP         0X54 /*0x54: IIC1 initial setup instruction*/

/*data type FPGA->FX3*/
#define DATA_RX_ACK      0x71 /*0x71: data receive acknowledge (This type data will be sent when received all kind of instructions)*/
#define IIC0_WR_RSP      0x72 /*0x72: IIC0 interface write data respond*/
#define IIC1_WR_RSP      0x73 /*0x73: IIC1 interface write data respond*/
#define IIC0_RD_RSP      0x74 /*0x74: IIC0 interface read data status respond (if iic data read request, if status had been send , read data respond will not be respond)*/
#define IIC1_RD_RSP      0x75 /*0x75: IIC1 interface read data status respond*/

#define IIC0_RA8D8_ONE_BYTE_RSP    0x81 /*0x81: IIC0 interface one by one read data respond (8bit addr+8bit data) */
#define IIC0_RA8D8_CNT_BYTES_RSP   0x82 /*0x82: IIC0 interface continuous read data respond (8bit addr+8bit data) */
#define IIC0_RA8D16_ONE_WORD_RSP   0x83 /*0x83: IIC0 interface one by one read data respond (8bit addr+16bit data)*/
#define IIC0_RA8D16_CNT_WORDS_RSP  0x84 /*0x84: IIC0 interface continuous read data respond (8bit addr+16bit data)*/
#define IIC0_RA16D8_ONE_BYTE_RSP   0x85 /*0x85: IIC0 interface one by one read data respond (16bit addr+8bit data)*/
#define IIC0_RA16D8_CNT_BYTES_RSP  0x86 /*0x86: IIC0 interface continuous read data respond (16bit addr+8bit data)*/
#define IIC0_RA16D16_ONE_WORD_RSP  0x87 /*0x87: IIC0 interface one by one read data respond (16bit addr+16bit data)*/
#define IIC0_RA16D16_CNT_WORDS_RSP 0x88 /*0x88: IIC0 interface continuous read data respond (16bit addr+16bit data)*/

#define IIC1_RA8D8_ONE_BYTE_RSP    0x91 /*0x81: IIC1 interface one by one read data respond (8bit addr+8bit data) */
#define IIC1_RA8D8_CNT_BYTES_RSP   0x92 /*0x82: IIC1 interface continuous read data respond (8bit addr+8bit data) */
#define IIC1_RA8D16_ONE_WORD_RSP   0x93 /*0x83: IIC1 interface one by one read data respond (8bit addr+16bit data)*/
#define IIC1_RA8D16_CNT_WORDS_RSP  0x94 /*0x84: IIC1 interface continuous read data respond (8bit addr+16bit data)*/
#define IIC1_RA16D8_ONE_BYTE_RSP   0x95 /*0x85: IIC1 interface one by one read data respond (16bit addr+8bit data)*/
#define IIC1_RA16D8_CNT_BYTES_RSP  0x96 /*0x86: IIC1 interface continuous read data respond (16bit addr+8bit data)*/
#define IIC1_RA16D16_ONE_WORD_RSP  0x97 /*0x87: IIC1 interface one by one read data respond (16bit addr+16bit data)*/
#define IIC1_RA16D16_CNT_WORDS_RSP 0x98 /*0x88: IIC1 interface continuous read data respond (16bit addr+16bit data)*/

#define REG_MAP_RD_RSP             0xA9 /*0xA9: register map read data respond*/

#define MAX_REGS_PER_ACCESS       (512)
#define MAX_PAYLOAD_SIZE          (MAX_REGS_PER_ACCESS*5)
#define MAX_BUFER_SIZE            (MAX_PAYLOAD_SIZE+9)

#define UART_EVT_RESPOND_FRAME    (1 << 0) /* Frame received event */
#define UART_EVT_ACK_FRAME        (1 << 1) /* Frame received event */
#define UART_EVT_DATA_RX_ACK_OK   (1 << 2) /* Frame DATA_RX_ACK OK event */
#define UART_EVT_DATA_RX_ACK_ERR  (1 << 3) /* Frame DATA_RX_ACK OK event */
#define UART_EVT_IIC0_WR_RSP      (1 << 4) /* Frame IIC0_WR_RSP event */
#define UART_EVT_IIC1_WR_RSP      (1 << 5) /* Frame IIC1_WR_RSP event */
#define UART_EVT_IIC0_RD_RSP      (1 << 6) /* Frame IIC0_RD_RSP event */
#define UART_EVT_IIC1_RD_RSP      (1 << 7) /* Frame IIC1_RD_RSP event */

#define UART_EVT_IIC0_RA8D8_ONE_BYTE    (1 << 8)  /* Frame IIC0_RA8D8_ONE_BYTE event */
#define UART_EVT_IIC0_RA8D8_CNT_BYTES   (1 << 9)  /* Frame IIC0_RA8D8_CNT_BYTES event */
#define UART_EVT_IIC0_RA8D16_ONE_WORD   (1 << 10) /* Frame IIC0_RA8D16_ONE_WORD event */
#define UART_EVT_IIC0_RA8D16_CNT_WORDS  (1 << 11) /* Frame IIC0_RA8D16_ONE_WORD event */
#define UART_EVT_IIC0_RA16D8_ONE_BYTE   (1 << 12) /* Frame IIC0_RA16D8_ONE_BYTE event */
#define UART_EVT_IIC0_RA16D8_CNT_BYTES  (1 << 13) /* Frame IIC0_RA16D8_CNT_BYTES event */
#define UART_EVT_IIC0_RA16D16_ONE_WORD  (1 << 14) /* Frame IIC0_RA16D16_ONE_WORD event */
#define UART_EVT_IIC0_RA16D16_CNT_WORDS (1 << 15) /* Frame IIC0_RA16D16_CNT_WORDS event */

#define UART_EVT_IIC1_RA8D8_ONE_BYTE    (1 << 16) /* Frame IIC1_RA8D8_ONE_BYTE event */
#define UART_EVT_IIC1_RA8D8_CNT_BYTES   (1 << 17) /* Frame IIC1_RA8D8_CNT_BYTES event */
#define UART_EVT_IIC1_RA8D16_ONE_WORD   (1 << 18) /* Frame IIC1_RA8D16_ONE_WORD event */
#define UART_EVT_IIC1_RA8D16_CNT_WORDS  (1 << 19) /* Frame IIC1_RA8D16_ONE_WORD event */
#define UART_EVT_IIC1_RA16D8_ONE_BYTE   (1 << 20) /* Frame IIC1_RA16D8_ONE_BYTE event */
#define UART_EVT_IIC1_RA16D8_CNT_BYTES  (1 << 21) /* Frame IIC1_RA16D8_CNT_BYTES event */
#define UART_EVT_IIC1_RA16D16_ONE_WORD  (1 << 22) /* Frame IIC1_RA16D16_ONE_WORD event */
#define UART_EVT_IIC1_RA16D16_CNT_WORDS (1 << 23) /* Frame IIC1_RA16D16_CNT_WORDS event */

#define UART_EVT_REG_MAP_RD_RSP    (1 << 24) /* register map write response event */

#define IIC_SPEED_50KBPS           0X11  
#define IIC_SPEED_100KBPS          0X22  
#define IIC_SPEED_200KBPS          0X33  
#define IIC_SPEED_400KBPS          0X44
#define IIC_SPEED_1000KBPS         0X55

#define I2C_READ_OK   0

typedef enum
{
   FPGA_ACTION_INVALID_IIC=-4,
   FPGA_ACTION_RD_LONG=-3,
   FPGA_ACTION_WR_LONG=-2,
   FPGA_ACTION_FAIL=-1,
   FPGA_ACTION_OK,
} E_FPGA_OPERATION;


typedef enum
{
   FPGA_IIC0=0,
   FPGA_IIC1,
   FPGA_IIC_LAST,
} E_FPGA_IIC;


typedef enum
{
   UART_ST_IDLE,
   UART_ST_REQ,
   UART_ST_WAIT_ACK,
} UARTState;

#define ACK_UNKNOWN   0XFF
#define ACK_OK         0
#define ACK_ERROR      1

#define MIN_FRAME_LENGTH         	10    /*eg. EB 90 AB 71 00 01 20 B9 13 1B */
#define ACK_FRAME_LENGTH         	MIN_FRAME_LENGTH
#define MIN_ACK_RSP_FRAMES_LENGTH   (2*MIN_FRAME_LENGTH)

#pragma pack(push,1)

typedef union
{
	uint8_t  raw[2];
	uint16_t value;
}U16_CNV;

typedef struct
{
    uint8_t  deviceAddress;
	uint8_t  regAddress;
	uint8_t  data;
} FPGA_IIC_A8D8_REG_t;

typedef struct
{
    uint8_t  deviceAddress;
	uint8_t  regAddress;
	U16_CNV  data;
} FPGA_IIC_A8D16_REG_t;

typedef struct
{
    uint8_t  deviceAddress;
	U16_CNV  regAddress;
	uint8_t  data;
} FPGA_IIC_A16D8_REG_t;

typedef struct
{
    uint8_t  deviceAddress;
	U16_CNV  regAddress;
	U16_CNV  data;
} FPGA_IIC_A16D16_REG_t;


typedef struct
{
	uint8_t  head1;
	uint8_t  head2;
    uint8_t  counter;
	uint8_t  type;
	uint16_t length; /*payload+checksum+tail*/
	uint8_t  data[MAX_PAYLOAD_SIZE];
	uint8_t  checksum;
	uint8_t  tail1;
	uint8_t  tail2;
} Uart_msg_t;

typedef struct
{
	uint8_t  head1;
	uint8_t  head2;
    uint8_t  counter;
	uint8_t  type;
	uint16_t length; /*payload+checksum+tail*/
	uint8_t  data;
	uint8_t  checksum;
	uint8_t  tail1;
	uint8_t  tail2;
} ack_msg_t;


typedef struct
{
   uint8_t     rx_buffer[MAX_BUFER_SIZE+MIN_FRAME_LENGTH];//ACK+replay frams
   uint32_t    rx_index;
   uint32_t    head_index;
   CyBool_t    head_received;
   uint32_t    length_expected;
   UARTState   req_state;
   Uart_msg_t  rx_msg;
   ack_msg_t   ack_msg;
   CyBool_t    ackEventSent;
   uint8_t     tx_buffer[MAX_BUFER_SIZE];

} UARTControl;

#pragma pack(pop)

#define UART_THREAD_STACK      (0x0800) /* UART application thread stack size */
#define UART_THREAD_PRIORITY   (7)      /* UART application thread priority */




extern CyU3PThread UartThread;
extern CyU3PEvent  UartEvent;
void UartThread_Entry ( uint32_t input);
void init_uart(void);

int random_A8D8_write_FPGAI2C(E_FPGA_IIC ch,FPGA_IIC_A8D8_REG_t *pRegTable, uint32_t number,uint8_t *op_result);
int continuous_A8D8_write_FPGAI2C(E_FPGA_IIC ch,FPGA_IIC_A8D8_REG_t *pRegTable, uint32_t number,uint8_t *op_result);
int random_A8D16_write_FPGAI2C(E_FPGA_IIC ch,FPGA_IIC_A8D16_REG_t *pRegTable, uint32_t number,uint8_t *op_result);
int continuous_A8D16_write_FPGAI2C(E_FPGA_IIC ch,FPGA_IIC_A8D16_REG_t *pRegTable, uint32_t number,uint8_t *op_result);
int random_A16D8_write_FPGAI2C(E_FPGA_IIC ch,FPGA_IIC_A16D8_REG_t *pRegTable, uint32_t number,uint8_t *op_result);
int continuous_A16D8_write_FPGAI2C(E_FPGA_IIC ch,FPGA_IIC_A16D8_REG_t *pRegTable, uint32_t number,uint8_t *op_result);
int random_A16D16_write_FPGAI2C(E_FPGA_IIC ch,FPGA_IIC_A16D16_REG_t *pRegTable, uint32_t number,uint8_t *op_result); 
int continuous_A16D16_write_FPGAI2C(E_FPGA_IIC ch,FPGA_IIC_A16D16_REG_t *pRegTable, uint32_t number,uint8_t *op_result);
int random_A8D8_read_FPGAI2C(E_FPGA_IIC ch,FPGA_IIC_A8D8_REG_t *pRegTable, uint32_t number,uint8_t *op_result);
int continuous_A8D8_read_FPGAI2C(E_FPGA_IIC ch,FPGA_IIC_A8D8_REG_t *pRegTable, uint32_t number,uint8_t *op_result) ;
int random_A8D16_read_FPGAI2C(E_FPGA_IIC ch,FPGA_IIC_A8D16_REG_t *pRegTable, uint32_t number,uint8_t *op_result);
int continuous_A8D16_read_FPGAI2C(E_FPGA_IIC ch,FPGA_IIC_A8D16_REG_t *pRegTable, uint32_t number,uint8_t *op_result);
int random_A16D8_read_FPGAI2C(E_FPGA_IIC ch,FPGA_IIC_A16D8_REG_t *pRegTable, uint32_t number,uint8_t *op_result);
int continuous_A16D8_read_FPGAI2C(E_FPGA_IIC ch,FPGA_IIC_A16D8_REG_t *pRegTable, uint32_t number,uint8_t *op_result);
int random_A16D16_read_FPGAI2C(E_FPGA_IIC ch,FPGA_IIC_A16D16_REG_t *pRegTable, uint32_t number,uint8_t *op_result);
int continuous_A16D16_read_FPGAI2C(E_FPGA_IIC ch,FPGA_IIC_A16D16_REG_t *pRegTable, uint32_t number,uint8_t *op_result);

int reg_map_write(uint8_t address ,uint8_t value);
int reg_map_read(uint8_t address,uint8_t *value);
int init_setup_FPGAI2C(E_FPGA_IIC ch,uint8_t speed);

#endif
