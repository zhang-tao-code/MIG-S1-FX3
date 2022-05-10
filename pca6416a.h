#ifndef _PCA6416A_H
#define _PCA6416A_H


#define PCA6416_SLAVE_ADDR                 0x21
#define PCA6416_REG_IN_PORT0               0x00
#define PCA6416_REG_IN_PORT1               0x01
#define PCA6416_REG_OUT_PORT0              0x02
#define PCA6416_REG_OUT_PORT1              0x03
#define PCA6416_REG_PIR_PORT0              0x04
#define PCA6416_REG_PIR_PORT1              0x05
#define PCA6416_REG_CFG_PORT0              0x06
#define PCA6416_REG_CFG_PORT1              0x07

typedef enum
{
   DPSRST_ST_NORMAL,
   DPSRST_ST_RESET,
} DPSRSTState;

typedef struct
{
    uint8_t       port0;
	uint16_t      duration;
	DPSRSTState   state;
	CyU3PTimer    resetTimer;
} PCA6416A_t;

#define ESET_DPS   0x60
#define BOOT_DPS   0X80

extern PCA6416A_t pca6416;
CyU3PReturnStatus_t init_pca6416a(void);
CyU3PReturnStatus_t set_pca6416a(uint8_t pinMask, CyBool_t state);
CyU3PReturnStatus_t reset_dps(uint16_t duration);

#endif
