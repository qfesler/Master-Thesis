#ifndef DWM1000lib_h
#define DWM1000lib_h

// register map
#define DWM1000_REG_DEV_ID      0x00
#define DWM1000_REG_EUI         0x01
#define DWM1000_REG_PANADR      0x03
#define DWM1000_REG_SYS_CFG     0x04
#define DWM1000_REG_SYS_TIME    0x06
#define DWM1000_REG_TX_FCTRL    0x08
#define DWM1000_REG_TX_BUFFER   0x09
#define DWM1000_REG_DX_TIME     0x0A
#define DWM1000_REG_RX_FWTO     0X0C
#define DWM1000_REG_SYS_CTRL    0x0D
#define DWM1000_REG_SYS_MASK    0x0E
#define DWM1000_REG_SYS_STATUS  0x0F
#define DWM1000_REG_RX_FINFO    0x10
#define DWM1000_REG_RX_BUFFER   0x11
#define DWM1000_REG_RX_FQUAL    0x12
#define DWM1000_REG_RX_TTCKI    0x13
#define DWM1000_REG_RX_TTCKO    0x14
#define DWM1000_REG_RX_TIME     0x15
#define DWM1000_REG_TX_TIME     0x17
#define DWM1000_REG_TX_ANTD     0x18
#define DWM1000_REG_SYS_STATE   0x19
#define DWM1000_REG_ACK_RESP_T  0x1A
#define DWM1000_REG_RX_SNIFF    0x1D
#define DWM1000_REG_TX_POWER    0x1E
#define DWM1000_REG_CHAN_CTRL   0x1F
#define DWM1000_REG_USR_SFD     0x21
#define DWM1000_REG_AGC_CTRL    0x23
#define DWM1000_REG_EXT_SYNC    0x24
#define DWM1000_REG_ACC_MEM     0x25
#define DWM1000_REG_GPIO_CTRL   0x26
#define DWM1000_REG_DRX_CONF    0x27
#define DWM1000_REG_RF_CONF     0x28
#define DWM1000_REG_TX_CAL      0x2A
#define DWM1000_REG_FS_CTRL     0x2B
#define DWM1000_REG_AON         0x2C
#define DWM1000_REG_OTP_IF      0x2D
#define DWM1000_REG_LDE_CTRL    0x2E
#define DWM1000_REG_DIG_DIAG    0x2F
#define DWM1000_REG_PMSC        0x36

// functions
/* SOFT RESET of the Decawave */
void DWM_reset(void);

/* Initialize the Decawave without adress filtering (2 tag mode)
Check for the decawave ID, reset and setup */
void DWM_init (void);

/* Write data into the register of the DW at address */
void DWM_WriteSPI(uint8_t address, uint8_t *data, uint16_t len);

/*Read register of the DW with given adress and length*/
void DWM_ReadSPI(uint8_t adress, uint8_t *data, uint16_t len);

/*array of uint8 in machine order to uint32 in human order*/
uint32_t uint8TOuint32(uint8_t *data);

/*uint32 human to array uint8 machine*/
void Uint32TOuint8 ( uint32_t from, uint8_t *to );

/*Error Function... blinks red  led if something wrong */
void Error_Fct(void);

#endif
