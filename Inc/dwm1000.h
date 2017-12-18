#ifndef __DWM1000_H
#define __DWM1000_H

#include <inttypes.h>


/* ------------------------------------- */
/* REGISTERS LENGTHS                     */
/* ------------------------------------- */
#define LEN_SYS_CTRL						4

/* ------------------------------------- */
/* REGISTERS BIT MAP                     */
/* ------------------------------------- */
#define TRXOFF_BIT							6

/* ------------------------------------- */
/* DEVICE MODE                           */
/* ------------------------------------- */
#define IDLE_MODE								0x00
#define RX_MODE									0x01
#define TX_MODE									0x02

/* ------------------------------------- */
/* READ-WRITE DEFINES                    */
/* ------------------------------------- */

#define NO_SUB									0xFF
#define WRITE										0x80
#define WRITE_SUB								0xC0
#define READ										0x00
#define READ_SUB								0x40
#define RW_SUB_EXT							0x80

/* ------------------------------------- */
/* UTILITIES DEFINES                     */
/* ------------------------------------- */
#define Bitset(var, bitno) ((var) |= 1UL<<(bitno))
#define Bitclear(var, bitno) ((var) &= ~(1UL<<(bitno)))

/* -------------------- Variables Defnitions ----------------------------- */

/* ------------------------------------- */
/* GLOBAL VARIABLES                      */
/* ------------------------------------- */
extern uint8_t _sysctrl[LEN_SYS_CTRL];

extern uint8_t _deviceMode;

extern SPI_HandleTypeDef* _deviceHandle;  // TODO this is dangerous because we could use deviceHandle before it's initialization



/* -------------------- Functions Definitions ---------------------------- */

/* ------------------------------------- */
/* UTILITIES FUNCTION                    */
/* ------------------------------------- */
void setBit(uint8_t *data, uint16_t len, uint8_t bit, uint8_t val);



/* ------------------------------------- */
/* STATE MANAGEMENT FUNCTION             */
/* ------------------------------------- */
void idle(void);


/* ------------------------------------- */
/*  RAW READ-WRITE FUNCTIONS             */
/* ------------------------------------- */

/**
* @brief Read data on the DW via SPI
* @param *spiHandle the STM32 spi
* @param address the register address to read
* @param offset the offset address to read
* @param *data the Rx buffer to receive data
* @param len The length of the data to read
*/
void DWM_ReadSPI_ext(uint8_t address, uint16_t offset, uint8_t *data, uint16_t len);

/**
* @brief Write data on the DW via SPI
* @param *spiHandle the STM32 spi
* @param address the register address to write
* @param offset the offset address to write
* @param *data the Tx buffer to transfer data
* @param len The length of the data to write
*/
void DWM_WriteSPI_ext(uint8_t address, uint16_t offset, uint8_t *data, uint16_t len);

#endif /* __DWM1000_H */
