
#include <inttypes.h>
#include <string.h>
#include "main.h"
#include "stm32f0xx_hal.h"

#include "dwm1000.h"

/* ----------------------------------------------------------------------- */
uint8_t _sysctrl[LEN_SYS_CTRL];

uint8_t _deviceMode;

SPI_HandleTypeDef* _deviceHandle;


/* ---------------------- Functions -------------------------------------- */

/* ------------------------------------- */
/* STATE MANAGEMENT FUNCTION             */
/* ------------------------------------- */
void idle() {
	// Set SYS_CTRL to 0
	memset(_sysctrl, 0, LEN_SYS_CTRL);
	// Set bit TRXOFF to 1
	setBit(_sysctrl, LEN_SYS_CTRL, TRXOFF_BIT, 1);
	// Set the device in IDLE mode
	_deviceMode = IDLE_MODE;
	// Update the DWM1000 module
	DWM_WriteSPI_ext(DWM1000_REG_SYS_CTRL, NO_SUB, _sysctrl, LEN_SYS_CTRL);
}

/* ------------------------------------- */
/*  BITs AND BYTEs  FUNCTIONS            */
/* ------------------------------------- */
void setBit(uint8_t *data, uint16_t len, uint8_t bit, uint8_t val) {
	uint16_t idx;
	uint8_t shift;
	
	idx = bit/8;
	if (idx >= len) return;
	
	uint8_t* targetByte = &data[idx];
	shift = bit%8;
	if (val) {
		Bitset(*targetByte, shift);
	} else {
		Bitclear(*targetByte, shift);
	}
}

/* ------------------------------------- */
/*  RAW READ-WRITE FUNCTIONS             */
/* ------------------------------------- */

void DWM_ReadSPI_ext(uint8_t address, uint16_t offset, uint8_t *data, uint16_t len) {
	uint8_t header[3];
	uint8_t headerLen = 1;
	uint8_t DUMMY_BYTE[len];
	memset(DUMMY_BYTE, 0, len);
	
	if (offset == NO_SUB) {
		header[0] = address | READ;
	} else {
		header[0] = address | READ_SUB;
		if(offset < 128) {
			header[1] = (uint8_t)offset;
			headerLen++;
		} else {
			header[1] = (uint8_t)offset | RW_SUB_EXT;
			header[2] = (uint8_t)(offset >> 7);
			headerLen += 2;
		}		
	}
	
	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(_deviceHandle, header, headerLen, HAL_MAX_DELAY);
	HAL_SPI_TransmitReceive(_deviceHandle, DUMMY_BYTE, data, len, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);
}

void DWM_WriteSPI_ext(uint8_t address, uint16_t offset, uint8_t *data, uint16_t len) {
	uint8_t header[3];
	uint8_t headerLen = 1;
	uint16_t i = 0;
	
	if (offset == NO_SUB) { 
		header[0] = address | WRITE;
	} else {
		header[0] = address | WRITE_SUB;
		if(offset < 128) {
			header[1] = (uint8_t)offset;
			headerLen++;
		} else {
			header[1] = (uint8_t)offset | RW_SUB_EXT;
			header[2] = (uint8_t)(offset >> 7);
			headerLen += 2;
		}		
	}
	
	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);
	// Hack write byte by byte to avoid hardfault on HAL_SPI_Transmit with non 2 bytes aligned data
	for (i=0; i < headerLen; i++) {
		HAL_SPI_Transmit(_deviceHandle, &header[i], 1, HAL_MAX_DELAY);
	}
	for (i=0; i < len; i++) {
		HAL_SPI_Transmit(_deviceHandle, &data[i], 1, HAL_MAX_DELAY);
	}

	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);
}

//Aim to change the order of the data to read/write from "human" order to "machine" order
//to see how to deal with C pointers.
//void Change_Data_order(
