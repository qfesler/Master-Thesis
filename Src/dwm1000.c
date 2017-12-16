
#include <inttypes.h>
#include <string.h>
#include "main.h"
#include "stm32f0xx_hal.h"

#include "dwm1000.h"

void DWM_ReadSPI_ext(SPI_HandleTypeDef *spiHandle, uint8_t address, uint16_t offset, uint8_t *data, uint16_t len) {
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
	HAL_SPI_Transmit(spiHandle, header, headerLen, HAL_MAX_DELAY);
	HAL_SPI_TransmitReceive(spiHandle, DUMMY_BYTE, data, len, HAL_MAX_DELAY);
	//HAL_Delay(100);
	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);
}
