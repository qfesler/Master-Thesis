// library for the use of DWM1000 on STM32

/* TODO -DWM_init (irq setup) (not in the library)
        - Frame filtering if several tags
        - Calibration antenna delay
        -DWM_init
*/
#include "DWM1000lib.h"
#include "stm32f0xx_hal.h"


void DWM_reset(void){
	/* reset : 	- set SYSCLK to 01
							- Clear SOFTRESET
							- set SOFTRESET
							- then, load LDE (see manual p 23-24) */

	uint8_t RxUint8[4];
	uint8_t TxUint8[4];

	// Getting PMSC_CTRL0 register
	DWM_ReadSPI(DW1000_REGISTER_PMSC_CTRL0, RxUint8,4);
	uint32_t RxUint32 = uint8TOuint32(RxUint8);

	// Set SYSCLKS bits to 01
	RxUint32 = ( RxUint32 & 0xFFFFFFFC ) | 1;
	Uint32TOuint8 ( RxUint32, TxUint8 );
	DWM_WriteSPI(DW1000_REGISTER_PMSC_CTRL0, TxUint8, 4);

	// Clear SOFTRESET bits
	RxUint32 &= 0x0FFFFFFF;
	Uint32TOuint8 ( RxUint32, TxUint8 );
	DWM_WriteSPI(DW1000_REGISTER_PMSC_CTRL0, TxUint8, 4);

	// Set SOFTRESET bits
	RxUint32 |= 0xF0000000;
	RxUint32 &= 0xFFFFFFFC;
	Uint32TOuint8 ( RxUint32, TxUint8 );
	DWM_WriteSPI(DW1000_REGISTER_PMSC_CTRL0, TxUint8, 4);

	HAL_Delay(5);

        // Load the LDE algorithm microcode into LDE RAM or disable LDE execution (clear LDERUNE)

	TxUint8[0] = 0xF6;
	TxUint8[1] = 0x00;
	TxUint8[2] = 0x01;
	TxUint8[3] = 0x03;
	DWM_WriteSPI(DW1000_REGISTER_PMSC_CTRL0, TxUint8, 4);

	TxUint8[0] = 0xED;
	TxUint8[1] = 0x06;
	TxUint8[2] = 0x00;
	TxUint8[3] = 0x80;
	DWM_WriteSPI(DW1000_REGISTER_PMSC_CTRL0, TxUint8, 4);

	HAL_Delay(1);

	TxUint8[0] = 0xF6;
	TxUint8[1] = 0x00;
	TxUint8[2] = 0x00;
	TxUint8[3] = 0x02;
	DWM_WriteSPI(DW1000_REGISTER_PMSC_CTRL0, TxUint8, 4);

}

/* Check DWM ID , goes into error loop if bad ID */
void DWM_Init(void){

  DWM_reset();

	uint8_t SPIRxBuffer8[4];
	uint32_t SPIRxBuffer32;
  uint8_t SPITxBuffer8[4];
	uint32_t SPITxBuffer32;

  // Check DW ID
	DWM_ReadSPI(DWM1000_REG_DEV_ID, SPIRxBuffer8, 4);
	SPIRxBuffer32 = uint8TOuint32(SPIRxBuffer8);
	if (SPIRxBuffer32 != 0xDECA0130){
		Error_Fct();
	}

  // RXAUTR: Receiver auto-re-enable
  DWM_ReadSPI(DWM1000_REG_SYS_CFG, SPIRxBuffer8, 4);
  SPIRxBuffer32 = uint8TOuint32(SPIRxBuffer8);
  SPITxBuffer32 = SPIRxBuffer32 | 0x20000000;
  Uint32TOuint8(SPITxBuffer32, SPITxBuffer8);
  DWM_WriteSPI(DWM1000_REG_SYS_CFG, SPITxBuffer8, 4);

  // setup of the irq : MASK 0x00000080 TX OK
                      //MASK 0x00002000 RX FINISHED
                      //MASK 0x00004000 RX NO ERROR
  DWM_ReadSPI(DWM1000_REG_SYS_MASK, SPIRxBuffer8, 4);
  SPIRxBuffer32 = uint8TOuint32(SPIRxBuffer8);
  SPITxBuffer32 = SPIRxBuffer32 | 0x00000080; // TX OK
  SPITxBuffer32 = SPITxBuffer32 | 0x00004000; // RX OK NO ERROR
  Uint32TOuint8(SPITxBuffer32, SPITxBuffer8);
  DWM_WriteSPI(DWM1000_REG_SYS_MASK, SPITxBuffer8, 4);

  // Frame filtering

  // antenna delay

	// /!\ ....
}

void DWM_WriteSPI(uint8_t address, uint8_t *data, uint16_t len){
	uint8_t writeAdr = 0 | (address & 0x3F) | 0x80;
	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, &writeAdr, 1, HAL_MAX_DELAY);
	HAL_SPI_Transmit(&hspi1, data, len, HAL_MAX_DELAY);
	HAL_Delay(100);
	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);
}

void DWM_ReadSPI(uint8_t adress, uint8_t *data, uint16_t len){
	uint8_t readAdr = 0 | (adress & 0x3F) ;
	uint8_t DUMMYBYTE[len];
	int i;
	for (i=0; i<len; i++){
		DUMMYBYTE[i] = 0;
	}
	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, &readAdr, 1, HAL_MAX_DELAY);
	HAL_SPI_TransmitReceive(&hspi1, DUMMYBYTE, data, len, HAL_MAX_DELAY);
	HAL_Delay(100);
	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);
}

uint32_t uint8TOuint32(uint8_t *data){
	return 0 | (data[3] << 24) | (data[2] << 16) | (data[1] << 8) | data[0];
}

void Uint32TOuint8 ( uint32_t from, uint8_t *to ) {
	to[3] = (from & 0xFF000000) >> 24;
	to[2] = (from & 0xFF0000) >> 16;
	to[1] = (from & 0xFF00) >> 8;
	to[0] = from & 0xFF;
}

void Error_Fct(void){
	while (1){
		HAL_GPIO_TogglePin(GPIOC, LD3_Pin);
		HAL_Delay(100);
	}
}
