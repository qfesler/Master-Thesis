#define NO_SUB									0xFF
#define WRITE										0x80
#define WRITE_SUB								0xC0
#define READ										0x00
#define READ_SUB								0x40
#define RW_SUB_EXT							0x80


void DWM_ReadSPI_ext(SPI_HandleTypeDef *spiHandle, uint8_t address, uint16_t offset, uint8_t *data, uint16_t len);
