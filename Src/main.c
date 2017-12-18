/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f0xx_hal.h"

/* USER CODE BEGIN Includes */
#include "EventRecorder.h"              // Keil.ARM Compiler::Compiler:Event Recorder
#include "stdio.h"
#include "dwm1000.h"
#include <string.h>
#include <inttypes.h>
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
uint8_t RxData[128];
uint8_t TxData[128];

uint64_t t1, t2, t3, t4, t5, t6;
uint8_t t2_8[5];
uint8_t t3_8[5];
uint8_t t6_8[5];

uint64_t tof;

uint64_t distance;

#define AIR_SPEED_OF_LIGHT 229702547.0
#define DW1000_TIMEBASE 15.65E-12
#define COEFF AIR_SPEED_OF_LIGHT*DW1000_TIMEBASE

// Boolean variables
int TxOk = 0;
int RxOk = 0;

int state;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void DWM_SendData(uint8_t* data, uint8_t len);
void DWM_WriteSpiUint32(uint8_t address, uint32_t ui32t);
void DWM_Disable_Rx(void);
uint32_t DWM_ReadSpiUint32(uint8_t address);
void DWM_ReceiveData(uint8_t* buffer);
void DWM_Init(void);
void DWM_reset(void);
uint64_t DWM_GetTimeStamp(uint8_t reg);
void DWM_Enable_Rx(void);
uint32_t uint8TOuint32(uint8_t *data);
void DWM_ReadSPI(uint8_t adress, uint8_t *data, uint16_t len);

void Error_Fct(void);
void Uint32TOuint8 ( uint32_t from, uint8_t *to );
uint64_t uint8TOuint64(uint8_t *data);
void DWM_WriteSPI(uint8_t address, uint8_t *data, uint16_t len);

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */



/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */
	EventRecorderInitialize (EventRecordAll, 1);
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI1_Init();

  /* USER CODE BEGIN 2 */
	_deviceHandle = &hspi1;					// Assign SPI handle
	
	printf("Hello World \n ");
	/* initialisation of the DecaWave */
	HAL_Delay(10); //time for the DW to go from Wakeup to init and then IDLE
	DWM_Init();
	state = STATE_INIT;
	
	DWM_WriteSpiUint32(DWM1000_REG_PANADR, ADRESS_AND_PAN);
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
#ifdef MASTER_BOARD
		switch (state){
			case STATE_INIT : 
				DWM_Disable_Rx();
					// can set the DW in IDLE to save battery and re-enable after it (16µs delay)
				HAL_Delay(1000); // 1sec between 2 measures
					
					//Send first data
				TxData[0] = TX_STANDARD_MESSAGE;
			TxData[1] = TX_STANDARD_MESSAGE;
			TxData[2] = TX_STANDARD_MESSAGE;
			TxData[3] = TX_STANDARD_MESSAGE;
			printf("sending data\n");
				DWM_SendData(TxData, 4);
					//Change state to wait TX OK (polling)
				printf("Wait TX \n");
				state = STATE_WAIT_FIRST_SEND;
				HAL_GPIO_WritePin(GPIOC, LD6_Pin, GPIO_PIN_SET);
			
				HAL_Delay(100);
			
			
	/*	EF	
	uint8_t RxBuffer[4];
	//uint8_t TxBuffer[4];
	uint32_t StatusRegister;
	//uint32_t ack = 0;
	// Getting status Register
	DWM_ReadSPI(DWM1000_REG_DX_TIME, RxBuffer, 4);
	StatusRegister = uint8TOuint32(RxBuffer);
	
	printf("%" PRIu32 "\n", StatusRegister);
		*/		
			break;
			
			case STATE_WAIT_FIRST_SEND:

				if (TxOk){
					//get tx time (T1)
					t1 = DWM_GetTimeStamp(DWM1000_REG_TX_TIME);
					state = STATE_WAIT_RESPONSE;
					TxOk = 0;
					DWM_Enable_Rx();
					printf("Tx OK \n");
				}
				
			break;
				
			case STATE_WAIT_RESPONSE:
				if (RxOk){
					// Read Rx buffer
					DWM_ReceiveData(RxData);
					printf("RX sth");
					
					// Check RxFrame
					if (RxData[0] == RX_STANDARD_MESSAGE){
						//get rx time (t4)
						t4 = DWM_GetTimeStamp(DWM1000_REG_RX_TIME);
						
						//Send second time
						DWM_SendData(TxData, 1);
						state = STATE_WAIT_SECOND_SEND;
					}
					RxOk = 0;
				}
			break;
				
			case STATE_WAIT_SECOND_SEND:
					if (TxOk){
					//get tx time (T5)
					t5 = DWM_GetTimeStamp(DWM1000_REG_TX_TIME);
					state = STATE_GET_TIMES;
					TxOk = 0;
					DWM_Enable_Rx();
				}
			break;
			
			case STATE_GET_TIMES:
				if (RxOk){
					//Read Rx Buffer
					DWM_ReceiveData(RxData);
					
					for (int i=0;i<5;i++){
						t2_8[i] = RxData[i];
						t3_8[i] = RxData[i+5];
						t6_8[i] = RxData[i+10];
					}
					t2 = uint8TOuint64(t2_8);
					t3 = uint8TOuint64(t3_8);
					t6 = uint8TOuint64(t6_8);
					state = STATE_COMPUTE_DISTANCE;
					RxOk = 0;
				}
			break;
				
			case STATE_COMPUTE_DISTANCE :
				tof = (2*t4 - t1 - 2*t3 + t2 + t6 - t5)/4;
				
			distance = tof*COEFF;
			printf("%"PRIu64"\n", distance);
				
				state = STATE_INIT;
			break;
		}
#endif
		
#ifdef SLAVE_BOARD
#endif
  }

  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* SPI1 init function */
static void MX_SPI1_Init(void)
{

  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
     PA2   ------> TSC_G1_IO3
     PA3   ------> TSC_G1_IO4
     PA6   ------> TSC_G2_IO3
     PA7   ------> TSC_G2_IO4
     PB0   ------> TSC_G3_IO2
     PB1   ------> TSC_G3_IO3
     PB10   ------> I2C2_SCL
     PB11   ------> I2C2_SDA
     PB13   ------> SPI2_SCK
     PB14   ------> SPI2_MISO
     PB15   ------> SPI2_MOSI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, NCS_MEMS_SPI_Pin|EXT_RESET_Pin|LD3_Pin|LD6_Pin 
                          |LD4_Pin|LD5_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : NCS_MEMS_SPI_Pin EXT_RESET_Pin LD3_Pin LD6_Pin 
                           LD4_Pin LD5_Pin */
  GPIO_InitStruct.Pin = NCS_MEMS_SPI_Pin|EXT_RESET_Pin|LD3_Pin|LD6_Pin 
                          |LD4_Pin|LD5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : MEMS_INT1_Pin MEMS_INT2_Pin */
  GPIO_InitStruct.Pin = MEMS_INT1_Pin|MEMS_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PA2 PA3 PA6 PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF3_TSC;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF3_TSC;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : I2C2_SCL_Pin I2C2_SDA_Pin */
  GPIO_InitStruct.Pin = I2C2_SCL_Pin|I2C2_SDA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF1_I2C2;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : SPI2_SCK_Pin SPI2_MISO_Pin SPI2_MOSI_Pin */
  GPIO_InitStruct.Pin = SPI2_SCK_Pin|SPI2_MISO_Pin|SPI2_MOSI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF0_SPI2;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI_IRQ_Pin */
  GPIO_InitStruct.Pin = SPI_IRQ_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SPI_IRQ_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI1_CS_Pin */
  GPIO_InitStruct.Pin = SPI1_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SPI1_CS_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_1_IRQn);

  HAL_NVIC_SetPriority(EXTI2_3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_3_IRQn);

}

/* USER CODE BEGIN 4 */

void DWM_SendData(uint8_t* data, uint8_t len){ // data limited to 125 byte long
	
	uint32_t ui32t;
	
	
	/* ** ERIC **/
	idle();
	/* ** ERIC ** */
	DWM_WriteSPI(DWM1000_REG_TX_BUFFER, data, len);
	
	// maj frame length
	ui32t = DWM_ReadSpiUint32(DWM1000_REG_TX_FCTRL);
	
	ui32t = (ui32t & 0xFFFFE000) | (len+2); // FCS is 2-bytes long
	printf("fx:%"PRIx32"\n", ui32t);
	DWM_WriteSPI_ext(DWM1000_REG_TX_FCTRL, NO_SUB, (uint8_t*)&ui32t, 4);
	//uint8_t flen = len+2; //FCS 2byte long
	//DWM_WriteSPI(DWM1000_REG_TX_FCTRL, &flen ,1);
	
	// START SENDING
	uint32_t TxStartBit = 2;
	DWM_WriteSpiUint32(DWM1000_REG_SYS_CTRL, TxStartBit);
}

void DWM_ReceiveData(uint8_t* buffer){
	// Get frame length
	uint8_t flen;
	DWM_ReadSPI(DWM1000_REG_RX_FINFO, &flen, 1);
	flen = flen-2; // FCS 2 Byte long
	
	//reading data
	DWM_ReadSPI(DWM1000_REG_RX_BUFFER, buffer, flen);
}


uint64_t DWM_GetTimeStamp(uint8_t reg){
	uint8_t time8[5];
	uint64_t time = 0;
	DWM_ReadSPI(reg, time8, 5);
	time = time8[4];
	time = time << 32; // doesnt work otherwise : says shift too large
	time = time |(time8[3] << 24) | (time8[2] << 16) | (time8[1] << 8) | time8[0];
	return time;
}


uint64_t uint8TOuint64(uint8_t *data){
	uint64_t tmp = 0;
	tmp = data[4];
	tmp = tmp << 32; // doesnt work otherwise : says shift too large
	tmp = tmp |(data[3] << 24) | (data[2] << 16) | (data[1] << 8) | data[0];
	return tmp;
	
}

void DWM_reset(void){
	/* reset : 	- set SYSCLK to 01
							- Clear SOFTRESET
							- set SOFTRESET
							- then, load LDE (see manual p 23-24) */

	uint8_t RxUint8[4];
	uint8_t TxUint8[4];

	// Getting PMSC_CTRL0 register
	DWM_ReadSPI(DWM1000_REG_PMSC, RxUint8,4);
	uint32_t RxUint32 = uint8TOuint32(RxUint8);
	HAL_Delay(1);

	// Set SYSCLKS bits to 01
	RxUint32 = ( RxUint32 & 0xFFFFFFFC ) | 1;
	Uint32TOuint8 ( RxUint32, TxUint8 );
	DWM_WriteSPI(DWM1000_REG_PMSC, TxUint8, 4);
	HAL_Delay(1);

	// Clear SOFTRESET bits
	RxUint32 &= 0x0FFFFFFF;
	Uint32TOuint8 ( RxUint32, TxUint8 );
	DWM_WriteSPI(DWM1000_REG_PMSC, TxUint8, 4);
	HAL_Delay(1);

	// Set SOFTRESET bits
	RxUint32 |= 0xF0000000;
	RxUint32 &= 0xFFFFFFFC;
	Uint32TOuint8 ( RxUint32, TxUint8 );
	DWM_WriteSPI(DWM1000_REG_PMSC, TxUint8, 4);

	HAL_Delay(5);

        // Load the LDE algorithm microcode into LDE RAM or disable LDE execution (clear LDERUNE)

	TxUint8[0] = 0xF6;
	TxUint8[1] = 0x00;
	TxUint8[2] = 0x01;
	TxUint8[3] = 0x03;
	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, TxUint8, 4, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);

	TxUint8[0] = 0xED;
	TxUint8[1] = 0x06;
	TxUint8[2] = 0x00;
	TxUint8[3] = 0x80;
	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, TxUint8, 4, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);
	
	HAL_Delay(1);

	TxUint8[0] = 0xF6;
	TxUint8[1] = 0x00;
	TxUint8[2] = 0x00;
	TxUint8[3] = 0x02;
	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, TxUint8, 4, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);
	
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
		printf("Wrong DW ID \n");
		Error_Fct();
	}

  // RXAUTR: Receiver auto-re-enable
  DWM_ReadSPI(DWM1000_REG_SYS_CFG, SPIRxBuffer8, 4);
  SPIRxBuffer32 = uint8TOuint32(SPIRxBuffer8);
  //SPITxBuffer32 = SPIRxBuffer32 | 0x20000000;
	SPITxBuffer32 = 0x20001200;
  Uint32TOuint8(SPITxBuffer32, SPITxBuffer8);
  DWM_WriteSPI(DWM1000_REG_SYS_CFG, SPITxBuffer8, 4);
	

	
	// CHAN_CTRL
	DWM_ReadSPI(DWM1000_REG_CHAN_CTRL, SPIRxBuffer8, 4);
  SPIRxBuffer32 = uint8TOuint32(SPIRxBuffer8);
	SPIRxBuffer32 &= 0xFFC5FFFF;
	SPIRxBuffer32 |= 0x00040000;
	Uint32TOuint8(SPITxBuffer32, SPITxBuffer8);
  DWM_WriteSPI(DWM1000_REG_CHAN_CTRL, SPITxBuffer8, 4);
	
	// F_CTRL
	uint8_t Rx5Buf[5];
	DWM_ReadSPI(DWM1000_REG_TX_FCTRL, Rx5Buf, 5);
		
	for(int i=0;i<5;i++){
		printf("%"PRIx8"\n", Rx5Buf[4-i]);
	}
	
	printf("He \n");
	uint8_t TxUint8[5];
	TxUint8[0] = DWM1000_REG_TX_FCTRL | 0x80;
	TxUint8[1] = 0x0C;
	TxUint8[2] = 0x80;
	TxUint8[3] = 0x15;
	TxUint8[4] = 0x00;
	//TxUint8[5] = 0x00;
	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, TxUint8, 5, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);
//	DWM_ReadSPI(DWM1000_REG_TX_FCTRL, SPIRxBuffer8, 4);
//  SPIRxBuffer32 = uint8TOuint32(SPIRxBuffer8);
//	//SPIRxBuffer32 &= 0xFFC083FF;
//	SPIRxBuffer32 &= 0xFFFF00FF;
//	//SPIRxBuffer32 |= 0x00292000;
//	SPIRxBuffer32 |= 0x00008000;
//	Uint32TOuint8(SPITxBuffer32, SPITxBuffer8);
//  DWM_WriteSPI(DWM1000_REG_TX_FCTRL, SPITxBuffer8, 4);
		DWM_ReadSPI(DWM1000_REG_TX_FCTRL, Rx5Buf, 5);
		
	for(int i=0;i<5;i++){
		printf("%"PRIx8"\n", Rx5Buf[4-i]);
	}
	
	
	// enable receiver
	SPITxBuffer32 = 0x00000100;
	Uint32TOuint8(SPITxBuffer32, SPITxBuffer8);
	DWM_WriteSPI(DWM1000_REG_SYS_CTRL, SPITxBuffer8, 4);

  // setup of the irq : MASK 0x00000080 TX OK
                      //MASK 0x00002000 RX FINISHED
                      //MASK 0x00004000 RX NO ERROR
  DWM_ReadSPI(DWM1000_REG_SYS_MASK, SPIRxBuffer8, 4);
  SPIRxBuffer32 = uint8TOuint32(SPIRxBuffer8);
  SPITxBuffer32 = SPIRxBuffer32 | TX_OK_MASK; // TX OK
  SPITxBuffer32 = SPITxBuffer32 | RX_NO_ERROR_MASK; // RX OK NO ERROR
  Uint32TOuint8(SPITxBuffer32, SPITxBuffer8);
  DWM_WriteSPI(DWM1000_REG_SYS_MASK, SPITxBuffer8, 4);

  // Frame filtering

  // antenna delay
	uint16_t delayuint16 = ANTENNA_DELAY;
	uint8_t delayuint8[2];
	delayuint8[1] = (delayuint16 & 0xFF00) >>8;
	delayuint8[0] = (delayuint16 & 0xFF);
	DWM_WriteSPI(DWM1000_REG_TX_ANTD, delayuint8, 2);

	// /!\ ....
	
	// ERIC - Check SYS_STATUS
		// clear IRQ flags on DW
	uint32_t ack = 0 | TX_OK_MASK;	
	Uint32TOuint8 ( ack, SPITxBuffer8 );
	DWM_WriteSPI(DWM1000_REG_SYS_STATUS, SPITxBuffer8, 4);
	HAL_Delay(100);
	/*
	DWM_ReadSPI(DWM1000_REG_SYS_STATUS, SPIRxBuffer8, 4);
  SPIRxBuffer32 = uint8TOuint32(SPIRxBuffer8);
  SPITxBuffer32 = SPIRxBuffer32 | TX_OK_MASK; // TX OK
  Uint32TOuint8(SPITxBuffer32, SPITxBuffer8);
  DWM_WriteSPI(DWM1000_REG_SYS_STATUS, SPITxBuffer8, 4);

  HAL_Delay(1000);
	*/
	
	printf("Check SYS_STATUS, last bit should be at 1\n");
	DWM_ReadSPI(DWM1000_REG_SYS_STATUS, Rx5Buf, 5);
		
	for(int i=0;i<5;i++){
		printf("%"PRIx8"\n", Rx5Buf[4-i]);
	}
	
	printf("Check SYS_MASK, bit 7 should be at 1\n");
	DWM_ReadSPI(DWM1000_REG_SYS_MASK, SPIRxBuffer8, 4);
  SPIRxBuffer32 = uint8TOuint32(SPIRxBuffer8);
	printf("%"PRIx32"\n", SPIRxBuffer32);
	

	
}

void DWM_WriteSPI(uint8_t address, uint8_t *data, uint16_t len){
	uint8_t writeAdr = 0 | (address & 0x3F) | 0x80;
	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, &writeAdr, 1, HAL_MAX_DELAY);
	HAL_SPI_Transmit(&hspi1, data, len, HAL_MAX_DELAY);
	//HAL_Delay(100);
	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);
}

void DWM_WriteSpiUint32(uint8_t address, uint32_t ui32t) {

	uint8_t buf[4];

	Uint32TOuint8(ui32t, buf);
	DWM_WriteSPI(address, buf, 4);
}

uint32_t DWM_ReadSpiUint32(uint8_t address) {

	uint8_t buf[4];

	DWM_ReadSPI(address, buf, 4);
	return uint8TOuint32(buf);
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
	//HAL_Delay(100);
	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);
}



uint32_t uint8TOuint32(uint8_t *data){
	return 0 | (data[3] << 24) | (data[2] << 16) | (data[1] << 8) | data[0];
}

void DWM_Enable_Rx(void){
	uint8_t TxBuf8[4];
	uint32_t TxBuf32 = 0x00000100;
	Uint32TOuint8(TxBuf32, TxBuf8);
	DWM_WriteSPI(DWM1000_REG_SYS_CTRL, TxBuf8, 4);
}

void DWM_Disable_Rx(void){
	uint8_t TxBuf8[4];
	uint32_t TxBuf32 = 0x00000000;
	Uint32TOuint8(TxBuf32, TxBuf8);
	DWM_WriteSPI(DWM1000_REG_SYS_CTRL, TxBuf8, 4);
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

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	if (GPIO_Pin != SPI_IRQ_Pin){return;}
	
	uint8_t RxBuffer[4];
	uint8_t TxBuffer[4];
	uint32_t StatusRegister;
	uint32_t ack = 0;
	
	
	// Getting status Register
	DWM_ReadSPI(DWM1000_REG_SYS_STATUS, RxBuffer, 4);
	StatusRegister = uint8TOuint32(RxBuffer);
	if (StatusRegister == 0xFFFFFFFF) {
		// Read again
		printf("DWM was in sleep mode\n");
		DWM_ReadSPI(DWM1000_REG_SYS_STATUS, RxBuffer, 4);
		StatusRegister = uint8TOuint32(RxBuffer);
	}
	//printf("%"PRIx32"\n", StatusRegister);
	// check if Tx OK
	if (StatusRegister & TX_OK_MASK){
		TxOk = 1;
		ack |= TX_OK_MASK;
	}
	// check if RX finished
	if (StatusRegister & RX_FINISHED_MASK){
		ack |= RX_FINISHED_MASK;
		// check in no error in RX
		if (StatusRegister & RX_NO_ERROR_MASK){
			RxOk = 1;
			ack |= RX_NO_ERROR_MASK;
		}
	}
	// clear IRQ flags on DW
	Uint32TOuint8 ( ack, TxBuffer );
	DWM_WriteSPI(DWM1000_REG_SYS_STATUS, TxBuffer, 4);
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
