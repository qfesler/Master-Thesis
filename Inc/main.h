/**
  ******************************************************************************
  * File Name          : main.hpp
  * Description        : This file contains the common defines of the application
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H
  /* Includes ------------------------------------------------------------------*/

/* Includes ------------------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define NCS_MEMS_SPI_Pin GPIO_PIN_0
#define NCS_MEMS_SPI_GPIO_Port GPIOC
#define MEMS_INT1_Pin GPIO_PIN_1
#define MEMS_INT1_GPIO_Port GPIOC
#define MEMS_INT1_EXTI_IRQn EXTI0_1_IRQn
#define MEMS_INT2_Pin GPIO_PIN_2
#define MEMS_INT2_GPIO_Port GPIOC
#define MEMS_INT2_EXTI_IRQn EXTI2_3_IRQn
#define B1_Pin GPIO_PIN_0
#define B1_GPIO_Port GPIOA
#define B1_EXTI_IRQn EXTI0_1_IRQn
#define EXT_RESET_Pin GPIO_PIN_5
#define EXT_RESET_GPIO_Port GPIOC
#define I2C2_SCL_Pin GPIO_PIN_10
#define I2C2_SCL_GPIO_Port GPIOB
#define I2C2_SDA_Pin GPIO_PIN_11
#define I2C2_SDA_GPIO_Port GPIOB
#define SPI2_SCK_Pin GPIO_PIN_13
#define SPI2_SCK_GPIO_Port GPIOB
#define SPI2_MISO_Pin GPIO_PIN_14
#define SPI2_MISO_GPIO_Port GPIOB
#define SPI2_MOSI_Pin GPIO_PIN_15
#define SPI2_MOSI_GPIO_Port GPIOB
#define LD3_Pin GPIO_PIN_6
#define LD3_GPIO_Port GPIOC
#define LD6_Pin GPIO_PIN_7
#define LD6_GPIO_Port GPIOC
#define LD4_Pin GPIO_PIN_8
#define LD4_GPIO_Port GPIOC
#define LD5_Pin GPIO_PIN_9
#define LD5_GPIO_Port GPIOC
#define USBF4_DM_Pin GPIO_PIN_11
#define USBF4_DM_GPIO_Port GPIOA
#define USBF4_DP_Pin GPIO_PIN_12
#define USBF4_DP_GPIO_Port GPIOA
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define SPI_IRQ_Pin GPIO_PIN_3
#define SPI_IRQ_GPIO_Port GPIOB
#define SPI_IRQ_EXTI_IRQn EXTI2_3_IRQn
#define SPI1_CS_Pin GPIO_PIN_6
#define SPI1_CS_GPIO_Port GPIOB

/* ########################## Assert Selection ############################## */
/**
  * @brief Uncomment the line below to expanse the "assert_param" macro in the 
  *        HAL drivers code
  */
/* #define USE_FULL_ASSERT    1U */

/* USER CODE BEGIN Private defines */

#define TX_STANDARD_MESSAGE			0xA5
#define RX_STANDARD_MESSAGE			0x5A

#define ANTENNA_DELAY 					0x8066 // /!\ From decarduino 

#define TX_OK_MASK							0x00000080 // TX OK
#define RX_FINISHED_MASK				0x00002000 // RX FINISHED
#define RX_NO_ERROR_MASK				0x00004000 // RX NO ERROR

// State Machine
#define STATE_INIT							1
#define STATE_WAIT_FIRST_SEND		2
#define STATE_WAIT_RESPONSE			3
#define STATE_WAIT_SECOND_SEND	4
#define STATE_GET_TIMES					5
#define STATE_COMPUTE_DISTANCE	6

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

/* USER CODE END Private defines */

#ifdef __cplusplus
 extern "C" {
#endif
void _Error_Handler(char *, int);

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)
#ifdef __cplusplus
}
#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

#endif /* __MAIN_H */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
