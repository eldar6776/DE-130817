/**
  ******************************************************************************
  * File Name          : main.hpp
  * Description        : Receiver modul main function header
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
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
#include "stm32f1xx.h"


/* Typedef -------------------------------------------------------------------*/
/* Define --------------------------------------------------------------------*/
//#define EE_WRITE_DEFAULT			1			// uncoment if need to write default values to eeprom
#define ULINK_DEBUGGER  			2			// uncoment if swdio and swclk line used by debugger
#define EE_BUFFER_SIZE				16			// reserved RAM buffer for eeprom related operation
#define EE_TIMEOUT     				20			// 20 ms eeprom operation timeout
#define EE_BLOCK_SIZE				256			// max. byte number in one block for single write operation
#define EE_READ  					0xaf		// command to read page 0 from i2c eeprom
#define EE_WRITE 					0xae		// command to write page 0 in i2c eeprom
#define EE_PAGE_SIZE				0x10000		// 64 kByte eeprom page size
#define EE_PAGE_COUNT				512			// number of pages
#define EE_WRITE_DELAY				10			// 10 ms delay to complete write operation
#define EE_TRIALS					100			// number of i2c operation trials


/** ==========================================================================*/
/**                                                                           */
/**     	I 2 C    E E P R O M    M E M O R Y    A D D R E S S E      	  */ 
/**                                                                           */
/** ==========================================================================*/
#define EE_NTC_BYPASS_VOLTAGE_ADDRESS				0x00
#define EE_NTC_OPEN_VOLTAGE_ADDRESS					0x02
#define EE_NTC_WARNING_TEMPERATURE_ADDRESS			0x04
#define EE_NTC_SHUTDOWN_TEMPERATURE_ADDRESS			0x06
#define EE_SUPPLY_VOLTAGE_SHUTDOWN_ADDRESS			0x08

#define EE_PWM_0_15_FREQUENCY_ADDRESS				0x10
#define EE_PWM_16_31_FREQUENCY_ADDRESS				0x12

#define EE_RS485_INTERFACE_ADDRESS					0x20
#define EE_RS485_GROUP_ADDRESS						0x22
#define EE_RS485_BROADCAST_ADDRESS					0x24
#define EE_RS485_BAUDRATE_ADDRESS					0x26

#define EE_HC12_BAUDRATE_ADDRESS					0x30
#define EE_HC12_CHANEL_ADDRESS						0x31
#define EE_HC12_LINK_TIMEOUT_ADDRESS				0x32

#define EE_NRF24L01_BAUDRATE_ADDRESS				0x40
#define EE_NRF24L01_CHANEL_ADDRESS					0x41
#define EE_NRF24L01_LINK_TIMEOUT_ADDRESS			0x42

#define EE_NRF905_BAUDRATE_ADDRESS					0x50
#define EE_NRF905_BAND_ADDRESS						0x51
#define EE_NRF905_CHANEL_ADDRESS					0x52
#define EE_NRF905_LINK_TIMEOUT_ADDRESS				0x53


/* ########################## Assert Selection ############################## */
/**
  * @brief Uncomment the line below to expanse the "assert_param" macro in the 
  *        HAL drivers code
  */
/* #define USE_FULL_ASSERT    1U */


/* Variable ------------------------------------------------------------------*/
extern uint8_t eeprom_buffer[EE_BUFFER_SIZE];

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
