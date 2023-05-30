/**
  ******************************************************************************
  * @file    uart.h
  * @author  CMS Application Team
  * @version Vx.x.x
  * @date    24-April-2022
  * @brief   This file provides firmware functions to manage the following 
  *          functionalities of the Universal asynchronous receiver
  *          transmitter (UART):
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 Cmsemicon.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/

#ifndef __UART_H__
#define __UART_H__
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include "sci_common.h"

//Need to delete after SCI framework is ready
#define UART0_TX      SCI0->SCI.TXD0
#define UART0_RX      SCI0->SCI.RXD0
#define UART1_TX      SCI0->SCI.TXD1
#define UART1_RX      SCI0->SCI.RXD1
#define UART2_TX      SCI1->SCI.TXD2
#define UART2_RX      SCI1->SCI.RXD2
#define UART3_TX      SCI2->SCI.TXD3
#define UART3_RX      SCI2->SCI.RXD3

#define PLL_SR_FIH_ENABLE						0
#if PLL_SR_FIH_ENABLE
	#define PLL_SR_FIH   
#endif
#define PLL_SR_FIH_ENABLE						0

#if PLL_SR_FMX_ENABLE
	#define PLL_SR_FMX_SYSCLK   
#endif
								
#define PLL_SR_FMX_FREQ                          640000000

#if defined( BAT32G1XX_100PIN) || defined( BAT32G1XX_80PIN)
#define IS_UART_ALL_PERIPH(PERIPH)	\
									(((PERIPH) == UART0) || ((PERIPH) == UART1) || ((PERIPH) == UART2)|| ((PERIPH) == UART3))
#else
#define IS_UART_ALL_PERIPH(PERIPH)	\
									(((PERIPH) == UART0) || ((PERIPH) == UART1) || ((PERIPH) == UART2))

#endif
/** @defgroup UART_Word_Length 
  * @{
  */ 
#define UART_WordLength_7b			((uint16_t)0x0006)
#define UART_WordLength_8b			((uint16_t)0x0007)
#define UART_WordLength_9b			((uint16_t)0x0008)
#define UART_WordLength_16b			((uint16_t)0x000E)
#define IS_UART_WORD_LENGTH(LENGTH)	(((LENGTH) == UART_WordLength_7b) || \
									((LENGTH) == UART_WordLength_8b) || \
									((LENGTH) == UART_WordLength_9b) || \
									((LENGTH) == UART_WordLength_16b))

/** @defgroup UART_Stop_Bits 
  * @{
  */ 
  
#define UART_StopBits_1				((uint16_t)0x0010)
#define UART_StopBits_2				((uint16_t)0x0020)
#define IS_UART_STOPBITS(STOPBITS)	(((STOPBITS) == UART_StopBits_1) || \
									((STOPBITS) == UART_StopBits_2))
/**
  * @}
  */ 

/** @defgroup UART_Parity 
  * @{
  */ 
  
#define UART_Parity_No				((uint16_t)0x0000)
#define UART_Parity_Even			((uint16_t)0x0200)
#define UART_Parity_Odd				((uint16_t)0x0300)
#define IS_UART_PARITY(PARITY)		(((PARITY) == UART_Parity_No)  || \
                                	((PARITY) == UART_Parity_Even) || \
                                	((PARITY) == UART_Parity_Odd))

/** @defgroup UART_Mode 
  * @{
  */ 
  
#define UART_Mode_Rx				((uint16_t)0x0012)
#define UART_Mode_Tx				((uint16_t)0x0022)
#define IS_UART_MODE(MODE)			((((MODE) & (uint16_t)0x00CD) == 0x00) && ((MODE) != (uint16_t)0x00))


/** @defgroup UART_bitorder 
  * @{
  */ 
#define UART_Bit_LSB				((uint16_t)0x0080)
#define UART_Bit_MSB				((uint16_t)0x0000)


/** @defgroup UART_phase 
  * @{
  */ 
#define UART_Phase_Normal			((uint16_t)0x0000)
#define UART_Phase_Reverse			((uint16_t)0x0040)

#define UART_PHASE_MASK				((uint16_t)0x0005)
#define UART0_PHASE_POS				((uint16_t)((1 << 0) & UART_PHASE_MASK))
#define UART1_PHASE_POS				((uint16_t)((1 << 2) & UART_PHASE_MASK))
#define UART2_PHASE_POS				((uint16_t)((1 << 0) & UART_PHASE_MASK))

#define UART_CTRL_POS				((uint16_t)0x01)

/** @defgroup UART_Flags 
  * @{
  */

#define UART_FLAG_TSF				((uint16_t)0x0040)
#define UART_FLAG_BFF               ((uint16_t)0x0020)
#define UART_FLAG_FEF               ((uint16_t)0x0004)
#define UART_FLAG_PEF               ((uint16_t)0x0002)
#define UART_FLAG_OVF				((uint16_t)0x0001)
#define IS_UART_FLAG(FLAG)			((((FLAG) & (uint16_t)0xFF98) == 0x00) && ((FLAG) != (uint16_t)0x00))

#define IS_UART_CLEAR_FLAG(FLAG)	((((FLAG) & (uint16_t)0xFFF8) == 0x00) && ((FLAG) != (uint16_t)0x00))

typedef struct
{
	uint32_t UART_BaudRate;				/*!< This member configures the UART communication baud rate. */

	uint16_t UART_WordLength;			/*!< Specifies the number of data bits transmitted or received in a frame.
											This parameter can be a value of @ref UART_Word_Length */

	uint16_t UART_StopBits;				/*!< Specifies the number of stop bits transmitted.
											This parameter can be a value of @ref UART_Stop_Bits */

	uint16_t UART_Parity;			  	/*!< Specifies the parity mode. This parameter can be a value of @ref UART_Parity
											@note When parity is enabled, the computed parity is inserted at the MSB position of the transmitted data (9th bit when
											the word length is set to 9 data bits; 8th bit when the word length is set to 8 data bits). */

	uint16_t UART_Mode;				  	/*!< Specifies wether the Receive or Transmit mode is enabled or disabled.
										This parameter can be a value of @ref UART_Mode */

	uint16_t bitorder; 				    /*!< Specifies the bit stream transit order: LSB or MSB */

	uint16_t phase;						/*!< Specifies UART output clock_Phase Normal or Reverse */

} UART_InitTypeDef;

/* Exported macro ------------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/  

/*  Function used to set the UART configuration to the default reset state ***/ 
void UART_DeInit(SCIAFSelect_TypeDef UARTx);

/* Initialization and Configuration functions *********************************/
int8_t UART_Init(SCIAFSelect_TypeDef UARTx, UART_InitTypeDef *UART_InitStruct);
void UART_Cmd(SCIAFSelect_TypeDef UARTx, FunctionalState NewState);
void UART_Start(SCIAFSelect_TypeDef UARTx,uint16_t uart_mode);
void UART_Stop(SCIAFSelect_TypeDef UARTx,uint16_t uart_mode);

/* Data transfers functions ***************************************************/ 
void UART_SendByte(SCIAFSelect_TypeDef UARTx, uint8_t Data);
char UART_ReceiveByte(SCIAFSelect_TypeDef UARTx);

FlagStatus UART_GetErrStaus(SCIAFSelect_TypeDef UARTx, uint16_t UART_FLAG);
void UART_ClearFlag(SCIAFSelect_TypeDef UARTx, uint16_t UART_FLAG);

#endif

