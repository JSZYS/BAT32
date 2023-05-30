#ifndef __SPI_H__
#define __SPI_H__

#include "common.h"

/** @addtogroup bat32g135_StdPeriph_Driver
  * @{
  */

/** @addtogroup SPI
  * @{
  */ 
#define _0001_SPI_UNDER_EXECUTE		0x0001
#define _0002_SPI_VALID_STORED		0x0002

#define SPI_WRDR 		(SPI->SDRO)
#define SPI_RDDR 		(SPI->SDRI)
#define SPI_BUSY 		(SPI->SPIM & _0001_SPI_UNDER_EXECUTE)
#define SPI_RRDY        (SPI->SPIM & _0002_SPI_VALID_STORED)

typedef void (*NSS_Func)(void);

typedef struct
{
	void(*Active)(void);
	void(*Inactive)(void);
}NSS_FUNC_T;
extern NSS_FUNC_T NSS;
/* Exported types ------------------------------------------------------------*/

/** 
  * @brief  SPI Init structure definition  
  */

typedef struct
{
	uint8_t SPI_Mode;					/*!< Specifies the SPI operating mode.
											This parameter can be a value of @ref SPI_mode */

	uint8_t SPI_DataSize;				/*!< Specifies the SPI data size.
											This parameter can be a value of @ref SPI_data_size */

	uint8_t SPI_CPOL;					/*!< Specifies the serial clock steady state.
											This parameter can be a value of @ref SPI_Clock_Polarity */

	uint8_t SPI_CPHA;					/*!< Specifies the clock active edge for the bit capture.
											This parameter can be a value of @ref SPI_Clock_Phase */

	uint8_t SPI_NSS;					/*!< Specifies whether the NSS signal is managed by
											hardware (NSS pin) or by software using the SSI bit.
											This parameter can be a value of @ref SPI_Slave_Select_management */

	uint8_t SPI_BaudRatePrescaler;		/*!< Specifies the Baud Rate prescaler value which will be
											used to configure the transmit and receive SCK clock.
											This parameter can be a value of @ref SPI_BaudRate_Prescaler
											@note The communication clock is derived from the master
												clock. The slave clock does not need to be set. */

	uint8_t SPI_FirstBit;				/*!< Specifies whether data transfers start from MSB or LSB bit.
											This parameter can be a value of @ref SPI_MSB_LSB_transmission */
}SPI_InitTypeDef;

/** @defgroup SPI_mode 
  * @{
  */
#define SPI_Mode_Master                 ((uint8_t)0x40)
#define SPI_Mode_Slave                  ((uint8_t)0x00)
#define IS_SPI_MODE(MODE)				(((MODE) == SPI_Mode_Master) || \
										((MODE) == SPI_Mode_Slave))
/**
  * @}
  */

/** @defgroup SPI_data_size 
  * @{
  */
#define SPI_DataSize_bit				((uint8_t)0x02)
#define SPI_DataSize_16b                ((uint8_t)0x04)
#define SPI_DataSize_8b                 ((uint8_t)0x00)
#define IS_SPI_DATASIZE(DATASIZE)		(((DATASIZE) == SPI_DataSize_16b) || \
										((DATASIZE) == SPI_DataSize_8b))
/**
  * @}
  */ 

/** @defgroup SPI_Clock_Polarity 
  * @{
  */
#define SPI_CPOL_Bit					((uint8_t)0x04)
#define SPI_CPOL_Low                    ((uint8_t)0x00)
#define SPI_CPOL_High                   ((uint8_t)0x10)
#define IS_SPI_CPOL(CPOL)				(((CPOL) == SPI_CPOL_Low) || \
										((CPOL) == SPI_CPOL_High))
/**
  * @}
  */

/** @defgroup SPI_Clock_Phase 
  * @{
  */
#define SPI_CPHA_Bit					((uint8_t)0x03)
#define SPI_CPHA_1Edge                  ((uint8_t)0x00)
#define SPI_CPHA_2Edge                  ((uint8_t)0x08)
#define IS_SPI_CPHA(CPHA)				(((CPHA) == SPI_CPHA_1Edge) || \
										((CPHA) == SPI_CPHA_2Edge))

/**
  * @}
  */

/** @defgroup SPI_Slave_Select_management 
  * @{
  */
#define SPI_NSS_Bit						((uint8_t)0x05)
#define SPI_NSS_Soft                    ((uint8_t)0x00)
#define SPI_NSS_Hard                    ((uint8_t)0x20)
#define IS_SPI_NSS(NSS)					(((NSS) == SPI_NSS_Soft) || \
										((NSS) == SPI_NSS_Hard))

/**
  * @}
  */ 

/** @defgroup SPI_BaudRate_Prescaler 
  * @{
  */
#define SPI_BaudRatePrescaler_0         ((uint8_t)0x00)
#define SPI_BaudRatePrescaler_2         ((uint8_t)0x01)
#define SPI_BaudRatePrescaler_4         ((uint8_t)0x02)
#define SPI_BaudRatePrescaler_8         ((uint8_t)0x03)
#define SPI_BaudRatePrescaler_16        ((uint8_t)0x04)
#define SPI_BaudRatePrescaler_32        ((uint8_t)0x05)
#define SPI_BaudRatePrescaler_64        ((uint8_t)0x06)
#define SPI_BaudRatePrescaler_Ext       ((uint8_t)0x07)
#define IS_SPI_BAUDRATE_PRESCALER(PRESCALER) 	(((PRESCALER) == SPI_BaudRatePrescaler_0) || \
												((PRESCALER) == SPI_BaudRatePrescaler_2) || \
												((PRESCALER) == SPI_BaudRatePrescaler_4) || \
												((PRESCALER) == SPI_BaudRatePrescaler_8) || \
												((PRESCALER) == SPI_BaudRatePrescaler_16) || \
												((PRESCALER) == SPI_BaudRatePrescaler_32) || \
												((PRESCALER) == SPI_BaudRatePrescaler_64) || \
												((PRESCALER) == SPI_BaudRatePrescaler_Ext))

/**
  * @}
  */ 

/** @defgroup SPI_MSB_LSB_transmission 
  * @{s
  */
#define SPI_FirstBit_Bit				((uint8_t)0x04)
#define SPI_FirstBit_MSB                ((uint8_t)0x00)
#define SPI_FirstBit_LSB                ((uint8_t)0x10)
#define IS_SPI_FIRST_BIT(BIT)			(((BIT) == SPI_FirstBit_MSB) || \
										((BIT) == SPI_FirstBit_LSB))
/**
  * @}
  */

/** @defgroup SPI_direction_transmit_receive 
  * @{
  */
 
#define SPI_Direction_Rx                  ((uint8_t)0xBF)
#define SPI_Direction_TxRx                ((uint8_t)0x40)
#define IS_SPI_DIRECTION(DIRECTION)		(((DIRECTION) == SPI_Direction_Rx) || \
										((DIRECTION) == SPI_Direction_TxRx))
										
/**
  * @}
  */

/** @defgroup SPI_flags_definition 
  * @{
  */
#define SPI_FLAG_RXNE					((uint8_t)0x02)
#define SPI_FLAG_RUNNING				((uint8_t)0x01)
#define IS_SPI_GET_FLAG(FLAG)			((((FLAG) & (uint16_t)0xFC) == 0x00) && ((FLAG) != (uint16_t)0x00))

/**
  * @}
  */

/* Exported macro ------------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/

/*  Function used to set the SPI configuration to the default reset state *****/
void SPI_DeInit(void);

/* Initialization and Configuration functions *********************************/
void SPI_Init(SPI_InitTypeDef* SPI_InitStruct);
void SPI_Cmd(FunctionalState NewState);

void SPI_DataSizeConfig(uint8_t SPI_DataSize);
void SPI_BiDirectionalLineConfig(uint8_t SPI_Direction);

/* Data transfers functions ***************************************************/ 
uint16_t SPI_ReceiveData(void);
void SPI_SendData(uint16_t Data);
uint8_t SPI_TransmitByte(uint8_t Data);
uint8_t SPI_ReceiveByte(void);

uint16_t SPI_TransmitWord(uint16_t Data);

/* Interrupts and flags management functions **********************************/
FlagStatus SPI_GetFlagStatus(uint8_t SPI_FLAG);

#endif
