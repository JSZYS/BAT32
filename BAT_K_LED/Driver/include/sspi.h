#ifndef __SSPI_H__
#define __SSPI_H__
#include <stdio.h>
#include <stdint.h>
#include "sci_common.h"

#define SSPI00_SDR      SCI0->SCI.SIO00
#define SSPI01_SDR      SCI0->SCI.SIO01
#define SSPI10_SDR      SCI0->SCI.SIO10
#define SSPI11_SDR      SCI0->SCI.SIO11
#define SSPI20_SDR      SCI1->SCI.SIO20
#define SSPI21_SDR      SCI1->SCI.SIO21

#if defined(BAT32G1XX_80PIN) || defined(BAT32G1XX_100PIN)
#define IS_SSPI_ALL_PERIPH(PERIPH)       (((PERIPH) == SSPI00) || ((PERIPH) == SSPI01) || ((PERIPH) == SSPI10) || \
							((PERIPH) == SSPI11) || ((PERIPH) == SSPI20) || ((PERIPH) == SSPI21)|| ((PERIPH) == SSPI30)|| ((PERIPH) == SSPI31))

#else
#define IS_SSPI_ALL_PERIPH(PERIPH)       (((PERIPH) == SSPI00) || ((PERIPH) == SSPI01) || ((PERIPH) == SSPI10) || \
                                        ((PERIPH) == SSPI11) || ((PERIPH) == SSPI20) || ((PERIPH) == SSPI21))
 
#endif
typedef struct
{

	uint16_t SPI_Mode;				  /*!< Specifies the SPI operating mode.
										   This parameter can be a value of @ref SPI_mode */

	uint16_t SPI_DataSize;			  /*!< Specifies the SPI data size.
										   This parameter can be a value of @ref SPI_data_size */
										   
	uint32_t SPI_ClockSpeed;  /*!< Specifies the Baud Rate which will transmit and receive clock.*/
											 
	uint16_t SPI_Bitorder;			  /*!< Specifies whether data transfers start from MSB or LSB bit.*/

	uint16_t SPI_Phase_Mode;          /*!< Specifies  data transfers starts before operation clock and its phase.*/

	  
}SPI_InitTypeDef;

/** 
  * @brief  SCI unit ST register  
  */
#define SSPI_STOP_EN						((uint16_t)0x0001)

/** 
  * @brief  SCI unit SS register  
  */
#define SSPI_START_EN					((uint16_t)0x0001)
#define SSPI_START_DIS					((uint16_t)0x0000)

/** 
  * @brief  SCI unit SPS register  
  */
#define SSPI_SPS_MAX_VALUE				((uint16_t)0x000F)
#define SSPI_SPS_MIN_VALUE				((uint16_t)0x0000)

/** 
  * @brief  SCI unit SSR register to read flags
  */
#define SSPI_FLAG_TSF					((uint16_t)0x0040)
#define SSPI_FLAG_BEF					((uint16_t)0x0020)
#define SSPI_FLAG_OVERRUN				((uint16_t)0x0001)
#define IS_SSPI_FLAG(FLAG)				((((FLAG) & (uint16_t)0xFF9E) == 0x00) && ((FLAG) != (uint16_t)0x00))
										
										
/** 
  * @brief  SCI unit SIR register to clear flags
  */
#define SSPI_CLEAR_FLAG_FEF				((uint16_t)0x0004)
#define SSPI_CLEAR_FLAG_PEF				((uint16_t)0x0002)
#define SSPI_CLEAR_FLAG_OVF				((uint16_t)0x0001)
#define IS_SSPI_CLEAR_FLAG(FLAG)         ((((FLAG) & (uint16_t)0x0007) == 0x00) && ((FLAG) != (uint16_t)0x00))

/** 
  * @brief  SCI unit SCR setting register
  */
#define SSPI_DATA_LENGTH_9               ((uint16_t)0x0001)
#define SSPI_DATA_LENGTH_7               ((uint16_t)0x0002)
#define SSPI_DATA_LENGTH_8               ((uint16_t)0x0003)

#define SSPI_DATA_STOPBIT_NONE			((uint16_t)0x0000)
#define SSPI_DATA_STOPBIT_1              ((uint16_t)0x0010)
#define SSPI_DATA_STOPBIT_2              ((uint16_t)0x0020)
#define SSPI_DATA_STOPBIT_INVALID		((uint16_t)0x0030)

#define SSPI_DIR_MSB                     ((uint16_t)0x0000)
#define SSPI_DIR_LSB                     ((uint16_t)0x0080)

#define SSPI_PARITY_NONE                 ((uint16_t)0x0000)
#define SSPI_INTSRE_NONE                 ((uint16_t)0x0000)
#define SSPI_PHASE_NONE                  ((uint16_t)0x0000)
/** 
  * @brief  SCI unit SO setting register
  */
#define SSPI_BusLevel_Low                ((uint16_t)0x0000)
#define SSPI_BusLevel_High               ((uint16_t)0x0001)
#define IS_SSPI_BUS_LEVEL(LEVEL)         (((LEVEL) == SSPI_BusLevel_Low) || \
                                        ((LEVEL) == SSPI_BusLevel_High))
#define SSPI_TransmitMode_Recv			((uint8_t)0x01)
#define SSPI_TransmitMode_Send			((uint8_t)0x02)
#define SSPI_TransmitMode_TxRx			((uint8_t)0x03)
#define SSPI_TransmitMode_Mask			((uint8_t)0x03)
#define IS_SSPI_TRANSMIT_MODE(MODE)		(((MODE) == SSPI_TransmitMode_Send) || \
                                        ((MODE) == SSPI_TransmitMode_Recv))



#define SPI_Phase_Mode0                 ((uint16_t)0x3000)//11: DAP_1: data sampling on first edge,data change on second edge;  CKP_1: Clock polarity is low when idle
#define SPI_Phase_Mode1                 ((uint16_t)0x1000)//01: DAP_0: Data sampling on second edge;  data change on first edge CKP_1: Clock polarity is low when idle
#define SPI_Phase_Mode2                 ((uint16_t)0x2000)//10: DAP_1: data sampling on first edge;                            CKP_0: Clock polarity is high when idle
#define SPI_Phase_Mode3                 ((uint16_t)0x0000)//00: DAP_0: Data sampling on second edge;                           CKP_0: Clock polarity is high when idle

#define SPI_Mode_Master                 ((uint16_t)0x0000)
#define SPI_Mode_Slave                  ((uint16_t)0x0001)

#define SPI_Data_Bits_16                 ((uint16_t)0x000F)
#define SPI_Data_Bits_9                 ((uint16_t)0x0008)
#define SPI_Data_Bits_8                 ((uint16_t)0x0007)
#define SPI_Data_Bits_7                 ((uint16_t)0x0006)
#define IS_SPI_DATASIZE(DATASIZE) (((DATASIZE) == SPI_Data_Bits_8) || ((DATASIZE) == SPI_Data_Bits_7)))

#define SPI_Bit_Msb				        ((uint16_t)0x0000)
#define SPI_Bit_Lsb                     ((uint16_t)0x0080)

typedef void (*NSS_Func)(void);

typedef struct
{
	void(*Active)(void);
	void(*Inactive)(void);
}NSS_FUNC_T;
extern NSS_FUNC_T NSS;

extern uint8_t g_spi_tx_end,g_spi_rx_end;
void SSPI_Set_TransmitMode(SCIAFSelect_TypeDef SSPIx, uint8_t Mode);
uint8_t SSPI_Get_MasterMode(SCIAFSelect_TypeDef SSPIx);
uint8_t SSPI_Get_TransmitMode(SCIAFSelect_TypeDef SSPIx);
uint8_t SSPI_TransmitData(SCIAFSelect_TypeDef SSPIx, uint8_t Data);

void SSPI_Start(SCIAFSelect_TypeDef func);
void SSPI_Stop(SCIAFSelect_TypeDef func);
int8_t SSPI_Init(SCIAFSelect_TypeDef func, SPI_InitTypeDef * SPI_InitStruct);
void SSPI_SendByte(SCIAFSelect_TypeDef SPIx, uint8_t Data);
uint8_t SSPI_ReceiveByte(SCIAFSelect_TypeDef SPIx);
FlagStatus SSPI_GetErrStaus(SCIAFSelect_TypeDef SSPIx, uint16_t SSPI_FLAG);
void SSPI_ClearFlag(SCIAFSelect_TypeDef SSPIx, uint16_t SSPI_FLAG);
uint8_t SSPI_TransmitData(SCIAFSelect_TypeDef SSPIx, uint8_t Data);

#endif
