/**
  ******************************************************************************
  * @file    i2c.h
  * @author  CMS Application Team
  * @version Vx.x.x
  * @date    24-April-2022
  * @brief   This file provides firmware functions to manage the following 
  *          functionalities of the I2C bus to transmit data or receive:           
  @verbatim       
 ===============================================================================
                        ##### How to use this driver #####
 ===============================================================================
    [..]
            
    @endverbatim        
  ******************************************************************************
  * @attention
  *
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/

#ifndef __I2C_H__
#define __I2C_H__

#include "sci_common.h"

/** 
  * @brief  I2C Init structure definition  
  */

typedef struct
{
    uint32_t I2C_ClockSpeed;          /*!< Specifies the clock frequency.
                                            This parameter must be set to a value lower than 400kHz */

    uint16_t I2C_Mode;                /*!< Specifies the I2C mode.
                                            This parameter can be a value of @ref I2C_mode */

    uint16_t I2C_DutyCycle;           /*!< Specifies the I2C fast mode duty cycle.
                                            This parameter can be a value of @ref I2C_duty_cycle_in_fast_mode */

    uint16_t I2C_OwnAddress1;         /*!< Specifies the first device own address.
                                            This parameter can be a 7-bit or 10-bit address. */

    uint16_t I2C_Ack;                 /*!< Enables or disables the acknowledgement.
                                            This parameter can be a value of @ref I2C_acknowledgement */

    uint16_t I2C_AcknowledgedAddress; /*!< Specifies if 7-bit or 10-bit address is acknowledged.
                                            This parameter can be a value of @ref I2C_acknowledged_address */
}I2C_InitTypeDef;

#if defined(BAT32G1XX_80PIN) || defined(BAT32G1XX_100PIN)
#define IS_I2C_ALL_PERIPH(PERIPH)       (((PERIPH) == I2C00) || ((PERIPH) == I2C01) || ((PERIPH) == I2C10) || \
                                        ((PERIPH) == I2C11) || ((PERIPH) == I2C20) || ((PERIPH) == I2C21) ||  \
										((PERIPH) == I2C30) || ((PERIPH) == I2C31))
#else
#define IS_I2C_ALL_PERIPH(PERIPH)       (((PERIPH) == I2C00) || ((PERIPH) == I2C01) || ((PERIPH) == I2C10) || \
                                        ((PERIPH) == I2C11) || ((PERIPH) == I2C20) || ((PERIPH) == I2C21))
#endif
/** 
  * @brief  SCI unit ST register  
  */
#define I2C_STOP_EN						((uint16_t)0x0001)

/** 
  * @brief  SCI unit SS register  
  */
#define I2C_START_EN					((uint16_t)0x0001)
#define I2C_START_DIS					((uint16_t)0x0000)

/** 
  * @brief  SCI unit SPS register  
  */
#define I2C_SPS_MAX_VALUE				((uint16_t)0x000F)
#define I2C_SPS_MIN_VALUE				((uint16_t)0x0000)

/** 
  * @brief  SCI unit SSR register to read flags
  */
#define I2C_FLAG_NOACK					((uint16_t)0x0002)
#define I2C_FLAG_OVERRUN				((uint16_t)0x0001)
#define IS_I2C_FLAG(FLAG)				((((FLAG) & (uint16_t)0xFFFC) == 0x00) && \
										((FLAG) != (uint16_t)0x00))
										
/** 
  * @brief  SCI unit SIR register to clear flags
  */
#define I2C_CLEAR_FLAG_FEF				((uint16_t)0x0004)
#define I2C_CLEAR_FLAG_PEF				((uint16_t)0x0002)
#define I2C_CLEAR_FLAG_OVF				((uint16_t)0x0001)
#define IS_I2C_CLEAR_FLAG(FLAG)         ((((FLAG) & (uint16_t)0x0007) == 0x00) && ((FLAG) != (uint16_t)0x00))

/** 
  * @brief  SCI unit SCR setting register
  */
#define I2C_DATA_LENGTH_9               ((uint16_t)0x0001)
#define I2C_DATA_LENGTH_7               ((uint16_t)0x0002)
#define I2C_DATA_LENGTH_8               ((uint16_t)0x0003)

#define I2C_DATA_STOPBIT_NONE			((uint16_t)0x0000)
#define I2C_DATA_STOPBIT_1              ((uint16_t)0x0010)
#define I2C_DATA_STOPBIT_2              ((uint16_t)0x0020)
#define I2C_DATA_STOPBIT_INVALID		((uint16_t)0x0030)

#define I2C_DIR_MSB                     ((uint16_t)0x0000)
#define I2C_DIR_LSB                     ((uint16_t)0x0080)

#define I2C_PARITY_NONE                 ((uint16_t)0x0000)
#define I2C_INTSRE_NONE                 ((uint16_t)0x0000)
#define I2C_PHASE_NONE                  ((uint16_t)0x0000)

/** 
  * @brief  SCI unit SO setting register
  */
#define I2C_BusLevel_Low                ((uint16_t)0x0000)
#define I2C_BusLevel_High               ((uint16_t)0x0001)
#define IS_I2C_BUS_LEVEL(LEVEL)         (((LEVEL) == I2C_BusLevel_Low) || \
                                        ((LEVEL) == I2C_BusLevel_High))

#define I2C_TransmitMode_Recv			((uint8_t)0x01)
#define I2C_TransmitMode_Send			((uint8_t)0x02)
#define I2C_TransmitMode_Mask			((uint8_t)0x03)
#define IS_I2C_TRANSMIT_MODE(MODE)		(((MODE) == I2C_TransmitMode_Send) || \
                                        ((MODE) == I2C_TransmitMode_Recv))

/* Exported macro ------------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/ 

/*  Function used to set the I2C configuration to the default reset state *****/
void I2C_DeInit(SCIAFSelect_TypeDef I2Cx);

/* Initialization and Configuration functions *********************************/
int I2C_Init(SCIAFSelect_TypeDef I2Cx, I2C_InitTypeDef* I2C_InitStruct);

void I2C_SendByte(SCIAFSelect_TypeDef I2Cx, uint8_t Data);
void I2C_WriteData(SCIAFSelect_TypeDef I2Cx, uint8_t Address, uint8_t Reg, uint8_t *Data, uint16_t Len);
uint8_t I2C_ReceiveByte(SCIAFSelect_TypeDef I2Cx);
void I2C_ReadData(SCIAFSelect_TypeDef I2Cx, uint8_t Address, uint8_t Reg, uint8_t *Data, uint16_t Len);

void I2C_Set_TransmitMode(SCIAFSelect_TypeDef I2Cx, uint8_t Mode);
uint8_t I2C_Get_TransmitMode(SCIAFSelect_TypeDef I2Cx);

void I2C_GenerateSTART(SCIAFSelect_TypeDef I2Cx);
void I2C_GenerateSTOP(SCIAFSelect_TypeDef I2Cx);

void I2C_Output_Cmd(SCIAFSelect_TypeDef I2Cx, FunctionalState NewState);
void I2C_ChannelStart_Cmd(SCIAFSelect_TypeDef I2Cx, FunctionalState NewState);

FlagStatus I2C_GetErrStaus(SCIAFSelect_TypeDef I2Cx, uint16_t I2C_FLAG);
void I2C_ClearFlag(SCIAFSelect_TypeDef I2Cx, uint16_t I2C_FLAG);

#endif
