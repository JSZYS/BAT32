/**
  ******************************************************************************
  * @file    i2ca.h
  * @author  CMS Application Team
  * @version Vx.x.x
  * @date    24-April-2022
  * @brief   This file provides firmware functions to manage the following 
  *          functionalities of the Universal Inter Integrated Circuit (I2CA):           
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
#ifndef __I2CA_H__
#define __I2CA_H__

/* Includes ------------------------------------------------------------------*/
#include "common.h"

/** @addtogroup bat32g135_StdPeriph_Driver
  * @{
  */

/** @defgroup I2CA 
  * @brief I2CA driver modules
  * @{
  */
/* Private define ------------------------------------------------------------*/
#if defined(BAT32G1XX_80PIN) || defined(BAT32G1XX_100PIN)	
#define I2CA_SET_IICCTL00(I2CAx,bitPos)	{		\
	if (I2CAx == I2CA0)							\
	{											\
		IICA_TypeDef *I2CA_Instance = IICA0;	\
		I2CA_Instance->IICCTL0 |= (1 << bitPos);\
	}											\
	if (I2CAx == I2CA1)							\
	{											\
		IICA_TypeDef *I2CA_Instance = IICA1;	\
		I2CA_Instance->IICCTL0 |= (1 << bitPos);\
	}											\
}


#define I2CA_CLEAR_IICCTL00(I2CAx,bitPos)	{		\
	if (I2CAx == I2CA0)							\
	{											\
		IICA_TypeDef *I2CA_Instance = IICA0;	\
		I2CA_Instance->IICCTL0 &= ~(1 << bitPos);\
	}											\
	if (I2CAx == I2CA1)							\
	{											\
		IICA_TypeDef *I2CA_Instance = IICA1;	\
		I2CA_Instance->IICCTL0 &= ~ (1 << bitPos);\
	}											\
}
#else
#define I2CA_SET_IICCTL00(I2CAx,bitPos)	{		\
		IICA_TypeDef *I2CA_Instance = IICA0;	\
		I2CA_Instance->IICCTL0 |= (1 << bitPos);\
}


#define I2CA_CLEAR_IICCTL00(I2CAx,bitPos)	{		\
		IICA_TypeDef *I2CA_Instance = IICA0;	\
		I2CA_Instance->IICCTL0 &= ~(1 << bitPos);\
}
#endif
#define I2CA_SET_REG(I2CAx,reg,bitPos)	I2CA_SET_##reg(I2CAx,bitPos)
#define I2CA_CLEAR_REG(I2CAx,reg,bitPos)	I2CA_CLEAR_##reg(I2CAx,bitPos)
/* Private typedef -----------------------------------------------------------*/
typedef enum
{
	I2CA0,
#if defined(BAT32G1XX_80PIN) || defined(BAT32G1XX_100PIN)	
	I2CA1,
#endif	
} I2CASelect_TypeDef;

typedef enum
{
	OK = 0,
	BUSY = -1,
	NSTART = -2,
	NACK = -3,
	NSTOP = -4,
} I2CA_Status;

typedef enum
{
	I2C_SLAVE_EVENT_NONE = 0,
	I2C_SLAVE_EVENT_ERROR = -1,
	I2C_SLAVE_EVENT_READ_ADDRESS_MATCHED = 1,
	I2C_SLAVE_EVENT_WRITE_ADDRESS_MATCHED = 2,
	I2C_SLAVE_EVENT_TRANSMIT_RUNNING = 3,
	I2C_SLAVE_EVENT_RECEIVE_RUNNING = 4,
	I2C_SLAVE_EVENT_TRANSMIT_COMPLETED = 5,
	I2C_SLAVE_EVENT_RECEIVE_COMPLETED = 6,

} I2CA_Slave_Event;

typedef struct
{
	uint32_t I2C_ClockSpeed;            /*!< Specifies the clock frequency.
											This parameter must be set to a value lower than 1000kHz */

	uint8_t I2C_Mode;                   /*!< Specifies the I2C mode.
											This parameter can be a value of @ref I2C_mode */

	uint8_t I2C_DutyCycle;              /*!< Specifies the I2C fast mode duty cycle.
											This parameter can be a value of @ref I2C_duty_cycle_in_fast_mode */

	uint8_t I2C_OwnAddress;             /*!< Specifies the first device own address.
											This parameter can be a 7-bit address. */

	uint8_t I2C_Ack;                    /*!< Enables or disables the acknowledgement.
											This parameter can be a value of @ref I2C_acknowledgement */
}I2CA_InitTypeDef;

/* Private variables ---------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/

/** @defgroup I2CA register for speed mode
  * @{
  */
#define I2CA_OPERATE_BIT				((uint8_t)0x03)
#define I2CA_OPERATE_STANDARD			((uint8_t)0x00)
#define I2CA_OPERATE_FAST				((uint8_t)0x08)

/** @defgroup I2CA register for digital filter
  * @{
  */
#define I2CA_DIGITAL_FILTER_BIT			((uint8_t)0x02)
#define I2CA_DIGITAL_FILTER_ON			((uint8_t)0x04)
#define I2CA_DIGITAL_FILTER_OFF			((uint8_t)0x00)

/** @defgroup I2CA register for clock division
  * @{
  */
#define I2CA_FCLK_DIV_BIT				((uint8_t)0x00)
#define I2CA_FCLK_DIV_2					((uint8_t)0x01)
#define I2CA_FCLK_DIV_0					((uint8_t)0x00)

/** @defgroup I2CA register to enable trigger for init start
  * @{
  */
#define I2CA_STOP_DETECTION_BIT			((uint8_t)0x01)
/* enable generation of a start condition without detecting a stop condition */
#define I2CA_WITHOUT_STOP_DETECTION		((uint8_t)0x02)
/* enable generation of a start condition upon detection of a stop condition */
#define I2CA_UOPN_STOP_DETECTION		((uint8_t)0x00)

/** @defgroup I2CA register for Communication reservation function
  * @{
  */
#define I2CA_RESERVATION_BIT			((uint8_t)(0x00U))
#define I2CA_RESERVATION_ENABLE			((uint8_t)(0x00U))
#define I2CA_RESERVATION_DISABLE		((uint8_t)(0x01U))

/** @defgroup I2CA_clock_speed
  * @{
  */
#define IS_I2CA_CLOCK_SPEED(SPEED)		(((SPEED) >= 0x1) && ((SPEED) <= 1000000))

/** @defgroup I2CA_mode 
  * @{
  */
#define I2CA_Mode_SMBusSlave			((uint8_t)0x00)
#define I2CA_Mode_SMBusMaster			((uint8_t)0x80)
#define IS_I2CA_MODE(MODE)				(((MODE) == I2CA_Mode_SMBusSlave) || \
                                        ((MODE) == I2CA_Mode_SMBusMaster))

/** @defgroup I2CA_DutyCycle 
  * @{
  */
#define IS_I2CA_DUTY_CYCLE(CYCLE)        (((CYCLE) > 0) && ((CYCLE) < 100))

/** @defgroup I2CA_own_address
  * @{
  */
#define IS_I2CA_OWN_ADDRESS(ADDRESS)     ((ADDRESS) <= 0xFE)

/** @defgroup I2CA_acknowledgement
  * @{
  */
#define I2CA_ACK_BIT					((uint8_t)0x02)
#define I2CA_Ack_Enable					((uint8_t)0x04)
#define I2CA_Ack_Disable				((uint8_t)0x00)
#define IS_I2CA_ACK_STATE(STATE)		(((STATE) == I2CA_Ack_Enable) || \
                                        ((STATE) == I2CA_Ack_Disable))

/** @defgroup I2CA register for waitting and interrupt control
  * @{
  */
#define I2CA_WTIM_BIT					((uint8_t)0x03)
#define I2CA_WTIM_ENABLE				((uint8_t)0x08)
#define I2CA_WTIM_DISABLE				((uint8_t)0x00)

#define I2CA_INTPOS_8CLK				((uint8_t)0x01)
#define I2CA_INTPOS_9CLK				((uint8_t)0x02)
#define IS_I2CA_INTPOS(POS)				(((POS) == I2CA_INTPOS_8CLK) || \
                                        ((POS) == I2CA_INTPOS_9CLK))

/** @defgroup I2CA register for periphal enable control
  * @{
  */
#define I2CA_EN_BIT						((uint8_t)0x07)
#define I2CA_EN_ENABLE					((uint8_t)0x80)
#define I2CA_EN_DISABLE					((uint8_t)0x00)

/** @defgroup I2CA register for communication exit control
  * @{
  */
#define I2CA_LREL_BIT					((uint8_t)0x06)
#define I2CA_LREL_ENABLE				((uint8_t)0x40)
#define I2CA_LREL_DISABLE				((uint8_t)0x00)

/** @defgroup I2CA register for communication wait unlock
  * @{
  */
#define I2CA_WREL_BIT					((uint8_t)0x05)
#define I2CA_WREL_ENABLE				((uint8_t)0x20)
#define I2CA_WREL_DISABLE				((uint8_t)0x00)

/** @defgroup I2CA register for interrupt of stop condition
  * @{
  */
#define I2CA_SPIE_BIT					((uint8_t)0x04)
#define I2CA_SPIE_ENABLE				((uint8_t)0x10)
#define I2CA_SPIE_DISABLE				((uint8_t)0x00)

/** @defgroup I2CA register for STOP and START condition control
  * @{
  */
#define I2CA_STOP_BIT					((uint8_t)0x00)
#define I2CA_STOP_ENABLE				((uint8_t)0x01)
#define I2CA_STOP_DISABLE				((uint8_t)0x00)

#define I2CA_START_BIT					((uint8_t)0x01)
#define I2CA_START_ENABLE				((uint8_t)0x02)
#define I2CA_START_DISABLE				((uint8_t)0x00)

/** @defgroup I2CA_Status 
  * @{
  */
#define I2CA_STATUS_SPD					((uint16_t)0x0001)
#define I2CA_STATUS_STD					((uint16_t)0x0002)
#define I2CA_STATUS_ACK					((uint16_t)0x0004)
#define I2CA_STATUS_TRC					((uint16_t)0x0008)
#define I2CA_STATUS_COI					((uint16_t)0x0010)
#define I2CA_STATUS_EXC					((uint16_t)0x0020)
#define I2CA_STATUS_ALD					((uint16_t)0x0040)
#define I2CA_STATUS_MSTS				((uint16_t)0x0080)
#define I2CA_STATUS_BUSBSY				((uint16_t)0x4000)
#define I2CA_STATUS_STCF				((uint16_t)0x8000)
#define IS_I2CA_STATUS(FLAG)			((((FLAG) & (uint16_t)0x3F00) == 0x00) && ((FLAG) != (uint16_t)0x00))

/** @defgroup I2CA_Flag_Register 
  * @{
  */
#define I2CA_I2CBUS_BUSY			((uint8_t)0x40)
#define I2CA_I2CBUS_IDLE			((uint8_t)0x00)

#define I2CA_BUS_MAX_NUM     2u

/* Private function prototypes -----------------------------------------------*/
extern IRQn_Type I2CA_IRQTable[I2CA_BUS_MAX_NUM];
/* Private functions ---------------------------------------------------------*/
void I2CA_DeInit(I2CASelect_TypeDef I2CAx);
void I2CA_Init(I2CASelect_TypeDef I2CAx,I2CA_InitTypeDef* I2CA_InitStruct);
void I2CA_Cmd(I2CASelect_TypeDef I2CAx,FunctionalState NewState);
void I2CA_Acknowledge_Cmd(I2CASelect_TypeDef I2CAx,FunctionalState NewState);
void I2CA_InterruptPosition_Config(I2CASelect_TypeDef I2CAx,uint8_t Intpos);
void I2CA_WaitRelieve_Cmd(I2CASelect_TypeDef I2CAx,FunctionalState NewState);
void I2CA_OwnAddress_Config(I2CASelect_TypeDef I2CAx,uint8_t Address);
uint8_t I2CA_ReadByte(I2CASelect_TypeDef I2CAx);
void I2CA_WriteByte(I2CASelect_TypeDef I2CAx,uint8_t Data);

void I2CA_GenerateSTART(I2CASelect_TypeDef I2CAx);
void I2CA_GenerateSTOP(I2CASelect_TypeDef I2CAx);
FlagStatus I2CA_GetFlagStaus(I2CASelect_TypeDef I2CAx,uint16_t I2CA_FLAG);

I2CA_Status I2CA_Master_WriteData(I2CASelect_TypeDef I2CAx,uint8_t Address, uint8_t Reg, uint8_t *Data, uint16_t Len);
I2CA_Status I2CA_Master_ReadData(I2CASelect_TypeDef I2CAx,uint8_t Address, uint8_t Reg, uint8_t *Data, uint16_t Len);

I2CA_Status I2CA_Slave_ReceiveData(I2CASelect_TypeDef I2CAx,uint8_t *Data, uint16_t Size, uint16_t *Len);
I2CA_Status I2CA_Slave_TransmitData(I2CASelect_TypeDef I2CAx,uint8_t *Reg, uint8_t *Data, uint16_t Size);
I2CA_Slave_Event I2CA_Slave_TransmitReceive_IT(I2CASelect_TypeDef I2CAx, uint8_t *TxBuffer, uint16_t *TxLength, 
                                                uint8_t *RxBuffer, uint16_t *RxLength, uint16_t RxTotLength);
#endif
