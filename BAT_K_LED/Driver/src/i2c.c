/**
  ******************************************************************************
  * @file    i2c.c
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

#include "i2c.h"
#include "cgc.h"

/** @addtogroup bat32g135_StdPeriph_Driver
  * @{
  */

/** @defgroup I2C 
  * @brief I2C driver modules
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* I2C init define by clock and clock source select */
#define I2Cx_Init(I2C_Instance, clock, clock_sel) \
{																		\
	uint16_t tmpreg = 0;                                              	\
																		\
	I2C_Instance->ST |= I2C_STOP_EN;                                    \
																		\
	tmpreg |= I2C_CLEAR_FLAG_FEF;                                       \
	tmpreg |= I2C_CLEAR_FLAG_PEF;                                       \
	tmpreg |= I2C_CLEAR_FLAG_OVF;                                       \
	I2C_Instance->SIR = tmpreg;                                         \
																		\
	tmpreg = I2C_Instance->SMR;                                         \
	tmpreg |= SCI_SMRMN_DEFAULT_VALUE;								    \
	tmpreg |= SCI_CLOCK_MODE_CKS | SCI_MODE_IIC | SCI_TRANSFER_END;	    \
	tmpreg |= clock_sel;												\
	I2C_Instance->SMR = tmpreg;                                         \
																		\
    tmpreg = 0;                                                         \
    tmpreg |= SCI_SCRMN_DEFAULT_VALUE;                                  \
    tmpreg |= SCI_TIMING_1;                                             \
	tmpreg |= I2C_DIR_MSB;                                              \
    tmpreg |= I2C_DATA_LENGTH_8;                                        \
    tmpreg |= I2C_DATA_STOPBIT_1;                                       \
    I2C_Instance->SCR = tmpreg;                                         \
																		\
	I2C_Instance->SPS&= ~I2C_SPS_MAX_VALUE;                             \
	I2C_Instance->SPS = clock.sps;										\
    I2C_Instance->SDR = clock.sdr;										\
																		\
	I2C_Instance->CKO = I2C_BusLevel_High;                              \
	I2C_Instance->SO = I2C_BusLevel_High;                               \
}

/* I2C BUS mode configurate by send or receive */
#define I2Cx_Chanel_ModeConfigurate(I2C_Instance, Mode)	\
{																		\
	I2C_Instance->SCR &= (uint16_t) ~(SCI_RECEPTION_TRANSMISSION);      \
	I2C_Instance->SCR |= (Mode == I2C_TransmitMode_Send) ?              \
							SCI_TRANSMISSION : SCI_RECEPTION;           \
}

/* I2C BUS generate START signal */
#define I2Cx_StartCondition(I2C_Instance)	\
{											\
	I2C_Instance->SO = I2C_BusLevel_Low;    \
	I2C_DelayCycle();                       \
	I2C_Instance->CKO = I2C_BusLevel_Low;   \
}

/* I2C BUS generate STOP signal */
#define I2Cx_StopCondition(I2C_Instance)	\
{                                           \
	I2C_Instance->SO = I2C_BusLevel_Low;    \
	I2C_Instance->CKO = I2C_BusLevel_High;  \
	I2C_DelayCycle();                       \
	I2C_Instance->SO = I2C_BusLevel_High;	\
}

/* I2C BUS channel enable */
#define I2Cx_Channel_Enable(I2C_Instance)	\
{											\
	I2C_Instance->SOE = I2C_START_EN;		\
	I2C_Instance->SS = I2C_START_EN;        \
}

/* I2C BUS channel disable */
#define I2Cx_Channel_Disable(I2C_Instance)	\
{                                           \
	I2C_Instance->ST = I2C_STOP_EN;         \
	I2C_Instance->SOE = I2C_START_DIS;      \
}

/* Private macro -------------------------------------------------------------*/
#define I2C_DELAY_CYCLE     1u
#define I2C_BUS_MAX_NUM     8u

/* Private variables ---------------------------------------------------------*/
static IRQn_Type I2C_IRQTable[I2C_BUS_MAX_NUM] = 
{IIC00_IRQn, IIC01_IRQn, IIC10_IRQn, IIC11_IRQn, IIC20_IRQn, IIC21_IRQn,IIC30_IRQn, IIC31_IRQn};

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

static uint32_t I2C_GetSysClock(void)
{
	return SystemCoreClock;
}

static inline void I2C_DelayCycle(void)
{
	for (int w_count = 0U; w_count <= I2C_DELAY_CYCLE; w_count++)
        __NOP();
}

/**
  * @brief  Deinitializes the I2Cx peripheral registers to their default reset values.
  * @param  I2Cx: where x can be 0, 1, 2, 3, 4 or 5 to select the I2C peripheral.
  * @retval None
  */
void I2C_DeInit(SCIAFSelect_TypeDef I2Cx)
{
	/* Check the parameters */
	assert_param(IS_I2C_ALL_PERIPH(I2Cx));

	CGC_PER0PeriphClockCmd(I2Cx >> 10, DISABLE);

	SCIChannel_Free(I2Cx, SCI_DEFAULT_MODE);
}

/**
  * @brief  Set I2Cx Bus init status and setting the bus speed
  * @param  I2Cx: where x can be 0, 1, 2, 3, 4 or 5 to select the I2C peripheral.
  * @param  I2C_InitStruct: I2C bus peripheral setting arg data structure.
  * @retval None.
  */
int I2C_Init(SCIAFSelect_TypeDef I2Cx, I2C_InitTypeDef* I2C_InitStruct)
{
	assert_param(IS_I2C_ALL_PERIPH(I2Cx));
	
	int8_t ret = SCI_SUCCESS;
    SCIPeriph_Clock_TypeDef pValue;
	
	ret = SCIChannel_Alloca(I2Cx, SCI_DEFAULT_MODE);
	if(ret != SCI_SUCCESS)
		return ret;

    switch(I2Cx)
    {
        case I2C00:
        {
			/* Clock gate control register setting to enable SCI0 clock */
			CGC_PER0PeriphClockCmd((uint8_t)((I2Cx >> 10)+SCI_UINT_OFFSET), ENABLE);
			
            I2C0_TypeDef *I2C_Instance = &SCI0->I2C0;
			SCIPeriphal_ClockUpdate(I2Cx, I2C_GetSysClock(), 
									I2C_InitStruct->I2C_ClockSpeed, 
									I2C_Instance->SPS,
									&pValue);
			I2Cx_Init(I2C_Instance, pValue, SCI_CLOCK_SELECT_CK00);
        }
        break;
        case I2C01:
        {
			CGC_PER0PeriphClockCmd((uint8_t)((I2Cx >> 10)+SCI_UINT_OFFSET), ENABLE);
            I2C1_TypeDef *I2C_Instance = &SCI0->I2C1;
			SCIPeriphal_ClockUpdate(I2Cx, I2C_GetSysClock(), 
									I2C_InitStruct->I2C_ClockSpeed, 
									I2C_Instance->SPS,
									&pValue);
			I2Cx_Init(I2C_Instance, pValue, SCI_CLOCK_SELECT_CK00);
        }
        break;
        case I2C10:
        {
			CGC_PER0PeriphClockCmd((uint8_t)((I2Cx >> 10)+SCI_UINT_OFFSET), ENABLE);
            I2C2_TypeDef *I2C_Instance = &SCI0->I2C2;
			SCIPeriphal_ClockUpdate(I2Cx, I2C_GetSysClock(), 
									I2C_InitStruct->I2C_ClockSpeed, 
									I2C_Instance->SPS,
									&pValue);
			I2Cx_Init(I2C_Instance, pValue, SCI_CLOCK_SELECT_CK01);
        }
        break;
        case I2C11:
        {
			CGC_PER0PeriphClockCmd((uint8_t)((I2Cx >> 10)+SCI_UINT_OFFSET), ENABLE);
            I2C3_TypeDef *I2C_Instance = &SCI0->I2C3;
			SCIPeriphal_ClockUpdate(I2Cx, I2C_GetSysClock(), 
									I2C_InitStruct->I2C_ClockSpeed, 
									I2C_Instance->SPS,
									&pValue);
			I2Cx_Init(I2C_Instance, pValue, SCI_CLOCK_SELECT_CK01);
        }
        break;
        case I2C20:
        {
			CGC_PER0PeriphClockCmd((uint8_t)((I2Cx >> 10)+SCI_UINT_OFFSET), ENABLE);
            I2C4_TypeDef *I2C_Instance = &SCI1->I2C4;
			SCIPeriphal_ClockUpdate(I2Cx, I2C_GetSysClock(), 
									I2C_InitStruct->I2C_ClockSpeed, 
									I2C_Instance->SPS,
									&pValue);
			I2Cx_Init(I2C_Instance, pValue, SCI_CLOCK_SELECT_CK00);
        }
        break;
        case I2C21:
        {
			CGC_PER0PeriphClockCmd((uint8_t)((I2Cx >> 10)+SCI_UINT_OFFSET), ENABLE);
            I2C5_TypeDef *I2C_Instance = &SCI1->I2C5;
			SCIPeriphal_ClockUpdate(I2Cx, I2C_GetSysClock(), 
									I2C_InitStruct->I2C_ClockSpeed, 
									I2C_Instance->SPS,
									&pValue);
			I2Cx_Init(I2C_Instance, pValue, SCI_CLOCK_SELECT_CK00);
        }
        break;
#if defined(BAT32G1XX_80PIN) || defined(BAT32G1XX_100PIN)
        case I2C30:
        {
			CGC_PER2PeriphClockCmd((uint8_t)((I2Cx >> 10)), ENABLE);
            I2C6_TypeDef *I2C_Instance = &SCI2->I2C6;
			SCIPeriphal_ClockUpdate(I2Cx, I2C_GetSysClock(), 
									I2C_InitStruct->I2C_ClockSpeed, 
									I2C_Instance->SPS,
									&pValue);
			I2Cx_Init(I2C_Instance, pValue, SCI_CLOCK_SELECT_CK00);
        }
        break;
        case I2C31:
        {
			CGC_PER0PeriphClockCmd((uint8_t)(I2Cx >> 10), ENABLE);
            I2C7_TypeDef *I2C_Instance = &SCI2->I2C7;
			SCIPeriphal_ClockUpdate(I2Cx, I2C_GetSysClock(), 
									I2C_InitStruct->I2C_ClockSpeed, 
									I2C_Instance->SPS,
									&pValue);
			I2Cx_Init(I2C_Instance, pValue, SCI_CLOCK_SELECT_CK00);
        }
        break;
#endif		
		default:
		break;
    }
	
	return ret;
}

/**
  * @brief  Set I2Cx Bus to generate a START signal and enable the bus.
  * @param  I2Cx: where x can be 0, 1, 2, 3, 4 or 5 to select the I2C peripheral.
  * @retval None.
  */
static inline void I2C_StartCondition(SCIAFSelect_TypeDef I2Cx)
{
	assert_param(IS_I2C_ALL_PERIPH(I2Cx));
	
    switch(I2Cx)
    {
        case I2C00:
        {
            I2C0_TypeDef *I2C_Instance = &SCI0->I2C0;
			I2Cx_StartCondition(I2C_Instance);
			I2Cx_Channel_Enable(I2C_Instance);
        }
        break;
        case I2C01:
        {
            I2C1_TypeDef *I2C_Instance = &SCI0->I2C1;
			I2Cx_StartCondition(I2C_Instance);
			I2Cx_Channel_Enable(I2C_Instance);
        }
        break;
        case I2C10:
        {
            I2C2_TypeDef *I2C_Instance = &SCI0->I2C2;
			I2Cx_StartCondition(I2C_Instance);
			I2Cx_Channel_Enable(I2C_Instance);
        }
        break;
        case I2C11:
        {
            I2C3_TypeDef *I2C_Instance = &SCI0->I2C3;
			I2Cx_StartCondition(I2C_Instance);
			I2Cx_Channel_Enable(I2C_Instance);
        }
        break;
        case I2C20:
        {
            I2C4_TypeDef *I2C_Instance = &SCI1->I2C4;
			I2Cx_StartCondition(I2C_Instance);
			I2Cx_Channel_Enable(I2C_Instance);
        }
        break;
        case I2C21:
        {
            I2C5_TypeDef *I2C_Instance = &SCI1->I2C5;
			I2Cx_StartCondition(I2C_Instance);
			I2Cx_Channel_Enable(I2C_Instance);
        }
        break;
#if defined(BAT32G1XX_80PIN) || defined(BAT32G1XX_100PIN)
        case I2C30:
        {
            I2C6_TypeDef *I2C_Instance = &SCI2->I2C6;
			I2Cx_StartCondition(I2C_Instance);
			I2Cx_Channel_Enable(I2C_Instance);
        }
        break;
        case I2C31:
        {
            I2C7_TypeDef *I2C_Instance = &SCI2->I2C7;
			I2Cx_StartCondition(I2C_Instance);
			I2Cx_Channel_Enable(I2C_Instance);
        }
        break;
#endif		
		default:
		break;
    }
}

/**
  * @brief  Set I2Cx Bus to generate a STOP signal and disable the bus.
  * @param  I2Cx: where x can be 0, 1, 2, 3, 4 or 5 to select the I2C peripheral.
  * @retval None.
  */
static inline void I2C_StopCondition(SCIAFSelect_TypeDef I2Cx)
{
	assert_param(IS_I2C_ALL_PERIPH(I2Cx));
	
	switch(I2Cx)
	{
		case I2C00:
		{
			I2C0_TypeDef *I2C_Instance = &SCI0->I2C0;
			I2Cx_Channel_Disable(I2C_Instance);
			I2Cx_StopCondition(I2C_Instance);
		}
		break;
		case I2C01:
		{
			I2C1_TypeDef *I2C_Instance = &SCI0->I2C1;
			I2Cx_Channel_Disable(I2C_Instance);
			I2Cx_StopCondition(I2C_Instance);
		}
		break;
		case I2C10:
		{
			I2C2_TypeDef *I2C_Instance = &SCI0->I2C2;
			I2Cx_Channel_Disable(I2C_Instance);
			I2Cx_StopCondition(I2C_Instance);
		}
		break;
		case I2C11:
		{
			I2C3_TypeDef *I2C_Instance = &SCI0->I2C3;
			I2Cx_Channel_Disable(I2C_Instance);
			I2Cx_StopCondition(I2C_Instance);
		}
		break;
		case I2C20:
		{
			I2C4_TypeDef *I2C_Instance = &SCI1->I2C4;
			I2Cx_Channel_Disable(I2C_Instance);
			I2Cx_StopCondition(I2C_Instance);
		}
		break;
		case I2C21:
		{
			I2C5_TypeDef *I2C_Instance = &SCI1->I2C5;
			I2Cx_Channel_Disable(I2C_Instance);
			I2Cx_StopCondition(I2C_Instance);
		}
		break;
#if defined(BAT32G1XX_80PIN) || defined(BAT32G1XX_100PIN)
        case I2C30:
        {
            I2C6_TypeDef *I2C_Instance = &SCI2->I2C6;
			I2Cx_Channel_Disable(I2C_Instance);
			I2Cx_StopCondition(I2C_Instance);
        }
        break;
        case I2C31:
        {
            I2C7_TypeDef *I2C_Instance = &SCI2->I2C7;
			I2Cx_Channel_Disable(I2C_Instance);
			I2Cx_StopCondition(I2C_Instance);
        }
        break;
#endif		
		default:
		break;
	}
}

/**
  * @brief  Set I2Cx bus output enable or disable
  * @param  I2Cx: where x can be 0, 1, 2, 3, 4 or 5 to select the I2C peripheral.
  * @param  NewState: I2Cx bus signal output status.
  *          This parameter can be one of the following values:
  *            @arg ENABLE: I2Cx bus signal output wave enable.
  *            @arg ISABLE: I2Cx bus signal output wave disable.
  * @retval None.
  */
void I2C_Output_Cmd(SCIAFSelect_TypeDef I2Cx, FunctionalState NewState)
{
	assert_param(IS_I2C_ALL_PERIPH(I2Cx));
	
	switch(I2Cx)
	{
		case I2C00:
		{
			I2C0_TypeDef *I2C_Instance = &SCI0->I2C0;
			I2C_Instance->SOE = (NewState == ENABLE) ? I2C_START_EN : I2C_START_DIS;
		}
		break;
		case I2C01:
		{
			I2C1_TypeDef *I2C_Instance = &SCI0->I2C1;
			I2C_Instance->SOE = (NewState == ENABLE) ? I2C_START_EN : I2C_START_DIS;
		}
		break;
		case I2C10:
		{
			I2C2_TypeDef *I2C_Instance = &SCI0->I2C2;
			I2C_Instance->SOE = (NewState == ENABLE) ? I2C_START_EN : I2C_START_DIS;
		}
		break;
		case I2C11:
		{
			I2C3_TypeDef *I2C_Instance = &SCI0->I2C3;
			I2C_Instance->SOE = (NewState == ENABLE) ? I2C_START_EN : I2C_START_DIS;
		}
		break;
		case I2C20:
		{
			I2C4_TypeDef *I2C_Instance = &SCI1->I2C4;
			I2C_Instance->SOE = (NewState == ENABLE) ? I2C_START_EN : I2C_START_DIS;
		}
		break;
		case I2C21:
		{
			I2C5_TypeDef *I2C_Instance = &SCI1->I2C5;
			I2C_Instance->SOE = (NewState == ENABLE) ? I2C_START_EN : I2C_START_DIS;
		}
		break;
#if defined(BAT32G1XX_80PIN) || defined(BAT32G1XX_100PIN)
        case I2C30:
        {
            I2C6_TypeDef *I2C_Instance = &SCI2->I2C6;
			I2C_Instance->SOE = (NewState == ENABLE) ? I2C_START_EN : I2C_START_DIS;
        }
        break;
        case I2C31:
        {
            I2C7_TypeDef *I2C_Instance = &SCI2->I2C7;
			I2C_Instance->SOE = (NewState == ENABLE) ? I2C_START_EN : I2C_START_DIS;
        }
        break;
#endif			
		default:
		break;
	}
}

/**
  * @brief  Set I2Cx bus output channel start or stop
  * @param  I2Cx: where x can be 0, 1, 2, 3, 4 or 5 to select the I2C peripheral.
  * @param  NewState: I2Cx bus signal output status.
  *          This parameter can be one of the following values:
  *            @arg ENABLE: I2Cx bus channle start enable.
  *            @arg DISABLE: I2Cx bus channle stop 
  * @retval None.
  */
void I2C_ChannelStart_Cmd(SCIAFSelect_TypeDef I2Cx, FunctionalState NewState)
{
	assert_param(IS_I2C_ALL_PERIPH(I2Cx));
	
	switch(I2Cx)
	{
		case I2C00:
		{
			I2C0_TypeDef *I2C_Instance = &SCI0->I2C0;
			if(NewState == ENABLE) 
				I2C_Instance->SS =  I2C_START_EN;
			else
				I2C_Instance->ST =  I2C_START_EN;
		}
		break;
		case I2C01:
		{
			I2C1_TypeDef *I2C_Instance = &SCI0->I2C1;
			if(NewState == ENABLE) 
				I2C_Instance->SS =  I2C_START_EN;
			else
				I2C_Instance->ST =  I2C_START_EN;
		}
		break;
		case I2C10:
		{
			I2C2_TypeDef *I2C_Instance = &SCI0->I2C2;
			if(NewState == ENABLE) 
				I2C_Instance->SS =  I2C_START_EN;
			else
				I2C_Instance->ST =  I2C_START_EN;
		}
		break;
		case I2C11:
		{
			I2C3_TypeDef *I2C_Instance = &SCI0->I2C3;
			if(NewState == ENABLE) 
				I2C_Instance->SS =  I2C_START_EN;
			else
				I2C_Instance->ST =  I2C_START_EN;
		}
		break;
		case I2C20:
		{
			I2C4_TypeDef *I2C_Instance = &SCI1->I2C4;
			if(NewState == ENABLE) 
				I2C_Instance->SS =  I2C_START_EN;
			else
				I2C_Instance->ST =  I2C_START_EN;
		}
		break;
		case I2C21:
		{
			I2C5_TypeDef *I2C_Instance = &SCI1->I2C5;
			if(NewState == ENABLE) 
				I2C_Instance->SS =  I2C_START_EN;
			else
				I2C_Instance->ST =  I2C_START_EN;
		}
		break;
#if defined(BAT32G1XX_80PIN) || defined(BAT32G1XX_100PIN)
        case I2C30:
        {
            I2C6_TypeDef *I2C_Instance = &SCI2->I2C6;
			if(NewState == ENABLE) 
				I2C_Instance->SS =  I2C_START_EN;
			else
				I2C_Instance->ST =  I2C_START_EN;
        }
        break;
        case I2C31:
        {
            I2C7_TypeDef *I2C_Instance = &SCI2->I2C7;
			if(NewState == ENABLE) 
				I2C_Instance->SS =  I2C_START_EN;
			else
				I2C_Instance->ST =  I2C_START_EN;
        }
        break;
#endif			
		
		default:
		break;
	}
}
/**
  * @brief  Set I2Cx communication mode in send or receive.
  * @param  I2Cx: where x can be 0, 1, 2, 3, 4 or 5 to select the I2C peripheral.
  * @param  Mode: I2Cx Transmit mode configurate.
  *          This parameter can be one of the following values:
  *            @arg I2C_TransmitMode_Send: Send mode setting for I2Cx bus.
  *            @arg I2C_TransmitMode_Recv: Receive mode setting for I2Cx bus.
  * @retval None.
  */
void I2C_Set_TransmitMode(SCIAFSelect_TypeDef I2Cx, uint8_t Mode)
{
	assert_param(IS_I2C_ALL_PERIPH(I2Cx));
	assert_param(IS_I2C_TRANSMIT_MODE(Mode));
	
	switch(I2Cx)
	{
		case I2C00:
		{
			I2C0_TypeDef *I2C_Instance = &SCI0->I2C0;
			I2Cx_Chanel_ModeConfigurate(I2C_Instance, Mode);
		}
		break;
		case I2C01:
		{
			I2C1_TypeDef *I2C_Instance = &SCI0->I2C1;
			I2Cx_Chanel_ModeConfigurate(I2C_Instance, Mode);
		}
		break;
		case I2C10:
		{
			I2C2_TypeDef *I2C_Instance = &SCI0->I2C2;
			I2Cx_Chanel_ModeConfigurate(I2C_Instance, Mode);
		}
		break;
		case I2C11:
		{
			I2C3_TypeDef *I2C_Instance = &SCI0->I2C3;
			I2Cx_Chanel_ModeConfigurate(I2C_Instance, Mode);
		}
		break;
		case I2C20:
		{
			I2C4_TypeDef *I2C_Instance = &SCI1->I2C4;
			I2Cx_Chanel_ModeConfigurate(I2C_Instance, Mode);
		}
		break;
		case I2C21:
		{
			I2C5_TypeDef *I2C_Instance = &SCI1->I2C5;
			I2Cx_Chanel_ModeConfigurate(I2C_Instance, Mode);
		}
		break;
#if defined(BAT32G1XX_80PIN) || defined(BAT32G1XX_100PIN)
        case I2C30:
        {
            I2C6_TypeDef *I2C_Instance = &SCI2->I2C6;
			I2Cx_Chanel_ModeConfigurate(I2C_Instance, Mode);
        }
        break;
        case I2C31:
        {
            I2C7_TypeDef *I2C_Instance = &SCI2->I2C7;
			I2Cx_Chanel_ModeConfigurate(I2C_Instance, Mode);
        }
        break;
#endif				
		default:
		break;
	}
}

/**
  * @brief  Get I2Cx communication mode in send or receive.
  * @param  I2Cx: where x can be 0, 1, 2, 3, 4 or 5 to select the I2C peripheral.
  * @retval  Mode: I2Cx Transmit mode in configurate.
  */
uint8_t I2C_Get_TransmitMode(SCIAFSelect_TypeDef I2Cx)
{
    uint8_t Mode = 0;

	assert_param(IS_I2C_ALL_PERIPH(I2Cx));
	
	switch(I2Cx)
	{
		case I2C00:
		{
			I2C0_TypeDef *I2C_Instance = &SCI0->I2C0;
            Mode = I2C_TransmitMode_Mask & ((I2C_Instance->SCR & SCI_RECEPTION_TRANSMISSION) >> 14);
		}
		break;
		case I2C01:
		{
			I2C1_TypeDef *I2C_Instance = &SCI0->I2C1;
            Mode = I2C_TransmitMode_Mask & ((I2C_Instance->SCR & SCI_RECEPTION_TRANSMISSION) >> 14);
		}
		break;
		case I2C10:
		{
			I2C2_TypeDef *I2C_Instance = &SCI0->I2C2;
            Mode = I2C_TransmitMode_Mask & ((I2C_Instance->SCR & SCI_RECEPTION_TRANSMISSION) >> 14);
		}
		break;
		case I2C11:
		{
			I2C3_TypeDef *I2C_Instance = &SCI0->I2C3;
            Mode = I2C_TransmitMode_Mask & ((I2C_Instance->SCR & SCI_RECEPTION_TRANSMISSION) >> 14);
		}
		break;
		case I2C20:
		{
			I2C4_TypeDef *I2C_Instance = &SCI1->I2C4;
            Mode = I2C_TransmitMode_Mask & ((I2C_Instance->SCR & SCI_RECEPTION_TRANSMISSION) >> 14);
		}
		break;
		case I2C21:
		{
			I2C5_TypeDef *I2C_Instance = &SCI1->I2C5;
            Mode = I2C_TransmitMode_Mask & ((I2C_Instance->SCR & SCI_RECEPTION_TRANSMISSION) >> 14);
		}
		break;
#if defined(BAT32G1XX_80PIN) || defined(BAT32G1XX_100PIN)
        case I2C30:
        {
            I2C6_TypeDef *I2C_Instance = &SCI2->I2C6;
            Mode = I2C_TransmitMode_Mask & ((I2C_Instance->SCR & SCI_RECEPTION_TRANSMISSION) >> 14);
        }
        break;
        case I2C31:
        {
            I2C7_TypeDef *I2C_Instance = &SCI2->I2C7;
            Mode = I2C_TransmitMode_Mask & ((I2C_Instance->SCR & SCI_RECEPTION_TRANSMISSION) >> 14);
        }
        break;
#endif				
		default:
		break;
	}

    return Mode;
}

/**
  * @brief  Generates I2Cx communication START condition.
  * @param  I2Cx: where x can be 0, 1, 2, 3, 4 or 5 to select the I2C peripheral.
  * @retval None.
  */
void I2C_GenerateSTART(SCIAFSelect_TypeDef I2Cx)
{
    /* Check the parameters */
    assert_param(IS_I2C_ALL_PERIPH(I2Cx));

    I2C_StartCondition(I2Cx);
}

/**
  * @brief  Generates I2Cx communication STOP condition.
  * @param  I2Cx: where x can be 0, 1, 2, 3, 4 or 5 to select the I2C peripheral.
  * @retval None.
  */
void I2C_GenerateSTOP(SCIAFSelect_TypeDef I2Cx)
{
    /* Check the parameters */
    assert_param(IS_I2C_ALL_PERIPH(I2Cx));

    I2C_StopCondition(I2Cx);
}

/** @defgroup I2C_Group Data transfers functions
 *  @brief   Data transfers functions 
 *
@verbatim   
===============================================================================
                ##### Data transfers functions #####
===============================================================================  

@endverbatim
  * @{
  */

/**
  * @brief  Sends a data byte through the I2Cx peripheral.
  * @param  I2Cx: where x can be 1, 2 or 3 to select the I2C peripheral.
  * @param  Data: Byte to be transmitted..
  * @retval None
  */
void I2C_SendByte(SCIAFSelect_TypeDef I2Cx, uint8_t Data)
{
    /* Check the parameters */
    assert_param(IS_I2C_ALL_PERIPH(I2Cx));

    /* Write in the SIO register the data to be sent */
    switch(I2Cx)
    {
        case I2C00:
        {
            I2C0_TypeDef *I2C_Instance = &SCI0->I2C0;
            I2C_Instance->SIO = Data;
        }
        break;
        case I2C01:
        {
            I2C1_TypeDef *I2C_Instance = &SCI0->I2C1;
            I2C_Instance->SIO = Data;
        }
        break;
        case I2C10:
        {
            I2C2_TypeDef *I2C_Instance = &SCI0->I2C2;
            I2C_Instance->SIO = Data;
        }
        break;
        case I2C11:
        {
            I2C3_TypeDef *I2C_Instance = &SCI0->I2C3;
            I2C_Instance->SIO = Data;
        }
        break;
        case I2C20:
        {
            I2C4_TypeDef *I2C_Instance = &SCI1->I2C4;
            I2C_Instance->SIO = Data;
        }
        break;
        case I2C21:
        {
            I2C5_TypeDef *I2C_Instance = &SCI1->I2C5;
            I2C_Instance->SIO = Data;
        }
        break;
#if defined(BAT32G1XX_80PIN) || defined(BAT32G1XX_100PIN)
        case I2C30:
        {
            I2C6_TypeDef *I2C_Instance = &SCI2->I2C6;
            I2C_Instance->SIO = Data;
        }
        break;
        case I2C31:
        {
            I2C7_TypeDef *I2C_Instance = &SCI2->I2C7;
            I2C_Instance->SIO = Data;
        }
        break;
#endif			
		default:
		break;
    }
}

/**
  * @brief  Send a data buffer through the I2Cx peripheral to write.
  * @param  I2Cx: where x can be 1, 2 or 3 to select the I2C peripheral.
  * @param  Address: Device address in the I2C bus.
  * @param  Reg: Register address in the Device.
  * @param  Data: Data buffer address.
  * @param  Len: Data buffer length need to transmit.
  * @retval None
  */
void I2C_WriteData(SCIAFSelect_TypeDef I2Cx, uint8_t Address, uint8_t Reg, uint8_t *Data, uint16_t Len)
{
    IRQn_Type irq;

    /* Check the parameters */
    assert_param(IS_I2C_ALL_PERIPH(I2Cx));

    /* Get the IQR number depend on the I2Cx in SCI unit */
    irq = I2C_IRQTable[(((I2Cx >> 12) + 1) * 2) + (I2Cx & 0x0F)];

    /* When the I2C bus start,we should set the bus register mode in Send */
	I2C_Set_TransmitMode(I2Cx, I2C_TransmitMode_Send);

    /* Generate a START signal to transmission */
	I2C_GenerateSTART(I2Cx);

    /* Device address send */
	I2C_SendByte(I2Cx, Address&0xFE);

    /* Wait device address send succes */
	while(!INTC_GetPendingIRQ(irq));
	INTC_ClearPendingIRQ(irq);
	
    /* Register address send */
	I2C_SendByte(I2Cx, Reg);
	
    /* Wait register address send succes */
	while(!INTC_GetPendingIRQ(irq));
	INTC_ClearPendingIRQ(irq);
	
    /* Data buffer send byte by byte */
	do {
		I2C_SendByte(I2Cx, *Data++);

		while(!INTC_GetPendingIRQ(irq));
		INTC_ClearPendingIRQ(irq);
	}while(--Len);
	
    /* Generate a STOP signal in transmission end */
	I2C_GenerateSTOP(I2Cx);
}

/**
  * @brief  Returns the most recent received data by the I2Cx peripheral.
  * @param  I2Cx: where x can be 1, 2 or 3 to select the I2C peripheral.
  * @retval The value of the received byte data.
  */
uint8_t I2C_ReceiveByte(SCIAFSelect_TypeDef I2Cx)
{
    uint8_t data = 0;

    /* Return the data in the SIO register */
    /* Check the parameters */
    assert_param(IS_I2C_ALL_PERIPH(I2Cx));

    switch(I2Cx)
    {
        case I2C00:
        {
            I2C0_TypeDef *I2C_Instance = &SCI0->I2C0;
            data = I2C_Instance->SIO;
        }
        break;
        case I2C01:
        {
            I2C1_TypeDef *I2C_Instance = &SCI0->I2C1;
            data = I2C_Instance->SIO;
        }
        break;
        case I2C10:
        {
            I2C2_TypeDef *I2C_Instance = &SCI0->I2C2;
            data = I2C_Instance->SIO;
        }
        break;
        case I2C11:
        {
            I2C3_TypeDef *I2C_Instance = &SCI0->I2C3;
            data = I2C_Instance->SIO;
        }
        break;
        case I2C20:
        {
            I2C4_TypeDef *I2C_Instance = &SCI1->I2C4;
            data = I2C_Instance->SIO;
        }
        break;
        case I2C21:
        {
            I2C5_TypeDef *I2C_Instance = &SCI1->I2C5;
            data = I2C_Instance->SIO;
        }
        break;
#if defined(BAT32G1XX_80PIN) || defined(BAT32G1XX_100PIN)
        case I2C30:
        {
            I2C6_TypeDef *I2C_Instance = &SCI2->I2C6;
            data = I2C_Instance->SIO;
        }
        break;
        case I2C31:
        {
            I2C7_TypeDef *I2C_Instance = &SCI2->I2C7;
            data = I2C_Instance->SIO;
        }
        break;
#endif		
		default:
		break;
    }

    return data;
}

/**
  * @brief  Read a data buffer through the I2Cx peripheral to device register address.
  * @param  I2Cx: where x can be 1, 2 or 3 to select the I2C peripheral.
  * @param  Address: Device address in the I2C bus.
  * @param  Reg: Register address in the Device.
  * @param  Data: Data buffer address.
  * @param  Len: Data buffer length need to transmit read.
  * @retval None
  */
void I2C_ReadData(SCIAFSelect_TypeDef I2Cx, uint8_t Address, uint8_t Reg, uint8_t *Data, uint16_t Len)
{
    IRQn_Type irq;

    /* Check the parameters */
    assert_param(IS_I2C_ALL_PERIPH(I2Cx));

    /* Get the IQR number depend on the I2Cx in SCI unit */
    irq = I2C_IRQTable[(((I2Cx >> 12) + 1) * 2) + (I2Cx & 0x0F)];

    /* When the I2C bus start,we should set the bus register mode in Send */
	I2C_Set_TransmitMode(I2Cx, I2C_TransmitMode_Send);

    /* Generate a START signal to transmission */
	I2C_GenerateSTART(I2Cx);

    /* Device address send */
	I2C_SendByte(I2Cx, Address & 0xFE);

    /* Wait device address send succes */
	while(!INTC_GetPendingIRQ(irq));
	INTC_ClearPendingIRQ(irq);
	
    /* Register address send */
	I2C_SendByte(I2Cx, Reg);

    /* Wait register address send succes */
	while(!INTC_GetPendingIRQ(irq));
	INTC_ClearPendingIRQ(irq);
	
	/* I2C bus restart and mode configurated by receive mode */
    I2C_GenerateSTOP(I2Cx);
    I2C_Set_TransmitMode(I2Cx, I2C_TransmitMode_Recv);
    I2C_GenerateSTART(I2Cx);

	/* Send device address retry and sign the WR bit to read */
	I2C_SendByte(I2Cx, Address | 0x01);
	while(!INTC_GetPendingIRQ(irq));
	INTC_ClearPendingIRQ(irq);

    /* Start to receive data to the dest memory buffer */
	do {
		/* When the last byte to receive, we should not output ACK */
		if(Len == 1U)
			I2C_Output_Cmd(I2Cx, DISABLE);
        
		/* Write the virtual data to SIO in order to start receive data */
		I2C_SendByte(I2Cx, 0xFFU);
		
		/* Wait the virtual data send success */
		while(!INTC_GetPendingIRQ(irq));
		INTC_ClearPendingIRQ(irq);
		
		/* Read the receive data from SIO register to target memory buffer */
		*Data++ = I2C_ReceiveByte(I2Cx);
		
	}while(--Len);

	/* Generate STOP signal to the read flow completed */
    I2C_GenerateSTOP(I2Cx);
}


/**
  * @brief  Clear the specified I2C flag is set.
  * @param  I2Cx: where x can be 0, 1, 2, 4, 5 to select the I2C peripheral.
  * @param  I2C_FLAG: specifies the flag to check.
  *          This parameter can be one of the following values:
  *            @arg I2C_FLAG_NOACK: Transmission frame have no ack check.
  *            @arg I2C_FLAG_OVERRUN: Data over run.
  * @retval None.
  */
FlagStatus I2C_GetErrStaus(SCIAFSelect_TypeDef I2Cx, uint16_t I2C_FLAG)
{
	FlagStatus status;
	
    /* Check the parameters */
	assert_param(IS_I2C_ALL_PERIPH(I2Cx));
    assert_param(IS_I2C_FLAG(I2C_FLAG));

    /* Clear the selected I2C flag */
	status = SCI_GetErrStaus(I2Cx,I2C_FLAG);
	
	return status;
}

/**
  * @brief  Clear the specified I2C flag is set.
  * @param  I2Cx: where x can be 0, 1, 2, 4, 5 to select the I2C peripheral.
  * @param  I2C_FLAG: specifies the flag to check.
  *          This parameter can be one of the following values:
  *            @arg I2C_CLEAR_FLAG_FEF: Transmission frame data error flag
  *            @arg I2C_CLEAR_FLAG_PEF: I2C Parity error flag
  *            @arg I2C_CLEAR_FLAG_OVF: Transmission over run error flag
  * @retval None.
  */
void I2C_ClearFlag(SCIAFSelect_TypeDef I2Cx, uint16_t I2C_FLAG)
{
    /* Check the parameters */
	assert_param(IS_I2C_ALL_PERIPH(I2Cx));
    assert_param(IS_I2C_CLEAR_FLAG(I2C_FLAG));

    /* Clear the selected I2C flag */
	SCI_ClearFlag(I2Cx,I2C_FLAG);
}
