/**
  ******************************************************************************
  * @file    sspi.c
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
#include "sspi.h"
#include "dma.h"
#include "cgc.h"
uint8_t g_spi_tx_end =0,g_spi_rx_end =0;


NSS_FUNC_T NSS;
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

/**
  * @brief  Deinitializes the UARTx peripheral registers to their default reset values.
  * @param  UARTx: where x can be 0, 1, 2 to select the UART peripheral.
  * @retval None
  */
void SSPI_DeInit(SCIAFSelect_TypeDef SSPIx)
{
	/* Check the parameters */
	assert_param(IS_SSPI_ALL_PERIPH(SSPIx));

	CGC_PER0PeriphClockCmd(SSPIx >> 10, DISABLE);

	SCIChannel_Free(SSPIx, SCI_DEFAULT_MODE);
}

/* SSPIx init define by clock and clock source select */
#define SSPIx_Init(SSPI_Instance,SPI_Mode,Phase_Mode,SPI_Bitorder,SPI_DataSize, clock, clock_sel) \
{																			\
	uint16_t tmpreg = 0;                                              		\
																			\
	SSPI_Instance->ST |= SSPI_STOP_EN;                                 	 	\
																			\
	tmpreg |= SSPI_CLEAR_FLAG_FEF;                                      	\
	tmpreg |= SSPI_CLEAR_FLAG_PEF;                                      	\
	tmpreg |= SSPI_CLEAR_FLAG_OVF;                                      	\
	SSPI_Instance->SIR = tmpreg;                                        	\
																			\
	tmpreg = SSPI_Instance->SMR;                                        	\
	tmpreg |= SCI_SMRMN_DEFAULT_VALUE;								    	\
	if(SPI_Mode ==SPI_Mode_Master)											\
		tmpreg |= SCI_CLOCK_MODE_CKS | SCI_MODE_SPI | SCI_TRANSFER_END;		\
	else																	\
		tmpreg |= SCI_CLOCK_MODE_TI0N | SCI_MODE_SPI | SCI_TRANSFER_END;	\
	tmpreg |= clock_sel;													\
	SSPI_Instance->SMR = tmpreg;                                        	\
																			\
    tmpreg = 0;                                                         	\
    tmpreg |= SCI_SCRMN_DEFAULT_VALUE;                                  	\
    tmpreg |= SCI_RECEPTION_TRANSMISSION;                               	\
    tmpreg |= Phase_Mode;                                               	\
	tmpreg |= SPI_Bitorder;                                             	\
    tmpreg |= SPI_DataSize;                                             	\
    SSPI_Instance->SCR = tmpreg;                                        	\
	if(SPI_Mode ==SPI_Mode_Master)											\
	{																		\
		SSPI_Instance->SPS&= ~SSPI_SPS_MAX_VALUE;                       	\
		SSPI_Instance->SPS = clock.sps;										\
		SSPI_Instance->SDR = clock.sdr;										\
	}																		\
	else																	\
	{																		\
		SSPI_Instance->SDR =0x0000;											\
	}																		\
	if(SPI_Mode ==SPI_Mode_Master)											\
	{																		\
		if((Phase_Mode ==SPI_Phase_Mode0) ||(Phase_Mode==SPI_Phase_Mode1))  \
			SSPI_Instance->CKO = SSPI_BusLevel_Low;                     	\
		else																\
			SSPI_Instance->CKO = SSPI_BusLevel_High;                    	\
	}																		\
	SSPI_Instance->SOE = SSPI_START_EN;                     		    	\
}

/* SSPIx BUS channel enable */
#define SSPIx_Channel_Enable(SSPI_Instance)		\
{												\
	SSPI_Instance->SOE = SSPI_START_EN;			\
	SSPI_Instance->SS = SSPI_START_EN;      	\
}

/* SSPIx BUS channel disable */
#define SSPIx_Channel_Disable(SSPI_Instance)	\
{                                           	\
	SSPI_Instance->ST = SSPI_STOP_EN;           \
	SSPI_Instance->SOE = SSPI_START_DIS;        \
}
static uint32_t SSPI_GetSysClock(void)
{
	return SystemCoreClock;
}
/* SSPI BUS mode configurate by send or receive */
#define SSPIx_Chanel_ModeConfigurate(SSPI_Instance, Mode)			\
{																	\
	SSPI_Instance->SCR &= (uint16_t) ~(SCI_RECEPTION_TRANSMISSION); \
	if(Mode == SSPI_TransmitMode_TxRx)								\
		SSPI_Instance->SCR |= SCI_RECEPTION_TRANSMISSION;			\
	else															\
	{																\
		SSPI_Instance->SCR |= (Mode == SSPI_TransmitMode_Send) ?    \
								SCI_TRANSMISSION : SCI_RECEPTION;   \
	}																\
}


/***********************************************************************************************************************
* Function Name: SCI_SPI_Init
* @brief  spi init  
* @param  SCIAFSelect_TypeDef /SPI_InitStruct
* @return init result 
***********************************************************************************************************************/
int8_t SSPI_Init(SCIAFSelect_TypeDef SSPIx, SPI_InitTypeDef * SPI_InitStruct)
{
	assert_param(IS_SSPI_ALL_PERIPH(SSPIx));
	
	int8_t ret = SCI_SUCCESS;
    SCIPeriph_Clock_TypeDef pValue;
	
	ret = SCIChannel_Alloca(SSPIx, SCI_DEFAULT_MODE);
	if(ret != SCI_SUCCESS)
		return ret;

	
    switch(SSPIx)
    {
        case SSPI00:
        {
			/* Clock gate control register setting to enable SCI0 clock */
			CGC_PER0PeriphClockCmd((uint8_t)((SSPIx >> 10)+SCI_UINT_OFFSET), ENABLE);
			
            I2C0_TypeDef *SSPI_Instance = &SCI0->I2C0;
			SCIPeriphal_ClockUpdate(SSPIx, SSPI_GetSysClock(), 
									SPI_InitStruct->SPI_ClockSpeed, 
									SSPI_Instance->SPS,
									&pValue);
			SSPIx_Init(SSPI_Instance,SPI_InitStruct->SPI_Mode,SPI_InitStruct->SPI_Phase_Mode,	       \
				SPI_InitStruct->SPI_Bitorder,SPI_InitStruct->SPI_DataSize,pValue,SCI_CLOCK_SELECT_CK00);
			if(SPI_InitStruct->SPI_Mode == SPI_Mode_Slave)
			{
				MISC->ISC |= MISC_ISC_SSIE00_Msk;		
			}
        }
        break;
        case SSPI01:
        {
			CGC_PER0PeriphClockCmd((uint8_t)((SSPIx >> 10)+SCI_UINT_OFFSET), ENABLE);
            I2C1_TypeDef *SSPI_Instance = &SCI0->I2C1;
			SCIPeriphal_ClockUpdate(SSPIx, SSPI_GetSysClock(), 
									SPI_InitStruct->SPI_ClockSpeed, 
									SSPI_Instance->SPS,
									&pValue);
			SSPIx_Init(SSPI_Instance,SPI_InitStruct->SPI_Mode,SPI_InitStruct->SPI_Phase_Mode,	       \
				SPI_InitStruct->SPI_Bitorder,SPI_InitStruct->SPI_DataSize,pValue,SCI_CLOCK_SELECT_CK00);
        }
        break;
        case SSPI10:
        {
			CGC_PER0PeriphClockCmd((uint8_t)((SSPIx >> 10)+SCI_UINT_OFFSET), ENABLE);
            I2C2_TypeDef *SSPI_Instance = &SCI0->I2C2;
			SCIPeriphal_ClockUpdate(SSPIx, SSPI_GetSysClock(), 
									SPI_InitStruct->SPI_ClockSpeed, 
									SSPI_Instance->SPS,
									&pValue);
			SSPIx_Init(SSPI_Instance,SPI_InitStruct->SPI_Mode,SPI_InitStruct->SPI_Phase_Mode,	       \
				SPI_InitStruct->SPI_Bitorder,SPI_InitStruct->SPI_DataSize,pValue,SCI_CLOCK_SELECT_CK01);
		
        }
        break;
        case SSPI11:
        {
			CGC_PER0PeriphClockCmd((uint8_t)((SSPIx >> 10)+SCI_UINT_OFFSET), ENABLE);
            I2C3_TypeDef *SSPI_Instance = &SCI0->I2C3;
			SCIPeriphal_ClockUpdate(SSPIx, SSPI_GetSysClock(), 
									SPI_InitStruct->SPI_ClockSpeed, 
									SSPI_Instance->SPS,
									&pValue);
			SSPIx_Init(SSPI_Instance,SPI_InitStruct->SPI_Mode,SPI_InitStruct->SPI_Phase_Mode,	       \
				SPI_InitStruct->SPI_Bitorder,SPI_InitStruct->SPI_DataSize,pValue,SCI_CLOCK_SELECT_CK01);
        }
        break;
        case SSPI20:
        {
			CGC_PER0PeriphClockCmd((uint8_t)((SSPIx >> 10)+SCI_UINT_OFFSET), ENABLE);
            I2C4_TypeDef *SSPI_Instance = &SCI1->I2C4;
			SCIPeriphal_ClockUpdate(SSPIx, SSPI_GetSysClock(), 
									SPI_InitStruct->SPI_ClockSpeed, 
									SSPI_Instance->SPS,
									&pValue);
			SSPIx_Init(SSPI_Instance,SPI_InitStruct->SPI_Mode,SPI_InitStruct->SPI_Phase_Mode,	       \
				SPI_InitStruct->SPI_Bitorder,SPI_InitStruct->SPI_DataSize,pValue,SCI_CLOCK_SELECT_CK00);
        }
        break;
        case SSPI21:
        {
			CGC_PER0PeriphClockCmd((uint8_t)((SSPIx >> 10)+SCI_UINT_OFFSET), ENABLE);
            I2C5_TypeDef *SSPI_Instance = &SCI1->I2C5;
			SCIPeriphal_ClockUpdate(SSPIx, SSPI_GetSysClock(), 
									SPI_InitStruct->SPI_ClockSpeed, 
									SSPI_Instance->SPS,
									&pValue);
			SSPIx_Init(SSPI_Instance,SPI_InitStruct->SPI_Mode,SPI_InitStruct->SPI_Phase_Mode,	       \
				SPI_InitStruct->SPI_Bitorder,SPI_InitStruct->SPI_DataSize,pValue,SCI_CLOCK_SELECT_CK00);
        }
        break;
#if defined(BAT32G1XX_80PIN) || defined(BAT32G1XX_100PIN)
        case SSPI30:
        {
			CGC_PER2PeriphClockCmd((SSPIx >> 10), ENABLE);
            I2C6_TypeDef *SSPI_Instance = &SCI2->I2C6;
			SCIPeriphal_ClockUpdate(SSPIx, SSPI_GetSysClock(), 
									SPI_InitStruct->SPI_ClockSpeed, 
									SSPI_Instance->SPS,
									&pValue);
			SSPIx_Init(SSPI_Instance,SPI_InitStruct->SPI_Mode,SPI_InitStruct->SPI_Phase_Mode,	       \
				SPI_InitStruct->SPI_Bitorder,SPI_InitStruct->SPI_DataSize,pValue,SCI_CLOCK_SELECT_CK00);
        }
        break;
        case SSPI31:
        {
            I2C7_TypeDef *SSPI_Instance = &SCI2->I2C7;
			SCIPeriphal_ClockUpdate(SSPIx, SSPI_GetSysClock(), 
									SPI_InitStruct->SPI_ClockSpeed, 
									SSPI_Instance->SPS,
									&pValue);
			SSPIx_Init(SSPI_Instance,SPI_InitStruct->SPI_Mode,SPI_InitStruct->SPI_Phase_Mode,	       \
				SPI_InitStruct->SPI_Bitorder,SPI_InitStruct->SPI_DataSize,pValue,SCI_CLOCK_SELECT_CK00);
        }
        break;		
#endif		
		default:
		break;
    }

	return ret;
}

/**
  * @brief  Generates SSPI communication START 
  * @param  SSPIx: where x can be 0, 1, 2, 3, 4 or 5 to select the SSPI peripheral.
  * @retval None.
  */
void SSPI_Start(SCIAFSelect_TypeDef SSPIx)
{
    /* Check the parameters */
    assert_param(IS_SSPI_ALL_PERIPH(SSPIx));

	NSS.Active();
	switch(SSPIx)
	{
		case SSPI00:
		{
			I2C0_TypeDef *SSPI_Instance = &SCI0->I2C0;
			SSPIx_Channel_Enable(SSPI_Instance);
		}
		break;
		case SSPI01:
		{
			I2C1_TypeDef *SSPI_Instance = &SCI0->I2C1;
			SSPIx_Channel_Enable(SSPI_Instance);
		}
		break;
		case SSPI10:
		{
			I2C2_TypeDef *SSPI_Instance = &SCI0->I2C2;
			SSPIx_Channel_Enable(SSPI_Instance);
		}
		break;
		case SSPI11:
		{
			I2C3_TypeDef *SSPI_Instance = &SCI0->I2C3;
			SSPIx_Channel_Enable(SSPI_Instance);
		}
		break;
		case SSPI20:
		{
			I2C4_TypeDef *SSPI_Instance = &SCI1->I2C4;
			SSPIx_Channel_Enable(SSPI_Instance);
		}
		break;
		case SSPI21:
		{
			I2C5_TypeDef *SSPI_Instance = &SCI1->I2C5;
			SSPIx_Channel_Enable(SSPI_Instance);
		}
		break;
#if defined(BAT32G1XX_80PIN) || defined(BAT32G1XX_100PIN)
		case SSPI30:
		{
			I2C6_TypeDef *SSPI_Instance = &SCI2->I2C6;
			SSPIx_Channel_Enable(SSPI_Instance);
		}
		break;
		case SSPI31:
		{
			I2C7_TypeDef *SSPI_Instance = &SCI2->I2C7;
			SSPIx_Channel_Enable(SSPI_Instance);
		}
		break;
#endif		
		default:
		break;
	}
}

/**
  * @brief  Generates SSPI communication STOP 
  * @param  SSPIx: where x can be 0, 1, 2, 3, 4 or 5 to select the I2C peripheral.
  * @retval None.
  */
void SSPI_Stop(SCIAFSelect_TypeDef SSPIx)
{
    /* Check the parameters */
    assert_param(IS_SSPI_ALL_PERIPH(SSPIx));

	switch(SSPIx)
	{
		case SSPI00:
		{
			I2C0_TypeDef *SSPI_Instance = &SCI0->I2C0;
			SSPIx_Channel_Disable(SSPI_Instance);
			INTC_ClearPendingIRQ(SPI00_IRQn);
			NVIC_ClearPendingIRQ(SPI00_IRQn);
		}
		break;
		case SSPI01:
		{
			I2C1_TypeDef *SSPI_Instance = &SCI0->I2C1;
			SSPIx_Channel_Disable(SSPI_Instance);
			INTC_ClearPendingIRQ(SPI01_IRQn);
			NVIC_ClearPendingIRQ(SPI01_IRQn);			
		}
		break;
		case SSPI10:
		{
			I2C2_TypeDef *SSPI_Instance = &SCI0->I2C2;
			SSPIx_Channel_Disable(SSPI_Instance);
			INTC_ClearPendingIRQ(SPI10_IRQn);
			NVIC_ClearPendingIRQ(SPI10_IRQn);				
		}
		break;
		case SSPI11:
		{
			I2C3_TypeDef *SSPI_Instance = &SCI0->I2C3;
			SSPIx_Channel_Disable(SSPI_Instance);
			INTC_ClearPendingIRQ(SPI11_IRQn);
			NVIC_ClearPendingIRQ(SPI11_IRQn);				
		}
		break;
		case SSPI20:
		{
			I2C4_TypeDef *SSPI_Instance = &SCI1->I2C4;
			SSPIx_Channel_Disable(SSPI_Instance);
			INTC_ClearPendingIRQ(SPI20_IRQn);
			NVIC_ClearPendingIRQ(SPI20_IRQn);				
		}
		break;
		case SSPI21:
		{
			I2C5_TypeDef *SSPI_Instance = &SCI1->I2C5;
			SSPIx_Channel_Disable(SSPI_Instance);
			INTC_ClearPendingIRQ(SPI21_IRQn);
			NVIC_ClearPendingIRQ(SPI21_IRQn);				
		}
		break;
#if defined(BAT32G1XX_80PIN) || defined(BAT32G1XX_100PIN)
		case SSPI30:
		{
			I2C6_TypeDef *SSPI_Instance = &SCI2->I2C6;
			SSPIx_Channel_Disable(SSPI_Instance);
			INTC_ClearPendingIRQ(SPI30_IRQn);
			NVIC_ClearPendingIRQ(SPI30_IRQn);				
		}
		break;
		case SSPI31:
		{
			I2C7_TypeDef *SSPI_Instance = &SCI2->I2C7;
			SSPIx_Channel_Disable(SSPI_Instance);
			INTC_ClearPendingIRQ(SPI31_IRQn);
			NVIC_ClearPendingIRQ(SPI31_IRQn);				
		}
		break;
#endif		
		default:
		break;
	}
	NSS.Inactive();
}
/**
  * @brief  Set SSPIx communication mode in send or receive.
  * @param  SSPIx: where x can be 0, 1, 2, 3, 4 or 5 to select the SSPI peripheral.
  * @param  Mode: SSPIx Transmit mode configurate.
  *          This parameter can be one of the following values:
  *            @arg SSPI_TransmitMode_Send: Send mode setting for SSPIx bus.
  *            @arg SSPI_TransmitMode_Recv: Receive mode setting for SSPIx bus.
  *            @arg SSPI_TransmitMode_TxRx: Receive and send mode setting for SSPIx bus.
  * @retval None.
  */
void SSPI_Set_TransmitMode(SCIAFSelect_TypeDef SSPIx, uint8_t Mode)
{
	assert_param(IS_SSPI_ALL_PERIPH(SSPIx));
	assert_param(IS_SSPI_TRANSMIT_MODE(Mode));
	
	switch(SSPIx)
	{
		case SSPI00:
		{
			I2C0_TypeDef *SSPI_Instance = &SCI0->I2C0;
			SSPIx_Chanel_ModeConfigurate(SSPI_Instance, Mode);
		}
		break;
		case SSPI01:
		{
			I2C1_TypeDef *SSPI_Instance = &SCI0->I2C1;
			SSPIx_Chanel_ModeConfigurate(SSPI_Instance, Mode);
		}
		break;
		case SSPI10:
		{
			I2C2_TypeDef *SSPI_Instance = &SCI0->I2C2;
			SSPIx_Chanel_ModeConfigurate(SSPI_Instance, Mode);
		}
		break;
		case SSPI11:
		{
			I2C3_TypeDef *SSPI_Instance = &SCI0->I2C3;
			SSPIx_Chanel_ModeConfigurate(SSPI_Instance, Mode);
		}
		break;
		case SSPI20:
		{
			I2C4_TypeDef *SSPI_Instance = &SCI1->I2C4;
			SSPIx_Chanel_ModeConfigurate(SSPI_Instance, Mode);
		}
		break;
		case SSPI21:
		{
			I2C5_TypeDef *SSPI_Instance = &SCI1->I2C5;
			SSPIx_Chanel_ModeConfigurate(SSPI_Instance, Mode);
		}
		break;
#if defined(BAT32G1XX_80PIN) || defined(BAT32G1XX_100PIN)
		case SSPI30:
		{
			I2C6_TypeDef *SSPI_Instance = &SCI2->I2C6;
			SSPIx_Chanel_ModeConfigurate(SSPI_Instance, Mode);
		}
		break;
		case SSPI31:
		{
			I2C7_TypeDef *SSPI_Instance = &SCI2->I2C7;
			SSPIx_Chanel_ModeConfigurate(SSPI_Instance, Mode);
		}
		break;
#endif		
		default:
		break;
	}
}

/**
  * @brief  Get SSPIx communication mode in send or receive.
  * @param  SSPIx: where x can be 0, 1, 2, 3, 4 or 5 to select the SSPI peripheral.
  * @retval  Mode: I2Cx Transmit mode in configurate.
  */
uint8_t SSPI_Get_TransmitMode(SCIAFSelect_TypeDef SSPIx)
{
    uint8_t Mode = 0;

	assert_param(IS_SSPI_ALL_PERIPH(SSPIx));
	
	switch(SSPIx)
	{
		case SSPI00:
		{
			I2C0_TypeDef *SSPI_Instance = &SCI0->I2C0;
            Mode = SSPI_TransmitMode_Mask & ((SSPI_Instance->SCR & SCI_RECEPTION_TRANSMISSION) >> 14);
		}
		break;
		case SSPI01:
		{
			I2C1_TypeDef *SSPI_Instance = &SCI0->I2C1;
            Mode = SSPI_TransmitMode_Mask & ((SSPI_Instance->SCR & SCI_RECEPTION_TRANSMISSION) >> 14);
		}
		break;
		case SSPI10:
		{
			I2C2_TypeDef *SSPI_Instance = &SCI0->I2C2;
            Mode = SSPI_TransmitMode_Mask & ((SSPI_Instance->SCR & SCI_RECEPTION_TRANSMISSION) >> 14);
		}
		break;
		case SSPI11:
		{
			I2C3_TypeDef *SSPI_Instance = &SCI0->I2C3;
            Mode = SSPI_TransmitMode_Mask & ((SSPI_Instance->SCR & SCI_RECEPTION_TRANSMISSION) >> 14);
		}
		break;
		case SSPI20:
		{
			I2C4_TypeDef *SSPI_Instance = &SCI1->I2C4;
            Mode = SSPI_TransmitMode_Mask & ((SSPI_Instance->SCR & SCI_RECEPTION_TRANSMISSION) >> 14);
		}
		break;
		case SSPI21:
		{
			I2C5_TypeDef *SSPI_Instance = &SCI1->I2C5;
            Mode = SSPI_TransmitMode_Mask & ((SSPI_Instance->SCR & SCI_RECEPTION_TRANSMISSION) >> 14);
		}
		break;
#if defined(BAT32G1XX_80PIN) || defined(BAT32G1XX_100PIN)
		case SSPI30:
		{
			I2C6_TypeDef *SSPI_Instance = &SCI2->I2C6;
            Mode = SSPI_TransmitMode_Mask & ((SSPI_Instance->SCR & SCI_RECEPTION_TRANSMISSION) >> 14);
		}
		break;
		case SSPI31:
		{
			I2C7_TypeDef *SSPI_Instance = &SCI2->I2C7;
            Mode = SSPI_TransmitMode_Mask & ((SSPI_Instance->SCR & SCI_RECEPTION_TRANSMISSION) >> 14);
		}
		break;
#endif		
		default:
		break;
	}

    return Mode;
}

/**
  * @brief  Get SSPIx work mode in master or slave.
  * @param  SSPIx: where x can be 0, 1, 2, 3, 4 or 5 to select the I2C peripheral.
  * @retval  Mode: SSPIx work mode in configurate.
  */
uint8_t SSPI_Get_MasterMode(SCIAFSelect_TypeDef SSPIx)
{
    uint8_t Mode = 0;

	assert_param(IS_SSPI_ALL_PERIPH(SSPIx));
	
	switch(SSPIx)
	{
		case SSPI00:
		{
			I2C0_TypeDef *SSPI_Instance = &SCI0->I2C0;
            Mode = ((SSPI_Instance->SMR & (SPI_Mode_Slave<<14)) >> 14);
		}
		break;
		case SSPI01:
		{
			I2C1_TypeDef *SSPI_Instance = &SCI0->I2C1;
            Mode = ((SSPI_Instance->SMR & (SPI_Mode_Slave<<14)) >> 14);
		}
		break;
		case SSPI10:
		{
			I2C2_TypeDef *SSPI_Instance = &SCI0->I2C2;
            Mode = ((SSPI_Instance->SMR & (SPI_Mode_Slave<<14)) >> 14);
		}
		break;
		case SSPI11:
		{
			I2C3_TypeDef *SSPI_Instance = &SCI0->I2C3;
            Mode = ((SSPI_Instance->SMR & (SPI_Mode_Slave<<14)) >> 14);
		}
		break;
		case SSPI20:
		{
			I2C4_TypeDef *SSPI_Instance = &SCI1->I2C4;
            Mode = ((SSPI_Instance->SMR & (SPI_Mode_Slave<<14)) >> 14);
		}
		break;
		case SSPI21:
		{
			I2C5_TypeDef *SSPI_Instance = &SCI1->I2C5;
            Mode = ((SSPI_Instance->SMR & (SPI_Mode_Slave<<14)) >> 14);
		}
		break;
#if defined(BAT32G1XX_80PIN) || defined(BAT32G1XX_100PIN)
		case SSPI30:
		{
			I2C6_TypeDef *SSPI_Instance = &SCI2->I2C6;
            Mode = ((SSPI_Instance->SMR & (SPI_Mode_Slave<<14)) >> 14);
		}
		break;
		case SSPI31:
		{
			I2C7_TypeDef *SSPI_Instance = &SCI2->I2C7;
            Mode = ((SSPI_Instance->SMR & (SPI_Mode_Slave<<14)) >> 14);
		}
		break;
#endif		
		default:
		break;
	}

    return Mode;
}


/**
  * @brief  Sends a data byte through the SSPIx peripheral.
  * @param  SSPIx: where x can be 1, 2 or 3 to select the SSPIx peripheral.
  * @param  Data: Byte to be transmitted..
  * @retval None
  */
void SSPI_SendByte(SCIAFSelect_TypeDef SSPIx, uint8_t Data)
{
    /* Check the parameters */
    assert_param(IS_SSPI_ALL_PERIPH(SSPIx));

    /* Write in the SIO register the data to be sent */
    switch(SSPIx)
    {
        case SSPI00:
        {
            I2C0_TypeDef *SSPI_Instance = &SCI0->I2C0;
            SSPI_Instance->SIO = Data;
        }
        break;
        case SSPI01:
        {
            I2C1_TypeDef *SSPI_Instance = &SCI0->I2C1;
            SSPI_Instance->SIO = Data;
        }
        break;
        case SSPI10:
        {
            I2C2_TypeDef *SSPI_Instance = &SCI0->I2C2;
            SSPI_Instance->SIO = Data;
        }
        break;
        case SSPI11:
        {
            I2C3_TypeDef *SSPI_Instance = &SCI0->I2C3;
            SSPI_Instance->SIO = Data;
        }
        break;
        case SSPI20:
        {
            I2C4_TypeDef *SSPI_Instance = &SCI1->I2C4;
            SSPI_Instance->SIO = Data;
        }
        break;
        case SSPI21:
        {
            I2C5_TypeDef *SSPI_Instance = &SCI1->I2C5;
            SSPI_Instance->SIO = Data;
        }
        break;
#if defined(BAT32G1XX_80PIN) || defined(BAT32G1XX_100PIN)
        case SSPI30:
        {
            I2C6_TypeDef *SSPI_Instance = &SCI2->I2C6;
            SSPI_Instance->SIO = Data;
        }
        break;
        case SSPI31:
        {
            I2C7_TypeDef *SSPI_Instance = &SCI2->I2C7;
            SSPI_Instance->SIO = Data;
        }
        break;
#endif		
		default:
		break;
    }
}
/**
  * @brief  Returns the most recent received data by the I2Cx peripheral.
  * @param  SSPIx: where x can be 1, 2 or 3 to select the I2C peripheral.
  * @retval The value of the received byte data.
  */
uint8_t SSPI_ReceiveByte(SCIAFSelect_TypeDef SSPIx)
{
    uint8_t data = 0;

    /* Return the data in the SIO register */
    /* Check the parameters */
    assert_param(IS_SSPI_ALL_PERIPH(SSPIx));

    switch(SSPIx)
    {
        case SSPI00:
        {
            I2C0_TypeDef *SSPI_Instance = &SCI0->I2C0;
            data = SSPI_Instance->SIO;
        }
        break;
        case SSPI01:
        {
            I2C1_TypeDef *SSPI_Instance = &SCI0->I2C1;
            data = SSPI_Instance->SIO;
        }
        break;
        case SSPI10:
        {
            I2C2_TypeDef *SSPI_Instance = &SCI0->I2C2;
            data = SSPI_Instance->SIO;
        }
        break;
        case SSPI11:
        {
            I2C3_TypeDef *SSPI_Instance = &SCI0->I2C3;
            data = SSPI_Instance->SIO;
        }
        break;
        case SSPI20:
        {
            I2C4_TypeDef *SSPI_Instance = &SCI1->I2C4;
            data = SSPI_Instance->SIO;
        }
        break;
        case SSPI21:
        {
            I2C5_TypeDef *SSPI_Instance = &SCI1->I2C5;
            data = SSPI_Instance->SIO;
        }
        break;
#if defined(BAT32G1XX_80PIN) || defined(BAT32G1XX_100PIN)
        case SSPI30:
        {
            I2C6_TypeDef *SSPI_Instance = &SCI2->I2C6;
            data = SSPI_Instance->SIO;
        }
        break;
        case SSPI31:
        {
            I2C7_TypeDef *SSPI_Instance = &SCI2->I2C7;
            data = SSPI_Instance->SIO;
        }
        break;
#endif		
		default:
		break;
    }

    return data;
}

/**
  * @brief  Checks whether the specified UART flag is set or not.
  * @param  reg: UART peripheral error register.
  * @param  UART_FLAG: specifies the flag to check.
  *          This parameter can be one of the following values:
  *            @arg UART_FLAG_TSF: Transmission running status flag
  *            @arg UART_FLAG_BFF: Transmission register save valid data flag
  *            @arg UART_FLAG_FEF: Transmission frame data error flag
  *            @arg UART_FLAG_PEF: UART Parity error flag
  *            @arg UART_FLAG_OVF: Transmission over run error flag
  * @retval The new state of UART_FLAG (SET or RESET).
  */
static FlagStatus SSPI_GetFlagStatus(uint16_t reg, uint16_t SSPI_FLAG)
{
	FlagStatus bitstatus = RESET;
	/* Check the parameters */
	assert_param(IS_SSPI_FLAG(SSPI_FLAG));

	bitstatus = SCI_GetFlagStatus(reg,SSPI_FLAG);
	
	return bitstatus;
}
/**
  * @brief  Transmits one Byte Data through the SSPI. 
  * @param  Data: Data to be transmitted.
  * @retval tmp: Receive the data by transmit.
  */
uint8_t SSPI_TransmitData(SCIAFSelect_TypeDef SSPIx, uint8_t Data)
{
	volatile uint8_t dummy;	
    /* Check the parameters */
    assert_param(IS_SSPI_ALL_PERIPH(SSPIx));

    /* Write in the SIO register the data to be sent */
    switch(SSPIx)
    {
        case SSPI00:
        {
            I2C0_TypeDef *SSPI_Instance = &SCI0->I2C0;
            SSPI_Instance->SIO = Data;
			while (SSPI_GetFlagStatus(SSPI_Instance->SSR, SSPI_FLAG_TSF ));
			dummy = SSPI_Instance->SIO;			
        }
        break;
        case SSPI01:
        {
            I2C1_TypeDef *SSPI_Instance = &SCI0->I2C1;
            SSPI_Instance->SIO = Data;
			while (SSPI_GetFlagStatus(SSPI_Instance->SSR, SSPI_FLAG_TSF ));	
			dummy = SSPI_Instance->SIO;			
        }
        break;
        case SSPI10:
        {
            I2C2_TypeDef *SSPI_Instance = &SCI0->I2C2;
            SSPI_Instance->SIO = Data;
			while (SSPI_GetFlagStatus(SSPI_Instance->SSR, SSPI_FLAG_TSF ));	
			dummy = SSPI_Instance->SIO;			
        }
        break;
        case SSPI11:
        {
            I2C3_TypeDef *SSPI_Instance = &SCI0->I2C3;
            SSPI_Instance->SIO = Data;
			while (SSPI_GetFlagStatus(SSPI_Instance->SSR, SSPI_FLAG_TSF ));	
			dummy = SSPI_Instance->SIO;			
        }
        break;
        case SSPI20:
        {
            I2C4_TypeDef *SSPI_Instance = &SCI1->I2C4;
            SSPI_Instance->SIO = Data;
			while (SSPI_GetFlagStatus(SSPI_Instance->SSR, SSPI_FLAG_TSF ));	
			dummy = SSPI_Instance->SIO;			
        }
        break;
        case SSPI21:
        {
            I2C5_TypeDef *SSPI_Instance = &SCI1->I2C5;
            SSPI_Instance->SIO = Data;
			while (SSPI_GetFlagStatus(SSPI_Instance->SSR, SSPI_FLAG_TSF ));
			dummy = SSPI_Instance->SIO;			
        }
        break;
#if defined(BAT32G1XX_80PIN) || defined(BAT32G1XX_100PIN)
        case SSPI30:
        {
            I2C6_TypeDef *SSPI_Instance = &SCI2->I2C6;
            SSPI_Instance->SIO = Data;
			while (SSPI_GetFlagStatus(SSPI_Instance->SSR, SSPI_FLAG_TSF ));	
			dummy = SSPI_Instance->SIO;			
        }
        break;
        case SSPI31:
        {
            I2C7_TypeDef *SSPI_Instance = &SCI2->I2C7;
            SSPI_Instance->SIO = Data;
			while (SSPI_GetFlagStatus(SSPI_Instance->SSR, SSPI_FLAG_TSF ));
			dummy = SSPI_Instance->SIO;			
        }
        break;
#endif		
		default:
		break;
    }
	return dummy;
}



/**
  * @brief  Checks whether the specified SSPI flag is set or not.
  * @param  SSPIx: where x can be 0, 1, 2, 4, 5 to select the SSPI peripheral.
  * @param  SSPI_FLAG: specifies the flag to check.
  *          This parameter can be one of the following values:
  *          This parameter can be one of the following values:
  *            @arg SSPI_FLAG_TSF: Transmission running status flag
  *            @arg SSPI_FLAG_BFF: Transmission register save valid data flag
  *            @arg SSPI_FLAG_OVF: Transmission over run error flag
  * @retval FlagStatus.
  */
FlagStatus SSPI_GetErrStaus(SCIAFSelect_TypeDef SSPIx, uint16_t SSPI_FLAG)
{
	FlagStatus status;
	
    /* Check the parameters */
	assert_param(IS_SSPI_ALL_PERIPH(SSPIx));
    assert_param(IS_SSPI_FLAG(SSPI_FLAG));

    /* Clear the selected I2C flag */
	status = SCI_GetErrStaus(SSPIx,SSPI_FLAG);
	
	return status;
}

/**
  * @brief  Clear the specified SSPI flag is set.
  * @param  SSPIx: where x can be 0, 1, 2, 4, 5 to select the SSPI peripheral.
  * @param  SSPI_FLAG: specifies the flag to check.
  *          This parameter can be one of the following values:
  *            @arg SSPI_CLEAR_FLAG_OVF: Transmission over run error flag
  * @retval None.
  */
void SSPI_ClearFlag(SCIAFSelect_TypeDef SSPIx, uint16_t SSPI_FLAG)
{
    /* Check the parameters */
	assert_param(IS_SSPI_ALL_PERIPH(SSPIx));
    assert_param(IS_SSPI_FLAG(SSPI_FLAG));

    /* Clear the selected I2C flag */
	SCI_ClearFlag(SSPIx,SSPI_FLAG);
}


