/**
  ******************************************************************************
  * @file    spi.c
  * @author  CMS Application Team
  * @version Vx.x.x
  * @date    24-April-2022
  * @brief   This file provides firmware functions to manage the following 
  *          functionalities of the Serial Peripheral interface (SPI):           
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
#include "spi.h"
#include "cgc.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/** @defgroup SPI_Private_Functions
  * @{
  */
NSS_FUNC_T NSS;
/**
  * @brief  De-initialize the SPI peripheral registers to their default reset values.
  * @note   None
  * @retval None
  */
void SPI_DeInit(void)
{
	CGC_PER1PeriphClockCmd(CGC_PER1Periph_SPI, DISABLE);
}

/**
  * @brief  Initializes the SPIx peripheral according to the specified 
  *         parameters in the SPI_InitStruct.
  * @param  SPI_InitStruct: pointer to a SPI_InitTypeDef structure that
  *         contains the configuration information for the specified SPI peripheral.
  * @retval None
  */
void SPI_Init(SPI_InitTypeDef* SPI_InitStruct)
{
	uint32_t tmpreg = 0;

	/* Check the SPI parameters */
	assert_param(IS_SPI_MODE(SPI_InitStruct->SPI_Mode));
	assert_param(IS_SPI_DATASIZE(SPI_InitStruct->SPI_DataSize));
	assert_param(IS_SPI_CPOL(SPI_InitStruct->SPI_CPOL));
	assert_param(IS_SPI_CPHA(SPI_InitStruct->SPI_CPHA));
	assert_param(IS_SPI_NSS(SPI_InitStruct->SPI_NSS));
	assert_param(IS_SPI_BAUDRATE_PRESCALER(SPI_InitStruct->SPI_BaudRatePrescaler));
	assert_param(IS_SPI_FIRST_BIT(SPI_InitStruct->SPI_FirstBit));

	/*---------------------------- SPI Clock gate control enable -----------------*/
	CGC_PER1PeriphClockCmd(CGC_PER1Periph_SPI, ENABLE);

	/*---------------------------- SPI SPIM flag clear ---------------------------*/
	SPI->SPIM &= ~(SPI_FLAG_RXNE);
	SPI->SPIM &= ~(SPI_FLAG_RUNNING);

	/*---------------------------- SPI SPIC Configuration ------------------------*/
	/* Get the SPIC value */
	tmpreg = SPI->SPIC;

	/* The SPI Master or Slave mode selected by clock setting */
	if (SPI_Mode_Master == SPI_InitStruct->SPI_Mode)
	{
		tmpreg |= SPI_InitStruct->SPI_BaudRatePrescaler;
	}
	else
	{
		/* When SPI mode is slave, select the ext clock */
		tmpreg |= SPI_BaudRatePrescaler_Ext;
	}

	/* SPI_Clock_Polarity setting */
	tmpreg |= SPI_InitStruct->SPI_CPOL;

	/* SPI_Clock_Phase setting */
	tmpreg |= SPI_InitStruct->SPI_CPHA;

	/* Set the SPIC value */
	SPI->SPIC = tmpreg;

	/*---------------------------- SPI SPIM Configuration ------------------------*/
	/* Get the SPIM value */
	tmpreg = SPI->SPIM;

	tmpreg |= SPI_InitStruct->SPI_DataSize;
	tmpreg |= SPI_InitStruct->SPI_NSS;
	tmpreg |= SPI_InitStruct->SPI_FirstBit;

	/* Set the SPIM value */
	SPI->SPIM = tmpreg;
}

/**
  * @brief  Enables or disables the specified SPI peripheral.
  * @param  NewState: new state of the SPI peripheral. 
  *          This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
void SPI_Cmd(FunctionalState NewState)
{
	/* Check the parameters */
	assert_param(IS_FUNCTIONAL_STATE(NewState));
	if (NewState != DISABLE)
	{
		/* Enable the selected SPI peripheral */
		SPI->SPIM |= (uint32_t)SPI_SPIM_SPIE_Msk;
		if(NSS.Active)
		{
			NSS.Active();
		}
	}
	else
	{
		/* Disable the selected SPI peripheral */
		SPI->SPIM &= (uint32_t)~((uint32_t)SPI_SPIM_SPIE_Msk);
		if(NSS.Inactive)
		{
			NSS.Inactive();	
		}
	}
}

/**
  * @brief  Configures the data size for the selected SPI.
  * @param  SPI_DataSize: specifies the SPI data size.
  *          This parameter can be one of the following values:
  *            @arg SPI_DataSize_16b: Set data frame format to 16bit
  *            @arg SPI_DataSize_8b: Set data frame format to 8bit
  * @retval None
  */
void SPI_DataSizeConfig(uint8_t SPI_DataSize)
{
	/* Check the parameters */
	assert_param(IS_SPI_DATASIZE(SPI_DataSize));
	/* Clear DLS bit */
	SPI->SPIM &= (uint32_t)~((uint32_t)SPI_SPIM_DLS_Msk);
	/* Set new DLS bit value */
	SPI->SPIM |= (uint32_t)SPI_DataSize;
}

/**
  * @brief  Selects the data transfer direction in bidirectional mode for the specified SPI.
  * @param  SPI_Direction: specifies the data transfer direction in bidirectional mode. 
  *          This parameter can be one of the following values:
  *            @arg SPI_Direction_Tx: Selects Tx transmission direction
  *            @arg SPI_Direction_Rx: Selects Rx receive direction
  * @retval None
  */
void SPI_BiDirectionalLineConfig(uint8_t SPI_Direction)
{
	/* Check the parameters */
	assert_param(IS_SPI_DIRECTION(SPI_Direction));
	if (SPI_Direction == SPI_Direction_TxRx)
	{
		/* Set the Tx/Rx mode */
		SPI->SPIM |= (uint32_t)SPI_Direction_TxRx;
	}
	else
	{
		/* Set the Rx only mode */
		SPI->SPIM &= (uint32_t)SPI_Direction_Rx;
	}
}

/**
  * @}
  */

/** @defgroup SPI_Group2 Data transfers functions
 *  @brief   Data transfers functions
 *
@verbatim   
 ===============================================================================
                      ##### Data transfers functions #####
 ===============================================================================  

 [..] This section provides a set of functions allowing to manage the SPI data 
      transfers. In reception, data are received and then stored into an internal 
      Rx buffer while. In transmission, data are first stored into an internal Tx 
      buffer before being transmitted.

 [..] The read access of the SPI_SDRI register can be done using the SPI_I2S_ReceiveData()
      function and returns the Rx buffered value. Whereas a write access to the SPI_SDRO 
      can be done using SPI_I2S_SendData() function and stores the written data into 
      Tx buffer.

@endverbatim
  * @{
  */

/**
  * @brief  Returns the most recent received data by the SPI peripheral. 
  * @retval The value of the received data.
  */
uint16_t SPI_ReceiveData(void)
{
	/* Return the data in the DR register */
	return (uint16_t)SPI->SDRI;
}

/**
  * @brief  Transmits a Data through the SPI peripheral. 
  * @param  Data: Data to be transmitted.
  * @retval None
  */
void SPI_SendData(uint16_t Data)
{
	/* Write in the DR register the data to be sent */
	SPI->SDRO = Data;
}

/**
  * @brief  Transmits one Byte Data through the SPI peripheral. 
  * @param  Data: Data to be transmitted.
  * @retval tmp: Receive the data by transmit.
  */
uint8_t SPI_TransmitByte(uint8_t Data)
{
	uint8_t tmp;
	
	/* Write in the DR register the data to be sent */
	SPI->SDRO = Data;
	
	/* Wait the Byte data transmit completed */
	while((SPI->SPIM & ((uint16_t)SPI_SPIM_SPTF_Msk)) != RESET);
	
	/* Receive Byte data */
	tmp = (uint8_t)SPI_ReceiveData();
	
	return tmp;
}


/**
  * @brief  Transmits one Byte Data through the SPI peripheral. 
  * @param  Data: Data to be transmitted.
  * @retval tmp: Receive the data by transmit.
  */
uint8_t SPI_ReceiveByte(void)
{
	uint8_t tmp;
	
	/* Write in the DR register the data to be sent */
	SPI->SDRO = 0xFF;
	
	/* Wait the Byte data transmit completed */
	while((SPI->SPIM & ((uint16_t)SPI_SPIM_SPTF_Msk)) != SET);
	
	/* Receive Byte data */
	tmp = (uint8_t)SPI_ReceiveData();
	
	return tmp;
}

/**
  * @brief  Transmits one Word Data through the SPI peripheral. 
  * @param  Data: Data to be transmitted.
  * @retval tmp: Receive the data by transmit.
  */
uint16_t SPI_TransmitWord(uint16_t Data)
{
	uint16_t tmp;
	
	/* Write in the DR register the data to be sent */
	SPI->SDRO = Data;
	
	/* Wait the Byte data transmit completed */
	while((SPI->SPIM & ((uint16_t)SPI_SPIM_SPTF_Msk)) != RESET);
	
	/* Receive Byte data */
	tmp = (uint16_t)SPI_ReceiveData();
	
	return tmp;
}

/**
  * @brief  Transmits one Word Data through the SPI peripheral. 
  * @param  Data: Data to be transmitted.
  * @retval tmp: Receive the data by transmit.
  */
uint16_t SPI_ReceiveWord(void)
{
	uint16_t tmp;
	
	/* Write in the DR register the data to be sent */
	SPI->SDRO = 0xFFFF;
	
	/* Wait the Byte data transmit completed */
	while((SPI->SPIM & ((uint16_t)SPI_SPIM_SPTF_Msk)) != SET);
	
	/* Receive Byte data */
	tmp = (uint16_t)SPI_ReceiveData();
	
	return tmp;
}

/**
  * @brief  Checks whether the specified SPI flag is set or not.
  * @param  SPI_FLAG: specifies the SPI flag to check. 
  *          This parameter can be one of the following values:
  *            @arg SPI_FLAG_RXNE: Receive buffer not empty flag.
  * 		   @arg SPI_FLAG_RUNNING: Communication status flag.
  * @retval The new state of SPI_FLAG (SET or RESET).
  */
FlagStatus SPI_GetFlagStatus(uint8_t SPI_FLAG)
{
	FlagStatus bitstatus = RESET;
	/* Check the parameters */
	assert_param(IS_SPI_GET_FLAG(SPI_FLAG));

	/* Check the status of the specified SPI flag */
	if ((SPI->SPIM & SPI_FLAG) != (uint16_t)RESET)
	{
		/* SPI_I2S_FLAG is set */
		bitstatus = SET;
	}
	else
	{
		/* SPI_I2S_FLAG is reset */
		bitstatus = RESET;
	}
	/* Return the SPI_I2S_FLAG status */
	return  bitstatus;
}
