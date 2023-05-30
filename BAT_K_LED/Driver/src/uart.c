/**
  ******************************************************************************
  * @file    uart.c
  * @author  CMS Application Team
  * @version Vx.x.x
  * @date    24-April-2022
  * @brief   This file provides firmware functions to manage the following 
  *          functionalities of the Universal asynchronous receiver
  *          transmitter (UART):           
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
#include "uart.h"
#include "cgc.h"


/** @addtogroup bat32g135_StdPeriph_Driver
  * @{
  */

/** @defgroup UART 
  * @brief UART driver modules
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Get the system clock in order to calculate UART peripheral baud rate.
  * @param  None
  * @retval None
  */
static uint32_t UART_GetSysClock(void)
{
	if (USE_HSE_SYSTYEM_CLOCK == SYSTYEM_CLOCK_OPEN)	
	{
		return 8000000;
	}
	else
	{
		return SystemCoreClock;
	}

}

/**
  * @brief  Deinitializes the UARTx peripheral registers to their default reset values.
  * @param  UARTx: where x can be 0, 1, 2 to select the UART peripheral.
  * @retval None
  */
void UART_DeInit(SCIAFSelect_TypeDef UARTx)
{
	int8_t ret = SCI_SUCCESS;
	/* Check the parameters */
	assert_param(IS_UART_ALL_PERIPH(UARTx));
	if(UARTx == UART0)
	{
		ret = SCIChannel_Alloca(UART1, UART_Mode_Rx | UART_Mode_Tx);
		if(ret == SCI_SUCCESS)
		{
			CGC_PER0PeriphClockCmd(UARTx >> 10, DISABLE);
			SCIChannel_Free(UART1, UART_Mode_Rx | UART_Mode_Tx);
		}
	}
	if(UARTx == UART1)
	{
		ret = SCIChannel_Alloca(UART0, UART_Mode_Rx | UART_Mode_Tx);
		if(ret == SCI_SUCCESS)
		{
			CGC_PER0PeriphClockCmd(UARTx >> 10, DISABLE);
			SCIChannel_Free(UART0, UART_Mode_Rx | UART_Mode_Tx);
		}
	}	
	SCIChannel_Free(UARTx, UART_Mode_Rx | UART_Mode_Tx);
}

/**
  * @brief  The specified UART0 peripheral initialization and set baud rate by UART_BAUD_t.
  * @param  UART_Instance: UART peripheral register base address.
  * @param  UART_InitStruct: pointer to a UART_InitTypeDef structure that contains
  *         the configuration information for the specified UART peripheral.
  * @param  pValue: UART baud rate setting option data structure.
  * @retval None
  */
static int UART0_Init(UART0_TypeDef *UART_Instance, UART_InitTypeDef* UART_InitStruct, SCIPeriph_Clock_TypeDef *pvalue)
{
	uint16_t tmpreg = 0x00;
	uint16_t phase_reg = UART0_PHASE_POS;
	uint16_t clk_reg = SCI_CLOCK_SELECT_CK00;

	assert_param(IS_UART_WORD_LENGTH(UART_InitStruct->UART_WordLength));
	assert_param(IS_UART_STOPBITS(UART_InitStruct->UART_StopBits));
	assert_param(IS_UART_PARITY(UART_InitStruct->UART_Parity));
	assert_param(IS_UART_MODE(UART_InitStruct->UART_Mode));
	
	/* UART transmit mode setting and serial special option setting for register operate */
	if((UART_InitStruct->UART_Mode & UART_Mode_Tx) == UART_Mode_Tx)
	{
		/* clear all trigger flag for UART transmit */
		{
			tmpreg = UART_Instance->TSIR;
			tmpreg |= SCI_SIRMN_FECTMN | SCI_SIRMN_PECTMN | SCI_SIRMN_OVCTMN;
			UART_Instance->TSIR = tmpreg;
		}

		/* SMR register setting for transmit : UART mode and clock source select fmck -> CK00 */
		{
			tmpreg = UART_Instance->TSMR;
			tmpreg |= SCI_SMRMN_DEFAULT_VALUE;
			tmpreg |= (UART_InitStruct->UART_Mode & 0x0FU);
			tmpreg |= clk_reg | SCI_TRANSFER_END;
			UART_Instance->TSMR = tmpreg;
		}

		/* SCR register setting for transmit : transmit mode and set serial option */
		{
			tmpreg = UART_Instance->TSCR;
			tmpreg |= SCI_TRANSMISSION;
			tmpreg |= SCI_SCRMN_DEFAULT_VALUE;
			tmpreg |= UART_InitStruct->UART_Parity;
			tmpreg |= UART_InitStruct->bitorder;
			tmpreg |= UART_InitStruct->UART_StopBits;
			tmpreg |= UART_InitStruct->UART_WordLength;
			UART_Instance->TSCR = tmpreg;
		}

		/* SDR register setting for transmit : select the division for fmck */
		{
			tmpreg = UART_Instance->TSDR;
			tmpreg |= pvalue->sdr;
			UART_Instance->TSDR = tmpreg;
		}
	}

	/* UART receive mode setting and serial special option setting for register operate */
	if((UART_InitStruct->UART_Mode & UART_Mode_Rx) == UART_Mode_Rx )
	{
		/* clear all trigger flag for UART receive */
		{
			tmpreg = UART_Instance->RSIR;
			tmpreg |= SCI_SIRMN_FECTMN | SCI_SIRMN_PECTMN | SCI_SIRMN_OVCTMN;
			UART_Instance->RSIR = tmpreg;
		}

		/* UART receive Noise filter enable */
		MISC->NFEN0 |= SCI_RXD0_FILTER_ON;

		/* SMR register setting for receive : UART mode and clock source select fmck -> CK00 */
		{
			tmpreg = UART_Instance->RSMR;
			tmpreg |= SCI_SMRMN_DEFAULT_VALUE;
			tmpreg |= clk_reg;
			tmpreg |= SCI_CLOCK_MODE_CKS;
			tmpreg |= SCI_TRIGGER_RXD;
			tmpreg |= UART_InitStruct->phase;
			tmpreg |= SCI_MODE_UART;
			tmpreg |= SCI_TRANSFER_END;
			UART_Instance->RSMR = tmpreg;
		}

		/* SCR register setting for receive : receive mode and set serial option */
		{
			tmpreg = UART_Instance->RSCR;
			tmpreg |= SCI_RECEPTION;
			tmpreg |= SCI_SCRMN_DEFAULT_VALUE;
			tmpreg |= UART_InitStruct->UART_Parity;
			tmpreg |= UART_InitStruct->bitorder;
			tmpreg |= UART_InitStruct->phase;
			tmpreg |= UART_StopBits_1;
			tmpreg |= UART_InitStruct->UART_WordLength;
			UART_Instance->RSCR = tmpreg;
		}

		/* SDR register setting for receive : select the division for fmck */
		{
			tmpreg = UART_Instance->RSDR;
			tmpreg |= pvalue->sdr;
			UART_Instance->RSDR = tmpreg;
		}
		
		/* SOL register setting for receive : phase output select normal or opposite */
		tmpreg = UART_Instance->SOL;
		if(UART_InitStruct->phase == UART_Phase_Normal)
			tmpreg &= ~ phase_reg;
		else
			tmpreg |= phase_reg;
		
		UART_Instance->SOL = tmpreg;
	}

	/* Serial channel stop register ST: channel stop in order to setting some register */
	if((UART_InitStruct->UART_Mode & UART_Mode_Rx) == UART_Mode_Rx)
		UART_Instance->ST |= (UART_CTRL_POS << 1);
	if((UART_InitStruct->UART_Mode & UART_Mode_Tx) == UART_Mode_Tx)
		UART_Instance->ST |= (UART_CTRL_POS);

	/* SCI0 unit clock division select */
	UART_Instance->SPS &= ~SCI0_SPS0_PRS00_Msk;
	UART_Instance->SPS = pvalue->sps;
	if((UART_InitStruct->UART_Mode & UART_Mode_Rx) == UART_Mode_Rx)
	{			
		UART_Instance->SS |= (UART_CTRL_POS << 1);
	}
	if((UART_InitStruct->UART_Mode & UART_Mode_Tx) == UART_Mode_Tx)
	{
		UART_Instance->SO |= UART_CTRL_POS;
		UART_Instance->SOE|= UART_CTRL_POS;
		UART_Instance->SS |= UART_CTRL_POS;
	}		

	return SCI_SUCCESS;
}

/**
  * @brief  The specified UART1 peripheral initialization and set baud rate by UART_BAUD_t.
  * @param  UART_Instance: UART peripheral register base address.
  * @param  UART_InitStruct: pointer to a UART_InitTypeDef structure that contains
  *         the configuration information for the specified UART peripheral.
  * @param  pValue: UART baud rate setting option data structure.
  * @retval None
  */
static int UART1_Init(UART1_TypeDef *UART_Instance, UART_InitTypeDef* UART_InitStruct, SCIPeriph_Clock_TypeDef *pValue)
{
	uint16_t tmpreg = 0x00;
	uint16_t phase_reg = UART1_PHASE_POS;
	uint16_t clk_reg = SCI_CLOCK_SELECT_CK01;

	assert_param(IS_UART_WORD_LENGTH(UART_InitStruct->UART_WordLength));
	assert_param(IS_UART_STOPBITS(UART_InitStruct->UART_StopBits));
	assert_param(IS_UART_PARITY(UART_InitStruct->UART_Parity));
	assert_param(IS_UART_MODE(UART_InitStruct->UART_Mode));
	
	/* UART transmit mode setting and serial special option setting for register operate */
	if((UART_InitStruct->UART_Mode & UART_Mode_Tx) == UART_Mode_Tx)
	{
		/* clear all trigger flag for UART transmit */
		{
			tmpreg = UART_Instance->TSIR;
			tmpreg |= SCI_SIRMN_FECTMN | SCI_SIRMN_PECTMN | SCI_SIRMN_OVCTMN;
			UART_Instance->TSIR = tmpreg;
		}

		/* SMR register setting for transmit : UART mode and clock source select fmck -> CK00 */
		{
			tmpreg = UART_Instance->TSMR;
			tmpreg |= SCI_SMRMN_DEFAULT_VALUE;
			tmpreg |= (UART_InitStruct->UART_Mode & 0x0FU);
			tmpreg |= clk_reg | SCI_TRANSFER_END;
			UART_Instance->TSMR = tmpreg;
		}

		/* SCR register setting for transmit : transmit mode and set serial option */
		{
			tmpreg = UART_Instance->TSCR;
			tmpreg |= SCI_TRANSMISSION;
			tmpreg |= SCI_SCRMN_DEFAULT_VALUE;
			tmpreg |= UART_InitStruct->UART_Parity;
			tmpreg |= UART_InitStruct->bitorder;
			tmpreg |= UART_InitStruct->UART_StopBits;
			tmpreg |= UART_InitStruct->UART_WordLength;
			UART_Instance->TSCR = tmpreg;
		}

		/* SDR register setting for transmit : select the division for fmck */
		{
			tmpreg = UART_Instance->TSDR;
			tmpreg |= pValue->sdr;
			UART_Instance->TSDR = tmpreg;
		}
	}

	/* UART receive mode setting and serial special option setting for register operate */
	if((UART_InitStruct->UART_Mode & UART_Mode_Rx) == UART_Mode_Rx )
	{
		/* clear all trigger flag for UART receive */
		{
			tmpreg = UART_Instance->RSIR;
			tmpreg |= SCI_SIRMN_FECTMN | SCI_SIRMN_PECTMN | SCI_SIRMN_OVCTMN;
			UART_Instance->RSIR = tmpreg;
		}

		/* UART receive Noise filter enable */
		MISC->NFEN0 |= SCI_RXD1_FILTER_ON;

		/* SMR register setting for receive : UART mode and clock source select fmck -> CK00 */
		{
			tmpreg = UART_Instance->RSMR;
			tmpreg |= SCI_SMRMN_DEFAULT_VALUE;
			tmpreg |= clk_reg;
			tmpreg |= SCI_CLOCK_MODE_CKS;
			tmpreg |= SCI_TRIGGER_RXD;
			tmpreg |= UART_InitStruct->phase;
			tmpreg |= SCI_MODE_UART;
			tmpreg |= SCI_TRANSFER_END;
			UART_Instance->RSMR = tmpreg;
		}

		/* SCR register setting for receive : receive mode and set serial option */
		{
			tmpreg = UART_Instance->RSCR;
			tmpreg |= SCI_RECEPTION;
			tmpreg |= SCI_SCRMN_DEFAULT_VALUE;
			tmpreg |= UART_InitStruct->UART_Parity;
			tmpreg |= UART_InitStruct->bitorder;
			tmpreg |= UART_InitStruct->phase;
			tmpreg |= UART_StopBits_1;
			tmpreg |= UART_InitStruct->UART_WordLength;

			UART_Instance->RSCR = tmpreg;
		}

		/* SDR register setting for receive : select the division for fmck */
		{
			tmpreg = UART_Instance->RSDR;
			tmpreg |= pValue->sdr;
			UART_Instance->RSDR = tmpreg;
		}
		
		/* SOL register setting for receive : phase output select normal or opposite */
		tmpreg = UART_Instance->SOL;
		if(UART_InitStruct->phase == UART_Phase_Normal)
			tmpreg &= ~ phase_reg;
		else
			tmpreg |= phase_reg;
		
		UART_Instance->SOL = tmpreg;;
	}

	/* Serial channel stop register ST: channel stop in order to setting some register */
	if((UART_InitStruct->UART_Mode & UART_Mode_Rx) == UART_Mode_Rx)
		UART_Instance->ST |= (UART_CTRL_POS << 1);
	if((UART_InitStruct->UART_Mode & UART_Mode_Tx) == UART_Mode_Tx)
		UART_Instance->ST |= (UART_CTRL_POS);

	/* SCI0 unit clock division select */
	UART_Instance->SPS &= ~SCI0_SPS0_PRS00_Msk;
	UART_Instance->SPS = pValue->sps;
	if((UART_InitStruct->UART_Mode & UART_Mode_Rx) == UART_Mode_Rx)
	{			
		UART_Instance->SS |= (UART_CTRL_POS << 1);
	}
	if((UART_InitStruct->UART_Mode & UART_Mode_Tx) == UART_Mode_Tx)
	{
		UART_Instance->SO |= UART_CTRL_POS;
		UART_Instance->SOE|= UART_CTRL_POS;
		UART_Instance->SS |= UART_CTRL_POS;
	}		

	return SCI_SUCCESS;
}

/**
  * @brief  The specified UART2 peripheral initialization and set baud rate by UART_BAUD_t.
  * @param  UART_Instance: UART peripheral register base address.
  * @param  UART_InitStruct: pointer to a UART_InitTypeDef structure that contains
  *         the configuration information for the specified UART peripheral.
  * @param  pValue: UART baud rate setting option data structure.
  * @retval None
  */
static int UART2_Init(UART2_TypeDef *UART_Instance, UART_InitTypeDef* UART_InitStruct, SCIPeriph_Clock_TypeDef *pValue)
{
	uint16_t tmpreg = 0x00;
	uint16_t phase_reg = UART2_PHASE_POS;
	uint16_t clk_reg = SCI_CLOCK_SELECT_CK00;

	assert_param(IS_UART_WORD_LENGTH(UART_InitStruct->UART_WordLength));
	assert_param(IS_UART_STOPBITS(UART_InitStruct->UART_StopBits));
	assert_param(IS_UART_PARITY(UART_InitStruct->UART_Parity));
	assert_param(IS_UART_MODE(UART_InitStruct->UART_Mode));
	
	/* UART transmit mode setting and serial special option setting for register operate */
	if((UART_InitStruct->UART_Mode & UART_Mode_Tx) == UART_Mode_Tx)
	{
		/* clear all trigger flag for UART transmit */
		{
			tmpreg = UART_Instance->TSIR;
			tmpreg |= SCI_SIRMN_FECTMN | SCI_SIRMN_PECTMN | SCI_SIRMN_OVCTMN;
			UART_Instance->TSIR = tmpreg;
		}

		/* SMR register setting for transmit : UART mode and clock source select fmck -> CK00 */
		{
			tmpreg = UART_Instance->TSMR;
			tmpreg |= SCI_SMRMN_DEFAULT_VALUE;
			tmpreg |= (UART_InitStruct->UART_Mode & 0x0FU);
			tmpreg |= clk_reg | SCI_TRANSFER_END;
			UART_Instance->TSMR = tmpreg;
		}

		/* SCR register setting for transmit : transmit mode and set serial option */
		{
			tmpreg = UART_Instance->TSCR;
			tmpreg |= SCI_TRANSMISSION;
			tmpreg |= SCI_SCRMN_DEFAULT_VALUE;
			tmpreg |= UART_InitStruct->UART_Parity;
			tmpreg |= UART_InitStruct->bitorder;
			tmpreg |= UART_InitStruct->UART_StopBits;
			tmpreg |= UART_InitStruct->UART_WordLength;
			UART_Instance->TSCR = tmpreg;
		}

		/* SDR register setting for transmit : select the division for fmck */
		{
			tmpreg = UART_Instance->TSDR;
			tmpreg |= pValue->sdr;
			UART_Instance->TSDR = tmpreg;
		}
	}

	/* UART receive mode setting and serial special option setting for register operate */
	if((UART_InitStruct->UART_Mode & UART_Mode_Rx) == UART_Mode_Rx )
	{
		/* clear all trigger flag for UART receive */
		{
			tmpreg = UART_Instance->RSIR;
			tmpreg |= SCI_SIRMN_FECTMN | SCI_SIRMN_PECTMN | SCI_SIRMN_OVCTMN;
			UART_Instance->RSIR = tmpreg;
		}

		/* UART receive Noise filter enable */
		MISC->NFEN0 |= SCI_RXD2_FILTER_ON;

		/* SMR register setting for receive : UART mode and clock source select fmck -> CK00 */
		{
			tmpreg = UART_Instance->RSMR;
			tmpreg |= SCI_SMRMN_DEFAULT_VALUE;
			tmpreg |= clk_reg;
			tmpreg |= SCI_CLOCK_MODE_CKS;
			tmpreg |= SCI_TRIGGER_RXD;
			tmpreg |= UART_InitStruct->phase;
			tmpreg |= SCI_MODE_UART;
			tmpreg |= SCI_TRANSFER_END;
			UART_Instance->RSMR = tmpreg;
		}

		/* SCR register setting for receive : receive mode and set serial option */
		{
			tmpreg = UART_Instance->RSCR;
			tmpreg |= SCI_RECEPTION;
			tmpreg |= SCI_SCRMN_DEFAULT_VALUE;
			tmpreg |= UART_InitStruct->UART_Parity;
			tmpreg |= UART_InitStruct->bitorder;
			tmpreg |= UART_InitStruct->phase;
			tmpreg |= UART_StopBits_1;
			tmpreg |= UART_InitStruct->UART_WordLength;
			UART_Instance->RSCR = tmpreg;
		}

		/* SDR register setting for receive : select the division for fmck */
		{
			tmpreg = UART_Instance->RSDR;
			tmpreg |= pValue->sdr;
			UART_Instance->RSDR = tmpreg;
		}
		
		/* SOL register setting for receive : phase output select normal or opposite */
		tmpreg = UART_Instance->SOL;
		if(UART_InitStruct->phase == UART_Phase_Normal)
			tmpreg &= ~ phase_reg;
		else
			tmpreg |= phase_reg;
		
		UART_Instance->SOL = tmpreg;;
	}

	/* Serial channel stop register ST: channel stop in order to setting some register */
	if((UART_InitStruct->UART_Mode & UART_Mode_Rx) == UART_Mode_Rx)
		UART_Instance->ST |= (UART_CTRL_POS << 1);
	if((UART_InitStruct->UART_Mode & UART_Mode_Tx) == UART_Mode_Tx)
		UART_Instance->ST |= (UART_CTRL_POS);

	/* SCI0 unit clock division select */
	UART_Instance->SPS &= ~SCI0_SPS0_PRS00_Msk;
	UART_Instance->SPS = pValue->sps;
	if((UART_InitStruct->UART_Mode & UART_Mode_Rx) == UART_Mode_Rx)
	{			
		UART_Instance->SS |= (UART_CTRL_POS << 1);
	}
	if((UART_InitStruct->UART_Mode & UART_Mode_Tx) == UART_Mode_Tx)
	{
		UART_Instance->SO |= UART_CTRL_POS;
		UART_Instance->SOE|= UART_CTRL_POS;
		UART_Instance->SS |= UART_CTRL_POS;
	}		

	return SCI_SUCCESS;
}

/**
  * @brief  Initializes the UARTx peripheral according to the specified
  *         parameters in the UART_InitStruct .
  * @param  UARTx: where x can be 0, 1, 2 to select the UART peripheral.
  * @param  UART_InitStruct: pointer to a UART_InitTypeDef structure that contains
  *         the configuration information for the specified UART peripheral.
  * @retval initial result
  */
int8_t UART_Init(SCIAFSelect_TypeDef UARTx, UART_InitTypeDef* UART_InitStruct)
{
	int8_t ret = SCI_SUCCESS;
	float max_error;
    SCIPeriph_Clock_TypeDef pValue;
	
	ret = SCIChannel_Alloca(UARTx, UART_InitStruct->UART_Mode);
	if(ret != SCI_SUCCESS)
		return ret;
 

	switch(UARTx)
	{
		case UART0:
		{
			/* Clock gate control register setting to enable SCI0 clock */
			CGC_PER0PeriphClockCmd((uint8_t)((UARTx >> 10)+SCI_UINT_OFFSET), ENABLE);

			UART0_TypeDef *UART_Instance = &SCI0->UART0;
			max_error = SCIPeriphal_ClockUpdate(UARTx, UART_GetSysClock(), 
									UART_InitStruct->UART_BaudRate, 
									UART_Instance->SPS,
									&pValue);
			if(max_error > 1)
			{
				return SCI_ERR_BAUDRATE;						
			}
			UART0_Init(UART_Instance, UART_InitStruct, &pValue);
		}
		break;

		case UART1:
		{
			/* Clock gate control register setting to enable SCI0 clock */
			CGC_PER0PeriphClockCmd((uint8_t)((UARTx >> 10)+SCI_UINT_OFFSET), ENABLE);

			UART1_TypeDef *UART_Instance = &SCI0->UART1;
			max_error = SCIPeriphal_ClockUpdate(UARTx, UART_GetSysClock(), 
									UART_InitStruct->UART_BaudRate, 
									UART_Instance->SPS,
									&pValue);
			if(max_error > 1)
			{
				return SCI_ERR_BAUDRATE;						
			}		
			UART1_Init(UART_Instance, UART_InitStruct, &pValue);
		}
		break;

		case UART2:
		{
			/* Clock gate control register setting to enable SCI0 clock */
			CGC_PER0PeriphClockCmd((uint8_t)((UARTx >> 10)+SCI_UINT_OFFSET), ENABLE);

			UART2_TypeDef *UART_Instance = &SCI1->UART2;
			max_error = SCIPeriphal_ClockUpdate(UARTx, UART_GetSysClock(), 
									UART_InitStruct->UART_BaudRate, 
									UART_Instance->SPS,
									&pValue);
			if(max_error > 1)
			{
				return SCI_ERR_BAUDRATE;						
			}
			UART2_Init(UART_Instance, UART_InitStruct, &pValue);
		}
		break;
#if defined(BAT32G1XX_80PIN) || defined(BAT32G1XX_100PIN)
		case UART3:
		{
			/* Clock gate control register setting to enable SCI0 clock */
			CGC_PER2PeriphClockCmd(UARTx  >> 10, ENABLE);

			UART3_TypeDef *UART_Instance = &SCI2->UART3;
			max_error = SCIPeriphal_ClockUpdate(UARTx, UART_GetSysClock(), 
									UART_InitStruct->UART_BaudRate, 
									UART_Instance->SPS,
									&pValue);
			if(max_error > 1)
			{
				return SCI_ERR_BAUDRATE;						
			}
			UART2_Init(UART_Instance, UART_InitStruct, &pValue);
		}
		break;	
#endif		
		default:
			break;
	}

	return ret;
}

/**
  * @brief  Enables the specified UART peripheral.
  * @param  UARTx: where x can be 0, 1, 2, to select UART peripheral.
  * @param  uart_mode: it can be as follows.
  *            @arg UART_Mode_Rx: uart receive mode 
  *            @arg UART_Mode_Tx: uart transimission mode 
  *            @arg UART_Mode_Tx|UART_Mode_Rx: uart receive and transimission mode 
  * @retval None
  */
void UART_Start(SCIAFSelect_TypeDef UARTx,uint16_t uart_mode)
{
	if (UARTx == UART0)
	{
		UART0_TypeDef *UART_Instance = &SCI0->UART0;
		if(uart_mode | UART_Mode_Tx)
		{
			UART_Instance->SO |= UART_CTRL_POS;
			UART_Instance->SOE|= UART_CTRL_POS;
			UART_Instance->SS |= UART_CTRL_POS;
		}
		if(uart_mode | UART_Mode_Rx)
		{
			UART_Instance->SO |= (UART_CTRL_POS <<1);
			UART_Instance->SOE|= (UART_CTRL_POS <<1);
			UART_Instance->SS |= (UART_CTRL_POS <<1);		
		}
	}
	else if (UARTx == UART1)
	{
		UART1_TypeDef *UART_Instance = &SCI0->UART1;
		if(uart_mode | UART_Mode_Tx)
		{
			UART_Instance->SO |= UART_CTRL_POS;
			UART_Instance->SOE|= UART_CTRL_POS;
			UART_Instance->SS |= UART_CTRL_POS;
		}
		if(uart_mode | UART_Mode_Rx)
		{
			UART_Instance->SO |= (UART_CTRL_POS <<1);
			UART_Instance->SOE|= (UART_CTRL_POS <<1);
			UART_Instance->SS |= (UART_CTRL_POS <<1);		
		}
	}
	else if (UARTx == UART2)
	{
		UART2_TypeDef *UART_Instance = &SCI1->UART2;
		if(uart_mode | UART_Mode_Tx)
		{
			UART_Instance->SO |= UART_CTRL_POS;
			UART_Instance->SOE|= UART_CTRL_POS;
			UART_Instance->SS |= UART_CTRL_POS;
		}
		if(uart_mode | UART_Mode_Rx)
		{
			UART_Instance->SO |= (UART_CTRL_POS <<1);
			UART_Instance->SOE|= (UART_CTRL_POS <<1);
			UART_Instance->SS |= (UART_CTRL_POS <<1);		
		}
	}
#if defined(BAT32G1XX_80PIN) || defined(BAT32G1XX_100PIN)
	else if (UARTx == UART3)
	{
		UART3_TypeDef *UART_Instance = &SCI2->UART3;
		if(uart_mode | UART_Mode_Tx)
		{
			UART_Instance->SO |= UART_CTRL_POS;
			UART_Instance->SOE|= UART_CTRL_POS;
			UART_Instance->SS |= UART_CTRL_POS;
		}
		if(uart_mode | UART_Mode_Rx)
		{
			UART_Instance->SO |= (UART_CTRL_POS <<1);
			UART_Instance->SOE|= (UART_CTRL_POS <<1);
			UART_Instance->SS |= (UART_CTRL_POS <<1);		
		}
	}
#endif
}

/**
  * @brief  Disables the specified UART peripheral.
  * @param  UARTx: where x can be 0, 1, 2, to select UART peripheral.
  * @param  uart_mode: it can be as follows.
  *            @arg UART_Mode_Rx: uart receive mode 
  *            @arg UART_Mode_Tx: uart transimission mode 
  *            @arg UART_Mode_Tx|UART_Mode_Rx: uart receive and transimission mode 

  * @retval None
  */
void UART_Stop(SCIAFSelect_TypeDef UARTx,uint16_t uart_mode)
{
	if (UARTx == UART0)
	{
		UART0_TypeDef *UART_Instance = &SCI0->UART0;
		if(uart_mode | UART_Mode_Tx)
		{
			UART_Instance->SOE&= ~ UART_CTRL_POS;
			UART_Instance->ST |= UART_CTRL_POS;
		}
		if(uart_mode | UART_Mode_Rx)
		{
			UART_Instance->SOE&= ~ (UART_CTRL_POS <<1);
			UART_Instance->SS |= (UART_CTRL_POS <<1);		
		}		
	}
	else if (UARTx == UART1)
	{
		UART1_TypeDef *UART_Instance = &SCI0->UART1;
		if(uart_mode | UART_Mode_Tx)
		{
			UART_Instance->SOE&= ~ UART_CTRL_POS;
			UART_Instance->ST |= UART_CTRL_POS;
		}
		if(uart_mode | UART_Mode_Rx)
		{
			UART_Instance->SOE&= ~ (UART_CTRL_POS <<1);
			UART_Instance->SS |= (UART_CTRL_POS <<1);		
		}	
	}
	else if (UARTx == UART2)
	{
		UART2_TypeDef *UART_Instance = &SCI1->UART2;
		if(uart_mode | UART_Mode_Tx)
		{
			UART_Instance->SOE&= ~ UART_CTRL_POS;
			UART_Instance->ST |= UART_CTRL_POS;
		}
		if(uart_mode | UART_Mode_Rx)
		{
			UART_Instance->SOE&= ~ (UART_CTRL_POS <<1);
			UART_Instance->SS |= (UART_CTRL_POS <<1);		
		}	
	}
#if defined(BAT32G1XX_80PIN) || defined(BAT32G1XX_100PIN)
	else if (UARTx == UART3)
	{
		UART3_TypeDef *UART_Instance = &SCI2->UART3;
		if(uart_mode | UART_Mode_Tx)
		{
			UART_Instance->SOE&= ~ UART_CTRL_POS;
			UART_Instance->ST |= UART_CTRL_POS;
		}
		if(uart_mode | UART_Mode_Rx)
		{
			UART_Instance->SOE&= ~ (UART_CTRL_POS <<1);
			UART_Instance->SS |= (UART_CTRL_POS <<1);		
		}	
	}
#endif
}


/**
  * @brief  Enables or disables the specified UART peripheral.
  * @param  UARTx: where x can be 0, 1, 2, to select UART peripheral.
  * @param  NewState: new state of the UARTx peripheral.
  *          This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
void UART_Cmd(SCIAFSelect_TypeDef UARTx, FunctionalState NewState)
{
	/* Check the parameters */
	assert_param(IS_UART_ALL_PERIPH(UARTx));
	assert_param(IS_FUNCTIONAL_STATE(NewState));

	if (NewState != DISABLE)
	{
		/* Enable the selected UART by setting the SO SOE SS register */
		UART_Start(UARTx,UART_Mode_Tx|UART_Mode_Rx);
	}
	else
	{
		/* Disable the selected UART by clearing the SOE register and set ST register */
		UART_Stop(UARTx,UART_Mode_Tx|UART_Mode_Rx);
	}
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
static FlagStatus UART_GetFlagStatus(uint16_t reg, uint16_t UART_FLAG)
{
	FlagStatus bitstatus = RESET;
	/* Check the parameters */
	assert_param(IS_UART_FLAG(UART_FLAG));

	bitstatus = SCI_GetFlagStatus(reg,UART_FLAG);
	
	return bitstatus;
}

/**
  * @brief  Transmits single data through the UARTx peripheral.
  * @param  UARTx: where x can be 0, 1, 2 to select the UART peripheral.
  * @param  Data: the data to transmit.
  * @retval None
  */
void UART_SendByte(SCIAFSelect_TypeDef UARTx, uint8_t Data)
{
	if (UARTx == UART0)
	{
		UART0_TypeDef *UART_Instance = &SCI0->UART0;
		UART_Instance->TXD = Data;
		while (UART_GetFlagStatus(UART_Instance->TSSR, UART_FLAG_TSF | UART_FLAG_BFF));
	}
	else if (UARTx == UART1)
	{
		UART1_TypeDef *UART_Instance = &SCI0->UART1;
    	UART_Instance->TXD = Data;
		while (UART_GetFlagStatus(UART_Instance->TSSR, UART_FLAG_TSF | UART_FLAG_BFF));
	}
	else if (UARTx == UART2)
	{
		UART2_TypeDef *UART_Instance = &SCI1->UART2;
    	UART_Instance->TXD = Data;
		while (UART_GetFlagStatus(UART_Instance->TSSR, UART_FLAG_TSF | UART_FLAG_BFF));
	}
#if defined(BAT32G1XX_80PIN) || defined(BAT32G1XX_100PIN)
	else if (UARTx == UART3)
	{
		UART3_TypeDef *UART_Instance = &SCI2->UART3;
    	UART_Instance->TXD = Data;
		while (UART_GetFlagStatus(UART_Instance->TSSR, UART_FLAG_TSF | UART_FLAG_BFF));
	}
#endif
}

/**
  * @brief  Returns the most recent received data by the UARTx peripheral.
  * @param  UARTx: where x can be 0, 1, 2 to select the UART peripheral.
  * @retval The received data.
  */
char UART_ReceiveByte(SCIAFSelect_TypeDef UARTx)
{
	char ch = 0;

	if (UARTx == UART0)
	{
		UART0_TypeDef *UART_Instance = &SCI0->UART0;
    	while (!UART_GetFlagStatus(UART_Instance->RSSR, UART_FLAG_BFF));
		ch = UART_Instance->RXD;
	}
	else if (UARTx == UART1)
	{
		UART1_TypeDef *UART_Instance = &SCI0->UART1;
    	while (!UART_GetFlagStatus(UART_Instance->RSSR, UART_FLAG_BFF));
		ch = UART_Instance->RXD;
	}
	else if (UARTx == UART2)
	{
		UART2_TypeDef *UART_Instance = &SCI1->UART2;
    	while (!UART_GetFlagStatus(UART_Instance->RSSR, UART_FLAG_BFF));
		ch = UART_Instance->RXD;
	}
#if defined(BAT32G1XX_80PIN) || defined(BAT32G1XX_100PIN)
	else if (UARTx == UART3)
	{
		UART3_TypeDef *UART_Instance = &SCI2->UART3;
    	while (!UART_GetFlagStatus(UART_Instance->RSSR, UART_FLAG_BFF));
		ch = UART_Instance->RXD;
	}
#endif
    return ch;  
}

/**
  * @brief  Clear the specified UART flag is set.
  * @param  UARTx: where x can be 0, 1, 2 to select the UART peripheral.
  * @param  UART_FLAG: specifies the flag to check.
  *          This parameter can be one of the following values:
  *            @arg UART_FLAG_FEF: Transmission frame data error flag
  *            @arg UART_FLAG_PEF: UART Parity error flag
  *            @arg UART_FLAG_OVF: Transmission over run error flag
  * @retval None.
  */
void UART_ClearFlag(SCIAFSelect_TypeDef UARTx, uint16_t UART_FLAG)
{
	/* Check the parameters */
	assert_param(IS_UART_ALL_PERIPH(UARTx));
	assert_param(IS_UART_CLEAR_FLAG(UART_FLAG));
	
	SCI_ClearFlag(UARTx,UART_FLAG);
}

/**
  * @brief  Checks whether the specified UART flag is set or not.
  * @param  UARTx: where x can be 0, 1, 2 to select the UART peripheral.
  * @param  UART_FLAG: specifies the flag to check.
  *          This parameter can be one of the following values:
  *            @arg UART_FLAG_TSF: Transmission running status flag
  *            @arg UART_FLAG_BFF: Transmission register save valid data flag
  *            @arg UART_FLAG_FEF: Transmission frame data error flag
  *            @arg UART_FLAG_PEF: UART Parity error flag
  *            @arg UART_FLAG_OVF: Transmission over run error flag
  * @retval The new state of USART_FLAG (SET or RESET).
  */
FlagStatus UART_GetErrStaus(SCIAFSelect_TypeDef UARTx, uint16_t UART_FLAG)
{
	FlagStatus status;
	
	/* Check the parameters */
	assert_param(IS_UART_ALL_PERIPH(UARTx));
	assert_param(IS_UART_FLAG(UART_FLAG));

	status = SCI_GetErrStaus(UARTx,UART_FLAG);

	return status;
}
