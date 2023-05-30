/**
  ******************************************************************************
  * @file    sci_common.c
  * @author  CMS Application Team
  * @version Vx.x.x
  * @date    24-April-2022
  * @brief   This file provides firmware functions to manage the following 
  *          functionalities of the Serial Controllor Unit(SCI):           
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


#include "sci_common.h"
#include <math.h>

/* Private define ------------------------------------------------------------*/
#define MAX_SCI_UNIT		3u
#define MAX_CHX_NUMR		4u
#define SCI_PARAMETER_NOT_USED(p)    (void) ((p))
#define WEAK_ERROR_ATTRIBUTE
#pragma weak sci_error_log   = sci_error_log_internal
/* Private typedef -----------------------------------------------------------*/
typedef struct {
	uint8_t status[MAX_SCI_UNIT][MAX_CHX_NUMR];
} SCICh_Status_Typedef;
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static SCICh_Status_Typedef SCI_ChxStat;
static const uint16_t sps_tab[16] = 
{1, 2, 4, 8, 16, 32, 64, 128, 256, 512, 1024, 2048, 4096, 8192, 16384, 32768};

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
  * @brief  This function is aimed to check SCI unit and its channel is used or not  
  * @param  unitId: SCI unit ID is 1 or 2 or 3.
  * @param  channelId: The channel ID in SCI unit is 0, 1, 2 or 3.
  * @retval unit channel status 
  */
static int8_t Alloc_ChannelFlag(uint16_t unitId, uint16_t channelId)
{
	/* check input parameter */
	assert_param(IS_ALL_SCIUNIT(unitId));
	assert_param(IS_ALL_SCICH(channelId));

	if(SCI_ChxStat.status[unitId][channelId])
		return SCI_ERR_CHANNEL_INVALID;
	else
	{
		SCI_ChxStat.status[unitId][channelId] = 1;
		return SCI_SUCCESS;
	}
}

/**
  * @brief  This function is to free SCI unit and its channel flag
  * @param  unitId: SCI unit ID is 0 or 1.
  * @param  channelId: The channel ID in SCI unit is 0, 1, 2 or 3.
  * @retval None
  */
static void Free_ChannelFlag(uint16_t unitId, uint16_t channelId)
{
	/* check input parameter */
	assert_param(IS_ALL_SCIUNIT(unitId));
	assert_param(IS_ALL_SCICH(channelId));

	if(SCI_ChxStat.status[unitId][channelId] == 1)
	{
		SCI_ChxStat.status[unitId][channelId] = 0;
	}	
}

/* Public functions ----------------------------------------------------------*/

/**
  * @brief  Default error logger function, used only if sci_error_log is not defined in the user application.
  * @param  err  The error code encountered.
  * @param  file The file name in which the error code was encountered.
  * @param  line The line number at which the error code was encountered.
  * @retval None
  */
void sci_error_log_internal(int8_t err, const char *file, int32_t line)
{
    SCI_PARAMETER_NOT_USED(err);
    SCI_PARAMETER_NOT_USED(file);
    SCI_PARAMETER_NOT_USED(line);
//	printf("%s,%d\r\n",file, line);
	__BKPT(0);
    while (1)
    {
        /* Do nothing. */
    }
}

/**
  * @brief  Calculate the register setting options for baud rate to UART peripheral.
  * @param  fclk_freq: System clock value on chip.
  * @param  baud: The target baud rate which want to setting.
  * @param  pValue: UART baud rate setting option data structure.
  * @retval None
  */
float SCIPeriphal_ClockUpdate(SCIAFSelect_TypeDef func, uint32_t fclk, uint32_t ftclk, uint8_t osps, SCIPeriph_Clock_TypeDef *clock)
{
#define CALCULATE_MAX_ERROR		2
#define CALCULATE_MAX_SPS		0x0F
#define CALCULATE_MAX_SDR		0x7F

	uint8_t i, j, tmp_i, tmp_j;
	uint8_t init_sdr, max_sps;
	uint32_t cal_fmck = 0, cal_ftclk = 0;
	float cur_err = 0, max_err = 100;

	if (func & SCI_UART_MASK)
		init_sdr = 2;
	else if (func & SCI_I2C_MASK)
		init_sdr = 1;
	else
		init_sdr = 0;

	if (osps)
	{
		clock->sps = osps;
		max_sps = 1;
	}
	else
		max_sps = CALCULATE_MAX_SPS + 1;

    for (i = 0; i < max_sps; i++)
    {
		if (osps)
		{
			i = osps;
			cal_fmck = fclk / sps_tab[osps];
		}
		else
			cal_fmck = fclk / sps_tab[i];

		for (j = init_sdr; j <= CALCULATE_MAX_SDR; j++)
		{
			cal_ftclk = cal_fmck / (j + 1) / 2;
			cur_err = 100 * (float)(cal_ftclk - ftclk) / (float)ftclk;

			if(fabs(max_err) > fabs(cur_err))
			{
				tmp_i = i;
				tmp_j = j;
				max_err = cur_err;
			}
		}
	}

	clock->sps = tmp_i;
	clock->sdr = tmp_j;

	return max_err;
}

/**
  * @brief  This function is aimed to check sci unit and its channel is used or not  
  * @param  func: The AF function for SCI channel periphal.
  * @param  mode: The mode for SCI channel periphal.
  * @retval status
  */
int SCIChannel_Alloca(SCIAFSelect_TypeDef func, uint16_t mode)
{
	int res;
	uint8_t channelId = 0;
	uint8_t uinitId = 0;

	/* check input parameter */
	assert_param(IS_ALL_SCIAF(func));

	uinitId = (func & SCI_UINT_MASK) >> 12;
#ifdef BAT32G1XX_80PIN	
	if(func == UART0 || func == UART1 || func == UART2 || func == UART3)
#else
	if(func == UART0 || func == UART1 || func == UART2)
#endif
	{
		if((mode & 0x0022)== 0x0022)  // UART_Mode_Tx
		{
			channelId = (0x0F & func);
			res = Alloc_ChannelFlag(uinitId, channelId);
		}
		
		if((mode & 0x0012) == 0x0012) // UART_Mode_Rx
		{
			channelId = ((0x0F & func) + 1);
			res = Alloc_ChannelFlag(uinitId, channelId);
		}
	}
	else
		res = Alloc_ChannelFlag(uinitId, (0x0F & func));
	
	return res;
}

/**
  * @brief  This function is free the SCI unit and its channel
  * @param  func: The AF function for SCI channel periphal.
  * @param  mode: The mode for SCI channel periphal.
  * @retval None
  */
void SCIChannel_Free(SCIAFSelect_TypeDef func, uint16_t mode)
{
	uint8_t channelId = 0;
	uint8_t uinitId = 0;

	/* check input parameter */
	assert_param(IS_ALL_SCIAF(func));

	uinitId = (func & 0x1000) >> 12;
#if defined( BAT32G1XX_100PIN) || defined( BAT32G1XX_80PIN)	
	if(func == UART0 || func == UART1 || func == UART2 || func == UART3)
#else
	if(func == UART0 || func == UART1 || func == UART2)	
#endif
	{
		if((mode & 0x0022)== 0x0022)  // UART_Mode_Tx
		{
			channelId = (0x0F & func);
			Free_ChannelFlag(uinitId, channelId);
		}
		
		if((mode & 0x0012) == 0x0012) // UART_Mode_Rx
		{
			channelId = ((0x0F & func) + 1);
			Free_ChannelFlag(uinitId, channelId);
		}
	}
	else
		Free_ChannelFlag(uinitId, (0x0F & func));
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
void SCI_Output_Cmd(SCIAFSelect_TypeDef SCIx, FunctionalState NewState)
{
	
	switch(SCIx)
	{
		case I2C00:
		{
			I2C0_TypeDef *I2C_Instance = &SCI0->I2C0;
			I2C_Instance->SOE = (NewState == ENABLE) ? SCI_START_EN : SCI_START_DIS;
		}
		break;
		case I2C01:
		{
			I2C1_TypeDef *I2C_Instance = &SCI0->I2C1;
			I2C_Instance->SOE = (NewState == ENABLE) ? SCI_START_EN : SCI_START_DIS;
		}
		break;
		case I2C10:
		{
			I2C2_TypeDef *I2C_Instance = &SCI0->I2C2;
			I2C_Instance->SOE = (NewState == ENABLE) ? SCI_START_EN : SCI_START_DIS;
		}
		break;
		case I2C11:
		{
			I2C3_TypeDef *I2C_Instance = &SCI0->I2C3;
			I2C_Instance->SOE = (NewState == ENABLE) ? SCI_START_EN : SCI_START_DIS;
		}
		break;
		case I2C20:
		{
			I2C4_TypeDef *I2C_Instance = &SCI1->I2C4;
			I2C_Instance->SOE = (NewState == ENABLE) ? SCI_START_EN : SCI_START_DIS;
		}
		break;
		case I2C21:
		{
			I2C5_TypeDef *I2C_Instance = &SCI1->I2C5;
			I2C_Instance->SOE = (NewState == ENABLE) ? SCI_START_EN : SCI_START_DIS;
		}
		break;
#if defined( BAT32G1XX_100PIN) || defined( BAT32G1XX_80PIN)
		case I2C30:
		{
			I2C6_TypeDef *I2C_Instance = &SCI2->I2C6;
			I2C_Instance->SOE = (NewState == ENABLE) ? SCI_START_EN : SCI_START_DIS;
		}
		break;
		case I2C31:
		{
			I2C7_TypeDef *I2C_Instance = &SCI2->I2C7;
			I2C_Instance->SOE = (NewState == ENABLE) ? SCI_START_EN : SCI_START_DIS;
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
void SCI_ChannelStart_Cmd(SCIAFSelect_TypeDef SCIx, FunctionalState NewState)
{
	assert_param(IS_ALL_SCIAF(SCIx));	
	switch(SCIx)
	{
		case SSPI00:
		case I2C00:
		{
			I2C0_TypeDef *I2C_Instance = &SCI0->I2C0;
			if(NewState == ENABLE) 
				I2C_Instance->SS =  SCI_START_EN;
			else
				I2C_Instance->ST =  SCI_STOP_EN;
		}
		break;
		case SSPI01:
		case I2C01:
		{
			I2C1_TypeDef *I2C_Instance = &SCI0->I2C1;
			if(NewState == ENABLE) 
				I2C_Instance->SS =  SCI_START_EN;
			else
				I2C_Instance->ST =  SCI_STOP_EN;
		}
		break;
		case SSPI10:
		case I2C10:
		{
			I2C2_TypeDef *I2C_Instance = &SCI0->I2C2;
			if(NewState == ENABLE) 
				I2C_Instance->SS =  SCI_START_EN;
			else
				I2C_Instance->ST =  SCI_STOP_EN;
		}
		break;
		case SSPI11:
		case I2C11:
		{
			I2C3_TypeDef *I2C_Instance = &SCI0->I2C3;
			if(NewState == ENABLE) 
				I2C_Instance->SS =  SCI_START_EN;
			else
				I2C_Instance->ST =  SCI_STOP_EN;
		}
		break;
		case SSPI20:
		case I2C20:
		{
			I2C4_TypeDef *I2C_Instance = &SCI1->I2C4;
			if(NewState == ENABLE) 
				I2C_Instance->SS =  SCI_START_EN;
			else
				I2C_Instance->ST =  SCI_STOP_EN;
		}
		break;
		case SSPI21:
		case I2C21:
		{
			I2C5_TypeDef *I2C_Instance = &SCI1->I2C5;
			if(NewState == ENABLE) 
				I2C_Instance->SS =  SCI_START_EN;
			else
				I2C_Instance->ST =  SCI_STOP_EN;
		}
		break;
		
		default:
		break;
	}
}

/**
  * @brief  Get the specified I2C flag is set.
  * @param  reg: The I2C peripheral for flag status.
  * @param  I2C_FLAG: specifies the flag to check.
  *          This parameter can be one of the following values:
  *            @arg I2C_FLAG_NOACK: Transmission frame have no ack check.
  *            @arg I2C_FLAG_OVERRUN: Data over run.
  * @retval None.
  */
FlagStatus SCI_GetFlagStatus(uint16_t reg, uint16_t FLAG)
{
	FlagStatus bitstatus = RESET;
	
	/* Check the parameters */

	if ((reg & FLAG) != (uint16_t)RESET)
	{
		bitstatus = SET;
	}
	else
	{
		bitstatus = RESET;
	}
	
	return bitstatus;
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
FlagStatus SCI_GetErrStaus(SCIAFSelect_TypeDef SCIx, uint16_t FLAG)
{
	FlagStatus status;
	
    /* Check the parameters */
	assert_param(IS_ALL_SCIAF(SCIx));

    /* get the selected I2C flag */
	switch(SCIx)
	{
		case SSPI00:
		case I2C00:
		{
			I2C0_TypeDef *I2C_Instance = &SCI0->I2C0;
			status = SCI_GetFlagStatus(I2C_Instance->SSR, FLAG);
		}
		break;
		case SSPI01:		
		case I2C01:
		{
			I2C1_TypeDef *I2C_Instance = &SCI0->I2C1;
			status = SCI_GetFlagStatus(I2C_Instance->SSR, FLAG);
		}
		break;
		case SSPI10:		
		case I2C10:
		{
			I2C2_TypeDef *I2C_Instance = &SCI0->I2C2;
			status = SCI_GetFlagStatus(I2C_Instance->SSR, FLAG);
		}
		break;
		case SSPI11:		
		case I2C11:
		{
			I2C3_TypeDef *I2C_Instance = &SCI0->I2C3;
			status = SCI_GetFlagStatus(I2C_Instance->SSR, FLAG);
		}
		break;
		case SSPI20:		
		case I2C20:
		{
			I2C4_TypeDef *I2C_Instance = &SCI1->I2C4;
			status = SCI_GetFlagStatus(I2C_Instance->SSR, FLAG);
		}
		break;
		case SSPI21:			
		case I2C21:
		{
			I2C5_TypeDef *I2C_Instance = &SCI1->I2C5;
			status = SCI_GetFlagStatus(I2C_Instance->SSR, FLAG);
		}
		break;
		case UART0:
		{			
			UART0_TypeDef *UART_Instance = &SCI0->UART0;
			status = SCI_GetFlagStatus(UART_Instance->RSSR, FLAG);
		}
		break;
		case UART1:
		{			
			UART1_TypeDef *UART_Instance = &SCI0->UART1;
			status = SCI_GetFlagStatus(UART_Instance->RSSR, FLAG);
		}
		break;
		case UART2:
		{			
			UART2_TypeDef *UART_Instance = &SCI1->UART2;
			status = SCI_GetFlagStatus(UART_Instance->RSSR, FLAG);
		}
		break;
#if defined( BAT32G1XX_100PIN) || defined( BAT32G1XX_80PIN)
		case UART3:
		{			
			UART3_TypeDef *UART_Instance = &SCI2->UART3;
			status = SCI_GetFlagStatus(UART_Instance->RSSR, FLAG);
		}
		break;
		case SSPI30:		
		case I2C30:
		{
			I2C6_TypeDef *I2C_Instance = &SCI2->I2C6;
			status = SCI_GetFlagStatus(I2C_Instance->SSR, FLAG);
		}
		break;
		case SSPI31:			
		case I2C31:
		{
			I2C7_TypeDef *I2C_Instance = &SCI2->I2C7;
			status = SCI_GetFlagStatus(I2C_Instance->SSR, FLAG);
		}
		break;
#endif		
		default:
		break;
	}
		
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
void SCI_ClearFlag(SCIAFSelect_TypeDef SCIx, uint16_t FLAG)
{
    /* Check the parameters */
	assert_param(IS_ALL_SCIAF(SCIx));


    /* Clear the selected I2C flag */
	switch(SCIx)
	{
		case SSPI00:
		case I2C00:
		{
			I2C0_TypeDef *I2C_Instance = &SCI0->I2C0;
			I2C_Instance->SIR |= FLAG;
		}
		break;
		case SSPI01:
		case I2C01:
		{
			I2C1_TypeDef *I2C_Instance = &SCI0->I2C1;
			I2C_Instance->SIR |= FLAG;
		}
		break;
		case SSPI10:
		case I2C10:
		{
			I2C2_TypeDef *I2C_Instance = &SCI0->I2C2;
			I2C_Instance->SIR |= FLAG;
		}
		break;
		case SSPI11:
		case I2C11:
		{
			I2C3_TypeDef *I2C_Instance = &SCI0->I2C3;
			I2C_Instance->SIR |= FLAG;
		}
		break;
		case SSPI20:
		case I2C20:
		{
			I2C4_TypeDef *I2C_Instance = &SCI1->I2C4;
			I2C_Instance->SIR |= FLAG;
		}
		break;
		case SSPI21:
		case I2C21:
		{
			I2C5_TypeDef *I2C_Instance = &SCI1->I2C5;
			I2C_Instance->SIR |= FLAG;
		}
		break;
		case UART0:
		{			
			UART0_TypeDef *UART_Instance = &SCI0->UART0;
			UART_Instance->RSIR |= FLAG;
			UART_Instance->TSIR |= FLAG;
		}
		break;
		case UART1:
		{			
			UART1_TypeDef *UART_Instance = &SCI0->UART1;
			UART_Instance->RSIR |= FLAG;
			UART_Instance->TSIR |= FLAG;
		}
		break;
		case UART2:
		{			
			UART2_TypeDef *UART_Instance = &SCI1->UART2;
			UART_Instance->RSIR |= FLAG;
			UART_Instance->TSIR |= FLAG;
		}
		break;
#if defined( BAT32G1XX_100PIN) || defined( BAT32G1XX_80PIN)
		case UART3:
		{			
			UART3_TypeDef *UART_Instance = &SCI2->UART3;
			UART_Instance->RSIR |= FLAG;
			UART_Instance->TSIR |= FLAG;
		}
		break;
		case SSPI30:		
		case I2C30:
		{
			I2C6_TypeDef *I2C_Instance = &SCI2->I2C6;
			I2C_Instance->SIR |= FLAG;
		}
		break;
		case SSPI31:			
		case I2C31:
		{
			I2C7_TypeDef *I2C_Instance = &SCI2->I2C7;
			I2C_Instance->SIR |= FLAG;
		}
		break;
#endif	
		default:
		break;
	}
}
