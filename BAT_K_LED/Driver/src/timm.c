/*******************************************************************************
*                  COPYRIGHT (C) 2021 CMS Technologies Ltd.                    *
*                                                                              *
********************************************************************************
* FileName      : tima.c                                                  *
* Author        :                                                              *
* Version       : 1.0                                                          *
* Date          : 2021.08.13                                                   *
* Description   :                                                              *
* Function List :                                                              *
********************************************************************************/
#include "timm.h"
#include "cgc.h"

static TMM_REG_t TMM_reg;
volatile uint16_t tmm_overflow_count_a = 0U;
volatile uint16_t tmm_overflow_count_b = 0U;


/***********************************************************************************************************************
* Function Name: TMM_RegSetting
* @brief  this function is to mapping TMM reg
* @param   
* @return   
***********************************************************************************************************************/	
void TMM_RegSetting(TMMSelect_TypeDef TMMx)
{
	switch (TMMx)
	{
		case TMM0:
			TMM_reg.TMMR  = &TMM->TMMR;
			TMM_reg.TMPMR = &TMM->TMPMR;
			TMM_reg.TMFCR = &TMM->TMFCR;				
			TMM_reg.TMOCR = &TMM->TMOCR;
			TMM_reg.TMCR[0] = &TMM->TMCR0;
			TMM_reg.TMIORA[0] = &TMM->TMIORA0;
			TMM_reg.TMIORC[0] = &TMM->TMIORC0;
			TMM_reg.TMIER[0] = &TMM->TMIER0;
		    TMM_reg.TMPOCR[0] = &TMM->TMPOCR0;
			TMM_reg.TMSR  = &TMM->TMSR0;
		
			TMM_reg.TMGRA = &TMM->TMGRA0;
			TMM_reg.TMGRB = &TMM->TMGRB0;
			TMM_reg.TMGRC = &TMM->TMGRC0;
			TMM_reg.TMGRD = &TMM->TMGRD0;

			TMM_reg.OPTL0 = &TMM->OPCTL0;
			TMM_reg.TMOER1 = &TMM->TMOER1;		
			TMM_reg.TMDF[0] = &TMM->TMDF0;		
			break;
		case TMM1:
			TMM_reg.TMMR  = &TMM->TMMR;
			TMM_reg.TMPMR = &TMM->TMPMR;
			TMM_reg.TMFCR = &TMM->TMFCR;
			TMM_reg.TMOCR = &TMM->TMOCR;		
			TMM_reg.TMCR[1] = &TMM->TMCR1;
			TMM_reg.TMIORA[1] = &TMM->TMIORA1;
			TMM_reg.TMIORC[1] = &TMM->TMIORC1;
			TMM_reg.TMIER[1] = &TMM->TMIER1;
		    TMM_reg.TMPOCR[1] = &TMM->TMPOCR1;
			TMM_reg.TMSR  = &TMM->TMSR1;
		
   		    TMM_reg.TMGRA = &TMM->TMGRA1;
			TMM_reg.TMGRB = &TMM->TMGRB1;
			TMM_reg.TMGRC = &TMM->TMGRC1;
			TMM_reg.TMGRD = &TMM->TMGRD1;
		
			TMM_reg.OPTL0 = &TMM->OPCTL0;
			TMM_reg.TMOER1 = &TMM->TMOER1;
			TMM_reg.TMDF[1] = &TMM->TMDF1;
			break;
		default:
			break;
	}
}
/**
  * @brief  Initializes the TIEMRM peripheral according to the specified
  *         parameters in the TIMM_InitStruct .
  * @param  TIMM_InitStruct: pointer to a TMM_InitTypeDef structure that contains
  *         the configuration information for the specified TIEMRM peripheral.
  * @retval none
  */
void TMM_Init(TMM_InitTypeDef *TMM_InitStruct)
{
	assert_param(IS_TMM_CLOCK(TMM_InitStruct->TMM_Clk));	
	assert_param(IS_TMM_CHANNEL(TMM_InitStruct->TMM_Channel));	

	CGC_PER1PeriphClockCmd(CGC_PER1Periph_TMM,ENABLE);
	TMM_RegSetting(TMM_InitStruct->TMM_Select);
	
	TMM->TMSTR &= (uint8_t)~((_01_TMM_TM0_COUNT_START << TMM1)|_01_TMM_TM0_COUNT_START);
	TMM->TMSTR |= _04_TMM_TM0_COUNT_CONTINUES | (_04_TMM_TM0_COUNT_CONTINUES << TMM1); 
	if((TMM_InitStruct->TMM_Clk == TMM_FCLK_TMCLK) && TMM_InitStruct->TMM_Mode != ( TMM_Mode_PWM3))
	{
		*TMM_reg.TMFCR |= 0x40;  // input clock is valid
	}

	if(TMM_InitStruct->TMM_Mode == TMM_Mode_Capture)
	{
		*TMM_reg.TMMR |= 0x00 | (TMM_InitStruct->TMM_IC.TMM_CHD_AS_BufferRegister << (TMM_InitStruct->TMM_Select << 1)) << 4 |\
						(TMM_InitStruct->TMM_IC.TMM_CHC_AS_BufferRegister << (TMM_InitStruct->TMM_Select << 1)) << 4;		
		*TMM_reg.TMDF[TMM_InitStruct->TMM_Select] |= TMM_InitStruct->TMM_IC.TMM_Filter_Clk;
		*TMM_reg.TMCR[TMM_InitStruct->TMM_Select] = TMM_InitStruct->TMM_CounterClear | TMM_InitStruct->TMM_Clk;
		if((TMM_InitStruct->TMM_Channel & TMM_Channel_A) == TMM_Channel_A)
		{
			*TMM_reg.TMIORA[TMM_InitStruct->TMM_Select] |= 0x04 | TMM_InitStruct->TMM_IC.TMM_CHA.TMM_ICPolarity;
			*TMM_reg.TMDF[TMM_InitStruct->TMM_Select] |= TMM_InitStruct->TMM_IC.TMM_CHA.TMM_ICFilter;
		}
		if((TMM_InitStruct->TMM_Channel & TMM_Channel_B)== TMM_Channel_B)
		{
			*TMM_reg.TMIORA[TMM_InitStruct->TMM_Select] |= 0x40 | TMM_InitStruct->TMM_IC.TMM_CHB.TMM_ICPolarity;
			*TMM_reg.TMDF[TMM_InitStruct->TMM_Select] |= TMM_InitStruct->TMM_IC.TMM_CHB.TMM_ICFilter << 1;			
		}
		if((TMM_InitStruct->TMM_Channel & TMM_Channel_C)== TMM_Channel_C)
		{
			*TMM_reg.TMIORC[TMM_InitStruct->TMM_Select] |= 0x08 |0x04 | TMM_InitStruct->TMM_IC.TMM_CHB.TMM_ICPolarity;	
			*TMM_reg.TMDF[TMM_InitStruct->TMM_Select] |= TMM_InitStruct->TMM_IC.TMM_CHC.TMM_ICFilter << 2;			
		}
		if((TMM_InitStruct->TMM_Channel & TMM_Channel_D)== TMM_Channel_D)
		{
			*TMM_reg.TMIORC[TMM_InitStruct->TMM_Select] |= 0x80 |0x40 | TMM_InitStruct->TMM_IC.TMM_CHB.TMM_ICPolarity;	
			*TMM_reg.TMDF[TMM_InitStruct->TMM_Select] |= TMM_InitStruct->TMM_IC.TMM_CHD.TMM_ICFilter << 3;			
		}		
	}
	if(TMM_InitStruct->TMM_Mode == TMM_Mode_Compare)
	{
		*TMM_reg.TMMR |= 0x00;                 //the TMGRC and TMGRD are used as general register	
		*TMM_reg.TMIORC[TMM_InitStruct->TMM_Select] &= 0x00;
		*TMM_reg.TMCR[TMM_InitStruct->TMM_Select] = TMM_InitStruct->TMM_CounterClear | TMM_InitStruct->TMM_Clk; //counter cleared condition 
		TMM->TMOER1 &= 0x00U;		
		if((TMM_InitStruct->TMM_Channel & TMM_Channel_A) == TMM_Channel_A)
		{
			*TMM_reg.TMIORA[TMM_InitStruct->TMM_Select] |= (TMM_InitStruct->TMM_OC.TMM_CHA.TMM_OCPolarity);	
			*TMM_reg.TMGRA = TMM_InitStruct->TMM_CHA_Pulse - 1;	
			TMM->TMOER1 |= 	0x00 << (TMM_InitStruct->TMM_Select << 2);
		}
		if((TMM_InitStruct->TMM_Channel & TMM_Channel_B)== TMM_Channel_B)
		{
			*TMM_reg.TMIORA[TMM_InitStruct->TMM_Select] |= ((TMM_InitStruct->TMM_OC.TMM_CHB.TMM_OCPolarity)<< 4);
			*TMM_reg.TMGRB = TMM_InitStruct->TMM_CHB_Pulse - 1;		
			TMM->TMOER1 |= 	0x00 << (TMM_InitStruct->TMM_Select << 2);			
		}
		if((TMM_InitStruct->TMM_Channel & TMM_Channel_C)== TMM_Channel_C)
		{
			*TMM_reg.TMIORC[TMM_InitStruct->TMM_Select] |= (TMM_InitStruct->TMM_OC.TMM_CHC.TMM_OCPolarity);	
			*TMM_reg.TMGRC = TMM_InitStruct->TMM_CHC_Pulse - 1;		
			TMM->TMOER1 |= 	0x00 << (TMM_InitStruct->TMM_Select << 2);
			if(TMM_InitStruct->TMM_OC.TMM_CHC_OutPin_Ctrl == TMM_OC_Output_PinCtrl_Enable)
			{
				*TMM_reg.TMIORC[TMM_InitStruct->TMM_Select] |= 0x00; 
			}
			else
			{
				*TMM_reg.TMIORC[TMM_InitStruct->TMM_Select] |= 0x08; // use TMGRC to control TMIOA pin output
			}
		}
		if((TMM_InitStruct->TMM_Channel & TMM_Channel_D)== TMM_Channel_D)
		{
			*TMM_reg.TMIORC[TMM_InitStruct->TMM_Select] |= ((TMM_InitStruct->TMM_OC.TMM_CHD.TMM_OCPolarity)<< 4);	
			*TMM_reg.TMGRD = TMM_InitStruct->TMM_CHD_Pulse - 1;		
			TMM->TMOER1 |= 	0x00 << (TMM_InitStruct->TMM_Select << 2);	
			if(TMM_InitStruct->TMM_OC.TMM_CHD_OutPin_Ctrl == TMM_OC_Output_PinCtrl_Enable)
			{
				*TMM_reg.TMIORC[TMM_InitStruct->TMM_Select] |= 0x00; 
			}
			else
			{
				*TMM_reg.TMIORC[TMM_InitStruct->TMM_Select] |= 0x80; // use TMGRD to control TMIOB pin output
			}
		}
//		*TMM_reg.TMIER[TMM_InitStruct->TMM_Select] |= TMM_InitStruct->TMM_Intp;
	}
	
	if(TMM_InitStruct->TMM_Mode == TMM_Mode_PWM || TMM_InitStruct->TMM_Mode == TMM_Mode_PWM_RSTSYN )
	{
		*TMM_reg.TMMR |= 0x00;                 //the TMGRC and TMGRD are used as general register	
		*TMM_reg.TMCR[TMM_InitStruct->TMM_Select] = TMM_InitStruct->TMM_CounterClear | TMM_InitStruct->TMM_Clk; //counter cleared condition 
		*TMM_reg.TMGRA = TMM_InitStruct->TMM_CHA_Pulse - 1;			
		if((TMM_InitStruct->TMM_Channel & TMM_Channel_B)== TMM_Channel_B)
		{
			*TMM_reg.TMPMR |=	TMM_InitStruct->TMM_Channel <<(TMM_InitStruct->TMM_Select << 2);
			*TMM_reg.TMGRB = TMM_InitStruct->TMM_CHB_Pulse - 1;	
			*TMM_reg.TMOER1 &= (~TMM_InitStruct->TMM_Channel) << (TMM_InitStruct->TMM_Select << 2);
			*TMM_reg.TMPOCR[TMM_InitStruct->TMM_Select] |= TMM_InitStruct->TMM_PWM.TMM_CHB.TMM_PWMActiveLevel;
			*TMM_reg.TMOCR |= ((TMM_InitStruct->TMM_PWM.TMM_CHB.TMM_PWMInitLevel << 1) << (TMM_InitStruct->TMM_Select << 2));
		}
		if((TMM_InitStruct->TMM_Channel & TMM_Channel_C)== TMM_Channel_C)
		{
			*TMM_reg.TMPMR |=	TMM_InitStruct->TMM_Channel <<(TMM_InitStruct->TMM_Select << 2);
			*TMM_reg.TMGRC = TMM_InitStruct->TMM_CHC_Pulse - 1;	
			*TMM_reg.TMOER1 &= (~TMM_InitStruct->TMM_Channel) << (TMM_InitStruct->TMM_Select << 2);
			*TMM_reg.TMPOCR[TMM_InitStruct->TMM_Select] |= TMM_InitStruct->TMM_PWM.TMM_CHC.TMM_PWMActiveLevel << 1;
			*TMM_reg.TMOCR |= ((TMM_InitStruct->TMM_PWM.TMM_CHC.TMM_PWMInitLevel << 2) << (TMM_InitStruct->TMM_Select << 2));
		}
		if((TMM_InitStruct->TMM_Channel & TMM_Channel_D)== TMM_Channel_D)
		{
			*TMM_reg.TMPMR |=	TMM_InitStruct->TMM_Channel <<(TMM_InitStruct->TMM_Select << 2);
			*TMM_reg.TMGRD = TMM_InitStruct->TMM_CHD_Pulse - 1;	
			*TMM_reg.TMOER1 &= (~TMM_InitStruct->TMM_Channel) << (TMM_InitStruct->TMM_Select << 2);
			*TMM_reg.TMPOCR[TMM_InitStruct->TMM_Select] |= TMM_InitStruct->TMM_PWM.TMM_CHD.TMM_PWMActiveLevel <<  2;
			*TMM_reg.TMOCR |= ((TMM_InitStruct->TMM_PWM.TMM_CHD.TMM_PWMInitLevel << 3) << (TMM_InitStruct->TMM_Select << 2));
		}
		if(TMM_InitStruct->TMM_Mode == TMM_Mode_PWM_RSTSYN)
		{
			*TMM_reg.TMFCR |= 0x01;
		}		
//		*TMM_reg.TMIER[TMM_InitStruct->TMM_Select] |= TMM_InitStruct->TMM_Intp;	
	}
	if(TMM_InitStruct->TMM_Mode == TMM_Mode_CompPWM)
	{
		CGC_PER1PeriphClockCmd(CGC_PER1Periph_PWMCUTOFF,ENABLE);
		*TMM_reg.TMMR |= _80_TMM_TMGRD1_BUFFER |_40_TMM_TMGRC1_BUFFER|_20_TMM_TMGRD0_BUFFER; //the TMGRD1 used as TMGRB1 buffer register¡¢TMGRC1 used as TMGRA1 buffer register¡¢ TMGRD0 are used TMGRB0 buffer register	
		*TMM_reg.TMCR[TMM_InitStruct->TMM_Select] = TMM_InitStruct->TMM_CounterClear | TMM_InitStruct->TMM_Clk; //counter cleared condition 
		*TMM_reg.TMFCR |= (TMM_InitStruct->TMM_BDTR.TMM_NegativeState << 3) | (TMM_InitStruct->TMM_BDTR.TMM_PostiveState << 2)| TMM_InitStruct->TMM_Combine_TransMode;
		
		*TMM_reg.TMDF[TMM_InitStruct->TMM_Select] |= (TMM_InitStruct->TMM_BDTR.TMIOA_BreakPolarity << 6) |(TMM_InitStruct->TMM_BDTR.TMIOB_BreakPolarity << 4)  \
													| (TMM_InitStruct->TMM_BDTR.TMIOC_BreakPolarity << 2) |(TMM_InitStruct->TMM_BDTR.TMIOD_BreakPolarity << 0);
		
		 TMM->TMELC |= (TMM_InitStruct->TMM_BDTR.TMM_Break <<1) << (TMM_InitStruct->TMM_Select << 4);
				
//		*TMM_reg.TMIER[TMM_InitStruct->TMM_Select] |= TMM_InitStruct->TMM_Intp;

		if((TMM_InitStruct->TMM_Channel & TMM_Channel_A) == TMM_Channel_A)
		{
			if(TMM_InitStruct->TMM_Select == TMM0)
			{
				*TMM_reg.TMGRA = TMM_InitStruct->TMM_CHA_Pulse - 1;	
				*TMM_reg.TMGRA = *TMM_reg.TMGRA + TMM_InitStruct->TMM_BDTR.TMM_DeadTime - 1;
				*TMM_reg.TMOER1 |= TMM_Channel_A << (TMM_InitStruct->TMM_Select << 2);
			}
			else
			{
				*TMM_reg.TMGRA = TMM_InitStruct->TMM_CHA_Pulse - 1;	
				*TMM_reg.TMOER1 &= ~((TMM_Channel_A) << (TMM_InitStruct->TMM_Select << 2));
			}
		}		
		if((TMM_InitStruct->TMM_Channel & TMM_Channel_B)== TMM_Channel_B)
		{
			*TMM_reg.TMGRB = TMM_InitStruct->TMM_CHB_Pulse - 1;	
			*TMM_reg.TMOER1 &= ~((TMM_Channel_B) << (TMM_InitStruct->TMM_Select << 2));
		}
		if((TMM_InitStruct->TMM_Channel & TMM_Channel_C)== TMM_Channel_C)
		{
			*TMM_reg.TMGRC = TMM_InitStruct->TMM_CHC_Pulse - 1;	
			*TMM_reg.TMOER1 &= ~(TMM_Channel_C << (TMM_InitStruct->TMM_Select << 2));
		}
		if((TMM_InitStruct->TMM_Channel & TMM_Channel_D)== TMM_Channel_D)
		{
			*TMM_reg.TMGRD = TMM_InitStruct->TMM_CHD_Pulse - 1;	
			*TMM_reg.TMOER1 &= ~(TMM_Channel_D << (TMM_InitStruct->TMM_Select << 2));
		}
		if(TMM_InitStruct->TMM_Select == TMM0)
		{
			TMM->TM0 = TMM_InitStruct->TMM_BDTR.TMM_DeadTime;
		}
		TMM->TM1 = 0;
	}
	*TMM_reg.TMIER[TMM_InitStruct->TMM_Select] |= TMM_InitStruct->TMM_Intp;

}

/**
  * @brief  This function starts TM0 and TM1 counter.
  * @param  None
  * @retval None
  */
void TMM_All_Start(void)
{
    TMM->TMSTR |= _01_TMM_TM0_COUNT_START | _02_TMM_TM1_COUNT_START;
}

/**
  * @brief  This function stops TM0 and TM1 counter.
  * @param  None
  * @retval None
  */
void TMM_All_Stop(void)
{
    TMM->TMSTR &= (uint8_t)~_01_TMM_TM0_COUNT_START;	
    TMM->TMSTR &= (uint8_t)~_02_TMM_TM1_COUNT_START;	
}



/**
  * @brief  This function starts TMM0 counter.
  * @param  match_flag : it indicated whether counter TM0 continues count 
  *			after matching compare register TMGRA0
  *		@arg ENABLE: countine count after matching compare register TMGRA0
  *		@arg DISABLE: stop count after matching compare register TMGRA0
  * @retval None
  */
void TMM0_Start(FunctionalState match_flag)
{
    volatile uint8_t tmsr_dummy;

    tmsr_dummy = TMM->TMSR0; /* read TMSR0 before write 0 */
    TMM->TMSR0 = 0x00U; /* clear TM0 each interrupt request */
	if(match_flag == ENABLE)
	{
		TMM->TMSTR |= _04_TMM_TM0_COUNT_CONTINUES;
	}
	else
	{
		TMM->TMSTR |= _00_TMM_TM0_COUNT_STOP_MATCH;
	}
    TMM->TMSTR |= _01_TMM_TM0_COUNT_START;
}



/**
  * @brief  This function stops TMM0 counter.
  * @param  None
  * @retval None
  */
void TMM0_Stop(void)
{
    volatile uint8_t tmsr_dummy;

    TMM->TMSTR |= _04_TMM_TM0_COUNT_CONTINUES;
    TMM->TMSTR &= (uint8_t)~_01_TMM_TM0_COUNT_START;
    tmsr_dummy = TMM->TMSR0; /* read TMSR0 before write 0 */
    TMM->TMSR0 = 0x00U; /* clear TM0 each interrupt request */
}

/**
  * @brief  This function starts TMM1 counter.
  * @param  match_flag : it indicated whether counter TM0 continues count 
  *			after matching compare register TMGRA0
  *		@arg ENABLE: countine count after matching compare register TMGRA0
  *		@arg DISABLE: stop count after matching compare register TMGRA0
  */
void TMM1_Start(FunctionalState match_flag)
{
    volatile uint8_t tmsr_dummy;

    tmsr_dummy = TMM->TMSR1; /* read TMSR1 before write 0 */
    TMM->TMSR1 = 0x00U; /* clear TM1 each interrupt request */
	if(match_flag == ENABLE)
	{
		TMM->TMSTR |= _08_TMM_TM1_COUNT_CONTINUES;
	}
	else
	{
		TMM->TMSTR |= _00_TMM_TM1_COUNT_STOP_MATCH;
	}

    TMM->TMSTR |= _02_TMM_TM1_COUNT_START;
}

/**
  * @brief  This function stops TMM1 counter.
  * @param  None
  * @retval None
  */
void TMM1_Stop(void)
{
    volatile uint8_t tmsr_dummy;

    TMM->TMSTR |= _08_TMM_TM1_COUNT_CONTINUES;
    TMM->TMSTR &= (uint8_t)~_02_TMM_TM1_COUNT_START;
    tmsr_dummy = TMM->TMSR1; /* read TMSR1 before write 0 */
    TMM->TMSR1 = 0x00U; /* clear TM0 each interrupt request */
}

/**
  * @brief  Checks whether the specified TMM_FLAG flag is set or not.
  * @param  TMMx: TMM peripheral select.
  * @param  TMM_FLAG: specifies the flag to check.
  *          This parameter can be one of the following values,whrer x is 0 or 1:
  *            @arg TMM_FLAG_MATCHA: TMx count matchs with Capture Compare RegisterTMGRA0 
  *            @arg TMM_FLAG_MATCHB: TMx count matchs with Capture Compare RegisterTMGRB0 
  *            @arg TMM_FLAG_MATCHC: TMx count matchs with Capture Compare RegisterTMGRC0 
  *            @arg TMM_FLAG_MATCHD: TMx count matchs with Capture Compare RegisterTMGRD0 
  *            @arg TMM_FLAG_OVFLOW: TMx count overflow flag
  * @retval The new state of UART_FLAG (SET or RESET).
  */
FlagStatus TMM_GetFlagStatus(TMMSelect_TypeDef TMMx, uint8_t TMM_FLAG)
{
	FlagStatus bitstatus = RESET;
	/* Check the parameters */
	assert_param(IS_TMM_FLAG(TMM_FLAG));
	assert_param(IS_TMM_SELECT(TMMx));
	/* Check the parameters */
	if(TMMx == TMM0)
	{
		if ((TMM->TMSR0 & TMM_FLAG) != (uint16_t)RESET)
		{
			bitstatus = SET;
		}
		else
		{
			bitstatus = RESET;
		}
	}
	if(TMMx == TMM1)
	{
		if ((TMM->TMSR1 & TMM_FLAG) != (uint16_t)RESET)
		{
			bitstatus = SET;
		}
		else
		{
			bitstatus = RESET;
		}	
	}

	return bitstatus;
}


uint32_t TMM_GetPulseWidth(TMMSelect_TypeDef TMMx, uint8_t channel)
{
    uint8_t  tmsr_temp;
    uint8_t  tmier_temp;
	int32_t width=0;
	
	/* Check the parameters */	
	assert_param(IS_TMM_SELECT(TMMx));
	
	TMM_RegSetting(TMMx);
	
	tmsr_temp = *TMM_reg.TMSR;
	tmier_temp = *TMM_reg.TMIER[TMMx];
	
	if (INTC_GetPendingIRQ(TMM0_IRQn) == 1)
	{
		INTC_ClearPendingIRQ(TMM0_IRQn); /* clear INTTMA interrupt flag */
	}
	else if(INTC_GetPendingIRQ(TMM1_IRQn) == 1)
	{
		INTC_ClearPendingIRQ(TMM1_IRQn); /* clear INTTMA interrupt flag */
	}
	else 
	{
		return 0;	
	}

    if (TMM_GetFlagStatus(TMMx,TMM_FLAG_OVFLOW) == SET)
    {
		*TMM_reg.TMSR = tmsr_temp & ~TMM_FLAG_OVFLOW;
        tmm_overflow_count_a += 1UL;
        tmm_overflow_count_b += 1UL;
    }
	
	if(channel == TMM_Channel_A)
	{
		if (TMM_GetFlagStatus(TMMx,TMM_FLAG_MATCHA) == SET)
		{
			*TMM_reg.TMSR = tmsr_temp & ~TMM_FLAG_MATCHA;

			if (tmm_overflow_count_a == 0UL)
			{
				width = (*TMM_reg.TMGRA - *TMM_reg.TMGRC);
				tmm_overflow_count_a = 0;
				if(width < 0)
				{
					width += 0xffff;
				}
			}
			else
			{
				width = (*TMM_reg.TMGRA - *TMM_reg.TMGRC) + (0x10000UL * tmm_overflow_count_a);
				tmm_overflow_count_a = 0UL;
			}
		}
	}
	if(channel == TMM_Channel_B)
	{
		if (TMM_GetFlagStatus(TMMx,TMM_FLAG_MATCHB) == SET)
		{
			*TMM_reg.TMSR = tmsr_temp & ~TMM_FLAG_MATCHB;

			if (tmm_overflow_count_b == 0UL)
			{
				width = (*TMM_reg.TMGRB - *TMM_reg.TMGRD);
			}
			else
			{
				width = (*TMM_reg.TMGRB - *TMM_reg.TMGRD) + (0x10000UL * tmm_overflow_count_b);
				tmm_overflow_count_b = 0UL;
			}
		}
	}
	*TMM_reg.TMIER[TMMx] = tmier_temp;
	
	return width;
}
