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
#include "tima.h"
#include "cgc.h"




/**
  * @brief  This function starts TMA counter.
  * @param  None
  * @retval None
  */
void TMA_Start(void)
{
    TMA->TACR0 |= _01_TMA_COUNT_START;
}

/**
  * @brief  This function stops TMA counter.
  * @param  None
  * @retval None
  */
void TMA_Stop(void)
{
    TMA->TACR0 &= (uint8_t)~_01_TMA_COUNT_START;
    INTC_DisableIRQ(TMA_IRQn);/* disable INTTMA interrupt */
    INTC_ClearPendingIRQ(TMA_IRQn);/* clear INTTMA interrupt flag */
}

/**
  * @brief  Enables or disables the specified TIMER channel.
  * @param  TIMx: where x can be 0, 1, select TIEMR peripheral.
  * @param  Chx: where x can be 0, 1,2,3 select channel of  specified TIMER peripheral.
  * @param  NewState: new state of the TIEMR channel.
  *          This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
//void TIM_Cmd(TIM_t TIMx,uint8_t Chx,TIM_FunState NewState)
//{

//	if(NewState == Enable)
//		TIM_Start(TIMx,Chx);
//	else
//		TIM_Stop(TIMx,Chx);
//}

/**
  * @brief  Set the specified TIMER channel's count num.
  * @param  TIMx: where x can be 0, 1, select TIEMR peripheral.
  * @param  Chx: where x can be 0, 1,2,3 select channel of  specified TIMER peripheral.
  * @param  counter: the count num of the TIEMR channel.          
  * @retval None
  */
//void TIM_SetCounter(TIM_t TIMx,uint8_t Chx,uint16_t counter)
//{
//	uint8_t chanPos =0,chanNum=0,pos=0,currentChan=0;	
//	assert_param(IS_TIM(TIMx));	
//	assert_param(IS_TIM_CHANNEL(Chx));
//	
//	Tim_RegSetting(TIMx);
//	
//	for(chanPos = 0; chanPos < 8; chanPos++)
//	{
//		pos = ((uint8_t)0x01) << chanPos;
//		chanNum = (uint8_t)Chx & pos;
//		if(chanNum == pos)
//		{
//		     currentChan=chanPos;
//			*TIM_reg.TDR[currentChan] = counter -1;	
//			break;
//		}
//	}		
//}

/**
  * @brief  Get the specified TIMER channel's status.
  * @param  TIMx: where x can be 0, 1, select TIEMR peripheral.
  * @param  Chx: where x can be 0, 1,2,3 select channel of  specified TIMER peripheral.        
  * @retval bool: 1 it occurs overflow 
  *               0 it does not occur overflow
  */
//bool TIM_GetStatus(TIM_t TIMx,uint8_t Chx)
//{
//	uint8_t chanPos =0,chanNum=0,pos=0,currentChan=0;	
//	
//	assert_param(IS_TIM(TIMx));	
//	assert_param(IS_TIM_CHANNEL(Chx));
//	
//	Tim_RegSetting(TIMx);	
//	for(chanPos = 0; chanPos < 8; chanPos++)
//	{
//		pos = ((uint8_t)0x01) << chanPos;
//		chanNum = (uint8_t)Chx & pos;
//		if(chanNum == pos)
//		{
//		    currentChan = chanPos;
//			break;
//		}
//	}
//	return (bool)(*TIM_reg.TSR[currentChan] & 0x0001);
//}

/**
  * @brief  Initializes the TIEMRA peripheral according to the specified
  *         parameters in the TIMA_InitStruct .
  * @param  TIMA_InitStruct: pointer to a TMA_InitTypeDef structure that contains
  *         the configuration information for the specified TIEMRA peripheral.
  * @retval none
  */
void TMA_Init(TMA_InitTypeDef *TIMA_InitStruct)
{
	CGC_PER1PeriphClockCmd(CGC_PER1Periph_TMA,ENABLE);
	
	TMA->TACR0 &= (uint8_t)~0x01;	//TMA STOP
    INTC_DisableIRQ(TMA_IRQn);           /* disable INTTMA interrupt */
    INTC_ClearPendingIRQ(TMA_IRQn);      /* clear INTTMA interrupt flag */
	
	TMA->TAMR0 = TIMA_InitStruct->TMA_Clk | TIMA_InitStruct->TMA_Mode;

	if(TIMA_InitStruct->TMA_Mode == TMA_Mode_Square)
	{
		TMA->TAIOC0 = TIMA_InitStruct->TMA_Polarity | 0x04; //0x00:ban TAO OUTPUT
	}
	else if(TIMA_InitStruct->TMA_Mode == TMA_Mode_PluseWidth || TIMA_InitStruct->TMA_Mode == TMA_Mode_PlusePeriod)
	{
		if(TIMA_InitStruct->TMA_Pulse ==TMA_Pulse_Width_Low || TIMA_InitStruct->TMA_Pulse ==TMA_Pulse_Rising)
			TMA->TAIOC0 = TMA_Polarity_0;
		if(TIMA_InitStruct->TMA_Pulse ==TMA_Pulse_Width_High ||TIMA_InitStruct->TMA_Pulse ==TMA_Pulse_Falling )
			TMA->TAIOC0 = TMA_Polarity_1;
	}
	else if(TIMA_InitStruct->TMA_Mode == TMA_Mode_EventCount)
	{
		assert_param(IS_TMA_EDGE(TIMA_InitStruct->TMA_EventPara.TIMA_Edge));
		assert_param(IS_TMA_CTRL(TIMA_InitStruct->TMA_EventPara.TMA_Ctrl));

		if(TIMA_InitStruct->TMA_EventPara.TMA_Ctrl & 0x000f)
		{
			TMA->TAIOC0 = 0x40;		
		}
		else if(TIMA_InitStruct->TMA_EventPara.TMA_Ctrl & 0x0ff0)
		{
			TMA->TAIOC0 = 0x80;			
		}
		else
		{
			TMA->TAIOC0 = 0x00;			
		}
		
		if(TIMA_InitStruct->TMA_EventPara.TMA_Ctrl & TMA_INTP4_LOW_LEVEL)
		{
			TMA->TAISR0 = _00_TMA_PWM_POLARITY_L;
		}
		if((TIMA_InitStruct->TMA_EventPara.TMA_Ctrl & TMM_TMIOD1_LOW_LEVEL)>>4)
		{
			TMA->TAISR0 = _00_TMA_PWM_POLARITY_L | _00_TMA_PWM_TMIOD1;
		}
		if((TIMA_InitStruct->TMA_EventPara.TMA_Ctrl & TMM_TMIOC1_LOW_LEVEL)>>6)
		{
			TMA->TAISR0 = _00_TMA_PWM_POLARITY_L | _01_TMA_PWM_TMIOC1;
		}
		if((TIMA_InitStruct->TMA_EventPara.TMA_Ctrl & TIM_TO02_LOW_LEVEL)>>8)
		{
			TMA->TAISR0 = _00_TMA_PWM_POLARITY_L | _02_TMA_PWM_TO02;
		}			
		if((TIMA_InitStruct->TMA_EventPara.TMA_Ctrl & TIM_TO03_LOW_LEVEL)>>10)
		{
			TMA->TAISR0 = _00_TMA_PWM_POLARITY_L | _03_TMA_PWM_TO03;
		}
		
		if((TIMA_InitStruct->TMA_EventPara.TMA_Ctrl & TMA_INTP4_HIGH_LEVEL)>>1)
		{
			TMA->TAISR0 = _04_TMA_PWM_POLARITY_H;		
		}		
		if((TIMA_InitStruct->TMA_EventPara.TMA_Ctrl & TMM_TMIOD1_HIGH_LEVEL)>>5)
		{
			TMA->TAISR0 = _04_TMA_PWM_POLARITY_H | _00_TMA_PWM_TMIOD1;
		}
		if((TIMA_InitStruct->TMA_EventPara.TMA_Ctrl & TMM_TMIOC1_HIGH_LEVEL)>>7)
		{
			TMA->TAISR0 = _04_TMA_PWM_POLARITY_H | _01_TMA_PWM_TMIOC1;
		}
		if((TIMA_InitStruct->TMA_EventPara.TMA_Ctrl & TIM_TO02_HIGH_LEVEL)>>9)
		{
			TMA->TAISR0 = _04_TMA_PWM_POLARITY_H | _02_TMA_PWM_TO02;
		}			
		if((TIMA_InitStruct->TMA_EventPara.TMA_Ctrl & TIM_TO03_HIGH_LEVEL)>>11)
		{
			TMA->TAISR0 = _04_TMA_PWM_POLARITY_H | _03_TMA_PWM_TO03;
		}
		
		if(TIMA_InitStruct->TMA_EventPara.TIMA_Edge == TMA_RISING_EDGE)
		{
			TMA->TAMR0 |= _00_TMA_TAIO_POLARITY_ONE;
			TMA->TAIOC0 |= 	TMA_Polarity_0;
		}
		else if(TIMA_InitStruct->TMA_EventPara.TIMA_Edge == TMA_FALLING_EDGE)
		{
			TMA->TAMR0 |= _00_TMA_TAIO_POLARITY_ONE;
			TMA->TAIOC0 |= 	TMA_Polarity_1;
		}
		else
		{
			TMA->TAMR0 |= _08_TMA_TAIO_POLARITY_BOTH;
		}
	}
	if(TIMA_InitStruct->TMA_Clk == TMA_FIL)
	{
        CGC->OSMC = 0x10U;  /* WUTMMCK0 = 1 */	
	}
    TMA->TA0 = TIMA_InitStruct->TMA_Period - 1;	
}


uint32_t TMA_GetPulseWidth(void)
{
	uint32_t width, period;
	static uint32_t tempwidth = 0xffff,tma_underflow_count=0;

	while(INTC_GetPendingIRQ(TMA_IRQn) == 0);
	INTC_ClearPendingIRQ(TMA_IRQn); /* clear INTTMA interrupt flag */

	if ((TMA->TACR0 & _20_TMA_UNDERFLOW_OCCUR) != 0U)
	{
		TMA->TACR0 &= (uint8_t)~_20_TMA_UNDERFLOW_OCCUR;
		tma_underflow_count += 1U;
	}		
	if((TMA->TACR0 & _10_TMA_ACTIVE_EDGE_RECEIVED)) //active edge ocuurs
	{
		TMA->TACR0 &= ~_10_TMA_ACTIVE_EDGE_RECEIVED;
		if (tma_underflow_count == 0U)
		{
			width = tempwidth - TMA->TA0;
			tempwidth = TMA->TA0;
		}
		else
		{
			width = tempwidth + (0xffff + 1) * tma_underflow_count - TMA->TA0;
			tempwidth = TMA->TA0;
			tma_underflow_count = 0;
		}	
	}
	return width;
}


uint32_t TMA_GetPulsePeriod(void)
{
	uint32_t width=0, tmp;
	static uint32_t tempwidth = 0xffff,tma_underflow_count=0;
	static uint8_t time =0;
	
	while(INTC_GetPendingIRQ(TMA_IRQn) == 0);
    INTC_ClearPendingIRQ(TMA_IRQn); /* clear INTTMA interrupt flag */
	
    if ((TMA->TACR0 & _20_TMA_UNDERFLOW_OCCUR) != 0U)
    {
        TMA->TACR0 &= (uint8_t)~_20_TMA_UNDERFLOW_OCCUR;
        tma_underflow_count += 1U;
    }	
	if((TMA->TACR0 & _10_TMA_ACTIVE_EDGE_RECEIVED)) // active edge ocuurs
	{
		time++;
		TMA->TACR0 &= ~_10_TMA_ACTIVE_EDGE_RECEIVED;

		if(time == 1)
		{
			tmp = TMA->TA0;    // dummy read to update readbuf	
		}
		else
		{
			if(tma_underflow_count == 0)
			{
				width = tempwidth - TMA->TA0;
			}
			else
			{
				 width = tempwidth - TMA->TA0 + 1U + (tempwidth + 1) * tma_underflow_count;
			}

			time = 0;			
		}
	}

	return width;
}