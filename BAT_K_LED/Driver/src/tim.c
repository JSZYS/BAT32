/*******************************************************************************
*                  COPYRIGHT (C) 2021 CMS Technologies Ltd.                    *
*                                                                              *
********************************************************************************
* FileName      : tim.c                                                  *
* Author        :                                                              *
* Version       : 1.0                                                          *
* Date          : 2021.08.13                                                   *
* Description   :                                                              *
* Function List :                                                              *
********************************************************************************/
#include "tim.h"
#include "cgc.h"
TIM_REG_t TIM_reg;
/* Private variables ---------------------------------------------------------*/

#if defined(BAT32G1XX_80PIN) || defined(BAT32G1XX_100PIN)
#define TIM_CHAN_MAX_NUM    8u
#define TIM_MAX_NUM    		2u

static IRQn_Type TIM_IRQTable[TIM_MAX_NUM][TIM_CHAN_MAX_NUM] = 
{{TM00_IRQn, TM01_IRQn, TM02_IRQn, TM03_IRQn,0,0,0,0},
{TM10_IRQn, TM11_IRQn, TM12_IRQn, TM13_IRQn,TM14_IRQn,TM15_IRQn,TM16_IRQn,TM17_IRQn},
};
#else
#define TIM_CHAN_MAX_NUM    4u
#define TIM_MAX_NUM    		1u

static IRQn_Type TIM_IRQTable[TIM_MAX_NUM][TIM_CHAN_MAX_NUM] = 
{{TM00_IRQn, TM01_IRQn, TM02_IRQn, TM03_IRQn}};
#endif
/**
  * @brief  the specified TIMER peripheral regiseter is mapped to specified TIM reg.
  * @param  TIMx: it contains two timer units 
  *            @arg TIM40 the specified timer unit TIM40
  *            @arg TIM41 the specified timer unit TIM40
  * @retval None
  */
void Tim_RegSetting(TIMSelect_TypeDef TIMx)
{
	switch (TIMx)
	{
		case TIM40:
			TIM_reg.TCR[0] = &TM40->TCR00;
			TIM_reg.TCR[1] = &TM40->TCR01;
			TIM_reg.TCR[2] = &TM40->TCR02;
			TIM_reg.TCR[3] = &TM40->TCR03;

			TIM_reg.TMR[0]  = &TM40->TMR00;
			TIM_reg.TMR[1]  = &TM40->TMR01;
			TIM_reg.TMR[2]  = &TM40->TMR02;
			TIM_reg.TMR[3]  = &TM40->TMR03;

			TIM_reg.TSR[0]  = &TM40->TSR00;
			TIM_reg.TSR[1]  = &TM40->TSR01;
			TIM_reg.TSR[2]  = &TM40->TSR02;
			TIM_reg.TSR[3]  = &TM40->TSR03;

			TIM_reg.TE  = &TM40->TE0;
			TIM_reg.TS  = &TM40->TS0;
			TIM_reg.TT  = &TM40->TT0;
			TIM_reg.TPS  = &TM40->TPS0;
			
			TIM_reg.TO  = &TM40->TO0;
			TIM_reg.TOE  = &TM40->TOE0;
			TIM_reg.TOL  = &TM40->TOL0;
			TIM_reg.TOM  = &TM40->TOM0;
			
			TIM_reg.TDR[0]  = &TM40->TDR00;
			TIM_reg.TDR[1]  = &TM40->TDR01;
			TIM_reg.TDR[2]  = &TM40->TDR02;
			TIM_reg.TDR[3]  = &TM40->TDR03;

//			TIM_reg.TDR1L  = &TM40->TDR01L;
//			TIM_reg.TDR1H  = &TM40->TDR01H;
//			TIM_reg.TDR3L  = &TM40->TDR03L;
//			TIM_reg.TDR3H  = &TM40->TDR03H;
			break;
#if defined(BAT32G1XX_80PIN) || defined(BAT32G1XX_100PIN)
		case TIM81:
			TIM_reg.TCR[0] = &TM81->TCR10;
			TIM_reg.TCR[1] = &TM81->TCR11;
			TIM_reg.TCR[2] = &TM81->TCR12;
			TIM_reg.TCR[3] = &TM81->TCR13;
			TIM_reg.TCR[4] = &TM81->TCR14;
			TIM_reg.TCR[5] = &TM81->TCR15;
			TIM_reg.TCR[6] = &TM81->TCR16;
			TIM_reg.TCR[7] = &TM81->TCR17;
		
			TIM_reg.TMR[0]  = &TM81->TMR10;
			TIM_reg.TMR[1]  = &TM81->TMR11;
			TIM_reg.TMR[2]  = &TM81->TMR12;
			TIM_reg.TMR[3]  = &TM81->TMR13;
			TIM_reg.TMR[4]  = &TM81->TMR14;
			TIM_reg.TMR[5]  = &TM81->TMR15;
			TIM_reg.TMR[6]  = &TM81->TMR16;
			TIM_reg.TMR[7]  = &TM81->TMR17;
		
			TIM_reg.TSR[0]  = &TM81->TSR10;
			TIM_reg.TSR[1]  = &TM81->TSR11;
			TIM_reg.TSR[2]  = &TM81->TSR12;
			TIM_reg.TSR[3]  = &TM81->TSR13;
			TIM_reg.TSR[4]  = &TM81->TSR14;
			TIM_reg.TSR[5]  = &TM81->TSR15;
			TIM_reg.TSR[6]  = &TM81->TSR16;
			TIM_reg.TSR[7]  = &TM81->TSR17;
			
			TIM_reg.TE  = &TM81->TE1;
			TIM_reg.TS  = &TM81->TS1;
			TIM_reg.TT  = &TM81->TT1;
			TIM_reg.TPS  = &TM81->TPS1;
			
			TIM_reg.TO  =  &TM81->TO1;
			TIM_reg.TOE  = &TM81->TOE1;
			TIM_reg.TOL  = &TM81->TOL1;
			TIM_reg.TOM  = &TM81->TOM1;
			
			TIM_reg.TDR[0]  = &TM81->TDR10;
			TIM_reg.TDR[1]  = &TM81->TDR11;
			TIM_reg.TDR[2]  = &TM81->TDR12;
			TIM_reg.TDR[3]  = &TM81->TDR13;
			TIM_reg.TDR[4]  = &TM81->TDR14;
			TIM_reg.TDR[5]  = &TM81->TDR15;
			TIM_reg.TDR[6]  = &TM81->TDR16;
			TIM_reg.TDR[7]  = &TM81->TDR17;
			break;
#endif
		default:
			break;
	}
}

/**
  * @brief  Enables the specified TIMER channel.
  * @param  TIMx: the specified tim, it can be TIM40 TIM41
  * @param  Chx : it is the specified channel of TIMER it is as follows
  *         @arg TTM_Channel_0
  *         @arg TTM_Channel_1
  *         @arg TTM_Channel_2
  *         @arg TTM_Channel_3
  * @retval None
  */
void TIM_Start(TIMSelect_TypeDef TIMx, uint8_t Chx)
{
	assert_param(IS_TIM(TIMx));	
	assert_param(IS_TIM_CHANNEL(Chx));
	
	Tim_RegSetting(TIMx);
	*TIM_reg.TS |= Chx;
}


/**
  * @brief  Disable the specified TIMER channel count.
  * @param  tim: the specified tim, it can be TIM40 TIM41
  * @param  ch : it is the specified channel of TIMER it is as follows
  *         @arg TTM_Channel_0
  *         @arg TTM_Channel_1
  *         @arg TTM_Channel_2
  *         @arg TTM_Channel_3
  * @retval None
  */
void TIM_Stop(TIMSelect_TypeDef TIMx, uint8_t Chx)
{
	assert_param(IS_TIM(TIMx));	
	assert_param(IS_TIM_CHANNEL(Chx));
	
	Tim_RegSetting(TIMx);
	*TIM_reg.TT |= Chx;
}

/**
  * @brief  Enables or disables the specified TIMER channel.
  * @param  TIMx: where x can be 0, 1, select TIEMR peripheral.
  * @param  Chx: where x can be 0, 1,2,3 select channel of  specified TIMER peripheral.
  * @param  NewState: new state of the TIEMR channel.
  *          This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
void TIM_Cmd(TIMSelect_TypeDef TIMx,uint8_t Chx,TIM_FunState NewState)
{

	if(NewState == Enable)
		TIM_Start(TIMx,Chx);
	else
		TIM_Stop(TIMx,Chx);
}

/**
  * @brief  Set the specified TIMER channel's count num.
  * @param  TIMx: where x can be 0, 1, select TIEMR peripheral.
  * @param  Chx: where x can be 0, 1,2,3 select channel of  specified TIMER peripheral.
  * 	@arg TTM_Channel_0、TTM_Channel_1、TTM_Channel_2、TTM_Channel_3
  * @param  counter: the count num of the TIEMR channel.          
  * @retval None
  */
void TIM_SetCounter(TIMSelect_TypeDef TIMx,uint8_t Chx,uint16_t counter)
{
	uint8_t chanPos =0,chanNum=0,pos=0,currentChan=0;	
	assert_param(IS_TIM(TIMx));	
	assert_param(IS_TIM_CHANNEL(Chx));
	
	Tim_RegSetting(TIMx);
	
	for(chanPos = 0; chanPos < 8; chanPos++)
	{
		pos = ((uint8_t)0x01) << chanPos;
		chanNum = (uint8_t)Chx & pos;
		if(chanNum == pos)
		{
		     currentChan=chanPos;
			*TIM_reg.TDR[currentChan] = counter;	
			break;
		}
	}		
}

/**
  * @brief  Get the specified TIMER channel's status.
  * @param  TIMx: where x can be 0, 1, select TIEMR peripheral.
  * @param  Chx: where x can be 0, 1,2,3 select channel of  specified TIMER peripheral.        
  * @retval bool: 1 it occurs overflow 
  *               0 it does not occur overflow
  */
bool TIM_GetStatus(TIMSelect_TypeDef TIMx,uint8_t Chx)
{
	uint8_t chanPos =0,chanNum=0,pos=0,currentChan=0;	
	
	assert_param(IS_TIM(TIMx));	
	assert_param(IS_TIM_CHANNEL(Chx));
	
	Tim_RegSetting(TIMx);	
	for(chanPos = 0; chanPos < 8; chanPos++)
	{
		pos = ((uint8_t)0x01) << chanPos;
		chanNum = (uint8_t)Chx & pos;
		if(chanNum == pos)
		{
		    currentChan = chanPos;
			break;
		}
	}
	return (bool)(*TIM_reg.TSR[currentChan] & 0x0001);
}

/**
  * @brief  Initializes the TIEMR peripheral according to the specified
  *         parameters in the TIM_InitStruct .
  * @param  TIM_InitStruct: pointer to a TIM_InitTypeDef structure that contains
  *         the configuration information for the specified TIEMR peripheral.
  * @retval initial result
  */
int TIM_Init(TIM_InitTypeDef *TIM_InitStruct)
{
	int ret = TIM_SUCC;
	uint8_t i=0,j=0;
	uint8_t chanPos =0,chanNum=0,pos=0,currentChan[8]={0};
	uint8_t masterPos =0,masterNum=0,posm=0,currentMChan[4]={0};
	
	assert_param(IS_TIM(TIM_InitStruct->TIM));	
	assert_param(IS_TIM_CHANNEL(TIM_InitStruct->TIM_Channel));
	
	if(TIM_InitStruct->TIM == TIM40)
	{
		CGC_PER0PeriphClockCmd(CGC_PER0Periph_TIM40,ENABLE);
	}
#if defined(BAT32G1XX_80PIN) || defined(BAT32G1XX_100PIN)
	if(TIM_InitStruct->TIM == TIM81)
	{
		CGC_PER2PeriphClockCmd(CGC_PER2Periph_TM81,ENABLE);
	}
#endif
	Tim_RegSetting(TIM_InitStruct->TIM);
	
	*TIM_reg.TPS = TIM_InitStruct->TIM_ClkDivision;	
	if(1==IS_TTM_Master_Chan(TIM_InitStruct->TIM_Selection_Master))//multi-tim combination
	{
		for(masterPos = 0; masterPos < 8; masterPos++)
		{
			posm = ((char)0x01) << masterPos;
			masterNum = TIM_InitStruct->TIM_Selection_Master & posm;
			if(masterNum == posm)
			{
				currentMChan[j]=masterPos;
	
				if(masterPos == 0)
				{
					*TIM_reg.TMR[currentMChan[j]] = _0000_TM_CLOCK_SELECT_CKM0 | TIM_InitStruct->TIM_Trigger | TIM_InitStruct->TIM_Pulse_Edge | TIM_InitStruct->TIM_Mode | TIM_InitStruct->TIM_StartInt;//								
				}
				else
				{
					*TIM_reg.TMR[masterPos] = _0000_TM_CLOCK_SELECT_CKM0 |0x0800|TIM_InitStruct->TIM_Trigger | TIM_InitStruct->TIM_Pulse_Edge | TIM_InitStruct->TIM_Mode | TIM_InitStruct->TIM_StartInt;								
				}
				*TIM_reg.TDR[currentMChan[j]] = TIM_InitStruct->TIM_Period[currentMChan[j]] -1;				 
				*TIM_reg.TO &= ~(1 << masterPos);
				*TIM_reg.TOE &= ~(1 << masterPos);	
				MISC->NFEN1 |=  (1<<currentMChan[j]);	 //后加			
				j++;
			}			
		}
		for(chanPos = 0; chanPos < 8; chanPos++)   //从属通道配置
		{
			pos = ((uint8_t)0x01) << chanPos;
			chanNum = TIM_InitStruct->TIM_Channel & pos;
			if(chanNum == pos)
			{
			    *TIM_reg.TMR[chanPos] = _0000_TM_CLOCK_SELECT_CKM0 |TIM_Trigger_UseMaster_Int |TIM_Mode_PWM_Slave| TIM_InitStruct->TIM_Mode |TIM_InitStruct->TIM_StartInt;// | TIM_InitStruct->TIM_Mode 
				*TIM_reg.TDR[chanPos] = TIM_InitStruct->TIM_Period[chanPos];
				*TIM_reg.TO &= ~(1 << chanPos);
				*TIM_reg.TOE |= (1 << chanPos);
				*TIM_reg.TOM |= (1 << chanPos);
			}
		}
		*TIM_reg.TS |= TIM_InitStruct->TIM_Channel |TIM_InitStruct->TIM_Selection_Master;   //TIM Start
		*TIM_reg.TOL|= TIM_InitStruct->TIM_Slave_Polarity;

	}
	else
	{
		for(chanPos = 0; chanPos < 8; chanPos++)
		{
			pos = ((uint8_t)0x01) << chanPos;
			chanNum = TIM_InitStruct->TIM_Channel & pos;
			if(chanNum == pos)
			{
			  currentChan[i]=chanPos;
			  i++;
			}
		}
		if(i == 1)
		{
			*TIM_reg.TDR[currentChan[0]] = TIM_InitStruct->TIM_Period[currentChan[0]] -1;
			*TIM_reg.TT |=  TIM_InitStruct->TIM_Channel;   //TIM stop

			*TIM_reg.TMR[currentChan[0]] = _0000_TM_CLOCK_SELECT_CKM0 | TIM_InitStruct->TIM_Trigger | TIM_InitStruct->TIM_Pulse_Edge | TIM_InitStruct->TIM_Mode | TIM_InitStruct->TIM_StartInt;

			*TIM_reg.TO &= ~ TIM_InitStruct->TIM_Channel;
			if(TIM_InitStruct->TIM_Mode == 0x0000)    //tim output
			{
				*TIM_reg.TOE |= TIM_InitStruct->TIM_Channel;
			}
			else   //input event for tim 
			{
				*TIM_reg.TOE &= ~ TIM_InitStruct->TIM_Channel;
				if(TIM_InitStruct->TIM_Mode == TIM_Mode_EventCount)
				{
					*TIM_reg.TMR[currentChan[0]] |= 0x1000;		
				}
				if(TIM_InitStruct->TIM == TIM40)
				{
					 MISC->NFEN1 |=  TIM_InitStruct->TIM_Channel;
				}
#if defined(BAT32G1XX_80PIN) || defined(BAT32G1XX_100PIN)
				if(TIM_InitStruct->TIM == TIM81)
				{
					 MISC->NFEN2 |=  TIM_InitStruct->TIM_Channel;
				}	
#endif						
				if((TIM_InitStruct->TIM_Channel == 0x01 || TIM_InitStruct->TIM_Channel == 0x02) && (TIM_InitStruct->TIM == TIM40))
				{
					MISC->TIOS0 |=  TIM_InitStruct->TIM4_Input;
					if(TIM_InitStruct->TIM4_Input)
					{
						MISC->NFEN1 &=  ~(TIM_InitStruct->TIM_Channel);					
					}	
				}		
			}				
			*TIM_reg.TS |=  TIM_InitStruct->TIM_Channel;   //TIM Start
		}
		else
		{
			return TIM_ERR;
		}
	}
	
	return ret;
}

/**
  * @brief  The Timer operates as pulse width measure.
  * @param  tim - choose timer unit it can choose TIM40 TIM41
  * @param  channel - choose TTM_Channel_0 ~ TTM_Channel_3    
  * @retval period - return the pulse period at specified edge
  */
uint32_t TIM_GetPulsePeriod(TIMSelect_TypeDef tim, uint8_t channel)
{
	assert_param(IS_TIM_CHANNEL(channel));
	assert_param(IS_TIM(tim));
	
	uint32_t width = 0;
	uint8_t chanpos =0;
	static uint8_t g_Int =0;
	
	if(channel == TTM_Channel_0)
	{
		chanpos = 0;
	}
	else if(channel == TTM_Channel_1)
	{
		chanpos = 1;	
	}
	else if(channel == TTM_Channel_2)
	{
		chanpos = 2;	
	}
	else if(channel == TTM_Channel_3)
	{
		chanpos = 3;
	}
	else if(channel == TTM_Channel_4)
	{
		chanpos = 4;
	}
	else if(channel == TTM_Channel_5)
	{
		chanpos = 5;
	}
	else if(channel == TTM_Channel_6)
	{
		chanpos = 6;
	}
	else if(channel == TTM_Channel_7)
	{
		chanpos = 7;
	}	
	Tim_RegSetting(tim);

	if(INTC_GetPendingIRQ(TIM_IRQTable[tim][chanpos]) == 1)
	{
		INTC_ClearPendingIRQ(TIM_IRQTable[tim][chanpos]); /* clear INTTMA interrupt flag */
	}
	else
	{
		return (width);
	}
    g_Int++;

	if(g_Int == 2)
	{
		if (1U == (*TIM_reg.TSR[chanpos] & _0001_TM4_OVERFLOW_OCCURS))
		{
			width= (uint32_t)(*TIM_reg.TDR[chanpos] + 1UL) + 0x10000UL;
		}
		else
		{
			width= (uint32_t)(*TIM_reg.TDR[chanpos] + 1UL);
		}
		g_Int = 0;
	}
    return (width);
}

/**
  * @brief  The Timer operates as pulse width measure.
  * @param  tim - choose timer unit it can choose TIM40 TIM41
  * @param  channel - choose TTM_Channel_0 ~ TTM_Channel_3    
  * @retval pulse width - return the width of pulse
  */
uint32_t TIM_GetPulseWidth(TIMSelect_TypeDef tim, uint8_t channel)
{
	assert_param(IS_TIM_CHANNEL(channel));
	assert_param(IS_TIM(tim));
	
	uint32_t width = 0;
	uint8_t chanpos =0;
	static uint8_t g_Int =0;
	
	if(channel == TTM_Channel_0)
	{
		chanpos = 0;
	}
	else if(channel == TTM_Channel_1)
	{
		chanpos = 1;	
	}
	else if(channel == TTM_Channel_2)
	{
		chanpos = 2;	
	}
	else if(channel == TTM_Channel_3)
	{
		chanpos = 3;
	}
	else if(channel == TTM_Channel_4)
	{
		chanpos = 4;
	}
	else if(channel == TTM_Channel_5)
	{
		chanpos = 5;
	}
	else if(channel == TTM_Channel_6)
	{
		chanpos = 6;
	}
	else if(channel == TTM_Channel_7)
	{
		chanpos = 7;
	}	
	Tim_RegSetting(tim);

	if(INTC_GetPendingIRQ(TIM_IRQTable[tim][chanpos]) == 1)
	{
		INTC_ClearPendingIRQ(TIM_IRQTable[tim][chanpos]); /* clear INTTM interrupt flag */
	}
	else
	{
		return (width);
	}
    g_Int++;

	if(g_Int == 1)
	{
		if (1U == (*TIM_reg.TSR[chanpos] & _0001_TM4_OVERFLOW_OCCURS))
		{
			width= (uint32_t)(*TIM_reg.TDR[chanpos] + 1UL) + 0x10000UL;
		}
		else
		{
			width= (uint32_t)(*TIM_reg.TDR[chanpos] + 1UL);
		}
		g_Int = 0;
	}
    return (width);
}
