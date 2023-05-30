#include "epwm.h"
#include "cgc.h"

/**
  * @brief  find real positon of channel  .
  * @param  channel: epwm channel
  * @retval channel positon 
  */
uint8_t EPWM_GetNum(uint8_t channel)
{
	uint8_t chanpos = 0x00, pos = 0x00 , currentchan = 0x00;
	assert_param(IS_EPWM_CHAN(channel));
	for (chanpos = 0x00; chanpos < 0x08; chanpos++)
	{
		pos = (0x01) << chanpos;
		/* Get the port pins position */
		currentchan = channel & pos;

		if (currentchan == pos)
		{
			return chanpos;
		}
	}
	return chanpos;		
}

/**
  * @brief  Initializes the EPWM peripheral according to the specified
  *         parameters in the EPWM_InitStruct .
  * @param  EPWM_InitStruct: pointer to a EPWM_InitTypeDef structure that contains
  *         the configuration information for the specified EPWM peripheral.
  * @retval initial result
  */
void EPWM_Init(EPWM_InitTypeDef *EPWM_InitStruct)
{
	assert_param(IS_EPWM_CHAN(EPWM_InitStruct->EPWM_Channel));
	
	CGC_PER1PeriphClockCmd(CGC_PER1Periph_EPWM,ENABLE);
	
	EPWM->EPWMSRC |=  EPWM_InitStruct->EPWM_Source;
	
	EPWM->EPWMCTL |= (EPWM_InitStruct->EPWM_Channel )|(EPWM_InitStruct->EPWM_OutPhase <<8);
	
	EPWM->EPWMSTC |= EPWM_InitStruct->EPWM_CutoffPara.Source | EPWM_InitStruct->EPWM_CutoffPara.Edge | EPWM_InitStruct->EPWM_CutoffPara.ReleaseMode | \
					 EPWM_InitStruct->EPWM_CutoffPara.ReleaseTiming;
	
	EPWM->EPWMSTC |= EPWM_InitStruct->EPWM_Cutoff_Outlv;
}
