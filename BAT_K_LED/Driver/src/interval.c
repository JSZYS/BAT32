#include "interval.h"
#include "cgc.h"


/**
  * @brief  This function initializes the interval module.
  * @param  IT_InitStruct: pointer to a IT_InitTypeDef structure that contains
  *         the configuration information for the specified interval peripheral.
  * @retval None
  */
void IT_Init(IT_InitTypeDef *IT_InitStruct)
{
	assert_param(IS_COUNTER_RANGER(IT_InitStruct->IT_Counter));
	assert_param(IS_IT_CLKSRC(IT_InitStruct->IT_ClkSource));

	CGC_PER0PeriphClockCmd(CGC_PER0Periph_RTC,ENABLE);
	
	switch(IT_InitStruct->IT_ClkSource)
	{
		case IT_FIL :
			CGC->OSMC  |= 1<<4;	
			MISC->RTCCL = 0x01;
			break;
		case IT_FSUB:
			MISC->RTCCL = 0x00;			
			break;
		case IT_FHOCO:
			MISC->RTCCL = 0x80;		
			MISC->RTCCL |= IT_InitStruct->IT_Freqscaler;		
			break;
		case IT_FMX:
			MISC->RTCCL = 0x00;
			MISC->RTCCL |= IT_InitStruct->IT_Freqscaler;		
			break;
		default:
			break;
	}

	RTC->ITMC = 0x0000;				// OPERATION_DISABLE
    INTC_DisableIRQ(IT_IRQn);       // disable INTIT interrupt
    INTC_ClearPendingIRQ(IT_IRQn);  // clear INTIT interrupt flag 
    RTC->ITMC = IT_InitStruct->IT_Counter;
}

/**
  * @brief  This function enables IntervalTimer count operation and clears interrupt flag.
  * @param  None
  * @retval None
  */
void IT_Start(void)
{
    INTC_ClearPendingIRQ(IT_IRQn); // clear INTIT interrupt flag 
    NVIC_ClearPendingIRQ(IT_IRQn); // clear INTIT interrupt flag 
    INTC_EnableIRQ(IT_IRQn);       // enable INTIT interrupt 
    RTC->ITMC |= 0x8000;           //OPERATION_ENABLE
	
    /* confirm the IntervalTimer is running */	
    while((RTC->ITMC & 0x8000) == 0U);  /* wait ITMC.RINTE == 1 */
}

/**
  * @brief  This function disables IntervalTimer count operation and clears interrupt flag.
  * @param  None
  * @retval None
  */
void IT_Stop(void)
{
    INTC_DisableIRQ(IT_IRQn);      // disable INTIT interrupt
    INTC_ClearPendingIRQ(IT_IRQn); // clear INTIT interrupt flag 
    NVIC_ClearPendingIRQ(IT_IRQn); // clear INTIT interrupt flag 
    RTC->ITMC &= (uint16_t)~0x8000;
}
