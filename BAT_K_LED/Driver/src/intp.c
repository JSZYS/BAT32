#include "intp.h"

/**
  * @brief  This function initializes the external interrupt module.
  * @param  Intp_InitStruct: pointer to a INTP_InitTypeDef structure that contains
  *         the configuration information for the specified INTP peripheral.
  * @retval None
  */
void INTP_Init(INTP_InitTypeDef* Intp_InitStruct)
{
	volatile uint8_t w_count;
	uint16_t pinpos = 0x00,pinnum=0x00, pos = 0x00, currentpin = 0x00;

    /* Check the parameters */
    assert_param(IS_INTP_TYPE(Intp_InitStruct->INTP_Select));
	for(pinpos = 0x00; pinpos < 0x08; pinpos++)
	{
		pos = ((uint16_t)0x01) << pinpos;
		pinnum = Intp_InitStruct->INTP_Select & pos;
		if(pinnum == pos)
		{
			currentpin = pinpos;
		}
	}
	if(Intp_InitStruct->INTP_Select < INTP8)
	{
		INTM->EGN0 &= ~Intp_InitStruct->INTP_Select;
		INTM->EGP0 &= ~Intp_InitStruct->INTP_Select;
		
		INTM->EGN0 |= ((Intp_InitStruct->EXTI_Trigger & 0x01U) >> 0) << currentpin;
		INTM->EGP0 |= ((Intp_InitStruct->EXTI_Trigger & 0x02U) >> 1) << currentpin;	
	}
	else
	{
		INTM->EGN1 &= ~Intp_InitStruct->INTP_Select;
		INTM->EGP1 &= ~Intp_InitStruct->INTP_Select;
		
		INTM->EGN1 |= ((Intp_InitStruct->EXTI_Trigger & 0x01U) >> 0) << (currentpin-8);
		INTM->EGP1 |= ((Intp_InitStruct->EXTI_Trigger & 0x02U) >> 1) << (currentpin-8);	
	
	}
    for (w_count = 0U; w_count <= KEY_WAITTIME; w_count++)
    {   
        __NOP();
    }	
}

/**
  * @brief  This function clears INTP interrupt flag and enables interrupt.
  * @param  pinMsk:select the active external interrupt pins
  * 	@arg  INTP0/INTP1/INTP2/INTP3
  * @retval None
  */
void INTP_Start(uint16_t pinMsk)
{
    if(pinMsk & (1 << 0))
    {
        INTC_ClearPendingIRQ(INTP0_IRQn); /* clear INTP interrupt flag */
        INTC_EnableIRQ(INTP0_IRQn);       /* enable INTP interrupt */
    }
    if(pinMsk & (1 << 1))
    {
        INTC_ClearPendingIRQ(INTP1_IRQn); /* clear INTP interrupt flag */
        INTC_EnableIRQ(INTP1_IRQn);       /* enable INTP interrupt */
    }
    if(pinMsk & (1 << 2))
    {
        INTC_ClearPendingIRQ(INTP2_IRQn); /* clear INTP interrupt flag */
        INTC_EnableIRQ(INTP2_IRQn);       /* enable INTP interrupt */
    }
    if(pinMsk & (1 << 3))
    {
        INTC_ClearPendingIRQ(INTP3_IRQn); /* clear INTP interrupt flag */
        INTC_EnableIRQ(INTP3_IRQn);       /* enable INTP interrupt */
    }
    if(pinMsk & (1 << 4))
    {
        INTC_ClearPendingIRQ(INTP4_IRQn); /* clear INTP interrupt flag */
        INTC_EnableIRQ(INTP4_IRQn);       /* enable INTP interrupt */
    }
    if(pinMsk & (1 << 5))
    {
        INTC_ClearPendingIRQ(INTP5_IRQn); /* clear INTP interrupt flag */
        INTC_EnableIRQ(INTP5_IRQn);       /* enable INTP interrupt */
    }
    if(pinMsk & (1 << 6))
    {
        INTC_ClearPendingIRQ(INTP6_IRQn); /* clear INTP interrupt flag */
        INTC_EnableIRQ(INTP6_IRQn);       /* enable INTP interrupt */
    }
    if(pinMsk & (1 << 7))
    {
        INTC_ClearPendingIRQ(INTP7_IRQn); /* clear INTP interrupt flag */
        INTC_EnableIRQ(INTP7_IRQn);       /* enable INTP interrupt */
    }
    if(pinMsk & (1 << 8))
    {
        INTC_ClearPendingIRQ(INTP8_IRQn); /* clear INTP interrupt flag */
        INTC_EnableIRQ(INTP8_IRQn);       /* enable INTP interrupt */
    }
    if(pinMsk & (1 << 9))
    {
        INTC_ClearPendingIRQ(INTP9_IRQn); /* clear INTP interrupt flag */
        INTC_EnableIRQ(INTP9_IRQn);       /* enable INTP interrupt */
    }
    if(pinMsk & (1 << 10))
    {
        INTC_ClearPendingIRQ(INTP10_IRQn); /* clear INTP interrupt flag */
        INTC_EnableIRQ(INTP10_IRQn);       /* enable INTP interrupt */
    }
    if(pinMsk & (1 << 11))
    {
        INTC_ClearPendingIRQ(INTP11_IRQn); /* clear INTP interrupt flag */
        INTC_EnableIRQ(INTP11_IRQn);       /* enable INTP interrupt */
    }
}

/**
  * @brief  This function disables INTP interrupt and clears interrupt flag.
  * @param  pinMsk:select the active external interrupt pins
  * 	@arg  INTP0/INTP1/INTP2/INTP3
  * @retval None
  */
void INTP_Stop(uint16_t pinMsk)
{
    if(pinMsk & (1 << 0))
    {
        INTC_DisableIRQ(INTP0_IRQn);       /* disable INTP interrupt */
        INTC_ClearPendingIRQ(INTP0_IRQn);  /* clear INTP interrupt flag */
    }
    if(pinMsk & (1 << 1))
    {
        INTC_DisableIRQ(INTP1_IRQn);       /* disable INTP interrupt */
        INTC_ClearPendingIRQ(INTP1_IRQn);  /* clear INTP interrupt flag */
    }
    if(pinMsk & (1 << 2))
    {
        INTC_DisableIRQ(INTP2_IRQn);       /* disable INTP interrupt */
        INTC_ClearPendingIRQ(INTP2_IRQn);  /* clear INTP interrupt flag */
    }
    if(pinMsk & (1 << 3))
    {
        INTC_DisableIRQ(INTP3_IRQn);       /* disable INTP interrupt */
        INTC_ClearPendingIRQ(INTP3_IRQn);  /* clear INTP interrupt flag */
    }
}
