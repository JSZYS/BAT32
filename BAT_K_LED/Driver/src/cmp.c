#include "cmp.h"
#include "cgc.h"
#define TESOPT  *((volatile uint8_t *)(0x4004384C))  /*!< (@ 0x4004384C) PGA protect register                          */
/***********************************************************************************************************************
* Function Name: CMP_VREF_Set_Value
* @brief   This function sets the internal reference voltate [#n] value.
* @param  ch - specify the comp channel number
* @param  reg_value - value of conversion
* @return None
***********************************************************************************************************************/
void CMP_VREF_Set_Value(CMP_Channel_t ch, uint8_t regvalue)
{
    if(ch & 0x01)
    {
		CMP->C0RVM = regvalue;
	}
    if(ch & 0x02)
    {
        CMP->C1RVM = regvalue; 
	}
}

/***********************************************************************************************************************
* Function Name: CMP_VREF_Set_Value
* @brief   This function sets the internal reference voltate [#n] value.
* @param  ch - specify the comp channel number
* @param  reg_value - value of conversion
* @return None
***********************************************************************************************************************/
void CMP_VREF_Init(CMP_Channel_t ch, CMP_Vref_t vref)
{
	TESOPT = 0x3D;		 /* Internal voltage reference test enable */
    if(vref == 0) 
    {
        CMP->CVRCTL = _00_VDD_INTERNAL_REFERVOL | _00_VSS_INTERNAL_REFERVOL;
	}
    if(vref == 1) 
    {
        CMP->CVRCTL = _01_AVREFP_INTERNAL_REFERVOL | _10_AVREFM_INTERNAL_REFERVOL;
	}

	if(ch & 0x01)
	{
        CMP->CVRCTL = _02_COMP0_REFVOLTAGE_ENABLE;
	}
	if(ch & 0x02)
	{
        CMP->CVRCTL = _20_COMP1_REFVOLTAGE_ENABLE; 
	}
}

/***********************************************************************************************************************
* Function Name: CMP_Input_Select
* @brief  This function sets the positive and negative side input of Comparator.
* @param  ch - specify the comp channel number
* @param  positive_side - specify the positive side input of Comparator
* @param  negative_side - specify the negative side input of Comparator
* @return None
***********************************************************************************************************************/
void CMP_Input_Select(CMP_Channel_t ch, CMP_Side_t positive_side, CMP_Side_t negative_side)
{
    if(ch & 0x01)
    {
        CMP->CMPSEL0 = positive_side << 7 | negative_side << 0;
	}
    if(ch & 0x02)
    {
        CMP->CMPSEL1 = positive_side << 6 | negative_side << 0;
	}
}

/***********************************************************************************************************************
* Function Name: CMP_Init
* @brief  This function initilize the CMP.
***********************************************************************************************************************/
void CMP_Init(CMP_InitTypeDef* CMP_InitStruct)
{
	CGC_PER1PeriphClockCmd(CGC_PER1Periph_PGACMP,ENABLE);

	if(CMP_InitStruct->CMP_Channel & 0x01)
	{
		if(CMP_InitStruct->CMP_NegativeSide == CMP0_NegativeSide_IVREF)
		{
            CMP_VREF_Set_Value(CMP_InitStruct->CMP_Channel, CMP_InitStruct->CMP_NegVrefVol);
            CMP_VREF_Init     (CMP_InitStruct->CMP_Channel, CMP_InitStruct->CMP_NegVref);
		}
        CMP_Input_Select  (CMP_InitStruct->CMP_Channel, CMP_InitStruct->CMP_PositiveSide, CMP_InitStruct->CMP_NegativeSide);
        CMP->COMPOCR = CMP_InitStruct->CMP_Polarity<<2 | CMP_InitStruct->CMP_Out2port<<1 | CMP_InitStruct->CMP_Int;        
		CMP->COMPMDR = _01_COMP0_OPERATION_ENABLED;

		if(CMP_InitStruct->CMP_Int)
		{
			if(CMP_InitStruct->CMP_IntReq == CMP_IntReq_Both)
			{
				CMP->COMPFIR |= 0x08;
			}
			else
			{
				CMP->COMPFIR |= (CMP_InitStruct->CMP_IntReq << 2);
			}
		}
		CMP->COMPFIR |= CMP_InitStruct->CMP_Filter;
//		CMP->CMP0HY |= ((CMP_InitStruct->CMP_HyVal << 4) | CMP_InitStruct->CMP_HyMode);
	}
	if(CMP_InitStruct->CMP_Channel & 0x02)
	{
		if(CMP_InitStruct->CMP_NegativeSide == CMP1_NegativeSide_IVREF)
		{
            CMP_VREF_Set_Value(CMP_InitStruct->CMP_Channel, CMP_InitStruct->CMP_NegVrefVol);
            CMP_VREF_Init     (CMP_InitStruct->CMP_Channel, CMP_InitStruct->CMP_NegVref);
		}
        CMP_Input_Select  (CMP_InitStruct->CMP_Channel, CMP_InitStruct->CMP_PositiveSide, CMP_InitStruct->CMP_NegativeSide);
        CMP->COMPOCR = CMP_InitStruct->CMP_Polarity<<6 |CMP_InitStruct->CMP_Out2port<<5 | (CMP_InitStruct->CMP_Int << 4);        
		CMP->COMPMDR = _10_COMP1_OPERATION_ENABLED;
		
		if(CMP_InitStruct->CMP_Int)
		{
			if(CMP_InitStruct->CMP_IntReq == CMP_IntReq_Both)
			{
				CMP->COMPFIR |= 0x80;
			}
			else
			{
				CMP->COMPFIR |= (CMP_InitStruct->CMP_IntReq << 6);
			}
		}

		CMP->COMPFIR |= (CMP_InitStruct->CMP_Filter << 4);
//		CMP->CMP1HY |= ((CMP_InitStruct->CMP_HyVal << 4) | CMP_InitStruct->CMP_HyMode);
	}	
}

/***********************************************************************************************************************
* Function Name: CMP_Start
* @brief  This function starts the Comparator.
* @param  ch - specify the channel
***********************************************************************************************************************/
void CMP_Start(CMP_Channel_t ch)
{
    if(ch & 0x01)
    {
        INTC_ClearPendingIRQ(CMP0_IRQn);    // clear INTCMP0 interrupt flag 
        INTC_EnableIRQ(CMP0_IRQn);          // enable INTCMP0 interrupt 
        CMP->COMPMDR = _01_COMP0_OPERATION_ENABLED;
	}
    if(ch & 0x02)
    {
        INTC_ClearPendingIRQ(CMP1_IRQn);    // clear INTCMP0 interrupt flag 
        INTC_EnableIRQ(CMP1_IRQn);          // enable INTCMP0 interrupt 
        CMP->COMPMDR = _10_COMP1_OPERATION_ENABLED;
	}
}
/***********************************************************************************************************************
* Function Name: CMP_Stop
* @brief  This function starts the Comparator.
* @param  ch - specify the channel
***********************************************************************************************************************/
void CMP_Stop(CMP_Channel_t ch)
{
    if(ch & 0x01)
    {
        CMP->COMPMDR &= ~_01_COMP0_OPERATION_ENABLED;
        INTC_DisableIRQ(CMP0_IRQn);         // disable INTCMP0 interrupt 
        INTC_ClearPendingIRQ(CMP0_IRQn);    // clear INTCMP0 interrupt flag 
	}
    if(ch & 0x02)
    {
        CMP->COMPMDR &= ~_10_COMP1_OPERATION_ENABLED;
        INTC_DisableIRQ(CMP1_IRQn);         // disable INTCMP1 interrupt
        INTC_ClearPendingIRQ(CMP1_IRQn);    // clear INTCMP1 interrupt flag
	}
}

/***********************************************************************************************************************
* Function Name: CMP_Filter_Setting
* @brief  This function sets the filter and edge detection of Comparator output.
* @param  ch - specify the comp channel number
* @param  fck - specify the clock of digital filter 
* @param  edge - specify the edge detector
* @return None
***********************************************************************************************************************/
void CMP_Filter_Setting(CMP_Channel_t ch, CMP_Filter_t fck, CMP_IntReq_t edge)
{
    if(ch & 0x01)
    {
        CMP->COMPFIR = edge << 2 | fck << 0;
    }
    if(ch & 0x02)
    {
        CMP->COMPFIR = edge << 6 | fck << 4;
    }
}
