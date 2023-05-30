#include "dac.h"
#include "cgc.h"

/***********************************************************************************************************************
* Function Name: DAC_Init
* @brief  This function initializes the D/A converter.
* @param  ch - spcify the dac channel to initialized
* @return None
***********************************************************************************************************************/
void DAC_Init(DAC_InitTypeDef* DAC_InitStruct)
{
	CGC_PER1PeriphClockCmd(CGC_PER1Periph_DAC,ENABLE);
	DAC->DAM   = 0x00U;

	if(DAC_InitStruct->DAC_Channel & 0x01)
	{
		DAC->DAM |= DAC_InitStruct->DAC_Mode;
		DAC->DACS0 = DAC_InitStruct->DAC_ConValue;
	}
	if(DAC_InitStruct->DAC_Channel & 0x02)
	{
		DAC->DAM |= DAC_InitStruct->DAC_Mode;
		DAC->DACS1 = DAC_InitStruct->DAC_ConValue;
	}
}

/***********************************************************************************************************************
* Function Name: DAC_Start
* @brief  This function enables the DA converter channel [#n].
* @param  ch - spcify the dac channel to start
* @return None
***********************************************************************************************************************/
void DAC_Start(DAC_Channel_t ch)
{
    if(ch & 0x01) 
    {
        DAC->DAM  |= DAC_DAM_DACE0_Msk;     /* enables D/A conversion operation */
    }
    if(ch & 0x02) 
    {
        DAC->DAM  |= DAC_DAM_DACE1_Msk;     /* enables D/A conversion operation */
    }
}
/***********************************************************************************************************************
* Function Name: DAC_Stop
* @brief  This function stops the DA converter channel [#n].
* @param  ch - spcify the dac channel to stop
* @return None
***********************************************************************************************************************/
void DAC_Stop(DAC_Channel_t ch)
{
    if(ch & 0x01) 
    {
        DAC->DAM  &= ~DAC_DAM_DACE0_Msk;     /* stops D/A conversion operation */
    }
    if(ch & 0x02) 
    {
        DAC->DAM  &= ~DAC_DAM_DACE1_Msk;     /* stops D/A conversion operation */
    }
}

/***********************************************************************************************************************
* Function Name: DAC_Set_Value
* @brief  This function sets the DA converter channel [#n] value.
* @param  reg_value - value of conversion
* @return None
***********************************************************************************************************************/
void DAC_Set_Value(DAC_Channel_t ch, uint8_t regvalue)
{
    if(ch & 0x01) 
    {
        DAC->DACS0 = regvalue;
    }
    if(ch & 0x02) 
    {
        DAC->DACS1 = regvalue;
    }
}

/***********************************************************************************************************************
* Function Name: DAC_Set_PowerOff
* @brief  This function stops supply of input clock and reset all SFR.
* @param  None
* @return None
***********************************************************************************************************************/
void DAC_Set_PowerOff(void)
{
    CGC->PER1 &= ~CGC_PER1_DACEN_Msk;     /* stops input clock supply */
}
