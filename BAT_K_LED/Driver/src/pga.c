#include "pga.h"
#include "cgc.h"

#define TESOPT  *((volatile uint8_t *)(0x4004384C))  /*!< (@ 0x4004384C) PGA protect register                          */
#define TESCTL 	*((volatile uint8_t *)(0x4004384D))  /*!< (@ 0x4004384D) PGA output control register                   */

/***********************************************************************************************************************
* Function Name: PGA_Init
* @brief  This function init the programmable gain amplifier.
* @param  PGA_InitTypeDef
* @return  none 
***********************************************************************************************************************/
void PGA_Init(PGA_InitTypeDef *PGA_InitStruct)
{
	CGC_PER1PeriphClockCmd(CGC_PER1Periph_PGACMP, ENABLE);

    TESOPT = 0x3DU; 

	if(PGA_InitStruct->PGA_Channel & 0x01)
	{
        PGA->PGA0CTL = 0x00U;     // PGA operation stopped 
        PGA->PGA0CTL = (PGA_InitStruct->PGA_Vref << 3) | PGA_InitStruct->PGA_Gain;
        TESCTL &= ~(1 << 0);      // PGAO0 output disable          
		TESCTL |= PGA_InitStruct->PGA_Out2Port << 0;    // PGAO0 output enable
	}
}

/***********************************************************************************************************************
* Function Name: PGA_Start
* @brief  This function starts the  programmable gain amplifier.
* @param  ch - specify the pga channel number
* @return  none 
***********************************************************************************************************************/
void PGA_Start(PGA_Channel_t ch)
{
    if(ch & 0x01)     
	{        
		PGA->PGA0CTL |= 0x80;     // PGA0 operation enabled 
	}
	if(ch & 0x02) 
    {
        PGA->PGA1CTL |= 0x80;     /* PGA1 operation enabled */
    }
}

/***********************************************************************************************************************
* Function Name: PGA_Stop
* @brief  This function stops the  programmable gain amplifier.
* @param  ch - specify the pga channel number
* @return  none 
***********************************************************************************************************************/
void PGA_Stop(PGA_Channel_t ch)
{
    if(ch & 0x01)     
	{        
		PGA->PGA0CTL &= ~0x80;     // PGA0 operation enabled 
	}
}

