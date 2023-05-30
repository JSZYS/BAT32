#include "isr.h"
volatile uint32_t g_ticks = 0xFFFFFFFFU;

ATE_FRAME_t pData = {0};
ISR_InitTypeDef_t IRQ_Fun;

/**
  * @brief  SysTick Handler Decreament the g_ticks value
  * @param  TM13 IRQ
  * @retval None
  */
void SysTick_Handler(void)
{
	g_ticks--;
}

/**
  * @brief  IRQ00_Handler
  * @param  LVI IRQ/OSDC_IRQn/OCRV_IRQn
  * @retval None
  */
void IRQ00_Handler(void)
{
	if(INTC_GetPendingIRQ(LVI_IRQn))
	{
		IRQ_Fun.IRQ[LVI_IRQn].intHandle(&pData);
	}
	
	if(INTC_GetPendingIRQ(OSDC_IRQn))
	{
		IRQ_Fun.IRQ[OSDC_IRQn].intHandle(&pData);
	}
	
	if(INTC_GetPendingIRQ(OCRV_IRQn))
	{
		IRQ_Fun.IRQ[OCRV_IRQn].intHandle(&pData);
	}	
}

/**
  * @brief  IRQ01_Handler
  * @param  INTP0 IRQ/INTP6_IRQn
  * @retval None
  */
void IRQ01_Handler(void)
{
	if(INTC_GetPendingIRQ(INTP0_IRQn))
	{
		IRQ_Fun.IRQ[INTP0_IRQn].intHandle(&pData);
	}

	if(INTC_GetPendingIRQ(INTP6_IRQn))
	{
		IRQ_Fun.IRQ[INTP6_IRQn].intHandle(&pData);
	}
}

/**
  * @brief  IRQ02_Handler
  * @param  INTP1 IRQ/INTP7_IRQn
  * @retval None
  */
void IRQ02_Handler(void)
{
	if(INTC_GetPendingIRQ(INTP1_IRQn))
	{
		IRQ_Fun.IRQ[INTP1_IRQn].intHandle(&pData);
	}

	if(INTC_GetPendingIRQ(INTP7_IRQn))
	{
		IRQ_Fun.IRQ[INTP7_IRQn].intHandle(&pData);
	}
}

/**
  * @brief  IRQ03_Handler
  * @param  INTP2 IRQ /INTP8_IRQn
  * @retval None
  */
void IRQ03_Handler(void)
{
	if(INTC_GetPendingIRQ(INTP2_IRQn))
	{
		IRQ_Fun.IRQ[INTP2_IRQn].intHandle(&pData);
	}
	
	if(INTC_GetPendingIRQ(INTP8_IRQn))
	{
		IRQ_Fun.IRQ[INTP8_IRQn].intHandle(&pData);
	}	
}

/**
  * @brief  IRQ04_Handler
  * @param  INTP3 IRQ /INTP9_IRQn
  * @retval None
  */
void IRQ04_Handler(void)
{
	if(INTC_GetPendingIRQ(INTP3_IRQn))
	{
		IRQ_Fun.IRQ[INTP3_IRQn].intHandle(&pData);
	}
	
	if(INTC_GetPendingIRQ(INTP9_IRQn))
	{
		IRQ_Fun.IRQ[INTP9_IRQn].intHandle(&pData);
	}	
}

/**
  * @brief  IRQ05_Handler
  * @param  INTP4_IRQn /INTP10_IRQn
  * @retval None
  */
void IRQ05_Handler(void)
{
	if(INTC_GetPendingIRQ(INTP4_IRQn))
	{
		IRQ_Fun.IRQ[INTP4_IRQn].intHandle(&pData);
	}
	
	if(INTC_GetPendingIRQ(INTP10_IRQn))
	{
		IRQ_Fun.IRQ[INTP10_IRQn].intHandle(&pData);
	}	
}

/**
  * @brief  IRQ06_Handler
  * @param  INTP5_IRQn /INTP11_IRQn
  * @retval None
  */
void IRQ06_Handler(void)
{
	if(INTC_GetPendingIRQ(INTP5_IRQn))
	{
		IRQ_Fun.IRQ[INTP5_IRQn].intHandle(&pData);
	}
	
	if(INTC_GetPendingIRQ(INTP11_IRQn))
	{
		IRQ_Fun.IRQ[INTP11_IRQn].intHandle(&pData);
	}	
}

/**
  * @brief  IRQ07_Handler
  * @param  ST2/SPI20/IIC20 IRQ 
  * @retval None
  */
void IRQ07_Handler(void)
{
	if(INTC_GetPendingIRQ(39))
	{
		IRQ_Fun.IRQ[39].intHandle(&pData);
	}
	else
	{
		IRQ_Fun.IRQ[7].intHandle(&pData);
	}
}

/**
  * @brief  IRQ08_Handler
  * @param  SR2/SPI21/IIC21 IRQ
  * @retval None
  */
void IRQ08_Handler(void)
{	
	if(INTC_GetPendingIRQ(40))
	{
		IRQ_Fun.IRQ[40].intHandle(&pData);
	}
	else
	{
		IRQ_Fun.IRQ[8].intHandle(&pData);
	}
}

/**
  * @brief  IRQ09_Handler
  * @param  SRE2 IRQ
  * @retval None
  */
void IRQ09_Handler(void)
{
	if(INTC_GetPendingIRQ(SRE2_IRQn))
	{
		IRQ_Fun.IRQ[SRE2_IRQn].intHandle(&pData);
	}
	
	if(INTC_GetPendingIRQ(CAN0ERR_IRQn))
	{
		IRQ_Fun.IRQ[CAN0ERR_IRQn].intHandle(&pData);
	}
}

/**
  * @brief  IRQ10_Handler
  * @param  ST0/SPI00/IIC00 IRQ  
  * @note   the num of ST0_IRQn is same as SPI00_IRQn and IIC00_IRQ, so SPI00_IRQn or IIC00_IRQ
  *			interrupt quote this function handler IRQ10_Handler
  * @retval None
  */
void IRQ10_Handler(void)
{
	IRQ_Fun.IRQ[10].intHandle(&pData);
}

/**
  * @brief  IRQ11_Handler
  * @param  SR0/SPI01/IIC01 IRQ 
  * @note   the num of SR0_IRQn is same as SPI01_IRQn and IIC01_IRQ, so SPI01_IRQn or IIC01_IRQ
  *			interrupt quote this function handler IRQ11_Handler
  * @retval None
  */
void IRQ11_Handler(void)
{
	IRQ_Fun.IRQ[11].intHandle(&pData);	
}

/**
  * @brief  IRQ12_Handler
  * @param   SRE0 or TM01H IRQ 
  * @note   the num of SRE0_IRQn is same as TM01H_IRQn, so TM01H_IRQn 
  *			interrupt quotes this function handler IRQ12_Handler
  * @retval None
  */
void IRQ12_Handler(void)
{
	if(INTC_GetPendingIRQ(SRE0_IRQn))
	{
		IRQ_Fun.IRQ[SRE0_IRQn].intHandle(&pData);	
	}
	
	if(INTC_GetPendingIRQ(TM01H_IRQn))
	{
		IRQ_Fun.IRQ[TM01H_IRQn].intHandle(&pData);	
	}	
}

/**
  * @brief  IRQ13_Handler
  * @param  ST1/SPI10/IIC10 IRQ  
  * @note   the num of ST1_IRQn is same as SPI10_IRQn and IIC10_IRQ, so SPI10_IRQn or IIC10_IRQ
  *			interrupt quote this function handler IRQ13_Handler
  * @retval None
  */
void IRQ13_Handler(void)
{
	IRQ_Fun.IRQ[13].intHandle(&pData);	
}

/**
  * @brief  IRQ14_Handler
  * @param  SR1/SPI11/IIC11 IRQ
  * @note   the num of SR1_IRQn is same as SPI11_IRQn and IIC11_IRQ, so SPI11_IRQn or IIC11_IRQ
  *			interrupt quote this function handler IRQ14_Handler
  * @retval None
  */
void IRQ14_Handler(void)
{
	IRQ_Fun.IRQ[14].intHandle(&pData);	
}

/**
  * @brief  IRQ15_Handler
  * @param  SRE1 IRQ
  * @note   the num of SRE1_IRQn is same as TM03H_IRQn, so TM03H_IRQn 
  *			interrupt quotes this function handler IRQ15_Handler
  * @retval None
  */
void IRQ15_Handler(void)
{
	if(INTC_GetPendingIRQ(SRE1_IRQn))
	{
		IRQ_Fun.IRQ[SRE1_IRQn].intHandle(&pData);	
	}
	
	if(INTC_GetPendingIRQ(TM03H_IRQn))
	{
		IRQ_Fun.IRQ[TM03H_IRQn].intHandle(&pData);	
	}	
}

/**
  * @brief  IRQ16_Handler
  * @param  IICA IRQ
  * @retval None
  */
void IRQ16_Handler(void)
{
	if(INTC_GetPendingIRQ(IICA0_IRQn))
	{
		IRQ_Fun.IRQ[IICA0_IRQn].intHandle(&pData);	
	}
	
	if(INTC_GetPendingIRQ(DIV_IRQn))
	{
		IRQ_Fun.IRQ[DIV_IRQn].intHandle(&pData);	
	}
	
	if(INTC_GetPendingIRQ(IICA1_IRQn))
	{
		IRQ_Fun.IRQ[IICA1_IRQn].intHandle(&pData);	
	}	
}

/**
  * @brief  IRQ17_Handler
  * @param  TM00 IRQ
  * @retval None
  */
void IRQ17_Handler(void)
{
	if(INTC_GetPendingIRQ(TM00_IRQn))
	{
		IRQ_Fun.IRQ[TM00_IRQn].intHandle(&pData);	
	}
	
	if(INTC_GetPendingIRQ(TM10_IRQn))
	{
		IRQ_Fun.IRQ[TM10_IRQn].intHandle(&pData);	
	}
	
	if(INTC_GetPendingIRQ(TM14_IRQn))
	{
		IRQ_Fun.IRQ[TM14_IRQn].intHandle(&pData);	
	}
}

/**
  * @brief  IRQ18_Handler
  * @param  TM01 IRQ
  * @retval None
  */
void IRQ18_Handler(void)
{
	if(INTC_GetPendingIRQ(TM01_IRQn))
	{
		IRQ_Fun.IRQ[TM01_IRQn].intHandle(&pData);	
	}
	
	if(INTC_GetPendingIRQ(TM11_IRQn))
	{
		IRQ_Fun.IRQ[TM11_IRQn].intHandle(&pData);	
	}
	
	if(INTC_GetPendingIRQ(TM15_IRQn))
	{
		IRQ_Fun.IRQ[TM15_IRQn].intHandle(&pData);	
	}
}

/**
  * @brief  IRQ19_Handler
  * @param  TM02 IRQ
  * @retval None
  */
void IRQ19_Handler(void)
{
	if(INTC_GetPendingIRQ(TM02_IRQn))
	{
		IRQ_Fun.IRQ[TM02_IRQn].intHandle(&pData);	
	}
	
	if(INTC_GetPendingIRQ(TM12_IRQn))
	{
		IRQ_Fun.IRQ[TM12_IRQn].intHandle(&pData);	
	}
	
	if(INTC_GetPendingIRQ(TM16_IRQn))
	{
		IRQ_Fun.IRQ[TM16_IRQn].intHandle(&pData);	
	}	
}

/**
  * @brief  IRQ20_Handler
  * @param  TM03 IRQ
  * @retval None
  */
void IRQ20_Handler(void)
{
	if(INTC_GetPendingIRQ(TM03_IRQn))
	{
		IRQ_Fun.IRQ[TM03_IRQn].intHandle(&pData);	
	}
	
	if(INTC_GetPendingIRQ(TM13_IRQn))
	{
		IRQ_Fun.IRQ[TM13_IRQn].intHandle(&pData);	
	}
	
	if(INTC_GetPendingIRQ(TM17_IRQn))
	{
		IRQ_Fun.IRQ[TM17_IRQn].intHandle(&pData);	
	}	
}

/**
  * @brief  IRQ21_Handler
  * @param  ADC IRQ
  * @retval None
  */
void IRQ21_Handler(void)
{
	IRQ_Fun.IRQ[21].intHandle(&pData);	
}

/**
  * @brief  IRQ22_Handler
  * @param  RTC IRQ
  * @retval None
  */
void IRQ22_Handler(void)
{
	if(INTC_GetPendingIRQ(RTC_IRQn))
	{
		IRQ_Fun.IRQ[RTC_IRQn].intHandle(&pData);	
	}
	
	if(INTC_GetPendingIRQ(IT_IRQn))
	{
		IRQ_Fun.IRQ[IT_IRQn].intHandle(&pData);	
	}	
}

/**
  * @brief  IRQ23_Handler
  * @param  KEY_IRQn
  * @retval None
  */
void IRQ23_Handler(void)
{
	if(INTC_GetPendingIRQ(KEY_IRQn))
	{
		IRQ_Fun.IRQ[KEY_IRQn].intHandle(&pData);	
	}
	
	if(INTC_GetPendingIRQ(CAN0REC_IRQn))
	{
		IRQ_Fun.IRQ[CAN0REC_IRQn].intHandle(&pData);	
	}	
}

/**
  * @brief  IRQ24_Handler
  * @param  CMP0_IRQn
  * @retval None
  */
void IRQ24_Handler(void)
{
	if(INTC_GetPendingIRQ(CMP0_IRQn))
	{
		IRQ_Fun.IRQ[CMP0_IRQn].intHandle(&pData);	
	}
	
	if(INTC_GetPendingIRQ(CAN0WUP_IRQn))
	{
		IRQ_Fun.IRQ[CAN0WUP_IRQn].intHandle(&pData);	
	}
}

void IRQ25_Handler(void)
{
	if(INTC_GetPendingIRQ(CMP1_IRQn))
	{
		IRQ_Fun.IRQ[CMP1_IRQn].intHandle(&pData);	
	}
	
	if(INTC_GetPendingIRQ(CAN0TRX_IRQn))
	{
		IRQ_Fun.IRQ[CAN0TRX_IRQn].intHandle(&pData);	
	}
}

void IRQ26_Handler(void)
{
	if(INTC_GetPendingIRQ(TMA_IRQn))
	{
		IRQ_Fun.IRQ[TMA_IRQn].intHandle(&pData);	
	}
	
	if(INTC_GetPendingIRQ(CAN1ERR_IRQn))
	{
		IRQ_Fun.IRQ[CAN1ERR_IRQn].intHandle(&pData);	
	}
}
/**
  * @brief  IRQ27_Handler
  * @param  TM0 IRQ
  * @retval None
  */
void IRQ27_Handler(void)
{
	IRQ_Fun.IRQ[27].intHandle(&pData);	
}

/**
  * @brief  IRQ28_Handler
  * @param  TM13 IRQ
  * @retval None
  */
void IRQ28_Handler(void)
{
	IRQ_Fun.IRQ[28].intHandle(&pData);	
}

/**
  * @brief  IRQ29_Handler
  * @param  TM12 IRQ
  * @retval None
  */
void IRQ29_Handler(void)
{
	if(INTC_GetPendingIRQ(TMB_IRQn))
	{
		IRQ_Fun.IRQ[TMB_IRQn].intHandle(&pData);	
	}
	
	if(INTC_GetPendingIRQ(CAN1REC_IRQn))
	{
		IRQ_Fun.IRQ[CAN1REC_IRQn].intHandle(&pData);	
	}	
}

/**
  * @brief  IRQ30_Handler
  * @param  TM13 IRQ
  * @retval None
  */
void IRQ30_Handler(void)
{
	if(INTC_GetPendingIRQ(TMC_IRQn))
	{
		IRQ_Fun.IRQ[TMC_IRQn].intHandle(&pData);	
	}
	
	if(INTC_GetPendingIRQ(CAN1WUP_IRQn))
	{
		IRQ_Fun.IRQ[CAN1WUP_IRQn].intHandle(&pData);	
	}
}

/**
  * @brief  IRQ31_Handler
  * @param  FMC IRQ 
  * @retval None
  */
void IRQ31_Handler(void)
{
	if(INTC_GetPendingIRQ(FMC_IRQn))
	{
		IRQ_Fun.IRQ[FMC_IRQn].intHandle(&pData);	
	}
	
	if(INTC_GetPendingIRQ(CAN1TRX_IRQn))
	{
		IRQ_Fun.IRQ[CAN1TRX_IRQn].intHandle(&pData);	
	}
}


/**
  * @brief  register interrupt function according to  IRQn num.
  * @param  irq_num: it relates to irq handler 
  * @param	interrupt:interrupt service function
  * @retval regiseter status 
  */
int ISR_Register(IRQn_Type irq_num, void *interrupt)
{
	int res = ISR_SUCC;

	if(IRQ_Fun.IRQ[irq_num].IRQ_Flag)
	{
		return ISR_ERR;
	}
	else
	{		
		IRQ_Fun.IRQ[irq_num].IRQ_Flag = 1;
		IRQ_Fun.IRQ[irq_num].intHandle = (isrFun_t)interrupt;
		INTC_ClearPendingIRQ(irq_num); // clear  interrupt flag 
		NVIC_ClearPendingIRQ(irq_num); // clear  interrupt flag 
		INTC_EnableIRQ(irq_num);
	}

	return res;
}
