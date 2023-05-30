#include "dma.h"
#include "cgc.h"
/**
  * @brief  The specified DMA peripheral initialization
  * @param  DMA_InitStruct: pointer to a DMA_InitTypeDef structure that contains
  *         the configuration information for the specified DMA peripheral.  
  * @retval None
  */
void DMA_Init(DMA_InitTypeDef * DMA_InitStruct)
{
	uint8_t DMA_ChainTrans_Flag = 0;
	
	if(DMA_InitStruct->DMA_ChainTrans)
	{
		if(DMA_InitStruct->DMA_ChainTrans == DMA_ChainTrans_Tail)
			DMA_ChainTrans_Flag = 0;
		else
			DMA_ChainTrans_Flag = 1;
	}
	if(DMA_InitStruct->DMA_ChainTrans == 0)
	{
		DMAVEC->VEC[DMA_InitStruct->DMA_Vector] = DMA_InitStruct->DMA_CtrlId;
	}
	else
	{
		if(DMA_InitStruct->DMA_ChainTrans == DMA_ChainTrans_Head)
		{
			DMAVEC->VEC[DMA_InitStruct->DMA_Vector] = DMA_InitStruct->DMA_CtrlId;
		}
	}
	DMAVEC->CTRL[DMA_InitStruct->DMA_CtrlId].DMACR = DMA_InitStruct->DMA_DataSize |  (DMA_InitStruct->DMA_RepeatInt << 5)  \
		| (DMA_ChainTrans_Flag << 4) | (DMA_InitStruct->DMA_DstInc << 3) | (DMA_InitStruct->DMA_SrcInc << 2) | DMA_InitStruct->DMA_Mode ;

	if(DMA_InitStruct->DMA_Mode == DMA_Mode_Repeat)
	{
		DMAVEC->CTRL[DMA_InitStruct->DMA_CtrlId].DMACR |= DMA_InitStruct->DMA_RepeatAreaAddr;
	}

	DMAVEC->CTRL[DMA_InitStruct->DMA_CtrlId].DMBLS = 1;   //the nums of data for in one time DMA transformation 
	DMAVEC->CTRL[DMA_InitStruct->DMA_CtrlId].DMACT = DMA_InitStruct->DMA_BufferSize; // times of DMA transformation
	DMAVEC->CTRL[DMA_InitStruct->DMA_CtrlId].DMRLD = DMA_InitStruct->DMA_BufferSize; // in repeat mode,it will be loaded into DMACT
	DMAVEC->CTRL[DMA_InitStruct->DMA_CtrlId].DMSAR = DMA_InitStruct->DMA_SrcAddr;
	DMAVEC->CTRL[DMA_InitStruct->DMA_CtrlId].DMDAR = DMA_InitStruct->DMA_DstAddr;	
	CGC_PER1PeriphClockCmd(CGC_PER1Periph_DMA, ENABLE);
	DMA->DMABAR  = DMAVEC_BASE;
}




/**
  * @brief  Enables the flag of specified DMA peripheral.
  * @param  vect_num: it choose DMA peripheral trigger source
  * @param  
  * @retval None
  */
void DMA_Start(DMA_VECTOR_t vect_num)
{
    volatile uint8_t  * sfr_addr;
    sfr_addr = &DMA->DMAEN0;
    *(sfr_addr + vect_num / 8) |= (1 << (vect_num % 8));    /* DMAENn = 1 */
}

/**
  * @brief  Stop the specified DMA peripheral.
  * @param  vect_num: it choose DMA peripheral trigger source
  * @param  
  * @retval None
  */
void DMA_Stop(DMA_VECTOR_t vect_num)
{
    volatile uint8_t  * sfr_addr;
    sfr_addr = &DMA->DMAEN0;
    *(sfr_addr + vect_num / 8) &= ~(1 << (vect_num % 8));    /* DMAENn = 0 */
}

/**
  * @brief  start the interrupt flag of specified DMA peripheral 
  * @param  vect_num: it choose DMA peripheral trigger source 
  * @param  
  * @retval None
  */
void DMA_Trigger(DMA_VECTOR_t vect_num)
{
    volatile uint8_t  * sfr_addr;
    DMA->IFPRCR = 0xF1;
    sfr_addr = &DMA->DMAIF0;
    *(sfr_addr + vect_num / 8) = (1 << (vect_num % 8));    /* DMAIFn = 1 */
    DMA->IFPRCR = 0x00;
}

