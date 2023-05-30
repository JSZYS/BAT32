#include "adc.h"
#include "cgc.h"


/**
  * @brief  Enable the specified ADC channel .
  * @param  None
  * @retval None
  */
void ADC_Start(ADC_Channel_t ch)
{
	 ADC->ADS = ch;             //specify adc channel 
	 INTC_ClearPendingIRQ(ADC_IRQn); // clear INTAD interrupt flag	 
	 NVIC_ClearPendingIRQ(ADC_IRQn); // clear INTAD interrupt flag 
	 INTC_EnableIRQ(ADC_IRQn);		 // enable INTAD interrupt 
	 NVIC_EnableIRQ(ADC_IRQn);		 // enable INTAD interrupt

	if((ADC->ADTRG >> 6) != 3)		// write ADCS except for hardwait mode 
	{
		ADC->ADM0 |= (1<<7);            // enables conversion operation 
	}
}

/**
  * @brief  Disable the specified ADC peripheral.
  * @param  None
  * @retval None
  */
void ADC_Stop(void)
{
	ADC->ADM0 &= ~ADC_Start_Cmp; 			/* stops conversion operation */
	INTC_DisableIRQ(ADC_IRQn);      /* disable INTAD interrupt */
	INTC_ClearPendingIRQ(ADC_IRQn); /* clear INTAD interrupt flag */
	NVIC_ClearPendingIRQ(ADC_IRQn); /* clear INTAD interrupt flag */
}

/**
  * @brief  Initializes the ADC peripheral according to the specified
  *         parameters in the ADC_InitStruct .
  * @param  ADC_InitTypeDef: pointer to a ADC_InitTypeDef structure that contains
  *         the configuration information for the specified ADC peripheral.
  * @retval None
  */
void ADC_Init(ADC_InitTypeDef* ADC_InitStruct)
{
	assert_param(IS_ADC_MODE(ADC_InitStruct->ADC_Mode));
	
	CGC_PER0PeriphClockCmd(CGC_PER0Periph_ADC,ENABLE);
	ADC->ADM0  = 0x00U;                 // disable AD conversion and clear ADM0 register 
	INTC_DisableIRQ(ADC_IRQn);          // disable INTAD interrupt 
	INTC_ClearPendingIRQ(ADC_IRQn);     // clear INTAD interrupt flag    
	NVIC_ClearPendingIRQ(ADC_IRQn);     // clear INTAD interrupt flag 

	ADC->ADM1 |= ADC_InitStruct->ADC_ConvSpeed; //0x03 low current convertion mode

	ADC->ADM1 |= ADC_InitStruct->ADC_Mode;

	if(ADC_InitStruct->ADC_Mode == ADC_Mode_Scan)
	{
		ADC->ADM2 |= ADC_InitStruct->ADC_ScanConf.ADC_ChannelSign << 1;
	}


	ADC->ADM1 |= ADC_InitStruct->ADC_ContinuousConvMode;

	if (ADC_InitStruct->ADC_RefVoltage == ADC_Ref_Vdd)
	{
		ADC->ADM2 |= 0x00;
	}
	else if (ADC_InitStruct->ADC_RefVoltage == ADC_Ref_Extern)
	{
		ADC->ADM2 |= 0x40|0x20|0x00; // (refer voltage +) and (refer voltage -) are provided by extern voltage
	}
	else
	{
		ADC->ADM2 |= 0x80 | 0x00 ; // (refer voltage +) provided by internal 1.45v voltage and (refer voltage -)  provided by VSS
	}

	ADC->ADTRG |= ADC_InitStruct->ADC_ExternalTrigConv;
	ADC->ADTRG |= ADC_InitStruct->ADC_HardwareTrigSour;

	ADC->ADUL = ADC_InitStruct->ADC_UpLimit;
	ADC->ADLL = ADC_InitStruct->ADC_LowLimit;

	ADC->ADM0 &= ~(7 << 3);
	ADC->ADM0 |= ADC_InitStruct->ADC_Prescaler << 3;
	ADC->ADNSMP = ADC_InitStruct->ADC_TwoSamplingDelay;
	
	ADC->ADM0 |= ADC_Enable;
}

/**
  * @brief  Set the ADC peripheral running clock.
  * @param  fr：the specific clock 
  *         @arg ADC_Prescaler_Div32 
  *         @arg ADC_Prescaler_Div16 
  *         @arg ADC_Prescaler_Div8
  *         @arg ADC_Prescaler_Div4
  *         @arg ADC_Prescaler_Div2
  *         @arg ADC_Prescaler_Div1
  * @retval None
  */
void ADC_Set_Clock(ADC_Prescaler_t fr)
{
	ADC->ADM0 &= ~(7 << 3);
	ADC->ADM0 |= fr << 3;
}

/**
  * @brief  get ADC scan result of the specified adc channel.it scans 4
			channels one time
  * @param  ch：the specific ADC channel channel0~channel12
  * @param  times：the numer of scan times 
  * @param  buf：the scan result of  adc channel  
  * @retval  the average value of scann adc channel
  */
extern uint32_t   g_AdcIntTaken;

uint16_t ADC_Converse_Scan(ADC_Channel_t ch, uint32_t times, uint16_t *buf)
{
	uint32_t i,j;
	volatile uint8_t  flag;
	uint32_t total = 0;

	assert_param(IS_SCAN_START_CHAN(ch));
	
	INTC_DisableIRQ(ADC_IRQn);     // disable INTAD interrupt 

	ADC->ADM0 &= ~ADC_Enable;

	ADC->ADM1 |= 0x08; //enable one-shot convertion
	ADC->ADM1 |= 0x80; //set sacn mode

	ADC->ADTRG = ADC_ExternalTrig_Software;
	ADC->ADM0 |= ADC_Enable;

	ADC->ADS  = ch;

	for(i=0; i<times;i++)
	{
		ADC->ADM0 |= ADC_Start_Cmp; //adc start
		for (j=0; j<4; j++)
		{
			while(INTC_GetPendingIRQ(ADC_IRQn) == 0);
			INTC_ClearPendingIRQ(ADC_IRQn); 	// clear INTAD interrupt flag 

			*buf++ = ADC->ADCR;
			total += ADC->ADCR;


		}
	}
	return (total / times); // return average value
}

/**
  * @brief  get ADC sample result of the specified adc channel.
  * @param  ch：the specific ADC channel
  * @param  times：the numer of adc sample times 
  * @param  buf：the scan result of  adc channel  
  * @retval  the average value of  adc sample
  */
uint16_t ADC_Converse(ADC_Channel_t ch, uint32_t times, uint16_t *buf)
{
	uint32_t i;
	volatile uint8_t  flag;
	uint32_t total = 0;

	INTC_DisableIRQ(ADC_IRQn);     // disable INTAD interrupt 

	ADC->ADM0 &= ~ADC_Enable;

	ADC->ADM1 |= ADC_Conv_Oneshot; //enable one-shot convertion
	ADC->ADTRG = ADC_ExternalTrig_Software;
	ADC->ADM0 |= ADC_Enable;

	ADC->ADS  = ch;

	for(i=0; i<times;i++)
	{
		ADC->ADM0 |= ADC_Start_Cmp;  //adc start
		
		while(INTC_GetPendingIRQ(ADC_IRQn) == 0);
		INTC_ClearPendingIRQ(ADC_IRQn); 	// clear INTAD interrupt flag 

		*buf++ = ADC->ADCR;
		total += ADC->ADCR;
	}
	return (total / times); // return average value
}



/**
  * @brief  Set ADC hard trigger   .
  * @param  wait：the different ways of adc hard trigger
			it contains ：ADC_ExternalTrig_Hardware_Wait
						ADC_ExternalTrig_Hardware_NoWait
  * @param  trg：it chooses hardware trigger source 
  * @retval  None
  */
void ADC_Set_HardTrigger(uint8_t wait, ADC_HardwareTrig_t trg)
{
	assert_param(IS_ADC_TRIGGER(wait));
	assert_param(IS_ADC_HARDTRIGGER(trg));	
	ADC->ADM0 &= ~ADC_Enable;

	ADC->ADTRG = wait | trg;

	ADC->ADM0 |= ADC_Enable;
}
