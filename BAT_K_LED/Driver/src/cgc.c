#include "cgc.h"

uint8_t USE_HSE_SYSTYEM_CLOCK = SYSTYEM_CLOCK_CLOSE;
uint8_t USE_HSI_SYSTYEM_CLOCK = SYSTYEM_CLOCK_CLOSE;
uint8_t USE_LSE_SYSTYEM_CLOCK = SYSTYEM_CLOCK_CLOSE;
uint8_t USE_LSI_SYSTYEM_CLOCK = SYSTYEM_CLOCK_CLOSE;

/**
  * @brief  Enables or disables the PER0 peripheral clock.
  * @note   After reset, the peripheral clock (used for registers read/write access)
  *         is disabled and the application software has to enable this clock before 
  *         using it.
  * @param  CGC_PER0Periph: specifies the PER0 peripheral to gates its clock.
  *          This parameter can be any combination of the following values:
  *            @arg CGC_PER0Periph_TIM40: TIM40 clock
  *            @arg CGC_PER0Periph_TIM41: TIM41 clock
  *            @arg CGC_PER0Periph_SCI0:  SCI0 clock
  *            @arg CGC_PER0Periph_SCI1:  SCI1 clock
  *            @arg CGC_PER0Periph_IICA0:  IICA clock
  *            @arg CGC_PER0Periph_ADC:   ADC clock
  *            @arg CGC_PER0Periph_IRDA:  IRDA clock
  *            @arg CGC_PER0Periph_RTC:   RTC clock
  * @param  NewState: new state of the specified peripheral clock.
  *          This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
void CGC_PER0PeriphClockCmd(uint8_t CGC_PER0Periph, FunctionalState NewState)
{
    /* Check the parameters */
    assert_param(IS_CGC_PER0_PERIPH(CGC_PER0Periph));
    assert_param(IS_FUNCTIONAL_STATE(NewState));

    if (NewState != DISABLE)
    {
        CGC->PER0 |= CGC_PER0Periph;
    }
    else
    {
        CGC->PER0 &= ~CGC_PER0Periph;
    }
}

/**
  * @brief  Enables or disables the PER1 peripheral clock.
  * @note   After reset, the peripheral clock (used for registers read/write access)
  *         is disabled and the application software has to enable this clock before 
  *         using it.
  * @param  CGC_PER1Periph: specifies the PER0 peripheral to gates its clock.
  *          This parameter can be any combination of the following values:
  *            @arg CGC_PER1Periph_TMA:     TIMA clock
  *            @arg CGC_PER1Periph_DMA:     DMA clock
  *            @arg CGC_PER1Periph_PGACMP:  PGA or CMP clock
  *            @arg CGC_PER1Periph_TMM:     Specifical TIMM clock
  *            @arg CGC_PER1Periph_DAC:		DAC clock
  *            @arg CGC_PER1Periph_TMB:     TIMB clock
  * @param  NewState: new state of the specified peripheral clock.
  *          This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
void CGC_PER1PeriphClockCmd(uint32_t CGC_PER1Periph, FunctionalState NewState)
{
    /* Check the parameters */
    assert_param(IS_CGC_PER1_PERIPH(CGC_PER1Periph));
    assert_param(IS_FUNCTIONAL_STATE(NewState));

    if (NewState != DISABLE)
    {
        CGC->PER1 |= CGC_PER1Periph;
    }
    else
    {
        CGC->PER1 &= ~CGC_PER1Periph;
    }
}

/**
  * @brief  Enables or disables the PER2 peripheral clock.
  * @note   After reset, the peripheral clock (used for registers read/write access)
  *         is disabled and the application software has to enable this clock before 
  *         using it.
  * @param  CGC_PER2Periph: specifies the PER2 peripheral to gates its clock.
  *          This parameter can be any combination of the following values:
  *            @arg CGC_PER2Periph_TM81:     TM81 clock
  *            @arg CGC_PER2Periph_CAN1:     CAN1 clock
  *            @arg CGC_PER2Periph_I2CA1:  IICA1 clock
  *            @arg CGC_PER2Periph_SCI2:     Specifical SCI2 clock
  *            @arg CGC_PER2Periph_OSDC:		DAC clock
  *            @arg CGC_PER2Periph_CAN2:     CAN2 clock
  *            @arg CGC_PER2Periph_SPIHS0:		SPIHS0 clock
  *            @arg CGC_PER2Periph_SPIHS1:     SPIHS1 clock
  * @param  NewState: new state of the specified peripheral clock.
  *          This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
void CGC_PER2PeriphClockCmd(uint32_t CGC_PER2Periph, FunctionalState NewState)
{
    /* Check the parameters */
    assert_param(IS_CGC_PER2_PERIPH(CGC_PER2Periph));
    assert_param(IS_FUNCTIONAL_STATE(NewState));

    if (NewState != DISABLE)
    {
        CGC->PER2 |= CGC_PER2Periph;
    }
    else
    {
        CGC->PER2 &= ~CGC_PER2Periph;
    }
}
/**
  * @brief  This function initializes the Main OSC and Sub OSC.
  * @param  main
  *             - OSC_PORT:        X1, X2 as PORT
  *             - OSC_OSCILLATOR:  X1, X2 as oscillator and connect crystal/ceramic resonator
  *             - OSC_EXCLK:       X1, as PORT, X2 as external clock input
  * @param  sub 
  *             - OSC_PORT:        XT1, XT2 as PORT
  *             - OSC_OSCILLATOR:  XT1, XT2 as oscillator and connect crystal resonator
  *             - OSC_EXCLK:       XT1 as PORT, XT2 as external clock input
  * @retval None
  */
void CGC_Osc_Setting(OSC_Pin_Mode_t main,OSC_Speed_Mode_t amph, OSC_Pin_Mode_t sub,OSC_Power_Mode_t amphs)
{
    volatile uint32_t w_count;
    uint8_t           temp_stab_set;
    uint8_t           temp_stab_wait;
    uint8_t           tmp;

    tmp = 0x00;
    if(main == OSC_PORT )
    {
        tmp |= (0 << CGC_CMC_EXCLK_Pos) | (0 << CGC_CMC_OSCSEL_Pos);
    }

    if(sub == OSC_PORT )
    {
        tmp |= (0 << CGC_CMC_EXCLKS_Pos) | (0 << CGC_CMC_OSCSELS_Pos);
    }

    if(main == OSC_OSCILLATOR)
    {
        tmp |= (0 << CGC_CMC_EXCLK_Pos) | (1 << CGC_CMC_OSCSEL_Pos) | (1 << CGC_CMC_AMPH_Pos);
    }

    if(sub == OSC_OSCILLATOR)
    {
        tmp |= (0 << CGC_CMC_EXCLKS_Pos) | (1 << CGC_CMC_OSCSELS_Pos) | (1 << CGC_CMC_AMPHS_Pos);
    }

    if(main == OSC_EXCLK)
    {
        tmp |= (1 << CGC_CMC_EXCLK_Pos) | (1 << CGC_CMC_OSCSEL_Pos);
    }

    if(sub == OSC_EXCLK)
    {
        tmp |= (1 << CGC_CMC_EXCLKS_Pos) | (1 << CGC_CMC_OSCSELS_Pos);
    }

    CGC->CMC = tmp;

    /* Set fMX */
    CGC->CSC &= ~(1<<7) ;   //MSTOP = 0

    if(main == OSC_OSCILLATOR)
    {
        temp_stab_set = _FF_CGC_OSCSTAB_STA18;
        
        do
        {
            temp_stab_wait = CGC->OSTC;
            temp_stab_wait &= temp_stab_set;
        }
        while (temp_stab_wait != temp_stab_set);
    }

    /* Set fSUB */
    CGC->CSC &= ~(1<<6) ;   //XTSTOP = 0

    if(sub == OSC_OSCILLATOR)
    {
        /* Change the waiting time according to the system */
        for (w_count = 0U; w_count <= CGC_SUBWAITTIME; w_count++)
        {
            __NOP();
        }
    }
}



/**
  * @brief  Configures the External Low Speed oscillator (LSE).
  * @note   External Low Speed oscillator can be called subsystem clock
  * @note   RCC_LSE: subsystem clock source can be OSC_PORT/OSC_OSCILLATOR/OSC_EXCLK.
  * 	 at same time, it can choose power consumption mode.  
  * @param  sub 
  *             - OSC_PORT:        XT1, XT2 as PORT
  *             - OSC_OSCILLATOR:  XT1, XT2 as oscillator and connect crystal resonator
  *             - OSC_EXCLK:       XT1 as PORT, XT2 as external clock input
  * @param  amphs
  *             - OSC_LOW_POWER:       low power consumption oscillation
  *             - OSC_NORMAL_POWER:    normal oscillation
  *             - OSC_ULTRA_LOW_POWER: ultra-low power consumption oscillation
  * @retval None
  */
void CGC_LSEConfig(OSC_Pin_Mode_t sub, OSC_Power_Mode_t amphs)
{
	/* Check the parameters */
	assert_param(IS_CGC_LSE_MODE(sub));
	assert_param(IS_CGC_LSE_PWR_MODE(amphs));

	volatile uint32_t w_count;
    uint8_t           tmp;

    tmp = 0x00;
    if(sub == OSC_PORT )
    {
        tmp |= (0 << CGC_CMC_EXCLKS_Pos) | (0 << CGC_CMC_OSCSELS_Pos);
    }

    if(sub == OSC_OSCILLATOR)
    {
        tmp |= (0 << CGC_CMC_EXCLKS_Pos) | (1 << CGC_CMC_OSCSELS_Pos) | (amphs << CGC_CMC_AMPHS_Pos);
    }

    if(sub == OSC_EXCLK)
    {
        tmp |= (1 << CGC_CMC_EXCLKS_Pos) | (1 << CGC_CMC_OSCSELS_Pos);
    }

    CGC->CMC = tmp;

    /* Set fSUB */
    CGC->CSC &= ~(1<<6) ;   //XTSTOP = 0

    if(sub == OSC_OSCILLATOR)
    {
        /* Change the waiting time according to the system */
        for (w_count = 0U; w_count <= CGC_SUBWAITTIME; w_count++)
        {
            __NOP();
        }
    }  
}


/**
  * @brief  Configures the External High Speed oscillator (HSE). 
  * @note   External High Speed oscillator clock source can be choose from PORT¡¢
  *         OSC_OSCILLATOR or external input clock.
  * 	    at same time, OSC_OSCILLATOR can be 1MHz < fx < 10MHz or 10MHz < fx < 20MHz 
  * @param  pinMode 
  *             - OSC_PORT:        XT1, XT2 as PORT
  *             - OSC_OSCILLATOR:  XT1, XT2 as oscillator and connect crystal resonator
  *             - OSC_EXCLK:       XT1 as PORT, XT2 as external clock input
  * @param  amph 
  *             - OSC_UNDER_10M:   X1 frequency: 1MHz < fx < 10MHz
  *             - OSC_OVER_10M:    X1 frequency: 10MHz < fx < 20MHz

  * @retval None
  */
void CGC_HSEConfig(OSC_Pin_Mode_t main, OSC_Speed_Mode_t amph)
{
	/* Check the parameters */
	assert_param(IS_CGC_LSE_MODE(main));
	assert_param(IS_CGC_HSE_OSC_SPEED(amph));

    volatile uint32_t w_count;
    uint8_t           temp_stab_set;
    uint8_t           temp_stab_wait;
    uint8_t           tmp;

    tmp = 0x00;
    if(main == OSC_PORT )
    {
        tmp |= (0 << CGC_CMC_EXCLK_Pos) | (0 << CGC_CMC_OSCSEL_Pos);
    }

    if(main == OSC_OSCILLATOR)
    {
        tmp |= (0 << CGC_CMC_EXCLK_Pos) | (1 << CGC_CMC_OSCSEL_Pos) | (amph << CGC_CMC_AMPH_Pos);
    }

    if(main == OSC_EXCLK)
    {
        tmp |= (1 << CGC_CMC_EXCLK_Pos) | (1 << CGC_CMC_OSCSEL_Pos);
    }

    CGC->CMC = tmp;

    /* Set fMX */
    CGC->CSC &= ~(1<<7) ;   //MSTOP = 0

    if(main == OSC_OSCILLATOR)
    {
        temp_stab_set = _FF_CGC_OSCSTAB_STA18;
        
        do
        {
            temp_stab_wait = CGC->OSTC;
            temp_stab_wait &= temp_stab_set;
        }
        while (temp_stab_wait != temp_stab_set);
    }
}



/**
  * @brief  Enables External Low Speed oscillator (LSE/Fsub) used as CPU 
  *		    system clock and Clock source of peripheral hardware circuit.	
  * @note     
  * @retval None
  */
void CGC_LSE_CFG_AS_FCLK()
{
	CGC->CKC = (1 << CGC_CKC_CSS_Pos) | (0 << CGC_CKC_MCM0_Pos);

    while ((CGC->CKC & CGC_CKC_CLS_Msk) == 0);
	USE_LSE_SYSTYEM_CLOCK = SYSTYEM_CLOCK_OPEN;
}

/**
  * @brief  Enables External High Speed oscillator (HSE/Fmx) used as CPU 
  *		    system clock and Clock source of peripheral hardware circuit.	
  * @note     
  * @retval None
  */
void CGC_HSE_CFG_AS_FCLK()
{
	CGC->CKC = (0 << CGC_CKC_CSS_Pos) | (1 << CGC_CKC_MCM0_Pos );
	while((CGC->CKC & CGC_CKC_MCS_Msk) == 0);
	USE_HSE_SYSTYEM_CLOCK = SYSTYEM_CLOCK_OPEN;	
}

/**
  * @brief  Enables Internal High Speed oscillator (HSI) used as CPU 
  *		    system clock and Clock source of peripheral hardware circuit.	
  * @note  internal High Speed oscillator FHOCO is divided to FIH,which will used as CPU system
  * 		clock
  * @retval None
  */
void CGC_HSI_CFG_AS_FCLK()
{
	CGC->CKC = 0 << CGC_CKC_CSS_Pos ;

	while((CGC->CKC & CGC_CKC_CSS_Msk) == 1);
	USE_HSI_SYSTYEM_CLOCK = SYSTYEM_CLOCK_OPEN;	
}

/**
  * @brief  Enables External High Speed oscillator (HSE) used as MAIN system clock 
  *	 which can provided for clock output/buzzer or CPU/peripheral hardware circuit.	
  * @note     
  * @retval None
  */
void CGC_HSE_CFG_AS_FMAIN()
{
	CGC->CKC =  1 << CGC_CKC_MCM0_Pos;
	while((CGC->CKC & CGC_CKC_MCS_Msk) == 0);
}
/**
  * @brief  Enables Internal High Speed oscillator (HSI/FIH) used as as MAIN system clock  
  *	  which can provided for clock output/buzzer or CPU/peripheral hardware circuit.	
  * @note     
  * @retval None
  */
void CGC_HSI_CFG_AS_FMAIN()
{
	CGC->CKC = 0 << CGC_CKC_MCM0_Pos;
	while((CGC->CKC & CGC_CKC_MCS_Msk) == 1);
}

/**
  * @brief Setting PLL used as system clock and Clock source of peripheral hardware circuit.
  * @param  PLL_Src_t 
  *             - PLL_SR_fIH:  PLL frequency source is generated by internal HSI
  *             - PLL_SR_fMX:  PLL frequency source is generated by FMX 
  * @param  PLL_Div_t   coefficient of division
  *             - PLL_DIV_1:     
  *             - PLL_DIV_2: 
  *             - PLL_DIV_4:  
  * @param  PLL_Mul_t   coefficient of multiplication
  *             - PLL_MUL_12:      
  *             - PLL_MUL_16: 
  * @note     
  * @retval None
  */
void CGC_PLL_Setting(PLL_Src_t src, PLL_Div_t div, PLL_Mul_t mul)
{
	uint8_t    tmp = 0;
    uint32_t i;
	
	if (src==PLL_SR_fIH)
	{
		tmp += _00_CGC_PLLSR_fIH;
	}
	else
	{
		tmp += _80_CGC_PLLSR_fMX;
	}

	if (div==PLL_DIV_2)
	{
		tmp += _04_CGC_PLL_DIV_2;
	}
	else if(div==PLL_DIV_4)
	{
		tmp += _08_CGC_PLL_DIV_4;
	}
	else
	{
		tmp += _00_CGC_PLL_DIV_1;
	}

	if (mul==PLL_MUL_16)
	{
		tmp += _02_CGC_PLL_MUL_16_0;
	}
	else
	{
		tmp += _00_CGC_PLL_MUL_12_0;
	}
	CGC->PLLCR = tmp;


    CGC->PLLCR |= 1<<0;        /* PLLON = 1 */
    for (i = 0U; i <= 2000; i++)
    {
        __NOP();
    }
	
}

/**
  * @brief  Enables output frequency by PLL used as CPU 
  *		    system clock and Clock source of peripheral hardware circuit.	
  * @note     
  * @retval None
  */
void CGC_PLL_CFG_AS_FCLK(void)
{
	CGC->MCKC = 0x01;
	while((CGC->MCKC & CGC_MCKC_CKSTR_Msk) == 0);
	USE_HSE_SYSTYEM_CLOCK = SYSTYEM_CLOCK_OPEN;	
}

/**
  * @brief  This function stops the main system clock oscilator (MOSC).
  * @param  None
  * @retval None
  */
void CGC_MainOsc_Stop(void)
{
	CGC->CSC |= 1<<7; 	/* MSTOP = 1 */
}

/**
  * @brief  This function starts the main system clock oscilator (MOSC). 
  * @param  None
  * @retval None
  */
void CGC_MainOsc_Start(void)
{
	CGC->CSC &= ~(1<<7); 	/* MSTOP = 0 */
}

/**
  * @brief  This function stops the sub system clock oscilator (SOSC).  
  * @param  None
  * @retval None
  */
void CGC_SubOsc_Stop(void)
{
	CGC->CSC |= 1<<6; 	/* XTSTOP = 1 */
}

/**
  * @brief  This function starts the sub system clock oscilator (SOSC). 
  * @param  None
  * @retval None
  */
void CLK_SubOsc_Start(void)
{
	CGC->CSC &= ~(1<<6); 	/* XTSTOP = 0 */
}

/**
  * @brief This function stops the high speed on chip oscilator (HOCO).
  * @param  None
  * @retval None
  */
void CGC_Hoco_Stop(void)
{
	CGC->CSC |= 1<<0; 	/* HIOSTOP = 1 */
}

/**
  * @brief This function starts the high speed on chip oscilator (HOCO). 
  * @param  None
  * @retval None
  */
void CGC_Hoco_Start(void)
{
	CGC->CSC &= ~(1<<0); 	/* HIOSTOP = 0 */
}
