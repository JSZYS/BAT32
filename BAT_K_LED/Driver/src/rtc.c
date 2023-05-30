#include "rtc.h"
#include "cgc.h"

/**
  * @brief  Start the RTC counter operation.
  * @param  None
  * @retval None
  */
void RTC_Start(void)
{

    INTC_ClearPendingIRQ(RTC_IRQn); // clear INTRTC interrupt flag 
    NVIC_ClearPendingIRQ(RTC_IRQn); // clear INTRTC interrupt flag 
    INTC_EnableIRQ(RTC_IRQn);       // enable INTRTC interrupt 

    RTC->RTCC0  |= 0x80;    // starts counter operation 

    /* confirm the RTC is running */
    RTC->RTCC1 |= RTC_RTCC1_RWAIT_Msk;              //stops SEC to YEAR counters. Mode to read or write counter value           
    while((RTC->RTCC1 & RTC_RTCC1_RWST_Msk) == 0);  // wait RWST = 1 
    RTC->RTCC1 &= ~RTC_RTCC1_RWAIT_Msk;             // sets counter operation 
    while((RTC->RTCC1 & RTC_RTCC1_RWST_Msk) != 0);  // wait RWST = 0 
}

/**
  * @brief  Stop the RTC counter operation.
  * @param  None
  * @retval None
  */
void RTC_Stop(void)
{
    RTC->RTCC0  = 0x00;     /* stops counter operation */
    INTC_DisableIRQ(RTC_IRQn);      /* disable INTRTC interrupt */
    INTC_ClearPendingIRQ(RTC_IRQn); /* clear INTRTC interrupt flag */
    NVIC_ClearPendingIRQ(RTC_IRQn); /* clear INTRTC interrupt flag */
}

/**
  * @brief  Convert from 2 digit BCD to Binary.
  * @param  Value: BCD value to be converted.
  * @retval Converted word
  */
static uint8_t RTC_Bcd2ToByte(uint8_t Value)
{
  uint8_t tmp = 0;
  tmp = ((uint8_t)(Value & (uint8_t)0xF0) >> (uint8_t)0x4) * 10;
  return (tmp + (Value & (uint8_t)0x0F));
}
/**
  * @brief  Converts a 2 digit decimal to BCD format.
  * @param  Value: Byte to be converted.
  * @retval Converted byte
  */
static uint8_t RTC_ByteToBcd2(uint8_t Value)
{
  uint8_t bcdhigh = 0;
  
  while (Value >= 10)
  {
    bcdhigh++;
    Value -= 10;
  }
  
  return  ((uint8_t)(bcdhigh << 4) | Value);
}

/**
  * @brief  Set the RTC hour format.
  * @param  RTC_HourFormat: 
				@arg RTC_HourFormat_12
				@arg RTC_HourFormat_24
  * @retval None
  */
void RTC_SetHourFormat(uint8_t RTC_HourFormat)
{
	/* Check the parameters */
	assert_param(IS_RTC_HOURFORMAT(RTC_HourFormat));
	

    RTC->RTCC1 |= RTC_RTCC1_RWAIT_Msk;              // stops SEC to YEAR counters. Mode to read or write counter value          
	
	RTC->RTCC0 |= RTC_HourFormat;    // select counter hour format:24hour or 12hour
	
	RTC->RTCC1 &= ~RTC_RTCC1_RWAIT_Msk; 			// sets counter operation 
    while((RTC->RTCC1 & RTC_RTCC1_RWST_Msk) != 0);  // wait RWST = 0 	
}


/**
  * @brief  Set the RTC current time.
  * @param  RTC_TimeStruct: pointer to a RTC_TimeTypeDef structure that contains 
  *                        the time configuration information for the RTC.     
  * @retval None
  */
void RTC_SetTime(RTC_TimeTypeDef* RTC_TimeStruct)
{
	/* Check the parameters */
	assert_param(IS_RTC_HOUR24(RTC_TimeStruct->RTC_Hours));
	assert_param(IS_RTC_MINUTES(RTC_TimeStruct->RTC_Minutes));
	assert_param(IS_RTC_SECONDS(RTC_TimeStruct->RTC_Seconds));	
	

    RTC->RTCC1 |= RTC_RTCC1_RWAIT_Msk;              // stops SEC to YEAR counters. Mode to read or write counter value          
	
	RTC->HOUR |= RTC_ByteToBcd2(RTC_TimeStruct->RTC_Hours);
	RTC->MIN  = RTC_ByteToBcd2(RTC_TimeStruct->RTC_Minutes);
	RTC->SEC  = RTC_ByteToBcd2(RTC_TimeStruct->RTC_Seconds);
	if(RTC_TimeStruct->RTC_Hours >12)
	{
		RTC->RTCC0 |= RTC_HourFormat_24;
	}
	else
	{
		RTC->RTCC0 |= RTC_HourFormat_12;
		RTC->HOUR |= RTC_TimeStruct->RTC_H12;
	}

	RTC->RTCC1 &= ~RTC_RTCC1_RWAIT_Msk; 			// sets counter operation 
    while((RTC->RTCC1 & RTC_RTCC1_RWST_Msk) != 0);  // wait RWST = 0 	
}

/**
  * @brief  Set the RTC current date.
  * @param  RTC_DateStruct: pointer to a RTC_DateTypeDef structure that contains 
  *                         the date configuration information for the RTC.
  * @retval None
  */
void RTC_SetDate(RTC_DateTypeDef* RTC_DateStruct)
{
	assert_param(IS_RTC_YEAR(RTC_DateStruct->RTC_Year));
	assert_param(IS_RTC_MONTH(RTC_DateStruct->RTC_Month));
	assert_param(IS_RTC_DATE(RTC_DateStruct->RTC_Day));
	assert_param(IS_RTC_DATE(RTC_DateStruct->RTC_WeekDay));	
	
    RTC->RTCC1 |= RTC_RTCC1_RWAIT_Msk;              // stops SEC to YEAR counters. Mode to read or write counter value          

	RTC->DAY   =  RTC_ByteToBcd2(RTC_DateStruct->RTC_Day);
	RTC->WEEK  =  RTC_ByteToBcd2(RTC_DateStruct->RTC_WeekDay);
	RTC->MONTH =  RTC_ByteToBcd2(RTC_DateStruct->RTC_Month);
	RTC->YEAR  =  RTC_ByteToBcd2(RTC_DateStruct->RTC_Year);	
	
	RTC->RTCC1 &= ~RTC_RTCC1_RWAIT_Msk; 			// sets counter operation 
    while((RTC->RTCC1 & RTC_RTCC1_RWST_Msk) != 0);  // wait RWST = 0 	
	
}

/**
  * @brief  Get the RTC current date and time.
  * @param  RTC_DateStruct: pointer to a RTC_DateTypeDef structure that contains 
  *                         the date configuration information for the RTC.
  * @retval None
  */

void RTC_Get_CounterValue(RTC_CounterTypeDef *counter_val)
{
    RTC->RTCC1 |= RTC_RTCC1_RWAIT_Msk;              // stops SEC to YEAR counters. Mode to read or write counter value          
    while((RTC->RTCC1 & RTC_RTCC1_RWST_Msk) == 0);  // wait RWST = 1 

    counter_val->time.RTC_Seconds   = RTC->SEC;    
    counter_val->time.RTC_Minutes   = RTC->MIN;   
    counter_val->time.RTC_Hours     = RTC->HOUR;   
    counter_val->date.RTC_Day   	= RTC->DAY;    
    counter_val->date.RTC_WeekDay 	= RTC->WEEK;   
    counter_val->date.RTC_Month		= RTC->MONTH;  
    counter_val->date.RTC_Year  	= RTC->YEAR;   

	RTC->RTCC1 &= ~RTC_RTCC1_RWAIT_Msk; 			// sets counter operation 
    while((RTC->RTCC1 & RTC_RTCC1_RWST_Msk) != 0);  // wait RWST = 0 
}

/**
  * @brief  Set the specified RTC Alarm.
  * @note   
  * @param  RTC_AlarmStruct: pointer to a RTC_AlarmTypeDef structure that 
  *                          contains the alarm configuration parameters.     
  * @retval None
  */
void RTC_SetAlarm( RTC_Alarm_t * RTC_AlarmStruct)
{
	INTC_DisableIRQ(RTC_IRQn);		// disable INTRTC interrupt 
	RTC->RTCC1 &= ~0x80;
	RTC->RTCC1 |= 0x40;
	
    RTC->ALARMWM = RTC_ByteToBcd2(RTC_AlarmStruct->Alarm_Minute);
    RTC->ALARMWH = RTC_ByteToBcd2(RTC_AlarmStruct->Alarm_Hour);
    RTC->ALARMWW = RTC_AlarmStruct->Alarm_Week;
	
	RTC->RTCC1 |= 0x80;
	RTC->RTCC1 &= ~0x10;//set alarm match-bit clear
	INTC_ClearPendingIRQ(RTC_IRQn); // clear INTRTC interrupt flag 
	NVIC_ClearPendingIRQ(RTC_IRQn); // clear INTRTC interrupt flag 
    INTC_EnableIRQ(RTC_IRQn);       // enable INTRTC interrupt
}

/**
  * @brief  Get the specified RTC Alarm Value.
  * @note   
  * @param  alarm_val: pointer to a RTC_Alarm_t structure that 
  *                          contains the alarm configuration parameters.     
  * @retval None
  */
void RTC_Get_AlarmValue(RTC_Alarm_t * alarm_val)
{
	INTC_DisableIRQ(RTC_IRQn);		// disable INTRTC interrupt 
	RTC->RTCC1 &= ~0x80;
    alarm_val->Alarm_Minute = RTC->ALARMWM;
    alarm_val->Alarm_Hour = RTC->ALARMWH;
    alarm_val->Alarm_Week = RTC->ALARMWW;
	RTC->RTCC1 |= 0x80;
	RTC->RTCC1 &= ~0x10;//set alarm match-bit clear
	INTC_ClearPendingIRQ(RTC_IRQn); // clear INTRTC interrupt flag 
	NVIC_ClearPendingIRQ(RTC_IRQn); // clear INTRTC interrupt flag 
    INTC_EnableIRQ(RTC_IRQn);       // enable INTRTC interrupt
}

/**
  * @brief  Initializes the RTC registers according to the specified parameters 
  *         in RTC_InitStruct.
  * @param  RTC_InitStruct: pointer to a RTC_InitTypeDef structure that contains 
  *         the configuration information for the RTC peripheral.
  * @note   The RTC Prescaler register is write protected and can be written in 
  *         initialization mode only.  
  * @retval None
  */
void RTC_Init(RTC_InitTypeDef *RTC_InitStruct)
{
	/* Check the parameters */
	assert_param(IS_RTC_HOURFORMAT(RTC_InitStruct->RTC_HourFormat));
	assert_param(IS_RTC_MINUTES(RTC_InitStruct->RTC_Time.RTC_Minutes));
	assert_param(IS_RTC_SECONDS(RTC_InitStruct->RTC_Time.RTC_Seconds));	
	
	assert_param(IS_RTC_DATE(RTC_InitStruct->RTC_Date.RTC_Day));
	assert_param(IS_RTC_MONTH(RTC_InitStruct->RTC_Date.RTC_Month));
	assert_param(IS_RTC_YEAR(RTC_InitStruct->RTC_Date.RTC_Year));	
	assert_param(IS_RTC_WEEKDAY(RTC_InitStruct->RTC_Date.RTC_WeekDay));	
	
	CGC_PER0PeriphClockCmd(CGC_PER0Periph_RTC,ENABLE);
	
	if(RTC_InitStruct->RTC_Clk == RTC_FIL)
	{
		CGC->OSMC  |= 1<<4;
	}	
	MISC->RTCCL = RTC_InitStruct->RTC_Clk;

	RTC->RTCC0  = 0x00;     // stops counter operation 
	RTC->RTCC0 |= RTC_InitStruct->RTC_HourFormat;    // select counter hour format:24hour or 12hour
	if(RTC_InitStruct->RTC_1HZ_Output)
	{
		RTC->RTCC0 |= 0x20;
	}
	RTC->RTCC0 =  (RTC->RTCC0 & (~0x07)) | RTC_InitStruct->RTC_Period;//set constant rtc period interrupt
	RTC->RTCC1 &= (uint8_t)~0x08;  //when generate period interrupt,bit-4 becomes 1; writing 0 to erase this flag

    RTC->SEC   = RTC_ByteToBcd2(RTC_InitStruct->RTC_Time.RTC_Seconds);
    RTC->MIN   = RTC_ByteToBcd2(RTC_InitStruct->RTC_Time.RTC_Minutes);
	RTC->HOUR  = RTC_ByteToBcd2(RTC_InitStruct->RTC_Time.RTC_Hours);	
	if(RTC_InitStruct->RTC_HourFormat == RTC_HourFormat_12)
		RTC->HOUR |= RTC_InitStruct->RTC_Time.RTC_H12;
    RTC->DAY   = RTC_ByteToBcd2(RTC_InitStruct->RTC_Date.RTC_Day);
    RTC->WEEK  = RTC_ByteToBcd2(RTC_InitStruct->RTC_Date.RTC_WeekDay);
    RTC->MONTH = RTC_ByteToBcd2(RTC_InitStruct->RTC_Date.RTC_Month);
    RTC->YEAR  = RTC_ByteToBcd2(RTC_InitStruct->RTC_Date.RTC_Year);

	RTC->RTCC1 &= (uint8_t)~0x80;//When set alarm registor, it must set RTCC1 wale-bit(bit-8) 0
	if(RTC_InitStruct->RTC_Alarm_Onoff)
	{
		RTC->RTCC1 |= 0x40;		
	}
	else
	{
		RTC->RTCC1 |= 0x00;		
	}

	RTC->ALARMWM = RTC_ByteToBcd2(RTC_InitStruct->RTC_Alarm.Alarm_Minute);
	RTC->ALARMWH = RTC_ByteToBcd2(RTC_InitStruct->RTC_Alarm.Alarm_Hour);
	RTC->ALARMWW = RTC_InitStruct->RTC_Alarm.Alarm_Week;
	RTC->RTCC1 &= ~0x10;//set alarm match-bit clear
}



/**
  * @brief  Enable the specified RTC Alarm function .
  * @param  
  * @retval None
  */
void RTC_Set_AlarmOn(void)
{
    volatile uint16_t w_count;
    INTC_DisableIRQ(RTC_IRQn); // disable INTRTC interrupt 
    RTC->RTCC1 |= 0x80;       //RTC_ALARM_ENABLE

    for (w_count = 0U; w_count < 2; w_count++)
    {
        __NOP();
	}
	RTC->RTCC1 &= ~0x10;//set alarm match-bit clear
    INTC_ClearPendingIRQ(RTC_IRQn); // clear INTRTC interrupt flag 
    NVIC_ClearPendingIRQ(RTC_IRQn); // clear INTRTC interrupt flag 
    INTC_EnableIRQ(RTC_IRQn);       // enable INTRTC interrupt
}

/**
  * @brief  Disable the specified RTC Alarm function .
  * @param  
  * @retval None
  */
void RTC_Set_AlarmOff(void)
{
	INTC_DisableIRQ(RTC_IRQn);		// disable INTRTC interrupt 
	RTC->RTCC1 &= ~0x80;
	RTC->RTCC1 &= ~0x10;

	INTC_ClearPendingIRQ(RTC_IRQn); // clear INTRTC interrupt flag 
	NVIC_ClearPendingIRQ(RTC_IRQn); // clear INTRTC interrupt flag 
}

/**
  * @brief  Enable the specified RTC constant period interrupt function .
  * @param  period ：set the constant period interrupt  
  * 	@arg    Half_Second
  * 	@arg    One_Second
  * 	@arg    One_Minute
  * 	@arg    One_Hour
  * 	@arg    One_Month
  * @retval None
  */
void  RTC_Set_ConstPeriodInterruptOn(RTC_Period_t period)
{
	INTC_DisableIRQ(RTC_IRQn);		// disable INTRTC interrupt */
	RTC->RTCC0 = (RTC->RTCC0 & (~0x07)) | period;
	RTC->RTCC1 &=~0x08;
	INTC_ClearPendingIRQ(RTC_IRQn); // clear INTRTC interrupt flag 
	NVIC_ClearPendingIRQ(RTC_IRQn); // clear INTRTC interrupt flag 
	INTC_EnableIRQ(RTC_IRQn);		// enable INTRTC interrupt 
}

/**
  * @brief  Disable the specified RTC constant period interrupt function .
  * @param 
  * @retval None
  */
void  RTC_Set_ConstPeriodInterruptOff(void)
{
    RTC->RTCC0 &= (uint8_t)~(0x07);
	RTC->RTCC1 &=~0x08;
	INTC_ClearPendingIRQ(RTC_IRQn); // clear INTRTC interrupt flag 
	NVIC_ClearPendingIRQ(RTC_IRQn); // clear INTRTC interrupt flag 
}


