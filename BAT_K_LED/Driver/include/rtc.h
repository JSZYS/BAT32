#ifndef __RTC_H__
#define __RTC_H__
#include <stdio.h>
#include <stdint.h>
#include "common.h"

/** @defgroup RTC_Time_Definitions 
  * @{
  */ 
#define IS_RTC_HOUR12(HOUR)            (((HOUR) > 0) && ((HOUR) <= 12))
#define IS_RTC_HOUR24(HOUR)            ((HOUR) <= 23)
#define IS_RTC_MINUTES(MINUTES)        ((MINUTES) <= 59)
#define IS_RTC_SECONDS(SECONDS)        ((SECONDS) <= 59)

#define IS_RTC_YEAR(YEAR)              ((YEAR) <= 99)
#define IS_RTC_MONTH(MONTH)            (((MONTH) >= 1) && ((MONTH) <= 12))
#define IS_RTC_DATE(DATE)              (((DATE) >= 1) && ((DATE) <= 31))
typedef enum 
{    
	SUNDAY   = 0x00U,    
	MONDAY   = 0x01U,    
	TUESDAY  = 0x02U,    
	WEDNESDAY= 0x03U,    
	THURSDAY = 0x04U,    
	FRIDAY   = 0x05U,    
	SATURDAY = 0x06U
}RTC_WeekDay_t;
#define IS_RTC_WEEKDAY(WEEKDAY) (((WEEKDAY) == SUNDAY) || \
                                 ((WEEKDAY) == MONDAY) || \
                                 ((WEEKDAY) == TUESDAY) || \
                                 ((WEEKDAY) == WEDNESDAY) || \
                                 ((WEEKDAY) == THURSDAY) || \
                                 ((WEEKDAY) == FRIDAY) || \
                                ((WEEKDAY) == SATURDAY))

/** @defgroup RTC_AM_PM_Definitions 
  * @{
  */ 
#define RTC_H12_AM                     ((uint8_t)0x00)
#define RTC_H12_PM                     ((uint8_t)0x20)
#define IS_RTC_H12(PM) (((PM) == RTC_H12_AM) || ((PM) == RTC_H12_PM))

typedef enum
{
	RTC_HourFormat_12 = 0x00,
	RTC_HourFormat_24 = 0x08,	
}RTC_HourFormat_t;
#define IS_RTC_HOURFORMAT(FORMAT) (((FORMAT) == RTC_HourFormat_12) || ((FORMAT) == RTC_HourFormat_24))



typedef enum
{
    RTC_FSUB = 0x00,    
	RTC_FIL  = 0x01,    
	RTC_64MHZ= 0xA2,  // fHOCO = 64MHz     
	RTC_48MHZ= 0x82,  // fHOCO = 48MHz     
	RTC_32MHZ= 0xC2,  // fHOCO = 32MHz     
	RTC_16MHZ= 0x03,  // fMX   = 16MHz     
	RTC_8MHZ = 0x43,   // fMX   =  8MHz 
}RTC_ClkSource_t;

typedef enum
{
	Period_None,
	Half_Second,  //each half_second generate an interrupt
	One_Second,
	One_Minute,
	One_Hour,
	One_Month,
}RTC_Period_t;   // period interrupt function

typedef enum
{
	RTC_Alarm_Off,
	RTC_Alarm_On
}RTC_Alarm_Onoff_t;   // period interrupt function

/** @defgroup RTC_Alarm_weekday
  * @{
  */ 
#define ALARM_WEEK(WEEKDAY)    (uint8_t)(1<<WEEKDAY)

/** @defgroup RTC_Alarm_Definitions
  * @{
  */ 
typedef struct
{
	uint8_t Alarm_Week;  //alarm  week day   it can be a value of @ref RTC_Alarm_weekday
	uint8_t Alarm_Hour;  //alarm  hour
	uint8_t Alarm_Minute;//alarm  minute
}RTC_Alarm_t;   // alarm  interrupt function



typedef struct
{
	uint8_t RTC_Hours;    /*!< Specifies the RTC Time Hour.
	                    This parameter must be set to a value in the 0-12 range
	                    if the RTC_HourFormat_12 is selected or 0-23 range if
	                    the RTC_HourFormat_24 is selected. */

	uint8_t RTC_Minutes;  /*!< Specifies the RTC Time Minutes.
	                    This parameter must be set to a value in the 0-59 range. */

	uint8_t RTC_Seconds;  /*!< Specifies the RTC Time Seconds.
	                    This parameter must be set to a value in the 0-59 range. */
							
	uint8_t RTC_H12;      /*!< Specifies the RTC AM/PM Time.
							This parameter can be a value of @ref RTC_AM_PM_Definitions */	
}RTC_TimeTypeDef;

typedef struct
{
	uint8_t RTC_WeekDay; /*!< Specifies the RTC Date WeekDay.
	                    This parameter can be a value of @ref RTC_WeekDay_Definitions */

	uint8_t RTC_Month;   /*!< Specifies the RTC Date Month (in BCD format).
	                    This parameter can be a value of @ref RTC_Month_Date_Definitions */

	uint8_t RTC_Day;     /*!< Specifies the RTC Date.
	                    This parameter must be set to a value in the 1-31 range. */

	uint8_t RTC_Year;     /*!< Specifies the RTC Date Year.
                        This parameter must be set to a value in the 0-99 range. */
}RTC_DateTypeDef;


typedef struct
{
	RTC_HourFormat_t RTC_HourFormat;   /*!< Specifies the RTC Hour Format.
							   This parameter can be a value of @ref RTC_Hour_Formats */

	RTC_TimeTypeDef RTC_Time; /*!< Specifies the RTC time include hour minute second.
	                    This parameter can be a value of @ref RTC_TimeTypeDef */

	RTC_DateTypeDef RTC_Date;   /*!< Specifies the RTC Date include year month day .
	                    This parameter can be a value of @ref RTC_DateTypeDef */

	RTC_ClkSource_t RTC_Clk;   /*!< Specifies the RTC Date Month (in BCD format).
											   This parameter can be a value of @ref RTC_Month_Date_Definitions */
						
	RTC_Period_t RTC_Period;   /*!< Specifies the RTC Date Month (in BCD format).
											   This parameter can be a value of @ref RTC_Month_Date_Definitions */
	RTC_Alarm_Onoff_t RTC_Alarm_Onoff;    /*!< Specifies the RTC Alarm interrupt is enable or disable */
											  											   
	RTC_Alarm_t RTC_Alarm;   /*!< Specifies the RTC Alarm (in BCD format).
											   This parameter can be a value of @ref RTC_Alarm_t_Definitions */
	uint8_t RTC_1HZ_Output;  
}RTC_InitTypeDef;

typedef struct 
{
	RTC_TimeTypeDef time;
    RTC_DateTypeDef date;
}RTC_CounterTypeDef;


void RTC_Init(RTC_InitTypeDef *RTC_InitStruct);
void RTC_Start(void);
void RTC_Stop(void);
void RTC_SetHourFormat(uint8_t RTC_HourFormat);
void RTC_SetTime(RTC_TimeTypeDef* RTC_TimeStruct);
void RTC_SetDate(RTC_DateTypeDef* RTC_DateStruct);
void RTC_Get_CounterValue(RTC_CounterTypeDef *counter_val);

void RTC_Set_AlarmOn(void);
void RTC_Set_AlarmOff(void);
void RTC_Set_AlarmValue(RTC_Alarm_t * alarm_val);

void  RTC_Set_ConstPeriodInterruptOn(RTC_Period_t period);
void  RTC_Set_ConstPeriodInterruptOff(void);
#endif
