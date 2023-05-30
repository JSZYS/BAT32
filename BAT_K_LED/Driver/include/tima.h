#ifndef __TIMA_H__
#define __TIMA_H__
#include <stdint.h>
#include <stdbool.h>
#include "common.h"

#define _01_TMA_COUNT_START                          (0x01U) /* count starts */
/* TAIO input polarity select bit (TEDGPL)输入边沿极性选择 */
#define _00_TMA_TAIO_POLARITY_ONE                    (0x00U) /* one edge */
#define _08_TMA_TAIO_POLARITY_BOTH                   (0x08U) /* both edges */
/* TAIO event input control bit (TIOGT1,TIOGT0) */
#define _00_TMA_EVENT_ENABLE_ALWAYS                  (0x00U) /* event is counted */
#define _40_TMA_EVENT_ENABLE_INTP4                   (0x40U) /* event is counted during INTP4 specified period */
#define _80_TMA_EVENT_ENABLE_TIMEROUTSIGNAL          (0x80U) /* event is counted during polarity period specified for timer output signal */
/* PWM signal and INTP4 polarity selection (RCCPSEL2) */
#define _00_TMA_PWM_POLARITY_L                       (0x00U) /* L period is counted */
#define _04_TMA_PWM_POLARITY_H                       (0x04U) /* H period is counted */
/* PWM signal selection (RCCPSEL1,RCCPSEL0) */
#define _00_TMA_PWM_TMIOD1                           (0x00U) /* TRDIOD1 */
#define _01_TMA_PWM_TMIOC1                           (0x01U) /* TRDIOC1 */
#define _02_TMA_PWM_TO02                             (0x02U) /* TO02 */
#define _03_TMA_PWM_TO03                             (0x03U) /* TO03 */
/* Timer A underflow flag (TUNDF) */
#define _00_TMA_UNDERFLOW_NOT_OCCUR                  (0x00U) /* no underflow */
#define _20_TMA_UNDERFLOW_OCCUR                      (0x20U) /* underflow */
/* Active edge judgment flag (TEDGF) */
#define _00_TMA_ACTIVE_EDGE_UNRECEIVED               (0x00U) /* active edge not received */
#define _10_TMA_ACTIVE_EDGE_RECEIVED                 (0x10U) /* active edge received (end of measurement period) */


/** @defgroup TMA_Edge_t 
  * @{
  */
#define TMA_RISING_EDGE         0x0001
#define TMA_FALLING_EDGE        0x0002
#define TMA_BOTH_EDGE        	(TMA_FALLING_EDGE | TMA_RISING_EDGE) 
#define IS_TMA_EDGE(edge)		((((edge) & (uint16_t)0xFFFC) == 0x0000) && ((edge) != (uint16_t)0x0000))

/** @defgroup TMA_Ctrl_t 
  * @{
  */
#define TMA_COUNT_AWLAYS		0x0000
#define TMA_INTP4_LOW_LEVEL		0x0001
#define TMA_INTP4_HIGH_LEVEL	0x0002

#define TMM_TMIOD1_LOW_LEVEL	0x0010
#define TMM_TMIOD1_HIGH_LEVEL	0x0020
#define TMM_TMIOC1_LOW_LEVEL	0x0040
#define TMM_TMIOC1_HIGH_LEVEL	0x0080

#define TIM_TO02_LOW_LEVEL		0x0100
#define TIM_TO02_HIGH_LEVEL		0x0200
#define TIM_TO03_LOW_LEVEL		0x0400
#define TIM_TO03_HIGH_LEVEL		0x0800
#define IS_TMA_CTRL(ctrl)		(((ctrl) & (uint16_t)0xF00C) == 0x0000)


typedef enum 
{
    TMA_FCLK_Div1		= 0x00U,
    TMA_FCLK_Div8		= 0x10U,
    TMA_FCLK_Div2		= 0x30U,
    TMA_FIL				= 0x40U,
    TMA_EventCount		= 0x50U,
    TMA_FSUB			= 0x60U,
}TMA_Clk_t;

typedef enum
{
	TMA_Mode_Interval   = 0x00,
	TMA_Mode_Square     = 0x01,
	TMA_Mode_EventCount = 0x02,
	TMA_Mode_PluseWidth = 0x03,
	TMA_Mode_PlusePeriod= 0x04,
}TMA_Mode_t;

typedef enum
{
	TMA_Polarity_0    =  0x00,  //事件计数时：上升沿计数； 方波模式：初始电平为高电平
	TMA_Polarity_1    = 0x01,
}TMA_Polarity_t;

typedef enum
{
	TMA_Pulse_Width_High= 0x00, // measure high level width of TI 
	TMA_Pulse_Width_Low = 0x01, // measure low level width of TI
	TMA_Pulse_Rising 	= 0x02, // measure rising edge interval of TI  	
	TMA_Pulse_Falling   = 0x03, // measure falling edge interval of TI	
}TMA_Pulse_t;

typedef struct
{
    uint16_t TIMA_Edge;	   /*!< Specifies the valid edge when eventcount
									 This parameter can be a value of @ref TIMA_Edge_t */
	
	uint16_t TMA_Ctrl;		 /*!< TMA counter events under the condition of the paramters of TMA_Ctrl_t
									 This parameter can be a value of @ref TMA_Ctrl_t */
}TMA_EVNT_t;

typedef struct
{		

	uint16_t TMA_Period;            /*!< Specifies the period value to be loaded into the active
	                                   Auto-Reload Register at the next update event.This parameter must be a number between 0x0000 and 0xFFFF.	*/ 
									   	
	uint16_t TMA_Clk;     /*!< Specifies the count clock source of TIMA
	                                  This parameter can be a value of @ref TMA_Clk_t */
																	  
	TMA_Mode_t TMA_Mode; 	         /*!< Specifies the TIM work mode of different TIM channel
									      This parameter can be a value of @ref TIMA_Mode_t */

	uint8_t TMA_Polarity;	      /*!< TAIO polarity switch bit (TEDGSEL) .*/
	
	TMA_EVNT_t TMA_EventPara;		 /*!< Specifies the TMA counter condition  
										 This parameter can be a value of @ref TMA_EVNT_t */
	TMA_Pulse_t TMA_Pulse;	
}TMA_InitTypeDef; 

void TMA_Init(TMA_InitTypeDef *TIMA_InitStruct);
void TMA_Start(void);
void TMA_Stop(void);
uint32_t TMA_GetPulseWidth(void);
uint32_t TMA_GetPulsePeriod(void);

#endif
