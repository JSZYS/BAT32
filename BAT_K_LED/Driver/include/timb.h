#ifndef __TIMB_H__
#define __TIMB_H__
#include <stdint.h>
#include <stdbool.h>
#include "common.h"
/****************************************************************************/
/***********************TMB PARAMATERS***************************************/
/****************************************************************************/
#define _80_TMB_COUNT_START                       (0x80U) /* count starts */
#define _08_TMB_REGISTER_OVERFLOW                 (0x08U) /* when the TB register overflows */
#define _01_TMB_INTERRUPT_TBIMFA_ENABLE           (0x01U) /* interrupt by TBIMFA bit enabled */
#define _02_TMB_INTERRUPT_TBIMFB_ENABLE           (0x02U) /* interrupt by TBIMFB bit enabled */


typedef enum 
{
    TMB_FCLK_Div1		= 0x00U,
    TMB_FCLK_Div2		= 0x01U,
    TMB_FCLK_Div4		= 0x02U,
	TMB_FCLK_Div8		= 0x03U,
    TMB_FCLK_Div32		= 0x04U,
    TMB_FCLK_TBCLKA		= 0x05U,
    TMB_FCLK_TBCLKB		= 0x07U,
}TMB_Clk_t;
#define IS_TMB_CLOCK(CLK)	(((CLK) == TMB_FCLK_Div1) || ((CLK) == TMB_FCLK_Div2) || ((CLK) == TMB_FCLK_Div4)|| \
                                ((CLK) == TMB_FCLK_Div8)||((CLK) == TMB_FCLK_Div32) ||((CLK) == TMB_FCLK_Div1)|| \
								((CLK) == TMB_FCLK_TBCLKA) || ((CLK) == TMB_FCLK_TBCLKB))
/** @defgroup TMB_Channel_t 
  * @{
  */
#define	TMB_Channel_A   	((uint8_t)0x01)
#define	TMB_Channel_B 		((uint8_t)0x02)
#define	TMB_Channel_ELC		((uint8_t)0x04)	
#define IS_TMB_CHANNEL(CHANN)			((((CHANN) & (uint8_t)0xF8) == 0x00) && ((CHANN) != (uint8_t)0x00))

/** @defgroup TMB_Counter_Mode 
  * @{
  */
#define TMB_CounterMode_Up		((uint8_t)0x00)
#define TMB_CounterMode_Down	((uint8_t)0x10)
#define IS_TIM_COUNTER_MODE(MODE) (((MODE) == TIM_CounterMode_Up) ||  \
                                   ((MODE) == TIM_CounterMode_Down))


/** @defgroup TMB_Counter_Clear 
  * @{
  */
#define TB_Clear_Disable        ((uint8_t)0x00)
#define TB_Clear_Match_TBGRA 	((uint8_t)0x20)
#define TB_Clear_Match_TBGRB	((uint8_t)0x40)

/** @defgroup TMB_Output_Compare_Polarity 
  * @{
  */
#define TMB_Output_Disable		((uint8_t)0x00)
#define TMB_Output_Low			((uint8_t)0x01)
#define TMB_Output_High			((uint8_t)0x02)
#define TMB_Output_Toggle		((uint8_t)0x03)

/** @defgroup TMB_Input_Capture_Polarity 
  * @{
  */
#define TMB_Capture_Rising		((uint8_t)0x00)
#define TMB_Capture_Falling		((uint8_t)0x01)
#define TMB_Capture_Both		((uint8_t)0x02)

/** @defgroup TMB_Input_Capture_Clock_Source 
  * @{
  */
#define TMA_Filter_FCLK_Div32		((uint8_t)0x00)
#define TMB_Filter_FCLK_Div8		((uint8_t)0x01)
#define TMB_Filter_FCLK_Div1		((uint8_t)0x02)
#define TMB_Filter_USE_CLK		    ((uint8_t)0x03) //use TMB opreation clock


/** @defgroup TMB_WorkMode 
  * @{
  */
typedef enum
{
	TMB_Mode_Capture = 0x00,
	TMB_Mode_Compare = 0x01,
	TMB_Mode_PWM     = 0x02,
	TMB_Mode_PhaseCount = 0x03,
}TMB_Mode_t;







/** 
  * @brief  TMB Input Capture Init structure definition  
  */

typedef struct
{


  uint8_t TMB_ICPolarity;   /*!< Specifies the active edge of the input signal.
                                  This parameter can be a value of @ref TIM_Input_Capture_Polarity */

  uint8_t TMB_ICFilter;     /*!< Specifies the input capture filter.
                                  This parameter can be a number between 0x0 and 0x1 */
  uint8_t TMB_Filter_Clk;     /*!< Specifies the clock source of input capture filter .
									This parameter can be a number between 0x0 and 0xF */
} TMB_ICInitTypeDef;


typedef struct
{
	TMB_ICInitTypeDef 	TMB_CHA;	/*!< TMB input capture channelA, for TMB channelA, it has Capture Compare Register TBGRA. 
												input capture parameter can be  a value of @ref TMB_ICInitTypeDef */

	TMB_ICInitTypeDef 	TMB_CHB;	/*!< TMB input capture channelB, for TMB channelB, it has Capture Compare Register TBGRB. 
												input capture parameter can be  a value of @ref TMB_ICInitTypeDef */

}TMB_IC_Channel_t;

/** 
  * @brief  TMB Output Compare Init structure definition  
  */

typedef struct
{
	uint8_t TMB_OCPolarity;    /*!< Specifies the output polarity.
								   This parameter can be a value of @ref TIM_Output_Compare_Polarity */
} TMB_OCInitTypeDef;

typedef struct
{

	TMB_OCInitTypeDef TMB_CHA;     /*!< TMB output compare channelA, for TMB channelA, it has Compare Register TBGRA. 
												output compare parameter can be  a value of @ref TMB_OCInitTypeDef */

	TMB_OCInitTypeDef TMB_CHB;   /*!< TMB output compare channelB, for TMB channelB, it has Compare Register TBGRB. 
												output compare parameter can be  a value of @ref TMB_OCInitTypeDef */
} TMB_OC_Channel_t;



typedef struct
{		
	uint8_t TMB_Channel;     /*!< Specifie the channel of TMB
	                                  This parameter can be a value of @ref TMB_Channel_t */
									   	
	uint8_t TMB_Clk;     /*!< Specifie the count clock source of TMB
	                                  This parameter can be a value of @ref TMB_Clk_t */

	uint16_t TMB_CounterMode;       /*!< Specifies the counter mode.
									   This parameter can be a value of @ref TMB_Counter_Mode */
	
	TMB_Mode_t TMB_Mode; 	         /*!< Specifies the TIM work mode of different TIM channel
									      This parameter can be a value of @ref TMB_Mode_t */
	
	uint16_t TMB_CHA_Pulse;         /*!< Specifies the pulse value to be loaded into the Capture Compare RegisterTBGRA. 
										   This parameter can be a number between 0x0000 and 0xFFFF */

	uint16_t TMB_CHB_Pulse;         /*!< Specifies the pulse value to be loaded into the Capture Compare Register TBGRB. 
										   This parameter can be a number between 0x0000 and 0xFFFF */

	TMB_IC_Channel_t 	TMB_IC;	 /*!< TMB input capture mode channel This parameter can be a value of @ref TMB_IC_Channel_t */

	
	TMB_OC_Channel_t	TMB_OC	;	/*!< TMB output compare channel This parameter can be a value of @ref TMB_OC_Channel_t */
	
	
	uint8_t		TMB_CounterClear;	/*!< TMB counter TB Register clear conditon This parameter can be a value of @ref TMB_OC_Channel_t */
	
}TMB_InitTypeDef; 

void TMB_Init(TMB_InitTypeDef *TIMB_InitStruct);
void TMB_Start(void);
void TMB_Stop(void);
uint32_t TMB_GetPulseWidth(uint8_t channel);
void TMB_SetCounter(uint16_t period, uint16_t duty);

#endif
