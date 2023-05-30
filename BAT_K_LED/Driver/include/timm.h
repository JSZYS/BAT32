#ifndef __TIMM_H__
#define __TIMM_H__
#include <stdint.h>
#include <stdbool.h>
#include "common.h"
/****************************************************************************/
/***********************TMM PARAMATERS***************************************/
/****************************************************************************/
#define _01_TMM_TM0_COUNT_START                   	(0x01U) /* TM0 Count starts */
#define _02_TMM_TM1_COUNT_START                   	(0x02U) /* TM1 Count starts */

#define _00_TMM_TM0_COUNT_STOP_MATCH              	(0x00U) /* count stops at compare match with TMGRA0 register */
#define _04_TMM_TM0_COUNT_CONTINUES               	(0x04U) /* count continues after compare match with TMGRA0 register */
#define _00_TMM_TM1_COUNT_STOP_MATCH              	(0x00U) /* count stops at compare match with TMGRA1 register */
#define _08_TMM_TM1_COUNT_CONTINUES                 (0x08U) /* count continues after compare match with TMGRA1 register */

#define _01_TMB_INTERRUPT_TBIMFA_ENABLE           	(0x01U) /* interrupt by TBIMFA bit enabled */
#define _02_TMB_INTERRUPT_TBIMFB_ENABLE           	(0x02U) /* interrupt by TBIMFB bit enabled */

#define _10_TMM_TMGRC0_BUFFER                       (0x10U) /* buffer register for TMGRA0 register */
#define _20_TMM_TMGRD0_BUFFER                       (0x20U) /* buffer register for TMGRB0 register */
#define _40_TMM_TMGRC1_BUFFER                       (0x40U) /* buffer register for TMGRA1 register */
#define _80_TMM_TMGRD1_BUFFER                       (0x80U) /* buffer register for TMGRB1 register */

#define TMM_FLAG_OVFLOW					((uint8_t)0x10)
#define TMM_FLAG_MATCHD               	((uint8_t)0x08)
#define TMM_FLAG_MATCHC               	((uint8_t)0x04)
#define TMM_FLAG_MATCHB               	((uint8_t)0x02)
#define TMM_FLAG_MATCHA					((uint8_t)0x01)
#define IS_TMM_FLAG(FLAG)			((((FLAG) & (uint8_t)0xE0) == 0x00) && ((FLAG) != (uint8_t)0x00))

/** @defgroup TMM_t 
  * @{
  */
typedef enum 
{
	TMM0 = 0x00,
	TMM1 = 0x01,	
}TMMSelect_TypeDef;
#define IS_TMM_SELECT(TMMx) (((TMMx) == TMM0) || ((TMMx) == TMM1))
/** @defgroup TMM_Clk_t 
  * @{
  */
typedef enum 
{
    TMM_FCLK_Div1		= 0x00U,
    TMM_FCLK_Div2		= 0x01U,
    TMM_FCLK_Div4		= 0x02U,
	TMM_FCLK_Div8		= 0x03U,
    TMM_FCLK_Div32		= 0x04U,
    TMM_FCLK_TMCLK		= 0x05U,
}TMM_Clk_t;
#define IS_TMM_CLOCK(CLK)	(((CLK) == TMM_FCLK_Div1) || ((CLK) == TMM_FCLK_Div2) || ((CLK) == TMM_FCLK_Div4)|| \
                                ((CLK) == TMM_FCLK_Div8)||((CLK) == TMM_FCLK_Div32) ||((CLK) == TMM_FCLK_Div1)|| \
								((CLK) == TMM_FCLK_TMCLK))
/** @defgroup TMM_Channel_t 
  * @{
  */
#define	TMM_Channel_A   	((uint8_t)0x01)
#define	TMM_Channel_B 		((uint8_t)0x02)
#define	TMM_Channel_C   	((uint8_t)0x04)
#define	TMM_Channel_D 		((uint8_t)0x08)
#define	TMM_Channel_ELC		((uint8_t)0x10)	
#define IS_TMM_CHANNEL(CHANN)			((((CHANN) & (uint8_t)0xE0) == 0x00) && ((CHANN) != (uint8_t)0x00))


/** @defgroup TMM_Counter_Clear_t 
  * @{
  */
#define TM_Clear_Disable        ((uint8_t)0x00)
#define TM_Clear_Match_TMGRA 	((uint8_t)0x20)
#define TM_Clear_Match_TMGRB	((uint8_t)0x40)
#define	TM_Clear_Sychronous   	((uint8_t)0x60)  // clear by the other timer under the circumstances of TMMR resister bit TMSYNC is 1
#define TM_Clear_Match_TMGRC 	((uint8_t)0xA0)
#define TM_Clear_Match_TMGRD	((uint8_t)0xC0)

/** @defgroup TMM_Combine_TransMode_t 
  * @{
  */
#define TMM_Trans_TM1_Underflow 		((uint8_t)0x02) /*data transfer from buffer register to general register when TM1 underflows */
#define TMM_Trans_TM0_Match				((uint8_t)0x03) /* data transfer from buffer register to general register when TM0 compare match TMGRA0 */

/** @defgroup TMM_Interrupt_t 
  * @{
  */
#define TMM_Int_NONE        ((uint8_t)0x00)
#define TMM_Int_IMIA 		((uint8_t)0x01)
#define TMM_Int_IMIB		((uint8_t)0x02)
#define	TMM_Int_IMIC   		((uint8_t)0x04)  
#define TMM_Int_IMID 		((uint8_t)0x08)
#define TMM_Int_OVIE 		((uint8_t)0x10) /* interrupt (OVI) by bits OVF and UDF enabled */

/** @defgroup TMM_Output_Compare_Polarity 
  * @{
  */
#define TMM_Output_Disable		((uint8_t)0x00)
#define TMM_Output_Low			((uint8_t)0x01)
#define TMM_Output_High			((uint8_t)0x02)
#define TMM_Output_Toggle		((uint8_t)0x03)

/** @defgroup TMM_Output_Compare_Initlevel 
  * @{
  */
#define TMM_OCInit_Low_Level		((uint8_t)0x00)
#define TMM_OCInit_High_Level		((uint8_t)0x01)

/** @defgroup TMM_Input_Capture_Polarity 
  * @{
  */
#define TMM_Capture_Rising		((uint8_t)0x00)
#define TMM_Capture_Falling		((uint8_t)0x01)
#define TMM_Capture_Both		((uint8_t)0x02)

/** @defgroup TMM_Input_Capture_Clock_Source 
  * @{
  */
#define TMM_Filter_FCLK_Div32		((uint8_t)0x00)
#define TMM_Filter_FCLK_Div8		((uint8_t)0x01)
#define TMM_Filter_FCLK_Div1		((uint8_t)0x02)
#define TMM_Filter_USE_CLK		    ((uint8_t)0x03) //use TMM opreation clock

/** @defgroup TMM_Input_Capture_Clock_Source 
  * @{
  */
#define TMM_GeneralRegister_Setting		0
#define TMM_BufferRegister_Setting		1

/** 
  * @brief  TMM_ICFilter_Set definition  
  */
#define TMM_ICFilter_Disable		0
#define TMM_ICFilter_Enable			1

/** @defgroup TMM_WorkMode 
  * @{
  */
typedef enum
{
	TMM_Mode_Capture 	= 0x00,
	TMM_Mode_Compare	= 0x01,
	TMM_Mode_PWM     	= 0x02,// normal PWM mode
	TMM_Mode_PWM_RSTSYN = 0x03,// RstSynchronous_PWM mode
	TMM_Mode_CompPWM 	= 0x04,//	complementary PWM mode
	TMM_Mode_PWM3 		= 0x05,//	output two PWM with same period
}TMM_Mode_t;




/** 
  * @brief  TMM Input Capture Init structure definition  
  */

typedef struct
{


  uint8_t TMM_ICPolarity;   /*!< Specifies the active edge of the input signal.
                                  This parameter can be a value of @ref TIM_Input_Capture_Polarity */

  uint8_t TMM_ICFilter;     /*!< Specifies the input capture filter flag.@ref TMM_ICFilter_Set,This parameter can be a number between 0x0 and 0x1 */
                                  
} TMM_ICInitTypeDef;


typedef struct
{
	uint8_t TMM_Filter_Clk;     /*!< Specifies the clock source of input capture filter .
									This parameter can be a number between 0x0 and 0x03, it provides clock source when using filter */

	TMM_ICInitTypeDef 	TMM_CHA;	/*!< TMM input capture channelA, for TMM channelA, it has Capture Compare Register TMGRA. 
												input capture parameter can be  a value of @ref TMM_ICInitTypeDef */

	TMM_ICInitTypeDef 	TMM_CHB;	/*!< TMM input capture channelB, for TMM channelB, it has Capture Compare Register TMGRB. 
												input capture parameter can be  a value of @ref TMM_ICInitTypeDef */

	TMM_ICInitTypeDef 	TMM_CHC;	/*!< TMM input capture channelC, for TMM channelC, it has Capture Compare Register TMGRC. 
												input capture parameter can be  a value of @ref TMM_ICInitTypeDef */
	uint8_t TMM_CHC_AS_BufferRegister;/*the flag of channelC worked as buffer Register for channelA,it @ref TMM_BufferRegister_Set*/
	
	TMM_ICInitTypeDef 	TMM_CHD;	/*!< TMM input capture channelD, for TMM channelD, it has Capture Compare Register TMGRD. 
												input capture parameter can be  a value of @ref TMM_ICInitTypeDef */
	uint8_t TMM_CHD_AS_BufferRegister;/*the flag of channelD worked as buffer Register for channelB,it @ref TMM_BufferRegister_Set*/

}TMM_IC_Channel_t;

/** 
  * @brief  TMM Output Compare Init structure definition  
  */

typedef struct
{
	uint8_t TMM_OCPolarity;    /*!< Specifies the output polarity.
								   This parameter can be a value of @ref TIM_Output_Compare_Polarity */
	uint8_t TMM_OCInitLevel;    /*!< Specifies the output init level.
								   This parameter can be a value of @ref TIM_Output_Compare_InitLevel */

} TMM_OCInitTypeDef;

/** 
  * @brief  TMM_OC_Output_PinCtrl  
  */
#define TMM_OC_Output_PinCtrl_Disable   ((uint8_t)0x00)
#define TMM_OC_Output_PinCtrl_Enable    ((uint8_t)0x01)

typedef struct
{

	TMM_OCInitTypeDef TMM_CHA;     /*!< TMM output compare channelA, for TMM channelA, it has Compare Register TMGRA. 
												output compare parameter can be  a value of @ref TMM_OCInitTypeDef */

	TMM_OCInitTypeDef TMM_CHB;   /*!< TMM output compare channelB, for TMM channelB, it has Compare Register TMGRB. 
												output compare parameter can be  a value of @ref TMM_OCInitTypeDef */

	TMM_OCInitTypeDef TMM_CHC;     /*!< TMM output compare channelC, for TMM channelC, it has Compare Register TMGRC. 
											output compare parameter can be  a value of @ref TMM_OCInitTypeDef */
	
	uint8_t TMM_CHC_OutPin_Ctrl;     /*!< TMM output compare channelC, Compare Register TMGRC, combined with Compare Register TMGRA,
											used to control TMIOA output pin. if TMM_OC_Output_PinCtrl_Enable is setted, TMIOA output pin can output PWM wave
											TMM_CHD_OutPin_Ctrl parameter can be  a value of @ref TMM_OC_Output_PinCtrl*/
	
	TMM_OCInitTypeDef TMM_CHD;   /*!< TMM output compare channelD, for TMM channelD, it has Compare Register TMGRD. 
											output compare parameter can be  a value of @ref TMM_OCInitTypeDef */
	
	uint8_t TMM_CHD_OutPin_Ctrl;     /*!< TMM output compare channelD, Compare Register TMGRD, combined with Compare Register TMGRB,
											used to control TMIOB output pin. if TMM_OC_Output_PinCtrl_Enable is setted, TMIOB output pin can output PWM wave
											TMM_CHD_OutPin_Ctrl parameter can be  a value of @ref TMM_OC_Output_PinCtrl*/
								

} TMM_OC_Channel_t;


/** 
  * @brief  TMM_PWM_InitLevel
  */
#define TMM_PWMInitLevel_Low		((uint8_t)0x00)	
#define TMM_PWMInitLevel_High		((uint8_t)0x01)	

/** 
  * @brief  TMM_PWM_ActiveLevel
  */
#define TMM_PWMActiveLevel_Low		((uint8_t)0x00)	
#define TMM_PWMActiveLevel_High		((uint8_t)0x01)	

/** 
  * @brief  TMM PWM Init structure definition  
  */
typedef struct
{
	uint8_t TMM_PWMInitLevel;    /*!< Specifies the output init level.
								   This parameter can be a value of @ref TIM_PWM_InitLevel */
	
	uint8_t TMM_PWMActiveLevel;    /*!< Specifies the output active level.
								   This parameter can be a value of @ref TIM_PWM_ActiveLevel */

} TMM_PWMInitTypeDef;




typedef struct
{

	TMM_PWMInitTypeDef TMM_CHA;     /*!< TMM output compare channelA, for TMM channelA, it has Compare Register TMGRA. 
												output compare parameter can be  a value of @ref TMM_OCInitTypeDef */

	TMM_PWMInitTypeDef TMM_CHB;   /*!< TMM output compare channelB, for TMM channelB, it has Compare Register TMGRB. 
												output compare parameter can be  a value of @ref TMM_OCInitTypeDef */

	TMM_PWMInitTypeDef TMM_CHC;     /*!< TMM output compare channelC, for TMM channelC, it has Compare Register TMGRC. 
											output compare parameter can be  a value of @ref TMM_OCInitTypeDef */
		
	TMM_PWMInitTypeDef TMM_CHD;   /*!< TMM output compare channelD, for TMM channelD, it has Compare Register TMGRD. 
											output compare parameter can be  a value of @ref TMM_OCInitTypeDef */
									

} TMM_PWM_Channel_t;

/** 
  * @brief TMM_Output_State_Selection 
  */
#define TMM_InitHigh_ActiveLow             (0x00U) /*  initial high level and active level is low level */
#define TMM_InitLow_ActiveHigh             (0x01U) /* initial low level and active level is high level */


/** 
  * @brief  BDTR structure definition 
  */

typedef struct
{

	uint8_t TMM_PostiveState;        /*!< Specifies the Postive output level selection .
									  This parameter can be a value of @ref TMM_Output_State_Selection */

	uint8_t TMM_NegativeState;        /*!< Specifies the Negative output level selection .
									  This parameter can be a value of @ref TMM_Output_State_Selection */

	uint8_t TMM_Break;            /*!< Specifies whether the TIM Break input is enabled or not. 
									  This parameter can be a value of @ref TIM_Break_Input_enable_disable */

	uint16_t TMM_DeadTime;         /*!< Specifies the delay time between the switching-off and the
								  switching-on of the outputs.
								  This parameter can be a number between 0x00 and 0xFF  */

	uint16_t TMIOA_BreakPolarity;    /*!< Specifies the TMIOA Break Output pin polarity.
                                      This parameter can be a value of @ref TMM_Break_Polarity */

	uint16_t TMIOB_BreakPolarity;    /*!< Specifies the TMIOB Break Output pin polarity.
                                      This parameter can be a value of @ref TMM_Break_Polarity */

	uint16_t TMIOC_BreakPolarity;    /*!< Specifies the TMIOC Break Output pin polarity.
                                      This parameter can be a value of @ref TMM_Break_Polarity */

	uint16_t TMIOD_BreakPolarity;    /*!< Specifies the TMIOD Break Output pin polarity.
                                      This parameter can be a value of @ref TMM_Break_Polarity */

} TMM_BDTRInitTypeDef;

typedef struct
{
	TMMSelect_TypeDef	TMM_Select;		/*!< Specifie the TMM uint of TMM, it can be TMM0 or TMM1
	                                  This parameter can be a value of @ref TMM_t */
	
	uint8_t TMM_Channel;     /*!< Specifie the channel of TMM
	                                  This parameter can be a value of @ref TMM_Channel_t */
										   	
	uint8_t TMM_Clk;     /*!< Specifie the count clock source of TMM
	                                  This parameter can be a value of @ref TMM_Clk_t */
	
	TMM_Mode_t TMM_Mode; 	         /*!< Specifies the TIM work mode of different TIM channel
									      This parameter can be a value of @ref TMM_Mode_t */
	
	uint16_t TMM_CHA_Pulse;         /*!< Specifies the pulse value to be loaded into the Capture Compare RegisterTMGRA. 
										   This parameter can be a number between 0x0000 and 0xFFFF */

	uint16_t TMM_CHB_Pulse;         /*!< Specifies the pulse value to be loaded into the Capture Compare Register TMGRB. 
										   This parameter can be a number between 0x0000 and 0xFFFF */

	uint16_t TMM_CHC_Pulse;         /*!< Specifies the pulse value to be loaded into the Capture Compare RegisterTMGRC. 
										   This parameter can be a number between 0x0000 and 0xFFFF */

	uint16_t TMM_CHD_Pulse;         /*!< Specifies the pulse value to be loaded into the Capture Compare Register TBMGRD. 
										   This parameter can be a number between 0x0000 and 0xFFFF */

	TMM_IC_Channel_t 	TMM_IC;	 /*!< TMM input capture mode channel This parameter can be a value of @ref TMM_IC_Channel_t */

	
	TMM_OC_Channel_t	TMM_OC	;	/*!< TMM output compare channel This parameter can be a value of @ref TMM_OC_Channel_t */
	
	TMM_PWM_Channel_t	TMM_PWM	;	/*!< TMM used as PWM channel This parameter can be a value of @ref TMM_PWM_Channel_t */
	
	TMM_BDTRInitTypeDef TMM_BDTR;	/*!< TMM used to set PWM break and deadtime, This parameter can be a value of @ref TMM_BDTRInitTypeDef */
	
	uint8_t		TMM_CounterClear;	/*!< TMM counter TM Register clear conditon This parameter can be a value of @ref TMM_Counter_Clear_t */
	
	uint8_t		TMM_Combine_TransMode;	/*!< TMM counter TM buffer Register transfer mode: 1¡¢buffer register copyed to general register when TM1 
										underflows 2¡¢buffer register copyed to general register when TM0 compare match to  TMGRA0; it only used 
											in complementary PWM mode This parameter can be a value of @ref TMM_Combine_TransMode_t */

	uint8_t TMM_Intp;         /*!< Specifies the whether open the interrupt flag of Capture Compare Register TBGRA.TBGRB/TBGRC/TBGRD 
										   when ocurs TM match Capture Compare Register,it will generates interrupt
										   This parameter can be  @ref TMM_Interrupt_t */	
}TMM_InitTypeDef; 

typedef struct 
{
	volatile uint8_t			*TMMR;
	volatile uint8_t			*TMPMR;
	volatile uint8_t			*TMFCR;
	volatile uint8_t            *TMOER[2];	
	volatile uint8_t			*TMOCR;	
	volatile uint8_t			*TMCR[2];	
	volatile uint8_t			*TMIORA[2];	
	volatile uint8_t			*TMIORC[2];			
	volatile uint8_t			*TMIER[2];	
	volatile uint8_t            *TMPOCR[2];
    volatile uint8_t            *TMSR;

	volatile uint8_t            *TMDF[2];
	volatile uint8_t            *TMOER1;
    volatile uint8_t            *OPTL0;
	
	volatile uint16_t            *TMGRA;
	volatile uint16_t            *TMGRB;
	volatile uint16_t            *TMGRC;
	volatile uint16_t            *TMGRD;

}TMM_REG_t;

void TMM_Init(TMM_InitTypeDef *TMM_InitStruct);
void TMM1_Start(FunctionalState match_flag);
void TMM0_Start(FunctionalState match_flag);
void TMM0_Stop(void);
void TMM_All_Start(void);
void TMM_All_Stop(void);
FlagStatus TMM_GetFlagStatus(TMMSelect_TypeDef TMMx, uint8_t TMM_FLAG);

uint32_t TMM_GetPulseWidth(TMMSelect_TypeDef TMMx, uint8_t channel);

#endif
