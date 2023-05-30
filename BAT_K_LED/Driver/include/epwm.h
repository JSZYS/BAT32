#ifndef __EPWM__H_
#define __EPWM__H_
#include "common.h"
#define		EPWM_CHANNUM	8 
/** @defgroup EPWM_Channel 
  * @{
  */ 
#define EPWM_CHAN_0    	 		0x01   
#define EPWM_CHAN_1     		0x02    
#define EPWM_CHAN_2     		0x04   
#define EPWM_CHAN_3     		0x08   
#define EPWM_CHAN_4     		0x10    
#define EPWM_CHAN_5     		0x20    
#define EPWM_CHAN_6    			0x40  
#define EPWM_CHAN_7     		0x80  
#define IS_EPWM_CHAN(CHAN)			( ((CHAN) != (uint8_t)0x00))
/** @defgroup EPWM_Source 
  * @{
  */ 
#define EPWM_SRC_TO01(chan)   	(0x00)  
#define EPWM_SRC_TO03(chan)   	(chan)

/** @defgroup EPWM_OutPhase 
  * @{
  */ 
#define EPWM_OutPhase_Normal(chan)  (0x00)  
#define EPWM_OutPhase_Reverse(chan)	(chan)  

/** @defgroup EPWM_Cutoff_t 
  * @{
  */ 
#define EPWM_CutoffSrc_None     0x00 
#define EPWM_CutoffSrc_CMP0     0x01 
#define EPWM_CutoffSrc_INTP0    0x02 
#define EPWM_CutoffSrc_EVENTC   0x03  

#define EPWM_SRising_EFalling   0x00  //cutoff start when rising edge; cutoff end when falling edge  
#define EPWM_SFalling_ERising   0x04  //cutoff start when falling edge; cutoff end when rising edge

#define EPWM_Release_Software   0x00  //software release
#define EPWM_Release_Hardware   0x08  //hardware release

#define EPWM_Release_Immediate  0x00  //cutoff release immediate when received release signal 
#define EPWM_Release_NextRising 0x10  //cutoff release at next rising edge

/** @defgroup EPWM_Cutoff_Outlevel 
  * @{
  */ 


#define EPWM_Cutoff_Out_BAN(chan)   (EPWM_GetNum(chan))  //ban cutoff
#define EPWM_Cutoff_Out_HIZ(chan)  	(1<<EPWM_GetNum(chan))  //HI-Z output
#define EPWM_Cutoff_Out_LOW(chan) 	(2<<EPWM_GetNum(chan))  //low level output
#define EPWM_Cutoff_Out_HIGH(chan)	(3<<EPWM_GetNum(chan))  //high level output


typedef struct
{
	uint8_t Source;
	uint8_t Edge;
	uint8_t ReleaseMode;
	uint8_t ReleaseTiming;
}EPWM_Cutoff_t;

typedef struct
{
	uint8_t	 EPWM_Channel;	 /*!< Specifies EPWM channel This parameter can be a value of @ref EPWM_Channel */
									 	
	uint8_t  EPWM_Source;    /*!< Specifies EPWM input source it can be : timer40 channel 1 or channel 3 output
									 This parameter can be a value of @ref EPWM_Source */
	
	uint8_t  EPWM_OutPhase;  /*!< Specifies EPWM output Phase: Normal or Reverse 
									 This parameter can be a value of @ref EPWM_OutPhase */
	
	EPWM_Cutoff_t  EPWM_CutoffPara;  /*!< Specifies EPWM cutoff paramter  
											This parameter can be a value of @ref EPWM_Cutoff_t */
	
	uint16_t  EPWM_Cutoff_Outlv;   /*!< Specifies EPWM output level when cutoff release  
											This parameter can be a value of @ref EPWM_Cutoff_Outlevel */
}EPWM_InitTypeDef;

void EPWM_Init(EPWM_InitTypeDef *EPWM_InitStruct);
uint8_t EPWM_GetNum(uint8_t channel);

#endif 
