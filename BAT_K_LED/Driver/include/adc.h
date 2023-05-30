#ifndef __ADC_H__
#define __ADC_H__
#include <stdio.h>
#include <stdint.h>
#include "common.h"

#define ADC_Start_Cmp                           ((uint8_t)0x80)       
#define ADC_Enable                              ((uint8_t)0x01) 

typedef enum
{
	ADC_ScanChanRes_NoId    = 0x00,  
	ADC_ScanChanRes_WithId   = 0x02
} ADC_ScanChanRes_t;

typedef enum
{
	ADC_Mode_Select = 0x00,  
	ADC_Mode_Scan   = 0x80
}ADC_Mode_t;
#define IS_ADC_MODE(MODE)			((MODE == ADC_Mode_Select) || (MODE == ADC_Mode_Scan))

typedef enum
{
	ADC_ConvSpeed_High  		= 0x00,  //high speed up to 64Mhz
	ADC_ConvSpeed_LowCurrent    = 0x03  //low current speed up to 27Mhz
}ADC_ConvSpeed_t;

typedef enum
{
	ADC_Prescaler_Div32  = 0x00,  
	ADC_Prescaler_Div16  = 0x01,
	ADC_Prescaler_Div8   = 0x02,
	ADC_Prescaler_Div4   = 0x03,
	ADC_Prescaler_Div2   = 0x04,
	ADC_Prescaler_Div1   = 0x05,
}ADC_Prescaler_t;



typedef enum
{
	ADC_Ref_Vdd      = 0x00,
	ADC_Ref_Extern   = 0x01,
	ADC_Ref_Internal = 0x10,	
}ADC_Ref_t;

typedef enum
{
	ADC_ExternalTrig_Software = 0x00,
	ADC_ExternalTrig_Hardware_NoWait = 0x80,
	ADC_ExternalTrig_Hardware_Wait = 0xC0,	
}ADC_ExternalTrigConv_t;
#define IS_ADC_TRIGGER(trig)		( (trig == ADC_ExternalTrig_Software)   	|| \
									  (trig == ADC_ExternalTrig_Hardware_NoWait)|| \
									  (trig == ADC_ExternalTrig_Hardware_Wait) )


#define ADC_UpLimit_Setting                             ((uint8_t)0xFF)
#define ADC_LowLimit_Setting                            ((uint8_t)0x00)


#define ADC_TwoSamplingDelay_5p5Cycles                 ((uint8_t)0x05) // 5.5 clk
#define ADC_TwoSamplingDelay_8p5Cycles                 ((uint8_t)0x08) // 8.5 clk
#define ADC_TwoSamplingDelay_10p5Cycles                ((uint8_t)0x0A) // 10.5 clk
#define ADC_TwoSamplingDelay_13p5Cycles                ((uint8_t)0x0D) // 13.5 clk
#define ADC_TwoSamplingDelay_17p5Cycles                ((uint8_t)0x11) // 17.5 clk
#define ADC_TwoSamplingDelay_21p5Cycles                ((uint8_t)0x15) // 21.5 clk
#define ADC_TwoSamplingDelay_255p5Cycles               ((uint8_t)0xFF) // 255.5 clk

typedef enum
{
    ADTES_Normal = 0U,      /* normal */
    ADTES_Zero_Code = 1U,   /* zero code test */
    ADTES_Half_Code = 3U,   /* half code test */
    ADTES_Full_Code = 5U,   /* full code test */
    ADTES_Free_Code = 6U    /* free code test */
} ADC_Test_t;

typedef enum
{
	ADC_HardwareTrig_INTTM01 = 0x00, // TIM1 count end or capture end interrupt
	ADC_HardwareTrig_ELC     = 0x01, // ELC 
	ADC_HardwareTrig_INTRTC  = 0x02, // RTC
	ADC_HardwareTrig_INTIT   = 0x03, // Interval timer interrupt
} ADC_HardwareTrig_t;
#define IS_ADC_HARDTRIGGER(trig)	( (trig == ADC_HardwareTrig_INTTM01)  || \
									  (trig == ADC_HardwareTrig_ELC)   	  || \
									  (trig == ADC_HardwareTrig_INTRTC)   || \
									  (trig == ADC_HardwareTrig_ELC)  )
#define	ADC_ScanNum_2	2
#define	ADC_ScanNum_3	3
#define	ADC_ScanNum_4	4

#define	ADC_Sign_Enable		0x01
#define	ADC_Sign_Disable	0x00

typedef struct
{
	uint8_t ADC_ScanNum;
	uint8_t ADC_ChannelSign;
}ADC_ScanNum_t;

typedef enum
{    
    ADC_Channel_0,    //P20
	ADC_Channel_1,    //P21
	ADC_Channel_2,    //P22
	ADC_Channel_3,    //P23
#ifndef  BAT32G137_32PIN	
	ADC_Channel_4,    //P24
	ADC_Channel_5,    //P25
	ADC_Channel_6,    //P26
	ADC_Channel_7,    //P27
#endif
	ADC_Channel_8,    //P11
	ADC_Channel_9,    //P10
	ADC_Channel_10,   //P03 
	ADC_Channel_11,   //P02
	ADC_Channel_12,   //P147 
#ifdef  BAT32G137_64PIN 
	ADC_Channel_13,   //P04 
#endif
	ADC_Channel_14,   //P120 
#ifndef  BAT32G137_32PIN
	ADC_Channel_15,   //P146 
#endif
	ADC_Channel_OFF = 31U,   
	ADC_Channel_PGA0 = 32U,
	ADC_Channel_PGA1 = 64U,	
	ADC_TemperSensor0 = 128U,    
	ADC_InterRefVolt
} ADC_Channel_t;
#define IS_SCAN_START_CHAN(CHAN)			(CHAN < ADC_Channel_12)

typedef enum 
{
	ADC_Conv_Continuous = 0x00, //sequential convertion mode
	ADC_Conv_Oneshot = 0x08 //one-shot convertion mode
} ADC_FuncState_t;

typedef struct
{
	ADC_Mode_t ADC_Mode;						/*!< Configures the ADC to operate in select or scan mode. 
												 This parameter can be a value of @ref ADC_mode */											   
	ADC_Prescaler_t ADC_Prescaler;				 /*!< Select the frequency of the clock to the ADC. The clock is common for all the ADCs.*/
																							   	
	ADC_Ref_t ADC_RefVoltage;                /*!< Select the external ref voltage or VDD . 
	                                           This parameter can be a value of @ref ADC_Ref_t */
	
	ADC_ConvSpeed_t  ADC_ConvSpeed;               /*!< Select the adc convert speed    
													This parameter can be a value of @ref ADC_ConvSpeed_t */ 
	
	ADC_ExternalTrigConv_t ADC_ExternalTrigConv;    /*!< Select the external event used to trigger the start of conversion of a regular group.
	                                           This parameter can be a value of @ref ADC_ExternalTrigConv_t for regular channels conversion */
	
	ADC_HardwareTrig_t ADC_HardwareTrigSour;		  /*!< Select the hardware trigger source  This parameter can be a value of @ref ADC_HardwareTrig_t */
											 
	ADC_ScanNum_t ADC_ScanConf;   /*!< Select the adc scan channel num  when in scan mode.
	                                           This parameter can be a value of @ref ADC_ScanNum_t */
	
	uint8_t ADC_UpLimit; 			       /*!< Configures the ADC result(high 8 bit) comprision uppest limit,used to generate INTAD 
											  This parameter can be a value of @ref ADC_UpLimit */	
											  	
	uint8_t ADC_LowLimit; 				 /*!< Configures the ADC result(high 8 bit) comprision lowest limit,used to generate INTAD 
											This parameter can be a value of @ref ADC_LowLimit */								  

	uint8_t ADC_TwoSamplingDelay;			 /*!< Configures the Delay between 2 sampling phases.*/ 				  

	ADC_FuncState_t ADC_ContinuousConvMode; /*!< Specifies whether the conversion  is performed in Continuous or Single conversion.											 
											  This parameter can be set to ENABLE or DISABLE. */	
}ADC_InitTypeDef;

void ADC_Init(ADC_InitTypeDef* ADC_InitStruct);
void ADC_Start(ADC_Channel_t ch);
void ADC_Stop(void);
uint16_t ADC_Converse(ADC_Channel_t ch, uint32_t times, uint16_t *buf);
uint16_t ADC_Converse_Scan(ADC_Channel_t ch, uint32_t times, uint16_t *buf);
void ADC_Set_HardTrigger(uint8_t wait, ADC_HardwareTrig_t trg);
uint16_t ADC_SelfTest(ADC_Test_t tes, uint32_t times, uint16_t *buf);

#endif

