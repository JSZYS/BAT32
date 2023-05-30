#ifndef __DAC_H__
#define __DAC_H__
#include <stdio.h>
#include <stdint.h>
#include "common.h"
#include "isr.h"

typedef enum
{
	DAC_Channel_0  =0x01,
	DAC_Channel_1  =0x02,	
	DAC_Channle_All=0x03,
}DAC_Channel_t;

typedef enum
{
	DAC_Mode_Normal,                   
	DAC_Mode_RealTime,
}DAC_Mode_t;
typedef struct
{
	DAC_Channel_t DAC_Channel;                 /*!< Configures the DAC channel.
												 This parameter can be a value of @ref DAC_Channel_t */	
	
	DAC_Mode_t    DAC_Mode;						/*!< Configures the DAC working mode. 
												 This parameter can be a value of @ref DAC_Mode_t */											   

	uint8_t       DAC_ConValue;                 /*!< Configures the DAC convertion value*/ 
												 
}DAC_InitTypeDef;


void DAC_Start(DAC_Channel_t ch);
void DAC_Init(DAC_InitTypeDef* DAC_InitStruct);
void DAC_Stop(DAC_Channel_t ch);
void DAC_Set_Value(DAC_Channel_t ch, uint8_t regvalue);
void DAC_Set_PowerOff(void);


#endif
