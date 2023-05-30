#ifndef __PGA_H__
#define __PGA_H__
#include <stdio.h>
#include <stdint.h>
#include "common.h"

typedef enum 
{    
	PGA_Channel_0 = 1,  // PGA0    
	PGA_Channel_1 = 2,  // PGA1    
	PGA_Channel_A = 3   // PGA0 & PGA1
} PGA_Channel_t;

typedef enum 
{    
    PGA_PGAIN_Ref_Voltage     = 0,  // Select PGAIN as PGA+ input 
	PGA_PGAGND_Ref_Voltage    = 1,	// Select PGAGND as PGA+ input	
} PGA_Vref_t;

typedef enum 
{    
    PGA_Gain_4  = 0, // Gain Amplifier:*4 
	PGA_Gain_8  = 1, // *8	 
	PGA_Gain_10	= 2, // *10
	PGA_Gain_12	= 3, // *12
	PGA_Gain_14	= 4, // *14
	PGA_Gain_16	= 5, // *16
	PGA_Gain_32	= 6, // *32
} PGA_Gain_t;
typedef enum
{
	 PGA_Out_Disable = 0, // disable PGAOUT output to port.
	 PGA_Out_Enable  = 1, //Enable PGAOUT output to port.
}PGA_Out_t;

typedef struct
{
	PGA_Channel_t PGA_Channel;	/*!< Specifies the channel of pga.*/
										   	
	PGA_Vref_t PGA_Vref;/*!< Specifies the GND selection of feedback resistance
								 This parameter can be a value of @ref PGA_Vref_t */
	PGA_Gain_t PGA_Gain;/*!< Specifies the Programmable gain amplifier amplification factor selection 
								 This parameter can be a value of @ref PGA_Gain_t */
	PGA_Out_t 	PGA_Out2Port;	/*!< Specifies the PGAOUT whether output to port or not
								 This parameter can be a value of @ref PGA_Out_t */					 
}PGA_InitTypeDef;


void PGA_Init(PGA_InitTypeDef *PGA_InitStruct);
void PGA_Start(PGA_Channel_t ch);
void PGA_Stop(PGA_Channel_t ch);

#endif
