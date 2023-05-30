#ifndef __INTERVAL_H__
#define __INTERVAL_H__
#include <stdio.h>
#include <stdint.h>
#include "common.h"
#define IS_COUNTER_RANGER(NUM)        ((NUM) <= 0x7fff)

/** @defgroup IT_ClkSource_definition 
  * @{
  */ 
#define IT_FSUB   0x00     //副系统时钟
#define IT_FIL    0x01     //低速内部振荡
#define IT_FHOCO  0x02     //高速内部振荡时钟
#define IT_FMX    0x03     //高速系统时钟
#define IS_IT_CLKSRC(CLK) (((CLK) == IT_FSUB) 		|| \
                                 ((CLK) == IT_FIL)  || \
                                 ((CLK) == IT_FHOCO)|| \
                                 ((CLK) == IT_FMX))
/** @defgroup IT_Frequence_definition 
  * @{
  */ 
#define IT_FreqScaler_1952	0x22     
#define IT_FreqScaler_1464	0x02     
#define IT_FreqScaler_976	0x42     
#define IT_FreqScaler_488	0x03     
#define IT_FreqScaler_244	0x43   

typedef struct
{
	uint8_t IT_ClkSource;   /*!< Specifies the interval timer clock source.
							   This parameter can be a value of @ref IT_ClkSource_definition */

	uint8_t IT_Freqscaler ;   /*!< Specifies the interval timer current working frequence when clock source choosed IT_FHOCO.
					 or IT_FMX ,This parameter can be a value of @ref IT_Frequence_definition */

	uint16_t IT_Counter; /*!< Specifies the interval timer counter.
	                    This parameter can be 0~0xfffe */

}IT_InitTypeDef;

void IT_Start(void);
void IT_Stop(void);
void IT_Init(IT_InitTypeDef *IT_InitStruct);
#endif

