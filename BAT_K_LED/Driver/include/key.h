#ifndef __KEY_INTP_H__
#define __KEY_INTP_H__
#include "common.h"
#include "gpio.h"
#define KEY_WAITTIME                    (1U)   /* change the waiting time according to the system */

#define KR0_INT   0x01
#define KR1_INT   0x02
#define KR2_INT   0x04
#define KR3_INT   0x08
#define KR4_INT   0x10
#define KR5_INT   0x20
#define KR6_INT   0x40
#define KR7_INT   0x80

#define IS_KEY_INT(KEY_INT) ((KEY_INT) != (uint8_t)0x00)


typedef struct
{
	uint8_t INT_Select;     /*!< Specifies the key interrupt trigger source.	*/
	                                        
}KEY_InitTypeDef;


void Key_Start(void);
void Key_Stop(void);
void KEY_Init(KEY_InitTypeDef* Key_InitStruct);

#endif


