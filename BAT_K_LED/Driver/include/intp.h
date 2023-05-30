#ifndef __INTP_H__
#define __INTP_H__
#include "common.h"
#include "gpio.h"
#define KEY_WAITTIME                    (1U)   /* change the waiting time according to the system */


#define INTP0     0x0001
#define INTP1     0x0002
#define INTP2     0x0004
#define INTP3     0x0008
#define INTP4     0x0010
#define INTP5     0x0020
#define INTP6     0x0040
#define INTP7     0x0080

#define INTP8     0x0100
#define INTP9     0x0200
#define INTP10    0x0400
#define INTP11    0x0800

#define IS_INTP_TYPE(INTP) ((((INTP) & (uint16_t)0xF000) == 0x0000) && ((INTP) != (uint16_t)0x0000))

typedef enum
{
	Trigger_None,
	Trigger_Falling ,
	Trigger_Rising ,	
	Trigger_Rising_Falling
}Trigger_TypeDef;

typedef struct
{
	uint16_t INTP_Select;     /*!< Specifies the trigger signal active edge for the EXTI lines.
                                         This parameter can be a value of @ref Trigger_TypeDef */
	
	Trigger_TypeDef EXTI_Trigger; /*!< Specifies the trigger signal active edge for the EXTI lines.
                                         This parameter can be a value of @ref Trigger_TypeDef */
}INTP_InitTypeDef;

void INTP_Init(INTP_InitTypeDef* Intp_InitStruct);

#endif
