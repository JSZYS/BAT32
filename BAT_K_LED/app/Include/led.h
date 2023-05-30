#ifndef __LED_H__
#define __LED_H__

#include "gpio.h"
#define LED1_ON    GPIO_ResetBits(GPIO_PORT7, GPIO_Pin_1);
#define LED1_OFF   GPIO_SetBits(GPIO_PORT7, GPIO_Pin_1);
#define LED2_ON    GPIO_ResetBits(GPIO_PORT7, GPIO_Pin_2);
#define LED2_OFF   GPIO_SetBits(GPIO_PORT7, GPIO_Pin_2);

void led_Init(void);


#endif
 




