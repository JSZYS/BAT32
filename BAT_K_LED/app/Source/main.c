#include "BAT32A239.h"
#include "led.h"
#include "delay_dm.h"

int main()
{
	  SystemCoreClockUpdate();
    	
	  delay_init(SystemCoreClock);
	  led_Init();
	  
	
	  while(1)
		{
			  LED1_ON;
			  LED2_OFF;
			  delay_ms(500);
			 
        LED1_OFF;
			  LED2_ON;
			  delay_ms(500);
			
			  LED1_ON;
			  LED2_OFF;
			  delay_ms(500);
			
			  LED1_OFF;
			  LED2_ON;
			  delay_ms(500);
		}
}

