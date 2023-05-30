#include "led.h"

void led_Init()
{
	 GPIO_InitTypeDef gpio_init  =  {0};                        //这里要赋值0，不然容易卡死在端口初始化里面
	 
	 gpio_init.GPIO_Pin          =  GPIO_Pin_1 | GPIO_Pin_2;    //配置要使用的引脚
	 gpio_init.GPIO_Mode         =  GPIO_Mode_OUT;              //配置引脚的模式
	 GPIO_Init(GPIO_PORT7, &gpio_init);                         //初始化端口
}




