#include "led.h"

void led_Init()
{
	 GPIO_InitTypeDef gpio_init  =  {0};                        //����Ҫ��ֵ0����Ȼ���׿����ڶ˿ڳ�ʼ������
	 
	 gpio_init.GPIO_Pin          =  GPIO_Pin_1 | GPIO_Pin_2;    //����Ҫʹ�õ�����
	 gpio_init.GPIO_Mode         =  GPIO_Mode_OUT;              //�������ŵ�ģʽ
	 GPIO_Init(GPIO_PORT7, &gpio_init);                         //��ʼ���˿�
}




