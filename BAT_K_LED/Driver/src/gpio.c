/*******************************************************************************
*                  COPYRIGHT (C) 2021 CMS Technologies Ltd.                    *
*                                                                              *
********************************************************************************
* FileName      : cms_gpio.c
* Author        : 
* Version       : 1.0
* Date          : 2021.08.13
* Description   :
* Function List :
********************************************************************************/
#include "gpio.h"
#define GPIO_PIOR_CFG_BASE     &PORT->PIOR2

void assert_failed(uint8_t* file, uint32_t line)
{
	while(1);
}


#ifdef  BAT32G1XX_100PIN
GPIO_PIN_AF_TABLE_t funcAfTab[] =
{
		{GROUP_AF_INTP1, {{GPIO_PIOR0,PIOR_BIT0,GPIO_P52},  {GPIO_PIOR4,PIOR_BIT5,GPIO_P56},  {PIOR_NONE,PIOR_BIT0,GPIO_DEF}}},
		{GROUP_AF_INTP2, {{GPIO_PIOR0,PIOR_BIT0,GPIO_P53},  {GPIO_PIOR4,PIOR_BIT5,GPIO_P47},  {PIOR_NONE,PIOR_BIT0,GPIO_DEF}}},
		{GROUP_AF_INTP3, {{GPIO_PIOR0,PIOR_BIT0,GPIO_P54},  {GPIO_PIOR4,PIOR_BIT5,GPIO_P57},  {PIOR_NONE,PIOR_BIT0,GPIO_DEF}}},
		{GROUP_AF_INTP4, {{GPIO_PIOR0,PIOR_BIT0,GPIO_P55},  {GPIO_PIOR4,PIOR_BIT5,GPIO_P146},  {PIOR_NONE,PIOR_BIT0,GPIO_DEF}}},
		{GROUP_AF_INTP5,  {{GPIO_PIOR0,PIOR_BIT4,GPIO_P12}, {PIOR_NONE,PIOR_BIT0,GPIO_DEF},  {PIOR_NONE,PIOR_BIT0,GPIO_DEF}}},
		{GROUP_AF_INTP6,  {{GPIO_PIOR4,PIOR_BIT5,GPIO_P84}, {PIOR_NONE,PIOR_BIT0,GPIO_DEF},  {PIOR_NONE,PIOR_BIT0,GPIO_DEF}}},
		{GROUP_AF_INTP7,  {{GPIO_PIOR4,PIOR_BIT5,GPIO_P85}, {PIOR_NONE,PIOR_BIT0,GPIO_DEF},  {PIOR_NONE,PIOR_BIT0,GPIO_DEF}}},
		{GROUP_AF_INTP8, {{GPIO_PIOR0,PIOR_BIT0,GPIO_P42},  {GPIO_PIOR0,PIOR_BIT7,GPIO_P00}, {GPIO_PIOR4,PIOR_BIT5,GPIO_P86}}},
		{GROUP_AF_INTP9, {{GPIO_PIOR0,PIOR_BIT0,GPIO_P43},  {GPIO_PIOR4,PIOR_BIT5,GPIO_P87},  {PIOR_NONE,PIOR_BIT0,GPIO_DEF}}},
		{GROUP_AF_INTP10,{{GPIO_PIOR0,PIOR_BIT1,GPIO_P05},	 {GPIO_PIOR0,PIOR_BIT7,GPIO_P01}, {GPIO_PIOR4,PIOR_BIT1,GPIO_P110}}},
		{GROUP_AF_INTP11,{{GPIO_PIOR0,PIOR_BIT1,GPIO_P06},  {GPIO_PIOR0,PIOR_BIT7,GPIO_P20}, {GPIO_PIOR4,PIOR_BIT1,GPIO_P111}}},

		{GROUP_AF_SDO00, {{GPIO_PIOR0,PIOR_BIT1,GPIO_P17},  {PIOR_NONE,PIOR_BIT0,GPIO_DEF},  {PIOR_NONE,PIOR_BIT0,GPIO_DEF}}},
		{GROUP_AF_SDI00, {{GPIO_PIOR0,PIOR_BIT1,GPIO_P16},  {PIOR_NONE,PIOR_BIT0,GPIO_DEF},  {PIOR_NONE,PIOR_BIT0,GPIO_DEF}}},
		{GROUP_AF_SCK00,{{GPIO_PIOR0,PIOR_BIT1,GPIO_P55},  {PIOR_NONE,PIOR_BIT0,GPIO_DEF},  {PIOR_NONE,PIOR_BIT0,GPIO_DEF}}},
		{GROUP_AF_CLKBUZ1,{{GPIO_PIOR0,PIOR_BIT4,GPIO_P55},{PIOR_NONE,PIOR_BIT0,GPIO_DEF},  {PIOR_NONE,PIOR_BIT0,GPIO_DEF}}},

		{GROUP_AF_RXD2,  {{GPIO_PIOR0,PIOR_BIT1,GPIO_P76},  {PIOR_NONE,PIOR_BIT0,GPIO_DEF},  {PIOR_NONE,PIOR_BIT0,GPIO_DEF}}},
		{GROUP_AF_TXD2,  {{GPIO_PIOR0,PIOR_BIT1,GPIO_P77},  {PIOR_NONE,PIOR_BIT0,GPIO_DEF},  {PIOR_NONE,PIOR_BIT0,GPIO_DEF}}},

		{GROUP_AF_TXD0,  {{GPIO_PIOR0,PIOR_BIT1,GPIO_P17},  {GPIO_PIOR3,PIOR_BIT4,GPIO_P40}, {GPIO_PIOR3,PIOR_BIT5,GPIO_P12}}},
		{GROUP_AF_RXD0,  {{GPIO_PIOR0,PIOR_BIT1,GPIO_P16},  {GPIO_PIOR3,PIOR_BIT4,GPIO_P137},{GPIO_PIOR3,PIOR_BIT5,GPIO_P11}}},
		{GROUP_AF_SCLAA0, {{GPIO_PIOR0,PIOR_BIT2,GPIO_P14},  {PIOR_NONE,PIOR_BIT0,GPIO_DEF},  {PIOR_NONE,PIOR_BIT0,GPIO_DEF}}},
		{GROUP_AF_SDAA0, {{GPIO_PIOR0,PIOR_BIT2,GPIO_P15},  {PIOR_NONE,PIOR_BIT0,GPIO_DEF},  {PIOR_NONE,PIOR_BIT0,GPIO_DEF}}},
		{GROUP_AF_CLKBUZ0,{{GPIO_PIOR0,PIOR_BIT3,GPIO_P31},{PIOR_NONE,PIOR_BIT0,GPIO_DEF},  {PIOR_NONE,PIOR_BIT0,GPIO_DEF}}},

		{GROUP_AF_TAIO,   {{GPIO_PIOR1,PIOR_BIT0,GPIO_P31}, {GPIO_PIOR1,PIOR_BIT1,GPIO_P41}, {GPIO_PIOR1,PIOR_CBIT10,GPIO_P06}}},
		{GROUP_AF_TAO,   {{GPIO_PIOR1,PIOR_BIT2,GPIO_P50}, {GPIO_PIOR1,PIOR_BIT3,GPIO_P00}, {PIOR_NONE,PIOR_BIT0,GPIO_DEF}}},
		{GROUP_AF_CRXD,  {{GPIO_PIOR3,PIOR_BIT3,GPIO_P50},  {PIOR_NONE,PIOR_BIT0,GPIO_DEF},  {PIOR_NONE,PIOR_BIT0,GPIO_DEF}}},
		{GROUP_AF_CTXD,  {{GPIO_PIOR3,PIOR_BIT3,GPIO_P51},  {PIOR_NONE,PIOR_BIT0,GPIO_DEF},  {PIOR_NONE,PIOR_BIT0,GPIO_DEF}}},		
		
		{GROUP_AF_TI_TO14,  {{GPIO_PIOR4,PIOR_BIT7,GPIO_P42}, {PIOR_NONE,PIOR_BIT0,GPIO_DEF}, {PIOR_NONE,PIOR_BIT0,GPIO_DEF}}},
		{GROUP_AF_TI_TO15,  {{GPIO_PIOR4,PIOR_BIT7,GPIO_P46}, {PIOR_NONE,PIOR_BIT0,GPIO_DEF}, {PIOR_NONE,PIOR_BIT0,GPIO_DEF}}},
		{GROUP_AF_TI_TO16,  {{GPIO_PIOR4,PIOR_BIT7,GPIO_P102},  {PIOR_NONE,PIOR_BIT0,GPIO_DEF},  {PIOR_NONE,PIOR_BIT0,GPIO_DEF}}},
		{GROUP_AF_TI_TO17,  {{GPIO_PIOR4,PIOR_BIT7,GPIO_P145},  {PIOR_NONE,PIOR_BIT0,GPIO_DEF},  {PIOR_NONE,PIOR_BIT0,GPIO_DEF}}},
		{GROUP_AF_SCLK31,   {{GPIO_PIOR4,PIOR_BIT6,GPIO_P54}, {PIOR_NONE,PIOR_BIT0,GPIO_DEF}, {PIOR_BIT0,PIOR_CBIT10,GPIO_DEF}}},
		{GROUP_AF_SCL31,    {{GPIO_PIOR4,PIOR_BIT6,GPIO_P54}, {PIOR_NONE,PIOR_BIT0,GPIO_DEF}, {PIOR_BIT0,PIOR_CBIT10,GPIO_DEF}}},
		{GROUP_AF_SDA31,    {{GPIO_PIOR4,PIOR_BIT6,GPIO_P53},  {PIOR_NONE,PIOR_BIT0,GPIO_DEF},  {PIOR_NONE,PIOR_BIT0,GPIO_DEF}}},
		
		{GROUP_AF_SCLK10,   {{GPIO_PIOR4,PIOR_BIT5,GPIO_P80}, {PIOR_NONE,PIOR_BIT0,GPIO_DEF}, {PIOR_BIT0,PIOR_BIT0,GPIO_DEF}}},
		{GROUP_AF_SCL10,    {{GPIO_PIOR4,PIOR_BIT5,GPIO_P80}, {PIOR_NONE,PIOR_BIT0,GPIO_DEF}, {PIOR_BIT0,PIOR_BIT0,GPIO_DEF}}},	
		{GROUP_AF_SDI10,    {{GPIO_PIOR4,PIOR_BIT5,GPIO_P81},  {PIOR_NONE,PIOR_BIT0,GPIO_DEF},  {PIOR_NONE,PIOR_BIT0,GPIO_DEF}}},
		{GROUP_AF_RXD1,     {{GPIO_PIOR4,PIOR_BIT5,GPIO_P81},  {PIOR_NONE,PIOR_BIT0,GPIO_DEF},  {PIOR_NONE,PIOR_BIT0,GPIO_DEF}}},		
		{GROUP_AF_SDO10,    {{GPIO_PIOR4,PIOR_BIT5,GPIO_P82},  {PIOR_NONE,PIOR_BIT0,GPIO_DEF},  {PIOR_NONE,PIOR_BIT0,GPIO_DEF}}},
		{GROUP_AF_TXD1,     {{GPIO_PIOR4,PIOR_BIT5,GPIO_P82},  {PIOR_NONE,PIOR_BIT0,GPIO_DEF},  {PIOR_NONE,PIOR_BIT0,GPIO_DEF}}},		
		{GROUP_AF_SDA10,    {{GPIO_PIOR4,PIOR_BIT5,GPIO_P81},  {PIOR_NONE,PIOR_BIT0,GPIO_DEF},  {PIOR_NONE,PIOR_BIT0,GPIO_DEF}}},

		{GROUP_AF_ODEFAULT,{{PIOR_NONE,PIOR_BIT0,GPIO_DEF}, {PIOR_NONE,PIOR_BIT0,GPIO_DEF}, {PIOR_NONE,PIOR_BIT0,GPIO_DEF}}},	
};
#elif defined(BAT32G1XX_64PIN) || defined(BAT32G1XX_80PIN)
GPIO_PIN_AF_TABLE_t funcAfTab[] =
{
		{GROUP_AF_INTP1, {{GPIO_PIOR0,PIOR_BIT0,GPIO_P52},  {PIOR_NONE,PIOR_BIT0,GPIO_DEF},  {PIOR_NONE,PIOR_BIT0,GPIO_DEF}}},
		{GROUP_AF_INTP2, {{GPIO_PIOR0,PIOR_BIT0,GPIO_P53},  {PIOR_NONE,PIOR_BIT0,GPIO_DEF},  {PIOR_NONE,PIOR_BIT0,GPIO_DEF}}},
		{GROUP_AF_INTP3, {{GPIO_PIOR0,PIOR_BIT0,GPIO_P54},  {PIOR_NONE,PIOR_BIT0,GPIO_DEF},  {PIOR_NONE,PIOR_BIT0,GPIO_DEF}}},
		{GROUP_AF_INTP4, {{GPIO_PIOR0,PIOR_BIT0,GPIO_P55},  {PIOR_NONE,PIOR_BIT0,GPIO_DEF},  {PIOR_NONE,PIOR_BIT0,GPIO_DEF}}},
		{GROUP_AF_INTP5,  {{GPIO_PIOR0,PIOR_BIT4,GPIO_P12}, {PIOR_NONE,PIOR_BIT0,GPIO_DEF},  {PIOR_NONE,PIOR_BIT0,GPIO_DEF}}},	
		{GROUP_AF_INTP8, {{GPIO_PIOR0,PIOR_BIT0,GPIO_P42},  {GPIO_PIOR0,PIOR_BIT7,GPIO_P00}, {PIOR_NONE,PIOR_BIT0,GPIO_DEF}}},
		{GROUP_AF_INTP9, {{GPIO_PIOR0,PIOR_BIT0,GPIO_P43},  {PIOR_NONE,PIOR_BIT0,GPIO_DEF},  {PIOR_NONE,PIOR_BIT0,GPIO_DEF}}},
		{GROUP_AF_INTP10,{{GPIO_PIOR0,PIOR_BIT1,GPIO_P05},	 {GPIO_PIOR0,PIOR_BIT7,GPIO_P01}, {PIOR_NONE,PIOR_BIT0,GPIO_DEF}}},
		{GROUP_AF_INTP11,{{GPIO_PIOR0,PIOR_BIT1,GPIO_P06},  {GPIO_PIOR0,PIOR_BIT7,GPIO_P20}, {PIOR_NONE,PIOR_BIT0,GPIO_DEF}}},
		{GROUP_AF_SDO00, {{GPIO_PIOR0,PIOR_BIT1,GPIO_P17},  {PIOR_NONE,PIOR_BIT0,GPIO_DEF},  {PIOR_NONE,PIOR_BIT0,GPIO_DEF}}},
		{GROUP_AF_SDI00, {{GPIO_PIOR0,PIOR_BIT1,GPIO_P16},  {PIOR_NONE,PIOR_BIT0,GPIO_DEF},  {PIOR_NONE,PIOR_BIT0,GPIO_DEF}}},
		{GROUP_AF_SCK00,{{GPIO_PIOR0,PIOR_BIT1,GPIO_P55},  {PIOR_NONE,PIOR_BIT0,GPIO_DEF},  {PIOR_NONE,PIOR_BIT0,GPIO_DEF}}},
		{GROUP_AF_CLKBUZ1,{{GPIO_PIOR0,PIOR_BIT4,GPIO_P55},{PIOR_NONE,PIOR_BIT0,GPIO_DEF},  {PIOR_NONE,PIOR_BIT0,GPIO_DEF}}},

		{GROUP_AF_RXD2,  {{GPIO_PIOR0,PIOR_BIT1,GPIO_P76},  {PIOR_NONE,PIOR_BIT0,GPIO_DEF},  {PIOR_NONE,PIOR_BIT0,GPIO_DEF}}},
		{GROUP_AF_TXD2,  {{GPIO_PIOR0,PIOR_BIT1,GPIO_P77},  {PIOR_NONE,PIOR_BIT0,GPIO_DEF},  {PIOR_NONE,PIOR_BIT0,GPIO_DEF}}},

		{GROUP_AF_TXD0,  {{GPIO_PIOR0,PIOR_BIT1,GPIO_P17},  {GPIO_PIOR3,PIOR_BIT4,GPIO_P40}, {GPIO_PIOR3,PIOR_BIT5,GPIO_P12}}},
		{GROUP_AF_RXD0,  {{GPIO_PIOR0,PIOR_BIT1,GPIO_P16},  {GPIO_PIOR3,PIOR_BIT4,GPIO_P137},{GPIO_PIOR3,PIOR_BIT5,GPIO_P11}}},
		{GROUP_AF_SCLAA0, {{GPIO_PIOR0,PIOR_BIT2,GPIO_P14},  {PIOR_NONE,PIOR_BIT0,GPIO_DEF},  {PIOR_NONE,PIOR_BIT0,GPIO_DEF}}},
		{GROUP_AF_SDAA0, {{GPIO_PIOR0,PIOR_BIT2,GPIO_P15},  {PIOR_NONE,PIOR_BIT0,GPIO_DEF},  {PIOR_NONE,PIOR_BIT0,GPIO_DEF}}},
		{GROUP_AF_CLKBUZ0,{{GPIO_PIOR0,PIOR_BIT3,GPIO_P31},{PIOR_NONE,PIOR_BIT0,GPIO_DEF},  {PIOR_NONE,PIOR_BIT0,GPIO_DEF}}},

		{GROUP_AF_VCOUT0,{{GPIO_PIOR3,PIOR_BIT0,GPIO_P70},{GPIO_PIOR2,PIOR_BIT0,GPIO_P120},  {PIOR_NONE,PIOR_BIT0,GPIO_DEF}}},
		{GROUP_AF_VCOUT1,{{GPIO_PIOR3,PIOR_BIT1,GPIO_P71},{GPIO_PIOR2,PIOR_BIT1,GPIO_P31},  {PIOR_NONE,PIOR_BIT0,GPIO_DEF}}},

		{GROUP_AF_TAIO,   {{GPIO_PIOR1,PIOR_BIT0,GPIO_P31}, {GPIO_PIOR1,PIOR_BIT1,GPIO_P41}, {GPIO_PIOR1,PIOR_CBIT10,GPIO_P06}}},
		{GROUP_AF_TAO,   {{GPIO_PIOR1,PIOR_BIT2,GPIO_P50}, {GPIO_PIOR1,PIOR_BIT3,GPIO_P00}, {PIOR_NONE,PIOR_BIT0,GPIO_DEF}}},
		{GROUP_AF_CRXD,  {{GPIO_PIOR3,PIOR_BIT3,GPIO_P50},  {PIOR_NONE,PIOR_BIT0,GPIO_DEF},  {PIOR_NONE,PIOR_BIT0,GPIO_DEF}}},
		{GROUP_AF_CTXD,  {{GPIO_PIOR3,PIOR_BIT3,GPIO_P51},  {PIOR_NONE,PIOR_BIT0,GPIO_DEF},  {PIOR_NONE,PIOR_BIT0,GPIO_DEF}}},		
		{GROUP_AF_ODEFAULT,{{PIOR_NONE,PIOR_BIT0,GPIO_DEF}, {PIOR_NONE,PIOR_BIT0,GPIO_DEF}, {PIOR_NONE,PIOR_BIT0,GPIO_DEF}}},	
};
#elif defined BAT32G1XX_48PIN	
GPIO_PIN_AF_TABLE_t funcAfTab[] =
{
		{GROUP_AF_INTP8,    {{PIOR_NONE,PIOR_BIT0,GPIO_DEF},  {GPIO_PIOR0,PIOR_BIT7,GPIO_P00}, {PIOR_NONE,PIOR_BIT0,GPIO_DEF}}},
		{GROUP_AF_RXD1,     {{GPIO_PIOR0,PIOR_BIT5,GPIO_P73},  {PIOR_NONE,PIOR_BIT0,GPIO_DEF},  {PIOR_NONE,PIOR_BIT0,GPIO_DEF}}},
		{GROUP_AF_TXD1,     {{GPIO_PIOR0,PIOR_BIT5,GPIO_P72},  {PIOR_NONE,PIOR_BIT0,GPIO_DEF},  {PIOR_NONE,PIOR_BIT0,GPIO_DEF}}},

		{GROUP_AF_RXD2,     {{GPIO_PIOR1,PIOR_BIT0,GPIO_DEF},  {GPIO_PIOR0,PIOR_BIT6,GPIO_P14},  {PIOR_NONE,PIOR_BIT0,GPIO_DEF}}},
		{GROUP_AF_TXD2,     {{GPIO_PIOR1,PIOR_BIT0,GPIO_DEF},  {GPIO_PIOR0,PIOR_BIT6,GPIO_P10},  {PIOR_NONE,PIOR_BIT0,GPIO_DEF}}},

		{GROUP_AF_TXD0,     {{GPIO_PIOR0,PIOR_BIT1,GPIO_P17},  {GPIO_PIOR3,PIOR_BIT4,GPIO_P40}, {GPIO_PIOR3,PIOR_BIT5,GPIO_P12}}},
		{GROUP_AF_RXD0,     {{GPIO_PIOR0,PIOR_BIT1,GPIO_P16},  {GPIO_PIOR3,PIOR_BIT4,GPIO_P137},{PIOR_NONE,PIOR_BIT0,GPIO_DEF}}},
		{GROUP_AF_SCLAA0,    {{GPIO_PIOR0,PIOR_BIT2,GPIO_P14},  {PIOR_NONE,PIOR_BIT0,GPIO_DEF},  {PIOR_NONE,PIOR_BIT0,GPIO_DEF}}},
		{GROUP_AF_SDAA0,    {{GPIO_PIOR0,PIOR_BIT2,GPIO_P15},  {PIOR_NONE,PIOR_BIT0,GPIO_DEF},  {PIOR_NONE,PIOR_BIT0,GPIO_DEF}}},
		{GROUP_AF_CLKBUZ0,  {{GPIO_PIOR0,PIOR_BIT3,GPIO_P31},{PIOR_NONE,PIOR_BIT0,GPIO_DEF},   {PIOR_NONE,PIOR_BIT0,GPIO_DEF}}},

		{GROUP_AF_TAIO,   {{GPIO_PIOR1,PIOR_BIT0,GPIO_P31}, {GPIO_PIOR1,PIOR_BIT1,GPIO_P41}, {GPIO_PIOR1,PIOR_CBIT10,GPIO_P06}}},
		{GROUP_AF_TAO,   {{GPIO_PIOR1,PIOR_BIT2,GPIO_P50}, {GPIO_PIOR1,PIOR_BIT3,GPIO_P00}, {PIOR_NONE,PIOR_BIT0,GPIO_DEF}}},
		{GROUP_AF_CRXD,  {{GPIO_PIOR3,PIOR_BIT3,GPIO_P50},  {PIOR_NONE,PIOR_BIT0,GPIO_DEF},  {PIOR_NONE,PIOR_BIT0,GPIO_DEF}}},
		{GROUP_AF_CTXD,  {{GPIO_PIOR3,PIOR_BIT3,GPIO_P51},  {PIOR_NONE,PIOR_BIT0,GPIO_DEF},  {PIOR_NONE,PIOR_BIT0,GPIO_DEF}}},		
		{GROUP_AF_ODEFAULT ,{{PIOR_NONE,PIOR_BIT0,GPIO_DEF},  {PIOR_NONE,PIOR_BIT0,GPIO_DEF},   {PIOR_NONE,PIOR_BIT0,GPIO_DEF}}},
};
#elif defined BAT32G1XX_32PIN	
GPIO_PIN_AF_TABLE_t funcAfTab[] =
{
		{GROUP_AF_RXD1,     {{GPIO_PIOR0,PIOR_BIT5,GPIO_P73},  {PIOR_NONE,PIOR_BIT0,GPIO_DEF},  {PIOR_NONE,PIOR_BIT0,GPIO_DEF}}},
		{GROUP_AF_TXD1,     {{GPIO_PIOR0,PIOR_BIT5,GPIO_P72},  {PIOR_NONE,PIOR_BIT0,GPIO_DEF},  {PIOR_NONE,PIOR_BIT0,GPIO_DEF}}},

		{GROUP_AF_RXD2,     {{GPIO_PIOR1,PIOR_BIT0,GPIO_DEF},  {GPIO_PIOR0,PIOR_BIT6,GPIO_P14},  {PIOR_NONE,PIOR_BIT0,GPIO_DEF}}},
		{GROUP_AF_TXD2,     {{GPIO_PIOR1,PIOR_BIT0,GPIO_DEF},  {GPIO_PIOR0,PIOR_BIT6,GPIO_P10},  {PIOR_NONE,PIOR_BIT0,GPIO_DEF}}},

		{GROUP_AF_TXD0,     {{GPIO_PIOR0,PIOR_BIT1,GPIO_P17},  {GPIO_PIOR3,PIOR_BIT4,GPIO_P40}, {GPIO_PIOR3,PIOR_BIT5,GPIO_P12}}},
		{GROUP_AF_RXD0,     {{GPIO_PIOR0,PIOR_BIT1,GPIO_P16},  {GPIO_PIOR3,PIOR_BIT4,GPIO_P137},{PIOR_NONE,PIOR_BIT0,GPIO_DEF}}},
		{GROUP_AF_SCLA0,    {{GPIO_PIOR0,PIOR_BIT2,GPIO_P14},  {PIOR_NONE,PIOR_BIT0,GPIO_DEF},  {PIOR_NONE,PIOR_BIT0,GPIO_DEF}}},
		{GROUP_AF_SDAA0,    {{GPIO_PIOR0,PIOR_BIT2,GPIO_P15},  {PIOR_NONE,PIOR_BIT0,GPIO_DEF},  {PIOR_NONE,PIOR_BIT0,GPIO_DEF}}},

		{GROUP_AF_TAIO,   {{GPIO_PIOR1,PIOR_BIT0,GPIO_P31}, {GPIO_PIOR1,PIOR_BIT1,GPIO_P41}, {GPIO_PIOR1,PIOR_CBIT10,GPIO_P06}}},
		{GROUP_AF_TAO,   {{GPIO_PIOR1,PIOR_BIT2,GPIO_P50}, {GPIO_PIOR1,PIOR_BIT3,GPIO_P00}, {PIOR_NONE,PIOR_BIT0,GPIO_DEF}}},
		{GROUP_AF_CRXD,  {{GPIO_PIOR3,PIOR_BIT3,GPIO_P50},  {PIOR_NONE,PIOR_BIT0,GPIO_DEF},  {PIOR_NONE,PIOR_BIT0,GPIO_DEF}}},
		{GROUP_AF_CTXD,  {{GPIO_PIOR3,PIOR_BIT3,GPIO_P51},  {PIOR_NONE,PIOR_BIT0,GPIO_DEF},  {PIOR_NONE,PIOR_BIT0,GPIO_DEF}}},		
		{GROUP_AF_ODEFAULT ,{{PIOR_NONE,PIOR_BIT0,GPIO_DEF},  {PIOR_NONE,PIOR_BIT0,GPIO_DEF},   {PIOR_NONE,PIOR_BIT0,GPIO_DEF}}},
};
#endif
/**
 * @brief Set specified GPIO as output function. 
 *
 * @param port port address, such as &P0, &P1, &P2...
 * @param pinMsk 
 *             e.g., bit0: 0x01, bit1: 0x02, bit0~3: 0x0F, bit0~7: 0xFF
 */
void GPIO_Output_Enable(__IO uint8_t *port, uint8_t pinMsk)
{
    *(port - 0x2A0) &= ~pinMsk;  /*!< PMC=0: Digital Function   */
    *(port + 0x020) &= ~pinMsk;  /*!< PM =0: Output Function    */
}

/**
 * @brief Set specified GPIO as input function. 
 *
 * @param port port address, such as &P0, &P1, &P2...
 * @param pinMsk 
 *             e.g., bit0: 0x01, bit1: 0x02, bit0~3: 0x0F, bit0~7: 0xFF
 */
void GPIO_Input_Enable(__IO uint8_t *port, uint8_t pinMsk)
{
    *(port - 0x2A0) &= ~pinMsk;  /*!< PMC=0: Digital Function   */
    *(port + 0x020) |=  pinMsk;  /*!< PM =1: Input Function     */
}

/**
 * @brief Enable pull up resister of input GPIO . 
 *
 * @param port port address, such as &P0, &P1, &P2...
 * @param pinMsk 
 *             e.g., bit0: 0x01, bit1: 0x02, bit0~3: 0x0F, bit0~7: 0xFF
 */
void GPIO_PullUp_Enable(__IO uint8_t *port, uint8_t pinMsk)
{
    *(port - 0x2D0) |=  pinMsk;  /*!< PU =1: Pull Up enable         */
}

/**
 * @brief Disable pull up resister of input GPIO . 
 *
 * @param port port address, such as &P0, &P1, &P2...
 * @param pinMsk 
 *             e.g., bit0: 0x01, bit1: 0x02, bit0~3: 0x0F, bit0~7: 0xFF
 */
void GPIO_PullUp_Disable(__IO uint8_t *port, uint8_t pinMsk)
{
    *(port - 0x2D0) &=  ~pinMsk;  /*!< PU =0: Pull Up disable        */
}

/**
 * @brief Set specified value to GPIO output
 *
 * @param port port address, such as &P0, &P1, &P2...
 * @param value 
 */
void GPIO_Set_Value(__IO uint8_t *port, uint8_t value)
{
    *port = value;           /*!< PL = value */
}

/**
 * @brief Get value from GPIO input 
 *
 * @param port port address, such as &P0, &P1, &P2...
 *
 * @return 
 */
uint8_t GPIO_Get_Value(__IO uint8_t *port)
{
//    PORT->PMS = 0x01;        /*!< Digital output level of the pin is read */        
    return (*port);          /*!< PL = value                              */
}

/**
 * @brief check GPIO . 
 *
 * @param port, such as P0, P1, P2...
 * @param pinMsk 
 *             e.g., bit0: 0x01, bit1: 0x02, bit0~3: 0x0F, bit0~7: 0xFF
 */
int GPIO_PinCheck(GPIO_Port_t PORTx, uint16_t GPIO_Pin)
{
	if(PORTx == GPIO_PORT0)
	{
		if(GPIO_Pin > GPIO_Pin_6)
			return GPIO_ERR;
	}
	if(PORTx == GPIO_PORT3)
	{
		if(GPIO_Pin > GPIO_Pin_1)
			return GPIO_ERR;
	}
	if(PORTx == GPIO_PORT4)
	{
		if(GPIO_Pin > GPIO_Pin_5)
			return GPIO_ERR;
	}
	if(PORTx == GPIO_PORT5)
	{
		if(GPIO_Pin > GPIO_Pin_5)
			return GPIO_ERR;
	}
	if(PORTx == GPIO_PORT12)
	{
		if(GPIO_Pin > GPIO_Pin_4)
			return GPIO_ERR;
	}	
	if(PORTx == GPIO_PORT13)
	{
		if((GPIO_Pin > GPIO_Pin_0) &&(GPIO_Pin < GPIO_Pin_6))
			return GPIO_ERR;
	}	
	if(PORTx == GPIO_PORT14)
	{
		if(GPIO_Pin == GPIO_Pin_5)
			return GPIO_ERR;
	}	
	return GPIO_SUCC;
}

void GPIO_Init(GPIO_Port_t PORTx,GPIO_InitTypeDef* GPIO_InitStruct)
{
	uint16_t pinpos = 0x00,pinnum=0x00, pos = 0x00, currentpin = 0x00;

	assert_param(IS_GPIO_ALL_PERIPH(PORTx));	  
	assert_param(IS_GPIO_PIN(GPIO_InitStruct->GPIO_Pin));	  
	assert_param(IS_GPIO_MODE(GPIO_InitStruct->GPIO_Mode));
	assert_param(IS_GPIO_Control(GPIO_InitStruct->GPIO_Ctrl));

	for(pinpos = 0x00; pinpos < 0x08; pinpos++)
	{
		pos = ((uint16_t)0x01) << pinpos;
		pinnum = (GPIO_InitStruct->GPIO_Pin) & pos;
		if(pinnum == pos)
		{
			currentpin = pinpos;
			
			if(GPIO_InitStruct->GPIO_Level == GPIO_Level_HIGH)
			{
				*((volatile uint8_t*)(&PORT->P0 + PORTx))|= (1<<currentpin);
			}
			else
			{
				*((volatile uint8_t*)(&PORT->P0 + PORTx))&= ~(1<<currentpin);
			}
			if(GPIO_InitStruct->GPIO_Ctrl == GPIO_Control_DIG ) /*digitial input/output*/
			{
				*((volatile uint8_t*)(&PORT->PMC0 +PORTx)) &= ~((!GPIO_InitStruct->GPIO_Ctrl) << currentpin);
				if(GPIO_InitStruct->GPIO_Mode == GPIO_Mode_OUT)
				{
					*((volatile uint8_t*)(&PORT->PM0 +PORTx)) &= ~((!GPIO_InitStruct->GPIO_Mode) << currentpin);
					assert_param(IS_GPIO_OTYPE(GPIO_InitStruct->GPIO_OType));
					if(GPIO_InitStruct->GPIO_OType == GPIO_OType_PP)
					{
						*((volatile uint8_t*)(&PORT->POM0 +PORTx)) &= ~((!GPIO_InitStruct->GPIO_OType) << currentpin);
					}
					else
					{
						*((volatile uint8_t*)(&PORT->POM0 +PORTx)) |= GPIO_InitStruct->GPIO_OType << currentpin;
					}					
				}
				else
				{
					*((volatile uint8_t*)(&PORT->PM0 +PORTx)) |= (GPIO_InitStruct->GPIO_Mode << currentpin);
				}
			}
			else /*analogy input*/                        
			{
				*((volatile uint8_t*)(&PORT->PMC0 +PORTx)) |= GPIO_InitStruct->GPIO_Ctrl << currentpin;
			}
			assert_param(IS_GPIO_PUPD(GPIO_InitStruct->GPIO_PuPd));	
			if(GPIO_InitStruct->GPIO_PuPd == GPIO_PuPd_UP)
			{
				*((volatile uint8_t*)(&PORT->PU0 +PORTx)) |= (1<<currentpin);
			}
			else if(GPIO_InitStruct->GPIO_PuPd == GPIO_PuPd_DOWN)
			{
//						*((volatile uint8_t*)(&PORT->PD0 +GPIOx)) |= (1<<currentpin);
			}
			else
			{
			}
		}
	}
}

/**
  * @brief  Sets the selected data port bits.
  * @note   This functions uses P register to allow atomic read/modify 
  *         accesses. 
  * @param  PORTx: where x can be (0..14) to select the GPIO peripheral for BAT32G137 devices
  * @param  GPIO_Pin: specifies the port bits to be written.
  *          This parameter can be any combination of GPIO_Pin_x where x can be (0..7).
  * @retval None
  */
void GPIO_SetBits(GPIO_Port_t  PORTx, uint16_t GPIO_Pin)
{ 
	/* Check the parameters */
	assert_param(IS_GPIO_ALL_PERIPH(PORTx));
	assert_param(IS_GPIO_PIN(GPIO_Pin));

	*((volatile uint8_t*)(&PORT->P0 +PORTx))|= GPIO_Pin;

}

/**
  * @brief  Clears the selected data port bits.
  * @note   This functions uses P register to allow atomic read/modify 
  *         accesses. 
  * @param  PORTx: where x can be (0..14) to select the GPIO peripheral for BAT32G137 devices
  * @param  GPIO_Pin: specifies the port bits to be written.
  *          This parameter can be any combination of GPIO_Pin_x where x can be (0..7).
  * @retval None
  */
void GPIO_ResetBits(GPIO_Port_t PORTx, uint16_t GPIO_Pin)
{
	/* Check the parameters */
	assert_param(IS_GPIO_ALL_PERIPH(PORTx));
	assert_param(IS_GPIO_PIN(GPIO_Pin));

	*((volatile uint8_t*)(&PORT->P0 +PORTx)) &= ~GPIO_Pin;
}

/**
  * @brief  toggle the selected data port bits.
  * @note   This functions uses P register to allow atomic read/modify 
  *         accesses. 
  * @param  PORTx: where x can be (0..14) to select the GPIO peripheral for BAT32G137 devices
  * @param  GPIO_Pin: specifies the port bits to be written.
  *          This parameter can be any combination of GPIO_Pin_x where x can be (0..7).
  * @retval None
  */
void GPIO_ToggleBits(GPIO_Port_t PORTx, uint16_t GPIO_Pin)
{
	/* Check the parameters */
	assert_param(IS_GPIO_ALL_PERIPH(PORTx));
	assert_param(IS_GPIO_PIN(GPIO_Pin));

	*((volatile uint8_t*)(&PORT->P0 +PORTx)) ^= GPIO_Pin;
}
/**
  * @brief  Reads the specified input port pin.
  * @param  PORTx: where x can be (0..14) to select the GPIO peripheral for BAT32G137 devices
  * @param  GPIO_Pin: specifies the port bits to be written.
  *          This parameter can be any combination of GPIO_Pin_x where x can be (0..7).
  * @retval The input port pin value.
  */
uint8_t GPIO_ReadInputDataBit(GPIO_Port_t PORTx, uint16_t GPIO_Pin)
{
	uint8_t bitstatus = 0x00;

	/* Check the parameters */
	assert_param(IS_GPIO_ALL_PERIPH(PORTx));
	assert_param(IS_GET_GPIO_PIN(GPIO_Pin));

	if(((*((volatile uint8_t*)(&PORT->P0 +PORTx))) & GPIO_Pin) != Bit_RESET)
	{
		bitstatus = (uint8_t)Bit_SET;
	}
	else
	{
		bitstatus = (uint8_t)Bit_RESET;
	}			

  return bitstatus;
}

/**
  * @brief  Reads the specified output data port bit.
  * @param  PORTx: where x can be (0..14) to select the GPIO peripheral for BAT32G137 devices
  * @param  GPIO_Pin: specifies the port bits to be written.
  *          This parameter can be any combination of GPIO_Pin_x where x can be (0..7).
  * @retval The output port pin value.
  */
uint8_t GPIO_ReadOutputDataBit(GPIO_Port_t PORTx, uint16_t GPIO_Pin)
{
	uint8_t bitstatus = 0x00;

	/* Check the parameters */
	assert_param(IS_GPIO_ALL_PERIPH(PORTx));
	assert_param(IS_GET_GPIO_PIN(GPIO_Pin));


	if(((*((volatile uint8_t*)(&PORT->P0 +PORTx))) & GPIO_Pin) != Bit_RESET)
	{
		bitstatus = (uint8_t)Bit_SET;
	}
	else
	{
		bitstatus = (uint8_t)Bit_RESET;
	}			

	return bitstatus;
}


void GPIO_SetPIOR(uint16_t idx,GPIO_Source_t GPIO_Source_Grp)
{
	uint8_t i;

	for(i=0;i<3;i++)
	{
		if(GPIO_Source_Grp == funcAfTab[idx].piorInfo[i].bitHigPin)
		{
			if(funcAfTab[idx].piorInfo[i].bitIndex == PIOR_CBIT10) //只针对pior1 的pior11 pior10组合bit 
			{
				*((volatile uint8_t*)(GPIO_PIOR_CFG_BASE + funcAfTab[idx].piorInfo[i].pior))  |= (1<<0);
				*((volatile uint8_t*)(GPIO_PIOR_CFG_BASE + funcAfTab[idx].piorInfo[i].pior))  |= (1<<1);
			}
			else
			{				
				*((volatile uint8_t*)(GPIO_PIOR_CFG_BASE + funcAfTab[idx].piorInfo[i].pior))  |= (1<<funcAfTab[idx].piorInfo[i].bitIndex);
			}
			break;
		}
	}	
}

/**
  * @brief  chip pins can be used as the specified digital fuctions.
  * @param  PORTx: where x can be (0..14) to select the GPIO peripheral for BAT32G137 devices
  * @param  GPIO_Pin: specifies the port bits to be written.
  *          This parameter can be any combination of GPIO_Pin_x where x can be (0..7).
  * @param  GPIO_Source_Grp: specifies the port bits and port,such as P70、P51......
  * @param  GPIO_AF: digital functions which can be mapped to any chip pin
  * @retval error or success .
  */
int GPIO_PinAFConfig(GPIO_Port_t PORTx, uint16_t GPIO_Pin, GPIO_Source_t GPIO_Source_Grp, GROUP_AF_t GPIO_AF)
{
	uint16_t id = 0;
	/* Check the parameters */
	assert_param(IS_GPIO_ALL_PERIPH(PORTx));
	assert_param(IS_GPIO_PIN(GPIO_Pin));
	assert_param(IS_GPIO_AF(GPIO_AF));

	if(GPIO_PinCheck(PORTx,GPIO_Pin))
	{
		return GPIO_ERR;
	}

	for(id =0;funcAfTab[id].func !=0;id++)
	{
		if(funcAfTab[id].func == GPIO_AF)
		{
			GPIO_SetPIOR(id,GPIO_Source_Grp);
			break;
		}
	}

	return GPIO_SUCC;
}

