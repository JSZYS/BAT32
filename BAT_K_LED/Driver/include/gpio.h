#ifndef __GPIO_H__
#define __GPIO_H__

#include "common.h"
#include <assert.h>


#define GPIO_SUCC  0
#define	GPIO_ERR   1

typedef enum
{ 
	GPIO_Control_DIG   = 0, /*!< GPIO Control Digital */
	GPIO_Control_ANA	 = 1, /*!< GPIO Control Analogy  */
}GPIOControl_TypeDef;
#define IS_GPIO_Control(Ctrl) (((Ctrl) == GPIO_Control_ANA)  || ((Ctrl) == GPIO_Control_DIG) )? 1:0

typedef enum
{ 
	GPIO_Mode_OUT  = 0, /*!< GPIO Output Mode */
	GPIO_Mode_IN	 = 1, /*!< GPIO Input Mode */
}GPIOMode_TypeDef;
#define IS_GPIO_MODE(MODE) (((MODE) == GPIO_Mode_IN)  || ((MODE) == GPIO_Mode_OUT) )? 1:0

typedef enum
{ 
	GPIO_OType_PP = 0,//普通输出
	GPIO_OType_OD = 1 //开漏输出
}GPIOOType_TypeDef;
#define IS_GPIO_OTYPE(OTYPE) (((OTYPE) == GPIO_OType_PP) || ((OTYPE) == GPIO_OType_OD))? 1:0
typedef enum
{ 
	GPIO_PuPd_NOPULL = 0x00,
	GPIO_PuPd_UP     = 0x01,
	GPIO_PuPd_DOWN   = 0x02
}GPIOPuPd_TypeDef;
#define IS_GPIO_PUPD(PUPD) (((PUPD) == GPIO_PuPd_NOPULL) || ((PUPD) == GPIO_PuPd_UP) || \
                            ((PUPD) == GPIO_PuPd_DOWN))? 1:0
typedef enum
{ 
	GPIO_Level_LOW	= 0x00,
	GPIO_Level_HIGH = 0x01,
}GPIOInit_Level;

typedef enum
{ 
	GPIO_PORT0 = 0x00,
	GPIO_PORT1 = 0x01,		
	GPIO_PORT2 = 0x02,
	GPIO_PORT3 = 0x03,
	GPIO_PORT4 = 0x04,
	GPIO_PORT5 = 0x05,
	GPIO_PORT6 = 0x06,
	GPIO_PORT7 = 0x07,
#ifdef BAT32G1XX_80PIN
	GPIO_PORT10 = 0x0A,
	GPIO_PORT11 = 0x0B,
	GPIO_PORT15 = 0x0F,
#endif	
	GPIO_PORT12 = 0x0C,
	GPIO_PORT13 = 0x0D,
	GPIO_PORT14 = 0x0E,
}GPIO_Port_t;

#ifdef BAT32G1XX_80PIN
#define IS_GPIO_ALL_PERIPH(PERIPH) (((PERIPH) == GPIO_PORT0) ||((PERIPH) == GPIO_PORT1) || ((PERIPH) == GPIO_PORT2) || \
									((PERIPH) == GPIO_PORT3) || ((PERIPH) == GPIO_PORT4) ||((PERIPH) == GPIO_PORT5) || \
									((PERIPH) == GPIO_PORT6) || ((PERIPH) == GPIO_PORT7) ||((PERIPH) == GPIO_PORT12)|| \
									((PERIPH) == GPIO_PORT13)|| ((PERIPH) == GPIO_PORT14)||((PERIPH) == GPIO_PORT10)|| \
									((PERIPH) == GPIO_PORT11)|| ((PERIPH) == GPIO_PORT15))
#else
#define IS_GPIO_ALL_PERIPH(PERIPH) (((PERIPH) == GPIO_PORT0) ||((PERIPH) == GPIO_PORT1) || ((PERIPH) == GPIO_PORT2) || \
									((PERIPH) == GPIO_PORT3) || ((PERIPH) == GPIO_PORT4) ||((PERIPH) == GPIO_PORT5) || \
									((PERIPH) == GPIO_PORT6) || ((PERIPH) == GPIO_PORT7) ||((PERIPH) == GPIO_PORT12)|| \
									((PERIPH) == GPIO_PORT13)|| ((PERIPH) == GPIO_PORT14))
#endif


/** 
  * @brief  GPIO Bit SET and Bit RESET enumeration 
  */ 
typedef enum
{ 
  Bit_RESET = 0,
  Bit_SET
}BitAction;
#define IS_GPIO_BIT_ACTION(ACTION) (((ACTION) == Bit_RESET) || ((ACTION) == Bit_SET))



#define GPIO_Pin_0                 ((uint16_t)0x0001)  /* Pin 0 selected */
#define GPIO_Pin_1                 ((uint16_t)0x0002)  /* Pin 1 selected */
#define GPIO_Pin_2                 ((uint16_t)0x0004)  /* Pin 2 selected */
#define GPIO_Pin_3                 ((uint16_t)0x0008)  /* Pin 3 selected */
#define GPIO_Pin_4                 ((uint16_t)0x0010)  /* Pin 4 selected */
#define GPIO_Pin_5                 ((uint16_t)0x0020)  /* Pin 5 selected */
#define GPIO_Pin_6                 ((uint16_t)0x0040)  /* Pin 6 selected */
#define GPIO_Pin_7                 ((uint16_t)0x0080)  /* Pin 7 selected */

#define GPIO_PIN_MASK              ((uint16_t)0x00FF) /* PIN mask for assert test */
#define IS_GPIO_PIN(PIN)           (((PIN) & GPIO_PIN_MASK ) != (uint16_t)0x00) ? 1:0

#define IS_GET_GPIO_PIN(PIN) (((PIN) == GPIO_Pin_0) || \
                              ((PIN) == GPIO_Pin_1) || \
                              ((PIN) == GPIO_Pin_2) || \
                              ((PIN) == GPIO_Pin_3) || \
                              ((PIN) == GPIO_Pin_4) || \
                              ((PIN) == GPIO_Pin_5) || \
                              ((PIN) == GPIO_Pin_6) || \
                              ((PIN) == GPIO_Pin_7))? 1:0
															

typedef enum
{
	GROUP_AF_ODEFAULT = 0,
#if defined(BAT32G1XX_64PIN) || defined(BAT32G1XX_80PIN) || defined(BAT32G1XX_100PIN)	
	GROUP_AF_INTP1 ,
	GROUP_AF_INTP2 ,
	GROUP_AF_INTP3 ,
	GROUP_AF_INTP4 ,
	GROUP_AF_INTP5 ,	
	GROUP_AF_INTP8 ,
	GROUP_AF_INTP9 ,

	GROUP_AF_INTP10 ,	
	GROUP_AF_INTP11 ,

	GROUP_AF_SDO00  ,
	GROUP_AF_SDI00  ,
	GROUP_AF_SCK00  ,	
	GROUP_AF_CLKBUZ1,	
	GROUP_AF_VCOUT0,	
	GROUP_AF_VCOUT1,		
#elif defined(BAT32G1XX_48PIN) 
	GROUP_AF_INTP8 ,
	GROUP_AF_RXD1  ,		
	GROUP_AF_TXD1  ,
#else
	GROUP_AF_RXD1  ,		
	GROUP_AF_TXD1  ,	
#endif
	GROUP_AF_RXD2 ,	
	GROUP_AF_TXD2 ,

	GROUP_AF_TXD0 ,	
	GROUP_AF_RXD0 ,	
	GROUP_AF_SCLAA0,		
	GROUP_AF_SDAA0 ,

#ifndef BAT32G1XX_32PIN
	GROUP_AF_CLKBUZ0 ,		
#endif

#ifdef BAT32G1XX_100PIN
	GROUP_AF_INTP6 ,
	GROUP_AF_INTP7 ,
	
	GROUP_AF_TI_TO14 ,
	GROUP_AF_TI_TO15 ,
	GROUP_AF_TI_TO16 ,
	GROUP_AF_TI_TO17 ,	

	GROUP_AF_SCLK31 ,
	GROUP_AF_SCL31 ,
	GROUP_AF_SDI31 ,
	GROUP_AF_SDO31 ,		
	GROUP_AF_SDA31 ,
	
	GROUP_AF_SCLK10 ,
	GROUP_AF_SCL10 ,
	
	GROUP_AF_SDI10 ,
	GROUP_AF_RXD1 ,
	
	GROUP_AF_SDO10 ,
	GROUP_AF_TXD1 ,	
	
	GROUP_AF_SDA10 ,	
#endif

	GROUP_AF_TAIO ,  //定时器A外部事件输入和脉冲输出
	GROUP_AF_TAO ,   //定时器A的脉冲输出

	GROUP_AF_CRXD ,
	GROUP_AF_CTXD ,	
}GROUP_AF_t;
#define IS_GPIO_AF(GPIO_AF) ((GPIO_AF) < GROUP_AF_CTXD || ((GPIO_AF) == GROUP_AF_CTXD))



typedef enum
{
	GPIO_DEF = 0x00,
	GPIO_P00 ,
	GPIO_P01 ,
	GPIO_P02 ,
	GPIO_P03 ,
	GPIO_P04 ,
	GPIO_P05 ,
	GPIO_P06 ,
	GPIO_P10 ,
	GPIO_P11 ,
	GPIO_P12 ,
	GPIO_P13 ,
	GPIO_P14 ,
	GPIO_P15 ,
	GPIO_P16 ,
	GPIO_P17 ,
	GPIO_P20 ,
	GPIO_P21 ,
	GPIO_P22 ,
	GPIO_P23 ,
	GPIO_P24 ,
	GPIO_P25 ,
	GPIO_P26 ,
	GPIO_P27 ,
	GPIO_P30 ,
	GPIO_P31 ,
	GPIO_P40 ,
	GPIO_P41 ,
	GPIO_P42 ,
	GPIO_P43 ,	
	GPIO_P50 ,
	GPIO_P51 ,
	GPIO_P52 ,
	GPIO_P53 ,
	GPIO_P54 ,	
	GPIO_P55 ,
	
	GPIO_P60 ,
	GPIO_P61 ,
	GPIO_P62 ,
	GPIO_P63 ,
	GPIO_P70 ,
	GPIO_P71 ,
	GPIO_P72 ,
	GPIO_P73 ,
	GPIO_P74 ,
	GPIO_P75 ,
	GPIO_P76 ,
	GPIO_P77 ,
	
	GPIO_P120 ,
	GPIO_P121 ,
	GPIO_P122 ,
	GPIO_P123 ,
	GPIO_P124 ,
	GPIO_P130 ,
	GPIO_P136 ,
	GPIO_P137 ,
	GPIO_P140 ,
	GPIO_P141 ,	
	GPIO_P146 ,
	GPIO_P147 ,

#ifdef BAT32G1XX_80PIN
	GPIO_P44 ,
	GPIO_P45 ,	
	
	GPIO_P64 ,
	GPIO_P65 ,	
	GPIO_P66 ,
	GPIO_P67 ,	

	GPIO_P100 ,	
	GPIO_P110 ,	
	GPIO_P111 ,	
	
	GPIO_P142 ,	
	GPIO_P143 ,	
	GPIO_P144 ,	
	
	GPIO_P150 ,	
	GPIO_P151 ,	
	GPIO_P152 ,		
	GPIO_P153 ,		
#endif
	GPIO_PIN_END ,
}GPIO_Source_t;

typedef enum
{
	PIOR_NONE  =-1,
	GPIO_PIOR2 =0,
	GPIO_PIOR0 =2,
	GPIO_PIOR1 =4,
	GPIO_PIOR3 =7,
}GPIO_PIOR_t;

typedef enum
{
	PIOR_BIT0 =0,
	PIOR_BIT1,
	PIOR_BIT2,
	PIOR_BIT3,
	PIOR_BIT4,
	PIOR_BIT5,
	PIOR_BIT6,
	PIOR_BIT7,
	PIOR_CBIT10,   //组合bit位（bit1 bit0）
}PIOR_BIT_t;

typedef struct
{
	GPIO_PIOR_t pior;
	PIOR_BIT_t  bitIndex;
	GPIO_Source_t   bitHigPin;
}PIOR_INFO_t;

typedef struct
{
	GROUP_AF_t  func;
	PIOR_INFO_t   piorInfo[3];
}GPIO_PIN_AF_TABLE_t;


typedef struct 
{
	uint16_t GPIO_Pin;						/*!< Specifies the GPIO pins to be configured.
										  This parameter can be any value of @ref GPIO_pins_define */
	GPIOMode_TypeDef GPIO_Mode;	  /*!< Specifies the operating mode for the selected pins.
									   This parameter can be a value of @ref GPIOMode_TypeDef */
	GPIOControl_TypeDef GPIO_Ctrl;    /*!< Specifies the control mode for the selected pins.
									   This parameter can be a value of @ref GPIOControl_TypeDef */
	GPIOOType_TypeDef GPIO_OType;   /*!< Specifies the operating output type for the selected pins.
										This parameter can be a value of @ref GPIOOType_TypeDef */
	GPIOPuPd_TypeDef GPIO_PuPd; 	/*!< Specifies the operating Pull-up/Pull down for the selected pins.
										 This parameter can be a value of @ref GPIOPuPd_TypeDef */
	GPIOInit_Level GPIO_Level;  /*!< Specifies the gpio init level for the selected pins.
										  This parameter can be a value of @ref GPIOInit_Level */
}GPIO_InitTypeDef;

void GPIO_Output_Enable(__IO uint8_t *port, uint8_t pinMsk);
void GPIO_Input_Enable(__IO uint8_t *port, uint8_t pinMsk);
void GPIO_PullUp_Enable(__IO uint8_t *port, uint8_t pinMsk);
void GPIO_PullUp_Disable(__IO uint8_t *port, uint8_t pinMsk);
void GPIO_PullDown_Enable(__IO uint8_t *port, uint8_t pinMsk);
void GPIO_PullDown_Disable(__IO uint8_t *port, uint8_t pinMsk);
void GPIO_Set_Value(__IO uint8_t *port, uint8_t value);
uint8_t GPIO_Get_Value(__IO uint8_t *port);

uint8_t GPIO_ReadInputDataBit(GPIO_Port_t PORTx, uint16_t GPIO_Pin);
uint8_t GPIO_ReadOutputDataBit(GPIO_Port_t PORTx, uint16_t GPIO_Pin);
void GPIO_SetBits(GPIO_Port_t  PORTx, uint16_t GPIO_Pin);
void GPIO_ResetBits(GPIO_Port_t PORTx, uint16_t GPIO_Pin);
void GPIO_ToggleBits(GPIO_Port_t PORTx, uint16_t GPIO_Pin);

void GPIO_Init(GPIO_Port_t PORTx,GPIO_InitTypeDef* GPIO_InitStruct);
int GPIO_PinAFConfig(GPIO_Port_t PORTx, uint16_t GPIO_Pin, GPIO_Source_t GPIO_Source_Grp, GROUP_AF_t GPIO_AF);
#endif

