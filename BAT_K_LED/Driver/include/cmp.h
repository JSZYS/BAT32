#ifndef __CMP_H__
#define __CMP_H__
#include <stdint.h>

#include "common.h"
#include "sci_common.h"

/* Comparator 1 monitor flag bit (C1MON) */
#define _00_COMP1_FLAG_REFERENCE_0                    (0x00U) // IVCMP1 less than comparator 1 reference voltage or comparator 1 stopped 
#define _80_COMP1_FLAG_REFERENCE_1                    (0x80U) // IVCMP1 more than comparator 1 reference voltage 
/* Comparator 1 operation enable bit (C1ENB) */
#define _00_COMP1_OPERATION_DISABLED                  (0x00U) // comparator 1 operation disabled 
#define _10_COMP1_OPERATION_ENABLED                   (0x10U) // comparator 1 operation enabled 
/* Comparator 0 monitor flag bit (C0MON) */
#define _00_COMP0_FLAG_REFERENCE_0                    (0x00U) // IVCMP0 less than comparator 0 reference voltage or comparator 0 stopped 
#define _08_COMP0_FLAG_REFERENCE_1                    (0x08U) // IVCMP0 more than comparator 0 reference voltage 
/* Comparator 0 operation enable bit (C0ENB) */
#define _00_COMP0_OPERATION_DISABLED                  (0x00U) // comparator 0 operation disabled 
#define _01_COMP0_OPERATION_ENABLED                   (0x01U) // comparator 0 operation enabled 

/* Comparator 1 edge detectioin selection (C1EDG) */
#define _00_COMP1_ONE_EDGE_INTERRUPT                  (0x00U) // interrupt request by comparator 1 one-edge detection 
#define _80_COMP1_BOTH_EDGES_INTERRUPT                (0x80U) // interrupt request by comparator 1 both-edge detection 
/* Comparator 1 edges polarity selection (C1EPO) */
#define _00_COMP1_RISING_INTERRUPT                    (0x00U) // interrupt request at comparator 1 rising edge 
#define _40_COMP1_FALLING_INTERRUPT                   (0x40U) // interrupt request at comparator 1 falling edge 

#define _00_COMP0_OUTPUT_DISABLE                      (0x00U) // comparator 0 VCOUT0 pin output disabled 
#define _02_COMP0_OUTPUT_ENABLE                       (0x02U) // comparator 0 VCOUT0 pin output enabled 

#define _00_COMP1_OUTPUT_DISABLE                      (0x00U) // comparator 1 VCOUT1 pin output disabled
#define _20_COMP1_OUTPUT_ENABLE                       (0x20U) // comparator 1 VCOUT1 pin output enabled 

#define _00_COMP1_INTERRUPT_DISABLE                   (0x00U) //comparator 1 interrupt request disabled 
#define _10_COMP1_INTERRPUT_ENABLE                    (0x10U) // comparator 1 interrupt request enabled 
#define _00_COMP0_INTERRUPT_DISABLE                   (0x00U) // comparator 0 interrupt request disabled
#define _01_COMP0_INTERRPUT_ENABLE                    (0x01U) // comparator 0 interrupt request enabled 
/* Power supply of internal reference voltage (CVRVS0) */
#define _00_VDD_INTERNAL_REFERVOL                     (0x00U) // VDD is selected as power supply of internal reference voltage  
#define _01_AVREFP_INTERNAL_REFERVOL                  (0x01U) // AVREFP is selected as power supply of internal reference voltage 
/* Control bit for internal reference voltage 0 (CVRE0) */
#define _00_COMP0_REFVOLTAGE_DISABLE                  (0x00U) // internal reference voltage 0 operation stopped 
#define _02_COMP0_REFVOLTAGE_ENABLE                   (0x02U) // internal reference voltage 0 operation enabled 
/* Ground selection bit for internal reference voltage (CVRVS1) */
#define _00_VSS_INTERNAL_REFERVOL                     (0x00U) // VSS selected as ground for internal reference voltage
#define _10_AVREFM_INTERNAL_REFERVOL                  (0x10U) // AVREFM selected as ground for internal reference voltage 
/* Control bit for internal reference voltage 1 (CVRE1) */
#define _00_COMP1_REFVOLTAGE_DISABLE                  (0x00U) // internal reference voltage 1 operation stopped
#define _20_COMP1_REFVOLTAGE_ENABLE                   (0x20U) // internal reference voltage 1 operation enabled 


typedef enum
{
    CMP_Vref_VDD_VSS = 0U,        // VDD/VSS selected as power/ground for internal reference voltage
	CMP_Vref_AVREFP_AVREFM = 1U   // AVREFP/AVREFM selected as power/ground for internal reference voltage
}CMP_Vref_t;


typedef enum
{
	CMP_Channel_0  = 1,
	CMP_Channel_1  = 2,
	CMP_Channel_2  = 3,
}	CMP_Channel_t;

typedef enum
{
    CMP0_PositiveSide_VCIN0 = 0,   // external VCIN0 PIN
    CMP0_PositiveSide_PGAO  = 1,   // PGA0OUT signal
    
	CMP0_NegativeSide_IVREF     = 0,	//bulit-in reference voltage,it can choose 256 steps
	CMP0_NegativeSide_BGR       = 1,   // internal voltage reference 1.45v
	CMP0_NegativeSide_VREF     = 2,   //external voltage reference 

	CMP1_PositiveSide_VCIN10   = 0,  // external VCIN10 PIN
	CMP1_PositiveSide_VCIN11   = 1,  // external VCIN11 PIN
	CMP1_PositiveSide_VCIN12   = 2,  // external VCIN12 PIN
	CMP1_PositiveSide_VCIN13   = 3,  // external VCIN13 PIN

	CMP1_NegativeSide_IVREF     = 0,	  //internal voltage reference 1.45v,it can choose 256 steps	
	CMP1_NegativeSide_BGR      = 1,   // cmp1 negative can choose internal voltage reference 1.45v
	CMP1_NegativeSide_VCIN10  = 2,    //cmp1 negative can choose external VCIN10 PIN
	CMP1_NegativeSide_VCIN11  = 3,    //cmp1 negative can choose external VCIN11 PIN
	CMP1_NegativeSide_VCIN12  = 4,    //cmp1 negative can choose external VCIN12 PIN
	CMP1_NegativeSide_VCIN13  = 5,	  //cmp1 negative can choose external VCIN13 PIN
}CMP_Side_t;

typedef enum
{
	 CMP_HyMode_None    = 0,  // None Hy
	 CMP_HyMode_Postive = 1, //positive hy
	 CMP_HyMode_Negative =2, //negative hy
	 CMP_HyMode_Both    = 3, //both side hy	 
}CMP_HyMode_t;

typedef enum
{
	 CMP_HyVal_None   = 0,  // None Hy
	 CMP_HyVal_20mv   = 1, //Comparator Hy 20mV
	 CMP_HyVal_40mv   = 2, //Comparator Hy 40mV
	 CMP_HyVal_60mv   = 3, //Comparator Hy 60mV
}CMP_HyVal_t;

typedef enum
{
	 CMP_IntReq_Rising   = 0,  // Interrupt request at comparator rising edge
	 CMP_IntReq_Falling  = 1, //Interrupt request at comparator falling edge
	 CMP_IntReq_Both     = 2, //Interrupt request at comparator both edge
}CMP_IntReq_t;
typedef enum
{
	 CMP_Int_Disable = 0, // Interrupt disable
	 CMP_Int_Enable  = 1, //Interrupt enable
}CMP_Int_t;
typedef enum
{
	 CMP_Filter_None   = 0, // None Filter
	 CMP_Filter_Fclk  = 1,  // Comparator filter enabled, sampling at fCLK
	 CMP_Filter_Fclk8 = 2,  // Comparator filter enabled, sampling at fCLK/8 
	 CMP_Filter_Fclk32= 3,  // Comparator filter enabled, sampling at fCLK/32
}CMP_Filter_t;

typedef enum
{
	 CMP_Out_Disable = 0, // Disable cmp result to outport
	 CMP_Out_Enable  = 1, //Enable cmp result to outport
}CMP_Out_t;
typedef enum
{
	 CMP_Out_Noraml   = 0, 
	 CMP_Out_Reverse  = 1, 
}CMP_Polarity_t;
typedef struct
{
	CMP_Channel_t CMP_Channel;	/*!< Specifies the channel of comparator.*/
										   	
	CMP_Side_t CMP_PositiveSide;/*!< Specifies the input signal of positive side of comparator 
								 This parameter can be a value of @ref CMP_Side_t */
	CMP_Side_t CMP_NegativeSide;/*!< Specifies the input signal of negative side of comparator 
								 This parameter can be a value of @ref CMP_Side_t */
	CMP_Vref_t CMP_NegVref;  /*!< Specifies the comparator use vdd or avrefp when negative side is CMP0_NegativeSide_IVREF
								 This parameter can be a value of @ref CMP_Vref_t */
	uint8_t CMP_NegVrefVol;  /*!< Specifies the negative side voltage */
	
	CMP_HyMode_t CMP_HyMode; /*!< Specifies the comparator hy mode
								 This parameter can be a value of @ref CMP_HyMode_t */
								 	
	CMP_HyVal_t CMP_HyVal; /*!< Specifies the comparator hy voltage
								 This parameter can be a value of @ref CMP_HyVal_t */	
								 	
	CMP_Filter_t CMP_Filter; /*!< Specifies the comparator hy voltage
							  This parameter can be a value of @ref CMP_HyVal_t */	 
							  	
	CMP_IntReq_t CMP_IntReq;/*!< Specifies generate comparator interrupt request condition 
							   This parameter can be a value of @ref CMP_IntReq_t */
							   	
	CMP_Int_t CMP_Int;/*!< Specifies generate comparator interrupt diasble or enbale
								This parameter can be a value of @ref CMP_Int_t */
	CMP_Out_t CMP_Out2port; /*!< Specifies the comparator result whether output to port or not
								 This parameter can be a value of @ref CMP_Out_t */
	CMP_Polarity_t CMP_Polarity; /*!< Specifies the comparator result whether output to port or not
								 This parameter can be a value of @ref CMP_Out_t */									 
}CMP_InitTypeDef;

void CMP_Init(CMP_InitTypeDef* CMP_InitStruct);
void CMP_Start(CMP_Channel_t ch);
void CMP_Stop(CMP_Channel_t ch);
void CMP_VREF_Set_Value(CMP_Channel_t ch, uint8_t regvalue);
void CMP_Filter_Setting(CMP_Channel_t ch, CMP_Filter_t fck, CMP_IntReq_t edge);

#endif
