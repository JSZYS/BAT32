/**
  ******************************************************************************
  * @file    sci_common.h
  * @author  CMS Application Team
  * @version Vx.x.x
  * @date    24-April-2022
  * @brief   This file provides firmware functions to manage the following 
  *          functionalities of the Serial Controllor Unit(SCI):           
  @verbatim       
 ===============================================================================
                        ##### How to use this driver #####
 ===============================================================================
    [..]
            
    @endverbatim        
  ******************************************************************************
  * @attention
  *
  *
  ******************************************************************************
  */

#ifndef __SCI_COMMON_H__
#define __SCI_COMMON_H__

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include "common.h"
#define SCI_ERROR_LOG(err)   sci_error_log((err),__FILE__, __LINE__);

#define SCI_DEFAULT_MODE	0

#define FCLK_DIV_1                      ((uint16_t)0x0000)//Operation clock slection 2^0
#define FCLK_DIV_2						((uint16_t)0x0001)//Operation clock slection 2^1
#define FCLK_DIV_4                      ((uint16_t)0x0002)//Operation clock slection 2^2
#define FCLK_DIV_8                      ((uint16_t)0x0003)//Operation clock slection 2^3
#define FCLK_DIV_16                     ((uint16_t)0x0004)//Operation clock slection 2^4
#define FCLK_DIV_32                     ((uint16_t)0x0005)//Operation clock slection 2^5
#define FCLK_DIV_64                     ((uint16_t)0x0006)//Operation clock slection 2^6
#define FCLK_DIV_128                    ((uint16_t)0x0007)//Operation clock slection 2^7
#define FCLK_DIV_256                    ((uint16_t)0x0008)//Operation clock slection 2^8
#define FCLK_DIV_512                    ((uint16_t)0x0009)//Operation clock slection 2^9
#define FCLK_DIV_1024                   ((uint16_t)0x000A)//Operation clock slection 2^10
#define FCLK_DIV_2048                   ((uint16_t)0x000B)//Operation clock slection 2^11
#define FCLK_DIV_4096                   ((uint16_t)0x000C)//Operation clock slection 2^12
#define FCLK_DIV_8192                   ((uint16_t)0x000D)//Operation clock slection 2^13
#define FCLK_DIV_16384                  ((uint16_t)0x000E)//Operation clock slection 2^14
#define FCLK_DIV_32768                  ((uint16_t)0x000F)//Operation clock slection 2^15

/**
 * SCI error code macro
 */
#define SCI_SUCCESS				(0)			//< SCI ok
#define SCI_ERR					(-1)		//< Normal error
#define SCI_ERR_BUSY			(-2)		//< Communication unit busy
#define SCI_ERR_ISR				(-3)		//< Interrupt be used by other function
#define SCI_ERR_CHANNEL_INVALID	(-4)		//< Selected channel does not exist
#define SCI_ERR_MODE_INVALID	(-5)		//< Unsupported or incorrect mode
#define SCI_ERR_BAUDRATE    	(-6)		//< Baudrate caculate error


/** 
  * @brief  SCI unit ST register  
  */
#define SCI_STOP_EN						((uint16_t)0x0001)
#define SCI_STOP_DIS					((uint16_t)0x0000)

/** 
  * @brief  SCI unit SS register  
  */
#define SCI_START_EN					((uint16_t)0x0001)
#define SCI_START_DIS					((uint16_t)0x0000)


/***********************************************************************************************************************
Macro definitions
***********************************************************************************************************************/
/* Clear trigger of framing error of channel n (FECTmn) */
#define SCI_SIRMN_FECTMN					(0x0004U) // clears the FEFmn bit of the SSRmn register to 0 
/* Clear trigger of parity error flag of channel n (PECTmn) */
#define SCI_SIRMN_PECTMN					(0x0002U) // clears the PEFmn bit of the SSRmn register to 0 
/* Clear trigger of overrun error flag of channel n (OVCTmn) */
#define SCI_SIRMN_OVCTMN					(0x0001U) // clears the OVFmn bit of the SSRmn register to 0 

/* Selection of operation clock (fMCK) of channel n (CKSmn) */
#define SCI_CLOCK_SELECT_CK00				(0x0000U) // operation clock CKm0 set by the SPSm register 
#define SCI_CLOCK_SELECT_CK01				(0x8000U) // operation clock CKm1 set by the SPSm register 

/* Selection of transfer clock (fTCLK) of channel n (CCSmn) */
#define SCI_CLOCK_MODE_CKS					(0x0000U) // divided operation clock fMCK specified by the CKSmn bit 
#define SCI_CLOCK_MODE_TI0N					(0x4000U) // clock input fSCK from the SCKp pin (slave transfer in SPI mode) 

/* Selection of start trigger source (STSmn) */
#define SCI_TRIGGER_SOFTWARE				(0x0000U) // only software trigger is valid (selected for SPI, UART transmission, and simplified I2C) */
#define SCI_TRIGGER_RXD						(0x0100U) // valid edge of the RxDq pin (selected for UART reception) 

/* Controls inversion of level of receive data of channel n in UART mode (SISmn0) */
#define SCI_EDGE_FALL						(0x0000U) // falling edge is detected as the start bit.
#define SCI_EDGE_RISING						(0x0040U) // rising edge is detected as the start bit.

/* Setting of operation mode of channel n (MDmn2,MDmn1) */
#define SCI_MODE_SPI						(0x0000U) // SPI mode 
#define SCI_MODE_UART						(0x0002U) // UART mode 
#define SCI_MODE_IIC						(0x0004U) // simplified I2C mode 

/* Selection of interrupt source of channel n (MDmn0) */
#define SCI_TRANSFER_END					(0x0000U) // transfer end interrupt 
#define SCI_BUFFER_EMPTY					(0x0001U) // buffer empty interrupt 

#define SCI_SMRMN_DEFAULT_VALUE				(0x0020U) // SMRmn default value 
#define SCI_SCRMN_DEFAULT_VALUE				(0x0004U) // SCRmn default value 
#define IIC_WAITTIME						(14U) // change the waiting time according to the system 

/* Setting of operation mode of channel n (TXEmn,RXEmn) */
#define SCI_NOT_COMMUNICATION				(0x0000U) // disable communication 
#define SCI_RECEPTION						(0x4000U) // reception only 
#define SCI_TRANSMISSION					(0x8000U) // transmission only 
#define SCI_RECEPTION_TRANSMISSION			(0xC000U) // transmission/reception 

/* Selection of data and clock phase in SPI mode (DAPmn,CKPmn) */
#define SCI_TIMING_1						(0x0000U) // type 1 
#define SCI_TIMING_2						(0x1000U) // type 2 
#define SCI_TIMING_3						(0x2000U) // type 3 
#define SCI_TIMING_4						(0x3000U) // type 4 

/* Mask control of error interrupt signal (INTSREx (x = 0 to 3)) (EOCmn) */
#define SCI_INTSRE_MASK						(0x0000U) // disables generation of error interrupt INTSREx (INTSRx is generated) */
#define SCI_INTSRE_ENABLE					(0x0400U) // enables generation of error interrupt INTSREx (INTSRx is not generated if an error occurs) */


/* Use of noise filter of RxD2 pin (SNFEN20) */
#define SCI_RXD2_FILTER_OFF					(0x00U) // noise filter off 
#define SCI_RXD2_FILTER_ON                  (0x10U) // noise filter on 

/* Use of noise filter of RxD1 pin (SNFEN10) */
#define SCI_RXD1_FILTER_OFF                 (0x00U) // noise filter off 
#define SCI_RXD1_FILTER_ON                  (0x04U) // noise filter on 

/* Use of noise filter of RxD0 pin (SNFEN00) */
#define SCI_RXD0_FILTER_OFF                 (0x00U) // noise filter off 
#define SCI_RXD0_FILTER_ON                  (0x01U) // noise filter on 

/*
    Serial status register mn (SSRmn) 
*/
/* Communication status indication flag of channel n (TSFmn) */
#define SCI_UNDER_EXECUTE					(0x0040U) // communication is in progress 
#define SCI_VALID_STORED					(0x0020U) // valid data is stored in the SDRmn register 

/* Framing error detection flag of channel n (FEFmn) */
#define SCI_FRAM_ERROR						(0x0004U) // an error occurs (during UART reception) 

/* Parity error detection flag of channel n (PEFmn) */
#define SCI_PARITY_ERROR					(0x0002U) // an error occurs (during UART reception) or ACK is not detected (during I2C transmission) */

/* Overrun error detection flag of channel n (OVFmn) */
#define SCI_OVERRUN_ERROR					(0x0001U) // an overrun error occurs 

#define SCI_CH00_PHASE_MASK					((uint16_t)(1 << 0))
#define SCI_CH02_PHASE_MASK					((uint16_t)(1 << 2))
#define SCI_CH10_PHASE_MASK					((uint16_t)(1 << 0))

/*
    Serial output register m (SOm) 
*/
/* Serial clock output of channel 3 (CKOm3) */
#define SCI_CH3_CLOCK_OUTPUT_0				(0x0000U) // serial clock output value is 0 
#define SCI_CH3_CLOCK_OUTPUT_1				(0x0800U) // serial clock output value is 1 

/* Serial clock output of channel 2 (CKOm2) */
#define SCI_CH2_CLOCK_OUTPUT_0				(0x0000U) // serial clock output value is 0 
#define SCI_CH2_CLOCK_OUTPUT_1				(0x0400U) // serial clock output value is 1 

/* Serial clock output of channel 1 (CKOm1) */
#define SCI_CH1_CLOCK_OUTPUT_0				(0x0000U) // serial clock output value is 0 
#define SCI_CH1_CLOCK_OUTPUT_1				(0x0200U) // serial clock output value is 1 



/***********************************************************************************************************************
* SCI Chanel Rules: |  bit15~bit12   |  bit7~bit4    |  bit3~bit0      |
*                   |  unit index   |function index |channel position |
* unit index      : unit0:0 unit1:1  unit2:2 
* function index  : uart:1  spi:2 iic:4 
* channel position: uart occupies 2 channels, position begins at firt channel position
***********************************************************************************************************************/
#define SCI_UINT_MASK	((uint16_t)0x3000)
#define SCI_UINT_OFFSET	((uint8_t)0x04)
#define SCI_UART_MASK	((uint8_t)0x10)
#define SCI_SSPI_MASK	((uint8_t)0x20)
#define SCI_I2C_MASK	((uint8_t)0x40)


typedef enum
{
	UART0	= 0x0010 + 0,
	UART1   = 0x0010 + 2,
	
	SSPI00  = 0x0020 + 0,
	SSPI01  = 0x0020 + 1,
	SSPI10  = 0x0020 + 2,
	SSPI11  = 0x0020 + 3,

	I2C00	= 0x0040 + 0,
	I2C01 	= 0x0040 + 1,
	I2C10	= 0x0040 + 2,
	I2C11 	= 0x0040 + 3,

	UART2	= 0x1010 + 0,	
	SSPI20  = 0x1020 + 0,
	SSPI21  = 0x1020 + 1,

	I2C20 	= 0x1040 + 0,
	I2C21 	= 0x1040 + 1,
#if defined(BAT32G1XX_80PIN) || defined(BAT32G1XX_100PIN)	
	UART3	= 0x2010 + 0,	
	SSPI30  = 0x2020 + 0,
	SSPI31  = 0x2020 + 1,

	I2C30 	= 0x2040 + 0,
	I2C31 	= 0x2040 + 1,
#endif	
} SCIAFSelect_TypeDef;

#define IS_ALL_SCIUNIT(unit)	(((unit) & (uint16_t)0xFFFC) == 0x0000)
#define IS_ALL_SCICH(chx)		(((chx) & (uint16_t)0xFFFC) == 0x0000)

#define IS_ALL_SCIAF(func)		((((func) & (uint16_t)0xCF8C) == 0x0000) && ((func) != (uint16_t)0x0000))


typedef struct 
{
    uint8_t sps;
    uint8_t sdr;
} SCIPeriph_Clock_TypeDef;
void sci_error_log(int8_t err, const char * file, int32_t line);
int SCIChannel_Alloca(SCIAFSelect_TypeDef func, uint16_t mode);
void SCIChannel_Free(SCIAFSelect_TypeDef func, uint16_t mode);
float SCIPeriphal_ClockUpdate(SCIAFSelect_TypeDef func, uint32_t fclk, uint32_t ftclk, uint8_t osps, SCIPeriph_Clock_TypeDef *clock);

FlagStatus SCI_GetErrStaus(SCIAFSelect_TypeDef SCIx, uint16_t FLAG);
FlagStatus SCI_GetFlagStatus(uint16_t reg, uint16_t FLAG);
void SCI_ClearFlag(SCIAFSelect_TypeDef SCIx, uint16_t FLAG);

#endif
