/**
  ******************************************************************************
  * @file    can.h
  * @author  MCD Application Team
  * @version V1.8.1
  * @date    27-January-2022
  * @brief   This file contains all the functions prototypes for the CAN firmware 
  *          library.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2016 Cmsemicon.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */


#ifndef __DRIVER_CAN_H__
#define __DRIVER_CAN_H__

#ifdef __cplusplus
    extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "common.h"

/** @addtogroup BAT32G137xx_StdPeriph_Driver
  * @{
  */

/** @addtogroup CAN
  * @{
  */

/* Exported types ------------------------------------------------------------*/
#define MAX_CAN_MSGCACHE_CNT			((uint8_t)0x10)
#if defined(BAT32G1XX_80PIN) || defined(BAT32G1XX_100PIN)
#define IS_CAN_ALL_PERIPH(PERIPH)       ((PERIPH) == CAN0 || (PERIPH) == CAN1 )
#define IS_CAN_ALL_MSGCACHE(MSGCACHE)   (((MSGCACHE) == CAN0MSG00) || ((MSGCACHE) == CAN0MSG01) || \
                                        ((MSGCACHE) == CAN0MSG02) || ((MSGCACHE) == CAN0MSG03) || \
                                        ((MSGCACHE) == CAN0MSG04) || ((MSGCACHE) == CAN0MSG05) || \
                                        ((MSGCACHE) == CAN0MSG06) || ((MSGCACHE) == CAN0MSG07) || \
                                        ((MSGCACHE) == CAN0MSG08) || ((MSGCACHE) == CAN0MSG09) || \
                                        ((MSGCACHE) == CAN0MSG10) || ((MSGCACHE) == CAN0MSG11) || \
                                        ((MSGCACHE) == CAN0MSG12) || ((MSGCACHE) == CAN0MSG13) || \
                                        ((MSGCACHE) == CAN0MSG14) || ((MSGCACHE) == CAN0MSG15)|| \
										((MSGCACHE) == CAN1MSG00) || ((MSGCACHE) == CAN1MSG01) || \
                                        ((MSGCACHE) == CAN1MSG02) || ((MSGCACHE) == CAN1MSG03) || \
                                        ((MSGCACHE) == CAN1MSG04) || ((MSGCACHE) == CAN1MSG05) || \
                                        ((MSGCACHE) == CAN1MSG06) || ((MSGCACHE) == CAN1MSG07) || \
                                        ((MSGCACHE) == CAN1MSG08) || ((MSGCACHE) == CAN1MSG09) || \
                                        ((MSGCACHE) == CAN1MSG10) || ((MSGCACHE) == CAN1MSG11) || \
                                        ((MSGCACHE) == CAN1MSG12) || ((MSGCACHE) == CAN1MSG13) || \
                                        ((MSGCACHE) == CAN1MSG14) || ((MSGCACHE) == CAN1MSG15))
#else
#define IS_CAN_ALL_PERIPH(PERIPH)       ((PERIPH) == CAN0 )
#define IS_CAN_ALL_MSGCACHE(MSGCACHE)   (((MSGCACHE) == CAN0MSG00) || ((MSGCACHE) == CAN0MSG01) || \
                                        ((MSGCACHE) == CAN0MSG02) || ((MSGCACHE) == CAN0MSG03) || \
                                        ((MSGCACHE) == CAN0MSG04) || ((MSGCACHE) == CAN0MSG05) || \
                                        ((MSGCACHE) == CAN0MSG06) || ((MSGCACHE) == CAN0MSG07) || \
                                        ((MSGCACHE) == CAN0MSG08) || ((MSGCACHE) == CAN0MSG09) || \
                                        ((MSGCACHE) == CAN0MSG10) || ((MSGCACHE) == CAN0MSG11) || \
                                        ((MSGCACHE) == CAN0MSG12) || ((MSGCACHE) == CAN0MSG13) || \
                                        ((MSGCACHE) == CAN0MSG14) || ((MSGCACHE) == CAN0MSG15))
#endif
/** 
  * @brief  CAN init structure definition
  */
typedef struct
{
    uint8_t CAN_Prescaler;      /*!< Specifies the length of a time quantum. 
                                    It ranges from 1 to 16. */
                                
    uint16_t CAN_BitRatePrescaler;  /*!< Specifies the Prescaler of bit rate. 
                                    It ranges from 1 to 256. */

    uint8_t CAN_OperateMode;    /*!< Specifies the CAN operating mode.
                                    This parameter can be a value of @ref CAN_operating_mode */

    uint8_t CAN_SJW;            /*!< Specifies the maximum number of time quanta 
                                    the CAN hardware is allowed to lengthen or 
                                    shorten a bit to perform resynchronization.
                                    This parameter can be a value of @ref CAN_synchronisation_jump_width */

    uint8_t CAN_BS1;            /*!< Specifies the number of time quanta in Bit 
                                    Segment 1. This parameter can be a value of 
                                    @ref CAN_time_quantum_in_bit_segment_1 */

    uint8_t CAN_BS2;            /*!< Specifies the number of time quanta in Bit Segment 2.
                                    This parameter can be a value of @ref CAN_time_quantum_in_bit_segment_2 */

    uint32_t MASK1;             /*!< Specifies the filter identification number (MSBs for a 32-bit configuration).
                                    Only 29 bits used to the filter such as MASK1, MASK2, MASK3, MASK4.
                                    This parameter can be a value between 0x00000000 and 0x1FFFFFFF */
	uint32_t MASK2;             /*!< When used standard frame, mask used 11bis; when used extend frame, mask used 29bits. */
	uint32_t MASK3;
	uint32_t MASK4;
} CAN_InitTypeDef;

/** 
  * @brief  CAN TxRx message structure definition  
  */
typedef struct
{
	uint8_t CacheType;	/*!< Specifies the message cache type select.
						This parameter can be a value of @ref CAN_msgcache_type  */
	
	uint32_t Id;		/*!< Specifies the standard identifier or extended identifier.
						When this parameter is standard identifier, This parameter can be a value between 0 to 0x7FF.
						When this parameter is extended identifier, This parameter can be a value between 0 to 0x1FFFFFFF. */

	uint8_t IDE;		/*!< Specifies the type of identifier for the message that 
						will be transmitted. This parameter can be a value 
						of @ref CAN_identifier_type */

	uint8_t RTR;		/*!< Specifies the type of frame for the message that will 
						be transmitted. This parameter can be a value of 
						@ref CAN_remote_transmission_request */

	uint8_t Interrupt;	/*!< Specifies the interrupt switch will be setting.
						This parameter can be a value for ENABLE or DISABLE */

	uint8_t DLC;		/*!< Specifies the length of the frame that will be 
						transmitted. This parameter can be a value between 
						0 to 8 */

	uint8_t Data[8];	/*!< Contains the data to be transmitted. It ranges from 0 
						to 0xFF. */
} CanTxRxMsg;


/** @defgroup CAN_InitStatus 
  * @{
  */
#define CAN_InitStatus_Failed              ((uint8_t)0x00) /*!< CAN initialization failed */
#define CAN_InitStatus_Success             ((uint8_t)0x01) /*!< CAN initialization OK */

/** @defgroup CAN_operating_mode 
  * @{
  */
#define CAN_OpMode_Initial      ((uint8_t)0x00)  /*!< initialization mode */
#define CAN_OpMode_Normal       ((uint8_t)0x01)  /*!< normal mode */
#define CAN_OpMode_NormalABT    ((uint8_t)0x02)  /*!< normal mode follow ABT */
#define CAN_OpMode_OnlyReceive  ((uint8_t)0x03)  /*!< only receive mode */
#define CAN_OpMode_Single       ((uint8_t)0x04)  /*!< single mode */
#define CAN_OpMode_SelfCheck    ((uint8_t)0x05)  /*!< self check mode */
#define CAN_OPMODE_MASK         ((uint16_t)0x0007)

#define IS_CANOP_MODE(MODE)     (((MODE) == CAN_OpMode_Initial) || \
                                ((MODE) == CAN_OpMode_Normal)|| \
                                ((MODE) == CAN_OpMode_NormalABT) || \
                                ((MODE) == CAN_OpMode_OnlyReceive) || \
                                ((MODE) == CAN_OpMode_Single) || \
                                ((MODE) == CAN_OpMode_SelfCheck))

/** @defgroup CAN_powersave_mode 
  * @{
  */
#define CAN_PsMode_None         ((uint8_t)0x00)
#define CAN_PsMode_Sleep        ((uint8_t)0x01)
#define CAN_PsMode_Invalid      ((uint8_t)0x02)
#define CAN_PsMode_Stop         ((uint8_t)0x03)
#define CAN_PSMODE_MASK         ((uint16_t)0x0018)

#define IS_CANPS_MODE(MODE)     (((MODE) == CAN_PsMode_None) || \
                                ((MODE) == CAN_PsMode_Sleep)|| \
                                ((MODE) == CAN_PsMode_Stop))

/**
  * @}
  */

/**
  * @defgroup CAN_operating_mode_status
  * @{
  */  

#define CAN_ModeStatus_Failed    ((uint8_t)0x00)                        /*!< CAN entering the specific mode failed */
#define CAN_ModeStatus_Success   ((uint8_t)!CAN_ModeStatus_Failed)      /*!< CAN entering the specific mode Succeed */

/**
  * @defgroup CAN_function_status
  * @{
  */  

#define CAN_FuncStatus_Idle		((uint8_t)0x00)							/*!< CAN function working is idle */
#define CAN_FuncStatus_Busy		((uint8_t)!CAN_FuncStatus_Idle)			/*!< CAN function working is busy */

/**
  * @defgroup CAN_MsgCache_init_status
  * @{
  */  

#define CAN_MsgcacheInit_Failed		((uint8_t)0x00)						/*!< CAN entering the specific mode failed */
#define CAN_MsgcacheInit_Success	((uint8_t)!CAN_MsgcacheInit_Failed)	/*!< CAN entering the specific mode Succeed */

/** @defgroup CAN_synchronisation_jump_width 
  * @{
  */
#define CAN_SJW_1tq             ((uint8_t)0x00)  /*!< 1 time quantum */
#define CAN_SJW_2tq             ((uint8_t)0x01)  /*!< 2 time quantum */
#define CAN_SJW_3tq             ((uint8_t)0x02)  /*!< 3 time quantum */
#define CAN_SJW_4tq             ((uint8_t)0x03)  /*!< 4 time quantum */

#define IS_CAN_SJW(SJW)         (((SJW) == CAN_SJW_1tq) || ((SJW) == CAN_SJW_2tq)|| \
                                ((SJW) == CAN_SJW_3tq) || ((SJW) == CAN_SJW_4tq))
/**
  * @}
  */

/** @defgroup CAN_time_quantum_in_bit_segment_1 
  * @{
  */
#define CAN_BS1_2tq             ((uint8_t)0x01)  /*!< 2 time quantum */
#define CAN_BS1_3tq             ((uint8_t)0x02)  /*!< 3 time quantum */
#define CAN_BS1_4tq             ((uint8_t)0x03)  /*!< 4 time quantum */
#define CAN_BS1_5tq             ((uint8_t)0x04)  /*!< 5 time quantum */
#define CAN_BS1_6tq             ((uint8_t)0x05)  /*!< 6 time quantum */
#define CAN_BS1_7tq             ((uint8_t)0x06)  /*!< 7 time quantum */
#define CAN_BS1_8tq             ((uint8_t)0x07)  /*!< 8 time quantum */
#define CAN_BS1_9tq             ((uint8_t)0x08)  /*!< 9 time quantum */
#define CAN_BS1_10tq            ((uint8_t)0x09)  /*!< 10 time quantum */
#define CAN_BS1_11tq            ((uint8_t)0x0A)  /*!< 11 time quantum */
#define CAN_BS1_12tq            ((uint8_t)0x0B)  /*!< 12 time quantum */
#define CAN_BS1_13tq            ((uint8_t)0x0C)  /*!< 13 time quantum */
#define CAN_BS1_14tq            ((uint8_t)0x0D)  /*!< 14 time quantum */
#define CAN_BS1_15tq            ((uint8_t)0x0E)  /*!< 15 time quantum */
#define CAN_BS1_16tq            ((uint8_t)0x0F)  /*!< 16 time quantum */

#define IS_CAN_BS1(BS1)         ((BS1) <= CAN_BS1_16tq)

/** @defgroup CAN_time_quantum_in_bit_segment_2 
  * @{
  */
#define CAN_BS2_1tq             ((uint8_t)0x00)  /*!< 1 time quantum */
#define CAN_BS2_2tq             ((uint8_t)0x01)  /*!< 2 time quantum */
#define CAN_BS2_3tq             ((uint8_t)0x02)  /*!< 3 time quantum */
#define CAN_BS2_4tq             ((uint8_t)0x03)  /*!< 4 time quantum */
#define CAN_BS2_5tq             ((uint8_t)0x04)  /*!< 5 time quantum */
#define CAN_BS2_6tq             ((uint8_t)0x05)  /*!< 6 time quantum */
#define CAN_BS2_7tq             ((uint8_t)0x06)  /*!< 7 time quantum */
#define CAN_BS2_8tq             ((uint8_t)0x07)  /*!< 8 time quantum */

#define IS_CAN_BS2(BS2)         ((BS2) <= CAN_BS2_8tq)

/** @defgroup CAN_interrupts 
  * @{
  */ 
#define CAN_IT_TRX              ((uint8_t)0x01)	/*!< Transmit complete from message cache Interrupt */
#define CAN_IT_REC              ((uint8_t)0x02)	/*!< Receive complete from message cache Interrupt */
#define CAN_IT_ERR_STATE        ((uint8_t)0x04)	/*!< CAN error Interrupt */
#define CAN_IT_ERR_PROTO        ((uint8_t)0x08)	/*!< CAN protocol error Interrupt */
#define CAN_IT_ERR_ARBLOST      ((uint8_t)0x10)	/*!< CAN arbitration loss Interrupt */
#define CAN_IT_WKU              ((uint8_t)0x20)	/*!< CAN wake-up by bus Interrupt */

#define IS_CAN_IT(IT)           (((IT) == CAN_IT_TRX) || ((IT) == CAN_IT_REC)  || \
                                ((IT) == CAN_IT_ERR_STATE) || ((IT) == CAN_IT_ERR_PROTO) || \
								((IT) == CAN_IT_ERR_ARBLOST) || ((IT) == CAN_IT_WKU))

/** @defgroup CAN_interrupts status flag
  * @{
  */ 
#define CAN_FLAG_TRX			((uint8_t)0x01)	/*!< Transmit complete from message cache Interrupt flag */
#define CAN_FLAG_REC			((uint8_t)0x02)	/*!< Receive complete from message cache Interrupt flag */
#define CAN_FLAG_ERR_STATE		((uint8_t)0x04)	/*!< CAN error Interrupt flag */
#define CAN_FLAG_ERR_PROTO		((uint8_t)0x08)	/*!< CAN protocol error Interrupt flag */
#define CAN_FLAG_ERR_ARBLOST	((uint8_t)0x10)	/*!< CAN arbitration loss Interrupt flag */
#define CAN_FLAG_WKU			((uint8_t)0x20)	/*!< CAN wake-up by bus Interrupt flag */

#define IS_CAN_FLAG(FLAG)		(((FLAG) == CAN_FLAG_TRX) || ((FLAG) == CAN_FLAG_REC)  || \
                                ((FLAG) == CAN_FLAG_ERR_STATE) || ((FLAG) == CAN_FLAG_ERR_PROTO) || \
								((FLAG) == CAN_FLAG_ERR_ARBLOST) || ((FLAG) == CAN_FLAG_WKU))

/** @defgroup CAN_msgcache_type 
  * @{
  */
#define CAN_CacheType_Tx		((uint8_t)0x00)
#define CAN_CacheType_Rx_NoMask	((uint8_t)0x01)
#define CAN_CacheType_Rx_1Mask	((uint8_t)0x02)
#define CAN_CacheType_Rx_2Mask	((uint8_t)0x03)
#define CAN_CacheType_Rx_3Mask	((uint8_t)0x04)
#define CAN_CacheType_Rx_4Mask	((uint8_t)0x05)

#define IS_CAN_CACHETYPE(TYPE)	(((TYPE) == CAN_CacheType_Tx) || ((TYPE) == CAN_CacheType_Rx_NoMask) || \
								((TYPE) == CAN_CacheType_Rx_1Mask) || ((TYPE) == CAN_CacheType_Rx_2Mask) || \
								((TYPE) == CAN_CacheType_Rx_3Mask) || ((TYPE) == CAN_CacheType_Rx_4Mask))

/** @defgroup CAN_identifier_type 
  * @{
  */
#define CAN_Id_Standard				((uint8_t)0x00)	/*!< Standard Id */
#define CAN_Id_Extended				((uint8_t)0x01)	/*!< Extended Id */
#define IS_CAN_IDTYPE(IDTYPE)		(((IDTYPE) == CAN_Id_Standard) || \
									((IDTYPE) == CAN_Id_Extended))

/** @defgroup CAN_remote_transmission_request 
  * @{
  */
#define CAN_RTR_Data                ((uint8_t)0x00)	/*!< Data frame */
#define CAN_RTR_Remote              ((uint8_t)0x01)	/*!< Remote frame */
#define IS_CAN_RTR(RTR)				(((RTR) == CAN_RTR_Data) || ((RTR) == CAN_RTR_Remote))

/** @defgroup CANMsgCache_InitTypeDef DLC 
  * @{
  */
#define IS_CAN_DLC(DLC)				(((DLC) >= 0) && ((PRESCALER) <= 8))

/**
  * @}
  */

/** @defgroup CAN_clock_prescaler 
  * @{
  */
#define IS_CAN_PRESCALER(PRESCALER) (((PRESCALER) >= 1) && ((PRESCALER) <= 16))

/** @defgroup CAN_BitRatePrescaler 
  * @{
  */
#define IS_CAN_BITRATEPRESCALER(PRESCALER) (((PRESCALER) >= 1) && ((PRESCALER) <= 256))

/**
  * @defgroup CAN_Error_Code_constants
  * @{
  */
#define CAN_ErrorCode_NoErr           ((uint8_t)0x00) /*!< No Error */ 
#define	CAN_ErrorCode_StuffErr        ((uint8_t)0x01) /*!< Stuff Error */ 
#define	CAN_ErrorCode_FormErr         ((uint8_t)0x02) /*!< Form Error */ 
#define	CAN_ErrorCode_ACKErr          ((uint8_t)0x03) /*!< Acknowledgment Error */ 
#define	CAN_ErrorCode_BitRecessiveErr ((uint8_t)0x04) /*!< Bit Recessive Error */ 
#define	CAN_ErrorCode_BitDominantErr  ((uint8_t)0x05) /*!< Bit Dominant Error */ 
#define	CAN_ErrorCode_CRCErr          ((uint8_t)0x06) /*!< CRC Error  */ 

/**
  * @defgroup CAN_Error_Status_constants
  * @{
  */
#define CAN_ErrorStat_ErrActive		((uint8_t)0x01)	/*!< Error active */ 
#define CAN_ErrorStat_ErrPassive	((uint8_t)0x02)	/*!< Error passive */ 
#define CAN_ErrorStat_BusOff		((uint8_t)0x03)	/*!< Error bus-off */

/** @defgroup CAN control and status registers
  * @{
  */
/*******************  Bit definition for GMCTRL register  ********************/
#define CAN_GMCTRL_SET_GOM      ((uint16_t)0x0100)
#define CAN_GMCTRL_SET_EFSD     ((uint16_t)0x0200)
#define CAN_GMCTRL_CLR_GOM      ((uint16_t)0x0001)

/*******************  Bit definition for CCTRL register  ********************/
#define CAN_CCTRL_SET_CCERC		((uint16_t)0x8000)
#define CAN_CCTRL_SET_AL		((uint16_t)0x4000)
#define CAN_CCTRL_CLR_CCERC		((uint16_t)0x0080)
#define CAN_CCTRL_CLR_AL		((uint16_t)0x0040)
#define CAN_CCTRL_CLR_VALID		((uint16_t)0x0020)

#define CAN_CCTRL_RSTAT_MASK	((uint16_t)0x0200)
#define CAN_CCTRL_TSTAT_MASK	((uint16_t)0x0100)
#define CAN_CCTRL_CCERC_MASK	((uint16_t)0x0080)
#define CAN_CCTRL_AL_MASK		((uint16_t)0x0040)
#define CAN_CCTRL_VALID_MASK	((uint16_t)0x0020)

/*******************  Bit definition for CLEC register  ********************/
#define CAN_CLEC_ERRNONE_MASK	((uint8_t)0x00)
#define CAN_CLEC_ERRFILL_MASK	((uint8_t)0x01)
#define CAN_CLEC_ERRFORMAT_MASK	((uint8_t)0x02)
#define CAN_CLEC_ERRACK_MASK	((uint8_t)0x03)
#define CAN_CLEC_ERRPBIT_MASK	((uint8_t)0x04)
#define CAN_CLEC_ERRNBIT_MASK	((uint8_t)0x05)
#define CAN_CLEC_ERRCRC_MASK	((uint8_t)0x06)
#define CAN_CLEC_ERRNDEF_MASK	((uint8_t)0x07)

/*******************  Bit definition for CINFO register  ********************/
#define CAN_CINFO_BOFF_MASK		((uint8_t)0x10)
#define CAN_CINFO_TECS_MASK		((uint8_t)0x0C)
#define CAN_CINFO_RECS_MASK		((uint8_t)0x03)

#define CAN_GET_TECS(CINFO)		((uint8_t)(((CINFO) & CAN_CINFO_TECS_MASK) >> 2))
#define CAN_GET_RECS(CINFO)		((uint8_t)(((CINFO) & CAN_CINFO_RECS_MASK) >> 0))

/*******************  Bit definition for CERC register  ********************/
#define CAN_CERC_REPS_MASK		((uint16_t)0x8000)
#define CAN_CERC_REC_MASK		((uint16_t)0x7F00)
#define CAN_CERC_TEC_MASK		((uint16_t)0x00FF)

#define CAN_GET_TEC(CERC)		((uint8_t)(((CERC) & CAN_CERC_TEC_MASK) >> 0))
#define CAN_GET_REC(CERC)		((uint8_t)(((CERC) & CAN_CERC_REC_MASK) >> 8))

/*******************  Bit definition for GMABT register  ********************/
#define CAN_GMABT_SET_ABTCLR	((uint16_t)0x0200)
#define CAN_GMABT_START_ABTTRG	((uint16_t)0x0100)
#define CAN_GMABT_STOP_ABTTRG	((uint16_t)0x0001)

#define CAN_GMABT_ABTTRG_MASK	((uint16_t)0x0001)
#define CAN_GMABT_ABTCLR_MASK	((uint16_t)0x0002)

/*******************  Bit definition for GMABTD register  ********************/
#define CAN_GMABTD_0DBT			((uint16_t)0x0000)
#define CAN_GMABTD_32DBT		((uint16_t)0x0001)
#define CAN_GMABTD_64DBT		((uint16_t)0x0002)
#define CAN_GMABTD_128DBT		((uint16_t)0x0003)
#define CAN_GMABTD_256DBT		((uint16_t)0x0004)
#define CAN_GMABTD_512DBT		((uint16_t)0x0005)
#define CAN_GMABTD_1024DBT		((uint16_t)0x0006)
#define CAN_GMABTD_2048DBT		((uint16_t)0x0007)
#define CAN_GMABTD_4096DBT		((uint16_t)0x0008)

#define IS_CAN_ALL_DBT(DBT)		(((DBT) == CAN_GMABTD_0DBT) || ((DBT) == CAN_GMABTD_32DBT) || \
								((DBT) == CAN_GMABTD_64DBT)  || ((DBT) == CAN_GMABTD_128DBT) || \
								((DBT) == CAN_GMABTD_256DBT)  || ((DBT) == CAN_GMABTD_512DBT) || \
								((DBT) == CAN_GMABTD_1024DBT)  || ((DBT) == CAN_GMABTD_2048DBT) || \
								((DBT) == CAN_GMABTD_4096DBT))

/*******************  Bit definition for CRGPT register  ********************/
#define CAN_CRGPT_CLR_ROVF		((uint16_t)0x0001)

#define CAN_CRGPT_RGPT_MASK		((uint16_t)0xFF00)
#define CAN_CRGPT_RHPM_MASK		((uint16_t)0x0002)
#define CAN_CRGPT_ROVF_MASK		((uint16_t)0x0001)

#define CAN_GET_RGPT(CRGPT)		((uint8_t)((((CRGPT) & CAN_CRGPT_RGPT_MASK) >> 8) & (MAX_CAN_MSGCACHE_CNT - 1)))

/*******************  Bit definition for CTGPT register  ********************/
#define CAN_CTGPT_CLR_TOVF		((uint16_t)0x0001)

#define CAN_CTGPT_TGPT_MASK		((uint16_t)0xFF00)
#define CAN_CTGPT_THPM_MASK		((uint16_t)0x0002)
#define CAN_CTGPT_TOVF_MASK		((uint16_t)0x0001)

#define CAN_GET_TGPT(CTGPT)		((uint8_t)((((CTGPT) & CAN_CTGPT_TGPT_MASK) >> 8) & (MAX_CAN_MSGCACHE_CNT - 1)))

/*******************  Bit definition for CLOPT register  ********************/
#define CAN_CLOPT_LOPT_MASK		((uint8_t)0xFF)
#define CAN_GET_LOPT(CLOPT)		((uint8_t)((((CLOPT) & CAN_CLOPT_LOPT_MASK) >> 0) & (MAX_CAN_MSGCACHE_CNT - 1)))

/*******************  Bit definition for CLIPT register  ********************/
#define CAN_CLIPT_LIPT_MASK		((uint8_t)0xFF)
#define CAN_GET_LIPT(CLIPT)		((uint8_t)((((CLIPT) & CAN_CLIPT_LIPT_MASK) >> 0) & (MAX_CAN_MSGCACHE_CNT - 1)))

/*******************  Bit definition for CTS register  **********************/
#define CAN_CTS_TSEN_MASK		((uint16_t)0x0001)
#define CAN_CTS_TSSEL_MASK		((uint16_t)0x0002)
#define CAN_CTS_TSLOCK_MASK		((uint16_t)0x0004)

#define CAN_CTS_SET_TSEN		((uint16_t)0x0100)
#define CAN_CTS_CLR_TSEN		((uint16_t)0x0001)
#define CAN_CTS_SET_TSSEL		((uint16_t)0x0200)
#define CAN_CTS_CLR_TSSEL		((uint16_t)0x0002)
#define CAN_CTS_SET_TSLOCK		((uint16_t)0x0400)
#define CAN_CTS_CLR_TSLOCK		((uint16_t)0x0004)

/** @defgroup CAN Message cache registers
  * @{
  */
/*******************  Bit definition for MCTRL register  ********************/
#define CAN_MCTRL_SET_RDY       ((uint16_t)0x0100)
#define CAN_MCTRL_SET_TRQ       ((uint16_t)0x0200)
#define CAN_MCTRL_SET_IE        ((uint16_t)0x0800)

#define CAN_MCTRL_CLR_RDY       ((uint16_t)0x0001)
#define CAN_MCTRL_CLR_TRQ       ((uint16_t)0x0002)
#define CAN_MCTRL_CLR_DN        ((uint16_t)0x0004)
#define CAN_MCTRL_CLR_IE        ((uint16_t)0x0008)
#define CAN_MCTRL_CLR_MOW       ((uint16_t)0x0010)

#define CAN_MCTRL_RDY_MASK      ((uint16_t)0x0001)
#define CAN_MCTRL_TRQ_MASK      ((uint16_t)0x0002)
#define CAN_MCTRL_DN_MASK       ((uint16_t)0x0004)
#define CAN_MCTRL_IE_MASK       ((uint16_t)0x0008)
#define CAN_MCTRL_MOW_MASK      ((uint16_t)0x0010)
#define CAN_MCTRL_MUC_MASK      ((uint16_t)0x2000)

/*******************  Bit definition for MCONF register  ********************/
#define CAN_MCONF_MA0           ((uint8_t)0x01)
#define CAN_MCONF_MT            ((uint8_t)0x38)
#define CAN_MCONF_RTR           ((uint8_t)0x40)
#define CAN_MCONF_OWS           ((uint8_t)0x80)

/* Exported functions --------------------------------------------------------*/

/*  Function used to set the CAN configuration to the default reset state *****/
void CAN_DeInit(CAN_Type* CANx);

/* Initialization and Configuration functions *********************************/ 
uint8_t CAN_Init(CAN_Type* CANx, CAN_InitTypeDef* CAN_InitStruct);

/* CAN working Mode request set by @CAN_OperateMode */
uint8_t CAN_OperatingModeRequest(CAN_Type* CANx, uint8_t CAN_OperateMode);

/* CAN powersave mode request set by @CAN_PowersaveMode */
uint8_t CAN_PowersaveModeRequest(CAN_Type* CANx, uint8_t CAN_PowersaveMode);

/* CAN ABT mode config and get status functions *******************************/
/* CAN ABT Mode configuration to enable and to set DBT value */
uint8_t CAN_ABTModeTransmitConfig(CAN_Type* CANx, uint16_t DBT);
uint8_t CAN_GetABTStatus(CAN_Type* CANx);

/* Function used to set the CAN Message Cache configuration to the default reset state *****/
uint8_t CAN_MessageCache_DeInit(CANMSG_Type *CANxMSGy);

/* Function used to set the CAN Message Cache configuration by @TxRxMessage */
uint8_t CAN_MessageCache_Init(CANMSG_Type *CANxMSGy, CanTxRxMsg *TxRxMessage);

/* Function used to set the CAN Message Cache over-write attribute */
void CAN_MessageCache_OverWriteConfig(CANMSG_Type *CANxMSGy, FunctionalState NewState);

/* Function used to transmit CAN frame data from @TxMessage to message cache */
uint8_t CAN_Transmit(CANMSG_Type *CANxMSGy, CanTxRxMsg* TxMessage);

/* Function used to receive CAN frame data from message cache to RxMessage, timeout unit is system clock tick */
uint8_t CAN_Receive(CAN_Type* CANx, CANMSG_Type *CANxMSGy, CanTxRxMsg* RxMessage, uint32_t Timeout);
uint8_t CAN_Receive_IT(CAN_Type* CANx, CANMSG_Type *CANxMSGy, CanTxRxMsg* RxMessage);

/* CAN Bus Error management functions *****************************************/
uint8_t CAN_GetLastErrorCode(CAN_Type* CANx);
uint8_t CAN_GetErrorStatus(CAN_Type* CANx);
uint8_t CAN_GetReceiveErrorCounter(CAN_Type* CANx);
uint8_t CAN_GetTransmitErrorCounter(CAN_Type* CANx);

/* Interrupts and flags management functions **********************************/
void CAN_ITConfig(CAN_Type* CANx, uint8_t CAN_IT, FunctionalState NewState);
FlagStatus CAN_GetFlagStatus(CAN_Type* CANx, uint8_t CAN_FLAG);
void CAN_ClearFlag(CAN_Type* CANx, uint8_t CAN_FLAG);

#ifdef __cplusplus
}
#endif

#endif /* __DRIVER_CAN_H__ */
