/**
  ******************************************************************************
  * @file    can.c
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    27-January-2022
  * @brief   This file provides firmware functions to manage the following 
  *          functionalities of the Controller area network (CAN) peripheral:
  *           + Initialization and Configuration 
  *           + CAN Frames Transmission
  *           + CAN Frames Reception
  *           + Operation modes switch
  *           + Error management
  *           + Interrupts and flags
  *
@verbatim
 ===============================================================================
                        ##### How to use this driver #####
 ===============================================================================
    [..]
      (#) Enable the CAN controller interface clock using 
          CGC_PER0PeriphClockCmd(CGC_PER0Periph_CAN0, ENABLE); for CAN0
      -@- In case you are using CAN1 only, you have to enable the CAN1 clock.
       
      (#) CAN pins configuration
        (++) Connect the involved CAN pins to GROUP_AF_CTXD/CRXD using the following function 
                GPIO_PinAFConfig(GPIO_PORTx, GPIO_Pin_y, GPIO_Pxy, GROUP_AF_CTXD);
                GPIO_PinAFConfig(GPIO_PORTx, GPIO_Pin_y, GPIO_Pxy, GROUP_AF_CRXD);
        (++) Configure these CAN pins in alternate function mode by calling
             the function  GPIO_Init();
        
      (#) Initialize and configure the CAN using CAN_Init() and 
          CAN_OperatingModeRequest() functions to configurate working mode.
        
      (#) Initialize and configure the CAN Message cache using 
          CAN_MessageCache_DeInit() and CAN_MessageCache_Init().

      (#) When we want to use ABT mode, on the base of fore used CAN_OperatingModeRequest() function
          to set CAN_OpMode_NormalABT mode, then to config ABT mode by CAN_ABTModeTransmitConfig()
          function to set each frame DBT value and trigger ABT mode start.
        (++) Use CAN_GetABTStatus() function to get the status by ABT.
        (++) ABT mode only support message cache from CAN0MSG00 to CAN0MSG07 and id must be continuous.
        
      (#) Transmit the desired CAN frame using CAN_Transmit() function.
        (++) CAN is in CAN_OpMode_Normal mode for polling type.
        
      (#) Receive a CAN frame using CAN_Receive() function.
        (++) CAN is in CAN_OpMode_Normal mode for polling type.
        
      (#) To control CAN events you can use one of the following two methods:
        (++) Check on CAN flags using the CAN_GetFlagStatus() function.  
        (++) Use CAN interrupts through the function CAN_ITConfig() at 
             initialization phase and CAN_GetITStatus() function into 
             interrupt routines to check if the event has occurred or not.
             After checking on a flag you should clear it using CAN_ClearFlag()
             function. 

@endverbatim
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


/* Includes ------------------------------------------------------------------*/
#include "can.h"
#include "cgc.h"

/** @addtogroup BAT32G137xx_StdPeriph_Driver
  * @{
  */

/** @defgroup CAN 
  * @brief CAN driver modules
  * @{
  */ 
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

/* Time out for Init operate */
#define INIT_TIMEOUT        ((uint32_t)0x0000FFFF)

/* Time out for mode setting */
#define SMODE_TIMEOUT       ((uint32_t)0x0000FFFF)

/* Time out for cache init */
#define CACHE_TIMEOUT       ((uint32_t)0x0000FFFF)

/**
  * @brief  Deinitializes the CAN peripheral registers to their default reset values.
  * @param  CANx: where x can be 0 select the CAN peripheral.
  * @retval None.
  */
void CAN_DeInit(CAN_Type* CANx)
{
    CGC_PER0PeriphClockCmd(CGC_PER0Periph_CAN0, ENABLE);
    CGC_PER0PeriphClockCmd(CGC_PER0Periph_CAN0, DISABLE);
}

/**
  * @brief  Initializes the CAN peripheral according to the specified
  *         parameters in the CAN_InitStruct.
  * @param  CANx: where x can be 0 to select the CAN peripheral.
  * @param  CAN_InitStruct: pointer to a CAN_InitTypeDef structure that contains
  *         the configuration information for the CAN peripheral.
  * @retval Constant indicates initialization succeed which will be 
  *         CAN_InitStatus_Failed or CAN_InitStatus_Success.
  */
uint8_t CAN_Init(CAN_Type* CANx, CAN_InitTypeDef* CAN_InitStruct)
{
    uint8_t InitStatus = CAN_InitStatus_Success;
    uint32_t wait_ack = 0x00000000;

    assert_param(IS_CAN_ALL_PERIPH(CANx));
    assert_param(IS_FUNCTIONAL_STATE(CAN_InitStruct->CAN_TTCM));
    assert_param(IS_FUNCTIONAL_STATE(CAN_InitStruct->CAN_ABTM));
    assert_param(IS_FUNCTIONAL_STATE(CAN_InitStruct->CAN_AWUM));
    assert_param(IS_CANOP_MODE(CAN_InitStruct->CAN_OperateMode));
    assert_param(IS_CAN_SJW(CAN_InitStruct->CAN_SJW));
    assert_param(IS_CAN_BS1(CAN_InitStruct->CAN_BS1));
    assert_param(IS_CAN_BS2(CAN_InitStruct->CAN_BS2));
    assert_param(IS_CAN_PRESCALER(CAN_InitStruct->CAN_Prescaler));
    assert_param(IS_CAN_BITRATEPRESCALER(CAN_InitStruct->CAN_BitRatePrescaler));

    /* Periphal clock enable for CAN module */
    if (CANx == CAN0)
    {
        CGC_PER0PeriphClockCmd(CGC_PER0Periph_CAN0, ENABLE);
    }
#if defined(BAT32G1XX_100PIN) ||  defined(BAT32G1XX_80PIN)
    if (CANx == CAN1)
    {
        CGC_PER2PeriphClockCmd(CGC_PER2Periph_CAN1, ENABLE);
    }
#endif	
    /* CAN periphal Initialization process */
    /* 1. Set the clock prescaler */
	CANx->CGMCS = CAN_InitStruct->CAN_Prescaler - 1;

    /* 2. Set SET_GOM and reset CLR_GOM to Enable CAN */
	CANx->CGMCTRL |=  CAN_GMCTRL_SET_GOM;
    CANx->CGMCTRL &= ~CAN_GMCTRL_CLR_GOM;

    /* 3. Set CBRP and CBTR to get the baudrate */
	CANx->CBRP = CAN_InitStruct->CAN_BitRatePrescaler - 1;
	CANx->CBTR = (uint16_t) ((uint16_t)CAN_InitStruct->CAN_SJW << 12) | \
                            ((uint16_t)CAN_InitStruct->CAN_BS2 << 8) | \
                            ((uint16_t)CAN_InitStruct->CAN_BS1 << 0);

    /* 4. Set All Mask for CAN filter */
	CANx->CMASK1L = CAN_InitStruct->MASK1 & 0xFFFF;
	CANx->CMASK1H = (CAN_InitStruct->MASK1 >> 16) & 0x1FFF;
	CANx->CMASK2L = CAN_InitStruct->MASK2 & 0xFFFF;
	CANx->CMASK2H = (CAN_InitStruct->MASK2 >> 16) & 0x1FFF;
	CANx->CMASK3L = CAN_InitStruct->MASK3 & 0xFFFF;
	CANx->CMASK3H = (CAN_InitStruct->MASK3 >> 16) & 0x1FFF;
	CANx->CMASK4L = CAN_InitStruct->MASK4 & 0xFFFF;
	CANx->CMASK4H = (CAN_InitStruct->MASK4 >> 16) & 0x1FFF;

    /* 5. Set CAN operating mode */
    CANx->CCTRL |=  (uint16_t)(CAN_InitStruct->CAN_OperateMode << 8);
    CANx->CCTRL &= ~(uint16_t)(CAN_InitStruct->CAN_OperateMode);

    /* Wait the operate complete */
    while (((CANx->CCTRL & CAN_OPMODE_MASK) != CAN_InitStruct->CAN_OperateMode) && (wait_ack != INIT_TIMEOUT))
    {
        wait_ack++;
    }

    /* Check register setting success or fail */
    if ((CANx->CCTRL & CAN_OPMODE_MASK) != CAN_InitStruct->CAN_OperateMode)
    {
        InitStatus = CAN_InitStatus_Failed;
    }

    return InitStatus;
}

/** @defgroup CAN_Group5 CAN Bus Error management functions
 *  @brief    CAN Bus Error management functions 
 *
@verbatim    
 ===============================================================================
                ##### CAN Bus Error management functions #####
 ===============================================================================  
    [..] This section provides functions allowing to 
      (+) Return the CANx's last error code (LEC)
      (+) Return the CANx Receive Error Counter (REC)
      (+) Return the LSB of the 8-bit CANx Transmit Error Counter(TEC).
   
      -@- If TEC is greater than 255, The CAN is in bus-off state.
      -@- if REC or TEC are greater than 96, an Error warning flag occurs.
      -@- if REC or TEC are greater than 127, an Error Passive Flag occurs.
                        
@endverbatim
  * @{
  */
  
/**
  * @brief  Returns the CANx's last error code (LEC).
  * @param  CANx: where x can be 0 to select the CAN peripheral.
  * @retval Error code: 
  *          - CAN_ERRORCODE_NoErr: No Error  
  *          - CAN_ERRORCODE_StuffErr: Stuff Error
  *          - CAN_ERRORCODE_FormErr: Form Error
  *          - CAN_ERRORCODE_ACKErr : Acknowledgment Error
  *          - CAN_ERRORCODE_BitRecessiveErr: Bit Recessive Error
  *          - CAN_ERRORCODE_BitDominantErr: Bit Dominant Error
  *          - CAN_ERRORCODE_CRCErr: CRC Error
  */
uint8_t CAN_GetLastErrorCode(CAN_Type* CANx)
{
    uint8_t errorcode = 0;

    /* Check the parameters */
    assert_param(IS_CAN_ALL_PERIPH(CANx));

    /* Get the error code*/
    errorcode = (((uint8_t)CANx->CLEC) & (uint8_t)CAN_CLEC_ERRNDEF_MASK);

    /* Return the error code*/
    return errorcode;
}

/**
  * @brief  Returns the CANx's error status.
  * @param  CANx: where x can be 0 to select the CAN peripheral.
  * @retval Error Status: 
  *          - CAN_ErrorStat_ErrActive: Error Active Status
  *          - CAN_ErrorStat_ErrPassive: Error Passive Status
  *          - CAN_ErrorStat_BusOff: Bus-Off Error
  */
uint8_t CAN_GetErrorStatus(CAN_Type* CANx)
{
    uint8_t errorstat = 0;

    /* Check the parameters */
    assert_param(IS_CAN_ALL_PERIPH(CANx));

    if (CANx->CINFO & CAN_CINFO_BOFF_MASK)
    {
        /* Bus Off Status: Transmit error counter greater than 255 */
        if (CAN_GET_TECS(CANx->CINFO) == 0x03)
        {
            errorstat = CAN_ErrorStat_BusOff;
        }
    }
    else
    {
        if ((CANx->CERC & CAN_CERC_REPS_MASK) && \
            ((CAN_GET_TECS(CANx->CINFO) == 0x03) || CAN_GET_RECS(CANx->CINFO) == 0x03))
        {
            /* Error Passive Status: */
            errorstat = CAN_ErrorStat_ErrPassive;
        }
        else
        {
            /* Error Active Status: */
            errorstat = CAN_ErrorStat_ErrActive;
        }
    }

    return errorstat;
}

/**
  * @brief  Returns the CANx Receive Error Counter (REC).
  * @note   In case of an error during reception, this counter is incremented 
  *         by 1 or by 8 depending on the error condition as defined by the CAN 
  *         standard. After every successful reception, the counter is 
  *         decremented by 1 or reset to 120 if its value was higher than 128. 
  *         When the counter value exceeds 127, the CAN controller enters the 
  *         error passive state.  
  * @param  CANx: where x can be 0 to select the CAN peripheral.
  * @retval CAN Receive Error Counter from 0 to 127. 
  */
uint8_t CAN_GetReceiveErrorCounter(CAN_Type* CANx)
{
    uint8_t counter = 0;

    /* Check the parameters */
    assert_param(IS_CAN_ALL_PERIPH(CANx));

    /* Get the Receive Error Counter*/
    counter = CAN_GET_REC(CANx->CERC);

    if (CAN_GET_RECS(CANx->CINFO) == 0x03)
    {
        counter = 128;
    }

    /* Return the Receive Error Counter*/
    return counter;
}

/**
  * @brief  Returns the LSB of the CANx Transmit Error Counter(TEC).
  * @param  CANx: where x can be 0 to select the CAN peripheral.
  * @retval LSB of the 8-bit CAN Transmit Error Counter. 
  */
uint8_t CAN_GetTransmitErrorCounter(CAN_Type* CANx)
{
    uint8_t counter = 0;

    /* Check the parameters */
    assert_param(IS_CAN_ALL_PERIPH(CANx));

    /* Get the LSB of the CANx Transmit Error Counter(TEC) */
    counter = CAN_GET_TEC(CANx->CERC);

    /* Return the LSB of the CANx Transmit Error Counter(TEC) */
    return counter;
}

/**
  * @brief  Enables or disables the specified CANx interrupts.
  * @param  CANx: where x can be 0 to select the CAN peripheral.
  * @param  CAN_IT: specifies the CAN interrupt sources to be enabled or disabled.
  *          This parameter can be: 
  *            @arg CAN_IT_TRX:         Transmit or receive from cache Interrupt 
  *            @arg CAN_IT_REC:         Cache receive frame Interrupt
  *            @arg CAN_IT_ERR_STATE:   CAN state error Interrupt
  *            @arg CAN_IT_ERR_PROTO:   CAN protocol error Interrupt
  *            @arg CAN_IT_ERR_ARBLOST: CAN Arbitration lost error Interrupt
  *            @arg CAN_IT_WKU:         Wake-up Interrupt
  * @param  NewState: new state of the CAN interrupts.
  * @note   This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
void CAN_ITConfig(CAN_Type* CANx, uint8_t CAN_IT, FunctionalState NewState)
{
    /* Check the parameters */
    assert_param(IS_CAN_ALL_PERIPH(CANx));
    assert_param(IS_CAN_IT(CAN_IT));
    assert_param(IS_FUNCTIONAL_STATE(NewState));

    if (NewState != DISABLE)
    {
        /* Enable the selected CANx interrupt */
        CANx->CIE |=  (uint16_t)(CAN_IT << 8);
        CANx->CIE &= ~(uint16_t)(CAN_IT);
    }
    else
    {
        /* Disable the selected CANx interrupt */
        CANx->CIE &= ~(uint16_t)(CAN_IT << 8);
        CANx->CIE |=  (uint16_t)(CAN_IT);
    }
}

/**
  * @brief  Get CAN flag status for events.
  * @param  CANx: where x can be 0 to select the CAN peripheral.
  * @param  CAN_FLAG: specifies the CAN flag can be set or reset.
  *          This parameter can be: 
  *            @arg CAN_FLAG_TRX:         Transmit or receive from cache flag.
  *            @arg CAN_FLAG_REC:         Cache receive frame flag.
  *            @arg CAN_FLAG_ERR_STATE:   CAN state error flag.
  *            @arg CAN_FLAG_ERR_PROTO:   CAN protocol error flag.
  *            @arg CAN_FLAG_ERR_ARBLOST: CAN Arbitration lost error flag.
  *            @arg CAN_FLAG_WKU:         Wake-up flag.
  * @retval This parameter can be: SET or RESET.
  */
FlagStatus CAN_GetFlagStatus(CAN_Type* CANx, uint8_t CAN_FLAG)
{
    FlagStatus bitstatus = RESET;

    /* Check the parameters */
    assert_param(IS_CAN_ALL_PERIPH(CANx));
    assert_param(IS_CAN_FLAG(CAN_FLAG));

    /* get the interrupt flag bit value */
    if ((CAN_FLAG & CANx->CINTS) != (uint16_t)RESET)
    {
        bitstatus = SET;
    }
    else
    {
        bitstatus = RESET;
    }

    return bitstatus;
}

/**
  * @brief  Clear CAN flag status for events.
  * @param  CANx: where x can be 0 to select the CAN peripheral.
  * @param  CAN_FLAG: specifies the CAN flag can be set or reset.
  *          This parameter can be: 
  *            @arg CAN_FLAG_TRX:         Transmit or receive from cache flag.
  *            @arg CAN_FLAG_REC:         Cache receive frame flag.
  *            @arg CAN_FLAG_ERR_STATE:   CAN state error flag.
  *            @arg CAN_FLAG_ERR_PROTO:   CAN protocol error flag.
  *            @arg CAN_FLAG_ERR_ARBLOST: CAN Arbitration lost error flag.
  *            @arg CAN_FLAG_WKU:         Wake-up flag.
  * @retval None.
  */
void CAN_ClearFlag(CAN_Type* CANx, uint8_t CAN_FLAG)
{
    /* Check the parameters */
    assert_param(IS_CAN_ALL_PERIPH(CANx));
    assert_param(IS_CAN_FLAG(CAN_FLAG));

    /* write 1 to CINTS bit to clear flag */
    CANx->CINTS = (uint16_t)CAN_FLAG;
}

/**
  * @brief  Selects the CAN Operation mode.
  * @param  CANx: where x can be 0 to select the CAN peripheral.
  * @param  CAN_OperateMode: CAN Operating Mode.
  *         This parameter can be one of @ref CAN_operating_mode.
  * @retval status of the requested mode which can be 
  *         - CAN_ModeStatus_Failed:  CAN failed entering the specific mode 
  *         - CAN_ModeStatus_Success: CAN Succeed entering the specific mode 
  */
uint8_t CAN_OperatingModeRequest(CAN_Type* CANx, uint8_t CAN_OperateMode)
{
    uint8_t status = CAN_ModeStatus_Failed;
    uint32_t timeout = SMODE_TIMEOUT; 

    /* Check the parameters */
    assert_param(IS_CAN_ALL_PERIPH(CANx));
    assert_param(IS_CANOP_MODE(CAN_OperateMode));

    /* Set CCTRL set bit and Clear clr bit for OPMODE0~OPMODE2 register */
    CANx->CCTRL |=  (uint16_t)(CAN_OperateMode << 8);
    CANx->CCTRL &= ~(uint16_t)(CAN_OperateMode);

    /* Wait the operate complete */
    while (((CANx->CCTRL & CAN_OPMODE_MASK) != CAN_OperateMode) && (timeout != 0))
    {
        timeout--;
    }

    /* read register OPMODE0~OPMODE2 to check setting success or fail */
    if ((CANx->CCTRL & CAN_OPMODE_MASK) != 0)
    {
        status = CAN_ModeStatus_Failed;
    }
    else
    {
        status = CAN_ModeStatus_Success;
    }

    return status;
}

/**
  * @brief  Selects the CAN power save mode.
  * @param  CANx: where x can be 0 to select the CAN peripheral.
  * @param  CAN_PowersaveMode: CAN Powersave Mode.
  *         This parameter can be one of @ref CAN_powersave_mode.
  * @retval status of the requested mode which can be 
  *         - CAN_ModeStatus_Failed:  CAN failed entering the specific mode 
  *         - CAN_ModeStatus_Success: CAN Succeed entering the specific mode 
  */
uint8_t CAN_PowersaveModeRequest(CAN_Type* CANx, uint8_t CAN_PowersaveMode)
{
    uint8_t status = CAN_ModeStatus_Failed;
    uint32_t timeout = SMODE_TIMEOUT;

    /* Check the parameters */
    assert_param(IS_CAN_ALL_PERIPH(CANx));
    assert_param(IS_CANPS_MODE(CAN_PowersaveMode));

    /* Set CCTRL set bit and Clear clr bit for PSMODE0~PSMODE1 register */
    CANx->CCTRL |=  (uint16_t)(CAN_PowersaveMode << 11);
    CANx->CCTRL &= ~(uint16_t)(CAN_PowersaveMode << 3);

    /* Wait the operate complete */
    while (((CANx->CCTRL & CAN_PSMODE_MASK) != CAN_PowersaveMode) && (timeout != 0))
    {
        timeout--;
    }

    /* read register PSMODE0~PSMODE1 to check setting success or fail */
    if ((CANx->CCTRL & CAN_PSMODE_MASK) != 0)
    {
        status = CAN_ModeStatus_Failed;
    }
    else
    {
        status = CAN_ModeStatus_Success;
    }

    return status;
}

/**
  * @brief  Configurate the CAN ABT transmit mode.
  * @param  CANx: where x can be 0 to select the CAN peripheral.
  * @param  DBT: CAN ABT transmit for each block delay time.
  * @retval status of the configurate which can be 
  *         - CAN_ModeStatus_Failed:  CAN failed configurate the specific mode 
  *         - CAN_ModeStatus_Success: CAN Succeed configurate the specific mode 
  */
uint8_t CAN_ABTModeTransmitConfig(CAN_Type* CANx, uint16_t DBT)
{
    /* Check the parameters */
    assert_param(IS_CAN_ALL_PERIPH(CANx));
	assert_param(IS_CAN_ALL_DBT(DBT));
	
	/* When ABTTRG is set, operate will fail */
	if (CANx->CGMABT & CAN_GMABT_ABTTRG_MASK)
	{
		return CAN_ModeStatus_Failed;
	}
	
	/* When TSTAT is set, operate will fail */
	if (CANx->CCTRL & CAN_CCTRL_TSTAT_MASK)
	{
		return CAN_ModeStatus_Failed;
	}
	
	/* Set ABT time delay for each block transmit */
	CANx->CGMABTD = DBT;
	
	/* Set ABTTRG to start trigger ABT mode */
	CANx->CGMABT = CAN_GMABT_START_ABTTRG;
	
	return CAN_ModeStatus_Success;
}

/**
  * @brief  Get the CAN ABT transmit status.
  * @param  CANx: where x can be 0 to select the CAN peripheral.
  * @retval status of the configurate which can be 
  *         - CAN_ModeStatus_Failed:  CAN ABT is busy. 
  *         - CAN_ModeStatus_Success: CAN ABT is idle.
  */
uint8_t CAN_GetABTStatus(CAN_Type* CANx)
{
    /* Check the parameters */
    assert_param(IS_CAN_ALL_PERIPH(CANx));
	
	/* When ABTTRG is set, operate will fail */
	if (CANx->CGMABT & CAN_GMABT_ABTTRG_MASK)
	{
		return CAN_FuncStatus_Busy;
	}
	
	return CAN_FuncStatus_Idle;
}

/**
  * @brief  CAN message cache register deinit.
  * @param  CANxMSGy: where x can be 0 to select the CAN peripheral.
  *                   where y can be 0 to 15 to select the cache.
  * @retval status of the requested mode which can be 
  *         - CAN_ModeStatus_Failed:  CAN failed entering the specific mode 
  *         - CAN_ModeStatus_Success: CAN Succeed entering the specific mode 
  */
uint8_t CAN_MessageCache_DeInit(CANMSG_Type *CANxMSGy)
{
    uint32_t timeout = CACHE_TIMEOUT;

    /* Check the parameters */
    assert_param(IS_CAN_ALL_MSGCACHE(CANxMSGy));

    /* Check message cache RDY bit and clear it */
    if ((CANxMSGy->CMCTRL & CAN_MCTRL_RDY_MASK) != 0x00)
    {
        CANxMSGy->CMCTRL = CAN_MCTRL_CLR_RDY;

        /* Wait the operate complete */
        while (((CANxMSGy->CMCTRL & CAN_MCTRL_RDY_MASK) != 0x00) && (timeout != 0))
        {
            timeout--;
        }
    }

    if ((CANxMSGy->CMCTRL & CAN_MCTRL_RDY_MASK) == 0x00)
    {
        /* clear TRQ and DN bits for CMCTRL */
        CANxMSGy->CMCTRL = CAN_MCTRL_CLR_TRQ;
        CANxMSGy->CMCTRL = CAN_MCTRL_CLR_DN;

        /* clear message cache enable bit for MA0 */
        CANxMSGy->CMCONF &= ~CAN_MCONF_MA0;

        /* clear message cache mask select for MT0~MT2  */
        CANxMSGy->CMCONF &= ~CAN_MCONF_MT;
		
		/* clear all bit for message config register */
		CANxMSGy->CMCONF = 0x00;

        return CAN_ModeStatus_Success;
    }
    
    return CAN_ModeStatus_Failed;
}

/**
  * @brief  CAN message cache register init.
  * @param  CANxMSGy: where x can be 0 to select the CAN peripheral.
  *                   where y can be 0 to 15 to select the cache.
  * @param  TxRxMessage: the message for tx or rx message to change RTR ID and CacheType.
  * @retval status of the requested mode which can be 
  *         - CAN_MsgcacheInit_Failed:  CAN failed to init message cache
  *         - CAN_MsgcacheInit_Success: CAN Succeed to init message cache
  */
uint8_t CAN_MessageCache_Init(CANMSG_Type *CANxMSGy, CanTxRxMsg *TxRxMessage)
{
    /* Check the parameters */
    assert_param(IS_CAN_ALL_MSGCACHE(CANxMSGy));
    assert_param(IS_CAN_CACHETYPE(TxRxMessage->CacheType));
    assert_param(IS_CAN_IDTYPE(TxRxMessage->IDE));
    assert_param(IS_CAN_RTR(TxRxMessage->RTR));
    assert_param(IS_CAN_DLC(TxRxMessage->DLC));

    /* Set message cache used flag */
	CANxMSGy->CMCONF |= CAN_MCONF_MA0;

    /* Set message cache type */
    CANxMSGy->CMCONF |= (TxRxMessage->CacheType << 3);

    /* Message cache frame ID setting by IDType */
    if(TxRxMessage->IDE == CAN_Id_Extended)
    {
        CANxMSGy->CMIDL = TxRxMessage->Id & ((uint16_t)0xFFFF);
        CANxMSGy->CMIDH = ((TxRxMessage->Id >> 16U) & ((uint16_t)0xFFFF)) | ((uint16_t)0x8000);
    }
    else if(TxRxMessage->IDE == CAN_Id_Standard)
    {
        CANxMSGy->CMIDL = 0;
        CANxMSGy->CMIDH = (TxRxMessage->Id << 2U) & ((uint16_t)0x1FFC);
    }

    /* RTR setting to select standard frame or remote frame */
    if (TxRxMessage->RTR == CAN_RTR_Remote)
    {
        CANxMSGy->CMCONF |=  CAN_MCONF_RTR;
    }
    else if (TxRxMessage->RTR == CAN_RTR_Data)
    {
        CANxMSGy->CMCONF &= ~CAN_MCONF_RTR;
    }
	
	/* When frame type is tx type, set frame data and length */
	if (TxRxMessage->CacheType == CAN_CacheType_Tx)
	{
		/* Set the frame data length */
		CANxMSGy->CMDLC = TxRxMessage->DLC;

		/* Set the frame data value from TxRxMessage->Data */
        CANxMSGy->CMDB0 = TxRxMessage->Data[0];
        CANxMSGy->CMDB1 = TxRxMessage->Data[1];
        CANxMSGy->CMDB2 = TxRxMessage->Data[2];
        CANxMSGy->CMDB3 = TxRxMessage->Data[3];
        CANxMSGy->CMDB4 = TxRxMessage->Data[4];
        CANxMSGy->CMDB5 = TxRxMessage->Data[5];
        CANxMSGy->CMDB6 = TxRxMessage->Data[6];
        CANxMSGy->CMDB7 = TxRxMessage->Data[7];
	}
	
	/* When the message cache interrupt enable, to set IE bit */
	if (TxRxMessage->Interrupt != DISABLE)
	{
		CANxMSGy->CMCTRL = CAN_MCTRL_SET_IE;
	}
	else
	{
		CANxMSGy->CMCTRL = CAN_MCTRL_CLR_IE;
	}
	
	/* When the message cache init, we should set RDY bit */
	CANxMSGy->CMCTRL = CAN_MCTRL_SET_RDY;
	
	return CAN_MsgcacheInit_Success;
}

/**
  * @brief  CAN periphal for nessage cache over write config.
  * @param  CANxMSGy: where x can be 0 to select the CAN peripheral.
  *                   where y can be 0 to 15 to select the cache.
  * @param  NewState: new state of the CAN interrupts.
  * @note   This parameter can be: ENABLE or DISABLE.
  * @retval none.
  */
void CAN_MessageCache_OverWriteConfig(CANMSG_Type *CANxMSGy, FunctionalState NewState)
{
    /* Check the parameters */
    assert_param(IS_CAN_ALL_MSGCACHE(CANxMSGy));
    assert_param(IS_FUNCTIONAL_STATE(NewState));
	
	if (NewState != DISABLE)
	{
		CANxMSGy->CMCONF |=  CAN_MCONF_OWS;
	}
	else
	{
		CANxMSGy->CMCONF &= ~CAN_MCONF_OWS;
	}
}

/**
  * @brief  Initiates and transmits a CAN frame message.
  * @param  CANxMSGy: where x can be 0 to select the CAN peripheral.
  *                   where y can be 0 to 15 to select the cache.
  * @param  TxMessage: pointer to a structure which contains CAN Id, CAN DLC and CAN data.
  * @retval 0 is failed and true value is success for send data length.
  */
uint8_t CAN_Transmit(CANMSG_Type *CANxMSGy, CanTxRxMsg* TxMessage)
{
	uint32_t timeout = CACHE_TIMEOUT;
	
    /* Check the parameters */
    assert_param(IS_CAN_ALL_MSGCACHE(CANxMSGy));
    assert_param(IS_CAN_IDTYPE(TxMessage->IDE));
    assert_param(IS_CAN_RTR(TxMessage->RTR));
    assert_param(IS_CAN_DLC(TxMessage->DLC));
	
    /* Check message cache RDY bit and clear it */
    if ((CANxMSGy->CMCTRL & CAN_MCTRL_RDY_MASK) != 0x00)
    {
        CANxMSGy->CMCTRL = CAN_MCTRL_CLR_RDY;
		
        /* Wait the operate complete */
        while (((CANxMSGy->CMCTRL & CAN_MCTRL_RDY_MASK) != 0x00) && (timeout != 0))
        {
            timeout--;
        }
    }
	
	/* check RDY clear is or not success */
    if (CANxMSGy->CMCTRL & CAN_MCTRL_RDY_MASK)
    {
        return 0;
    }

    /* Set message cache used flag */
	CANxMSGy->CMCONF |= CAN_MCONF_MA0;
	
    /* clear mask select bits */
	CANxMSGy->CMCONF &= ~CAN_MCONF_MT;

    /* Set the frame data length */
    CANxMSGy->CMDLC = TxMessage->DLC;

    /* Set the frame data value from TxMessage->Data */
    if (TxMessage->RTR == CAN_RTR_Data)
    {
        CANxMSGy->CMDB0 = TxMessage->Data[0];
        CANxMSGy->CMDB1 = TxMessage->Data[1];
        CANxMSGy->CMDB2 = TxMessage->Data[2];
        CANxMSGy->CMDB3 = TxMessage->Data[3];
        CANxMSGy->CMDB4 = TxMessage->Data[4];
        CANxMSGy->CMDB5 = TxMessage->Data[5];
        CANxMSGy->CMDB6 = TxMessage->Data[6];
        CANxMSGy->CMDB7 = TxMessage->Data[7];
    }

    /* Message cache ready bit set */
    CANxMSGy->CMCTRL = CAN_MCTRL_SET_RDY;

    /* Message cache TRQ bit set */
    CANxMSGy->CMCTRL = CAN_MCTRL_SET_TRQ;
	
	return CANxMSGy->CMDLC;
}

/**
  * @brief  A CAN frame message receive cache to memory to store.
  * @param  CANx: where x can be 0 to select the CAN peripheral.
  * @param  CANxMSGy: where x can be 0 to select the CAN peripheral.
  *                   where y can be 0 to 15 to select the cache.
  * @param  RxMessage: pointer to a structure which contains CAN Id, CAN DLC and CAN data.
  * @param  Timeout: wait timeout value for wait time.
  * @retval 0 is failed and true value is success for receive data length.
  * @note   This function is used by polling type.
  */
uint8_t CAN_Receive(CAN_Type* CANx, CANMSG_Type *CANxMSGy, CanTxRxMsg* RxMessage, uint32_t Timeout)
{
	uint32_t timeout_temp = Timeout;
	uint16_t reg_crgpt = 0;
    uint8_t cache_num = 0;
    uint8_t recv_flag = 0;
	
    /* Check the parameters */
	assert_param(IS_CAN_ALL_PERIPH(CANx));
    assert_param(IS_CAN_ALL_MSGCACHE(CANxMSGy));
	assert_param(RxMessage == NULL);

	/* wait interrupt flag set */
    while (timeout_temp--)
    {
        /* To wait CINTS receive flag set */
        if ((CAN_FLAG_REC & CANx->CINTS) != (uint16_t)RESET)
        {
			/* Read CRGPT register value to memory */
			reg_crgpt = CANx->CRGPT;
			
			/* clear interrupt status flag */
            CANx->CINTS = CAN_FLAG_REC;
            recv_flag = 1;
        }
    }

    /* When REC flag is not set, return 0 to completed this operate */
    if (!recv_flag)
    {
        return 0;
    }

	/* check ROVF register set or not and to clear it */
    if (reg_crgpt & CAN_CRGPT_ROVF_MASK)
    {
        CANx->CRGPT = CAN_CRGPT_CLR_ROVF;
    }
	
	/* When RHPM register is set, receive operate will finish. */
    if (reg_crgpt & CAN_CRGPT_RHPM_MASK)
    {
        return 0;
    }

	/* clear DN register to enable next frame data cache */
    CANxMSGy->CMCTRL = CAN_MCTRL_CLR_DN;

	/* Get cache number and receive data length and valid data */
    cache_num = (((reg_crgpt) & CAN_CRGPT_RGPT_MASK) >> 8) & 0x0F;
	
	/* judge frame type is standard or extended */
	if (CANxMSGy->CMIDH & 0x8000)
	{
		/* Extended frame to fetch ID0~ID28 */
		RxMessage->IDE = CAN_Id_Extended;
		RxMessage->Id  = ((CANxMSGy->CMIDH & 0x1FFF) << 16) | (CANxMSGy->CMIDL);
	}
	else
	{
		/* Standard frame to fetch ID18~ID28 */
		RxMessage->IDE = CAN_Id_Standard;
		RxMessage->Id  = (CANxMSGy->CMIDH & 0x1FFC) >> 2;
	}
	
	/* Get receive frame data length */
    RxMessage->DLC = CANxMSGy->CMDLC;
	
	/* Get receive frame valid data to memory */
    for(int i = 0; i < RxMessage->DLC; i++)
    {
        RxMessage->Data[i] = *(((uint8_t *)&(CANxMSGy->CMDB0)) + i);
    }

	/* when DN or MUC register bit is set, the frame cache data is invalid */
    if ((CANxMSGy->CMCTRL & CAN_MCTRL_DN_MASK) || (CANxMSGy->CMCTRL & CAN_MCTRL_MUC_MASK))
    {
        return 0;
    }

    return RxMessage->DLC;
}

/**
  * @brief  A CAN frame message receive cache to memory to store by interrupt.
  * @param  CANx: where x can be 0 to select the CAN peripheral.
  * @param  CANxMSGy: where x can be 0 to select the CAN peripheral.
  *                   where y can be 0 to 15 to select the cache.
  * @param  RxMessage: pointer to a structure which contains CAN Id, CAN DLC and CAN data.
  * @retval 0 is failed and true value is success for receive data length.
  * @note   This function is used by interrupt type.
  */
uint8_t CAN_Receive_IT(CAN_Type* CANx, CANMSG_Type *CANxMSGy, CanTxRxMsg* RxMessage)
{
	uint16_t reg_crgpt = 0;
    uint8_t cache_num = 0;

    /* Check the parameters */
	assert_param(IS_CAN_ALL_PERIPH(CANx));
    assert_param(IS_CAN_ALL_MSGCACHE(CANxMSGy));
	assert_param(RxMessage == NULL);

    /* Read CRGPT register value to memory */
    reg_crgpt = CANx->CRGPT;

	/* check ROVF register set or not and to clear it */
    if (reg_crgpt & CAN_CRGPT_ROVF_MASK)
    {
        CANx->CRGPT = CAN_CRGPT_CLR_ROVF;
    }
	
	/* When RHPM register is set, receive operate will finish. */
    if (reg_crgpt & CAN_CRGPT_RHPM_MASK)
    {
        return 0;
    }

	/* clear DN register to enable next frame data cache */
    CANxMSGy->CMCTRL = CAN_MCTRL_CLR_DN;

	/* Get cache number and receive data length and valid data */
    cache_num = (((reg_crgpt) & CAN_CRGPT_RGPT_MASK) >> 8) & 0x0F;
	
	/* judge frame type is standard or extended */
	if (CANxMSGy->CMIDH & 0x8000)
	{
		/* Extended frame to fetch ID0~ID28 */
		RxMessage->IDE = CAN_Id_Extended;
		RxMessage->Id  = ((CANxMSGy->CMIDH & 0x1FFF) << 16) | (CANxMSGy->CMIDL);
	}
	else
	{
		/* Standard frame to fetch ID18~ID28 */
		RxMessage->IDE = CAN_Id_Standard;
		RxMessage->Id  = (CANxMSGy->CMIDH & 0x1FFC) >> 2;
	}
	
	/* Get receive frame data length */
    RxMessage->DLC = CANxMSGy->CMDLC;
	
	/* Get receive frame valid data to memory */
    for(int i = 0; i < RxMessage->DLC; i++)
    {
        RxMessage->Data[i] = *(((uint8_t *)&(CANxMSGy->CMDB0)) + i);
    }

	/* when DN or MUC register bit is set, the frame cache data is invalid */
    if ((CANxMSGy->CMCTRL & CAN_MCTRL_DN_MASK) || (CANxMSGy->CMCTRL & CAN_MCTRL_MUC_MASK))
    {
        return 0;
    }

    return RxMessage->DLC;
}

