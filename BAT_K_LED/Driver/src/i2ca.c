/**
  ******************************************************************************
  * @file    i2ca.c
  * @author  CMS Application Team
  * @version Vx.x.x
  * @date    24-April-2022
  * @brief   This file provides firmware functions to manage the following 
  *          functionalities of the Universal Inter Integrated Circuit (I2CA):           
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

/* Includes ------------------------------------------------------------------*/
#include "i2ca.h"
#include "cgc.h"

/** @addtogroup bat32g135_StdPeriph_Driver
  * @{
  */

/** @defgroup I2CA 
  * @brief I2CA driver modules
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define I2CA_MAX_CLOCK  (20000000U)
#define I2CA_DIR_TRANSMIT   0x01
#define I2CA_DIR_RECEIVE    0x02

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
IRQn_Type I2CA_IRQTable[I2CA_BUS_MAX_NUM] = 
{IICA0_IRQn, IICA1_IRQn};

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
  * @brief  I2CA module Deinit.
  * @param  None
  * @retval None
  */
void I2CA_DeInit(I2CASelect_TypeDef I2CAx)
{
	if(I2CAx == I2CA0)
	{
		CGC_PER0PeriphClockCmd(CGC_PER0Periph_I2CA0, DISABLE);
	}
#if defined(BAT32G1XX_80PIN) || defined(BAT32G1XX_100PIN)	
	else if(I2CAx == I2CA1)
	{
		CGC_PER2PeriphClockCmd(CGC_PER2Periph_I2CA1, DISABLE);
	}
#endif
}

/**
  * @brief  The specified UART1 peripheral initialization and set baud rate by UART_BAUD_t.
  * @param  UART_Instance: UART peripheral register base address.
  * @param  UART_InitStruct: pointer to a UART_InitTypeDef structure that contains
  *         the configuration information for the specified UART peripheral.
  * @param  pValue: UART baud rate setting option data structure.
  * @retval None
  */
static void IICA_Init(IICA_TypeDef *IICA_Instance, I2CA_InitTypeDef* I2CA_InitStruct)
{
    uint8_t tmpreg = 0;
    uint8_t tmp_wl, tmp_wh;
    uint32_t fmck;
	
	tmpreg = IICA_Instance->IICCTL1;
    /* I2CA register setting for control register n1 */
    tmpreg = IICA_Instance->IICCTL1;
    if (I2CA_InitStruct->I2C_ClockSpeed <= 100000)
    {
        /* operates in standard mode */
        tmpreg &= ~(1 << I2CA_OPERATE_BIT);
        tmpreg &= ~(1 << I2CA_DIGITAL_FILTER_BIT);
    }
    else
    {
        /* operates in fast mode and open filter */
        tmpreg |=  (1 << I2CA_OPERATE_BIT);
        tmpreg |=  (1 << I2CA_DIGITAL_FILTER_BIT);
    }

    /* clock for I2CA division select */
    if (SystemCoreClock >= 1000000 && SystemCoreClock <= I2CA_MAX_CLOCK)
    {
        tmpreg &= ~(1 << I2CA_FCLK_DIV_BIT);
        fmck = SystemCoreClock;
    }
    else if (SystemCoreClock > I2CA_MAX_CLOCK)
    {
        tmpreg |=  (1 << I2CA_FCLK_DIV_BIT);
        fmck = ((SystemCoreClock / 2) > I2CA_MAX_CLOCK) ? I2CA_MAX_CLOCK : (SystemCoreClock / 2);
    }

    IICA_Instance->IICCTL1 = tmpreg;

    /* Own address setting for I2C bus */
    IICA_Instance->SVA = I2CA_InitStruct->I2C_OwnAddress;

    /* I2CA register setting for flag register n */
    tmpreg = IICA_Instance->IICF0;

    /* enable generation of a start condition without detecting a stop condition */
    tmpreg |=  (1 << I2CA_STOP_DETECTION_BIT);

    /* enable  Communication reservation */
    tmpreg |=  (1 << I2CA_RESERVATION_BIT);

    IICA_Instance->IICF0 = tmpreg;

    /* I2CA bus clock option setting */
    if (I2CA_InitStruct->I2C_ClockSpeed <= 100000)
    {
        if (I2CA_InitStruct->I2C_Mode == I2CA_Mode_SMBusSlave)
        {
            tmp_wl = 4.7 * fmck / 1000000;
            tmp_wh = 5.3 * fmck / 1000000;
        }
        else if (I2CA_InitStruct->I2C_Mode == I2CA_Mode_SMBusMaster)
        {
            tmp_wl = (fmck) * 0.47 / (I2CA_InitStruct->I2C_ClockSpeed);
            tmp_wh = (fmck) * 0.53 / (I2CA_InitStruct->I2C_ClockSpeed);
        }
    }
    else if (I2CA_InitStruct->I2C_ClockSpeed <= 400000)
    {
        if (I2CA_InitStruct->I2C_Mode == I2CA_Mode_SMBusSlave)
        {
            tmp_wl = 1.3 * fmck / 1000000;
            tmp_wh = 1.2 * fmck / 1000000;
        }
        else if (I2CA_InitStruct->I2C_Mode == I2CA_Mode_SMBusMaster)
        {
            tmp_wl = (fmck) * 0.52 / (I2CA_InitStruct->I2C_ClockSpeed);
            tmp_wh = (fmck) * 0.48 / (I2CA_InitStruct->I2C_ClockSpeed);
        }
    }
    else
    {
        if (I2CA_InitStruct->I2C_Mode == I2CA_Mode_SMBusSlave)
        {
            tmp_wl = 0.5 * fmck / 1000000;
            tmp_wh = 0.5 * fmck / 1000000;
        }
        else if (I2CA_InitStruct->I2C_Mode == I2CA_Mode_SMBusMaster)
        {
            tmp_wl = (fmck) * 0.5 / (I2CA_InitStruct->I2C_ClockSpeed);
            tmp_wh = (fmck) * 0.5 / (I2CA_InitStruct->I2C_ClockSpeed);
        }
    }

    /* IICWL0 and IICWH0 register setting to config clock square option */
    IICA_Instance->IICWL = tmp_wl;
    IICA_Instance->IICWH = tmp_wh;

    /* I2CA bus option setting for control register n0 */
    /* interrupt request is generated at the ninth clock's falling edge */
    IICA_Instance->IICCTL0 |=  (1 << I2CA_WTIM_BIT);

    /* acknowledgment config */
    if (I2CA_InitStruct->I2C_Ack)
        IICA_Instance->IICCTL0 |=  (1 << I2CA_ACK_BIT);
    else
        IICA_Instance->IICCTL0 &= ~(1 << I2CA_ACK_BIT);

    /* enable operation */
    IICA_Instance->IICCTL0 |=  (1 << I2CA_EN_BIT);

    /* this exits from the current communications and sets standby mode */
    IICA_Instance->IICCTL0 |=  (1 << I2CA_LREL_BIT);
	
}
/**
  * @brief  Initializes the I2CA peripheral according to the specified
  *         parameters in the I2CA_InitStruct .
  * @param  I2CA_InitStruct: pointer to a I2CA_InitTypeDef structure that contains
  *         the configuration information for the specified I2CA peripheral.
  * @retval None
  */
void I2CA_Init(I2CASelect_TypeDef I2CAx, I2CA_InitTypeDef* I2CA_InitStruct)
{
    /* Check the parameters */
    assert_param(IS_I2CA_CLOCK_SPEED(I2CA_InitStruct->I2C_ClockSpeed));
    assert_param(IS_I2CA_MODE(I2CA_InitStruct->I2C_Mode));
    assert_param(IS_I2CA_DUTY_CYCLE(I2CA_InitStruct->I2C_DutyCycle));
    assert_param(IS_I2CA_OWN_ADDRESS(I2CA_InitStruct->I2C_OwnAddress));
    assert_param(IS_I2CA_ACK_STATE(I2CA_InitStruct->I2C_Ack));

	switch(I2CAx)
	{
		case I2CA0:
		{
			CGC_PER0PeriphClockCmd(CGC_PER0Periph_I2CA0, ENABLE);			
			IICA_TypeDef *I2CA_Instance = IICA0;	
			IICA_Init(I2CA_Instance,I2CA_InitStruct);			
		}
		break;
#if defined(BAT32G1XX_80PIN) || defined(BAT32G1XX_100PIN)	
		case I2CA1:
		{
			CGC_PER2PeriphClockCmd(CGC_PER2Periph_I2CA1, ENABLE);			
			IICA_TypeDef *I2CA_Instance = IICA1;
			IICA_Init(I2CA_Instance,I2CA_InitStruct);	
		}
		break;
#endif
		default:
			break;
	}
}
/**
  * @brief  Enables or disables the specified I2CA peripheral.
  * @param  NewState: new state of the I2CA peripheral.
  *          This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
void I2CA_Cmd(I2CASelect_TypeDef I2CAx,FunctionalState NewState)
{
	/* Check the parameters */
	assert_param(IS_FUNCTIONAL_STATE(NewState));
	

	if (NewState != DISABLE)
	{
		/* Enable the I2CA periphal by setting the SPIE register */
		I2CA_SET_REG(I2CAx,IICCTL00,IICA0_IICCTL00_SPIE_Pos);
	}
	else
	{
		/* Disable the I2CA periphal by setting the SPIE register */
		I2CA_CLEAR_REG(I2CAx,IICCTL00,IICA0_IICCTL00_SPIE_Pos);
	}

}

/**
  * @brief  Enables or disables the specified I2CA acknowledgement.
  * @param  NewState: new state of the I2CA acknowledgement.
  *          This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
void I2CA_Acknowledge_Cmd(I2CASelect_TypeDef I2CAx,FunctionalState NewState)
{
	/* Check the parameters */
	assert_param(IS_FUNCTIONAL_STATE(NewState));

	if (NewState != DISABLE)
	{
		/* Enable the I2CA acknowledgement by setting the ACKE register */
		I2CA_SET_REG(I2CAx,IICCTL00,IICA0_IICCTL00_ACKE_Pos);
	}
	else
	{
		/* Disable the I2CA acknowledgement by setting the ACKE register */
		I2CA_CLEAR_REG(I2CAx,IICCTL00,IICA0_IICCTL00_ACKE_Pos);
	}

}

/**
  * @brief  Configurate the Interrupt positon depend on the register WTIM bit in IICCTL00.
  * @param  Intpos: Interrupt position config.
  *          This parameter can be one of the following values:
  *            @arg I2CA_INTPOS_8CLK	: In the 8th clock to generate intterupt signal.
  *            @arg I2CA_INTPOS_9CLK	: In the 9th clock to generate intterupt signal.
  * @retval None
  */
void I2CA_InterruptPosition_Config(I2CASelect_TypeDef I2CAx,uint8_t Intpos)
{
    /* Check the parameters */
    assert_param(IS_I2CA_INTPOS(Intpos));

    /* 
    Generate the interrupt signal int the 8th clock falling edge.
    Master role : when the Nth clock is completed, set the clock is low to wait.
    Slave role  : when the Nth clock is completed, set the clock is low to wait master.
    Note : N is depend on param intpos.
    */	
	if (Intpos == I2CA_INTPOS_8CLK)
	{
		I2CA_CLEAR_REG(I2CAx,IICCTL00,IICA0_IICCTL00_WTIM_Pos);			
	}
	else if (Intpos == I2CA_INTPOS_9CLK)
	{
		I2CA_SET_REG(I2CAx,IICCTL00,IICA0_IICCTL00_WTIM_Pos);						
	}
}

/**
  * @brief  To relieve the wait for I2CA bus state
  * @param  NewState: new state of the I2CA waiting relieve.
  *          This parameter can be: ENABLE or DISABLE.
  * @retval None
  * @Note   the WREL bit will auto reset when the relieve is completed.
  */
void I2CA_WaitRelieve_Cmd(I2CASelect_TypeDef I2CAx,FunctionalState NewState)
{
	/* Check the parameters */
	assert_param(IS_FUNCTIONAL_STATE(NewState));


	if (NewState != DISABLE)
	{
		/* Relieve the wait for bus state */
		I2CA_SET_REG(I2CAx,IICCTL00,IICA0_IICCTL00_WREL_Pos);						
	}
	else
	{
		I2CA_CLEAR_REG(I2CAx,IICCTL00,IICA0_IICCTL00_WREL_Pos);			
	}

}

/**
  * @brief  Configurate the I2CA own address to SVA0 register.
  * @param  Address: Set the device address to register.
  * @retval None
  */
void I2CA_OwnAddress_Config(I2CASelect_TypeDef I2CAx,uint8_t Address)
{
	/* Check the parameters */
	assert_param(IS_I2CA_OWN_ADDRESS(Address));
	
	IICA_TypeDef *I2CA_Instance;
	
	if(I2CAx == I2CA0)	
	{
		I2CA_Instance = (IICA_TypeDef *)IICA0;
	}	
#if defined(BAT32G1XX_80PIN) || defined(BAT32G1XX_100PIN)
	else if(I2CAx == I2CA1)
	{
		I2CA_Instance = (IICA_TypeDef *)IICA1;
	}	
#endif
	I2CA_Instance->SVA = Address;	
}

/**
  * @}
  */

/** @defgroup I2CA Data transfers functions
 *  @brief   Data transfers functions
 *
@verbatim   
 ===============================================================================
                      ##### Data transfers functions #####
 ===============================================================================  

 [..] This section provides a set of functions allowing to manage the I2CA data 
      transfers. In reception, data are received and then stored into an internal 
      buffer while. In transmission, data are first stored into an internal send
      buffer before being transmitted.

 [..] The read access of the IICA0 register can be done using the I2CA_ReadByte()
      function and returns the Rx register value. Whereas a write access to the IICA0 
      can be done using I2CA_WriteByte() function and stores the written data into 
      Tx register.

@endverbatim
  * @{
  */

/**
  * @brief  Read the register data for IICA0.
  * @param  None.
  * @retval The data value for IICA0.
  */
uint8_t I2CA_ReadByte(I2CASelect_TypeDef I2CAx)
{
	IICA_TypeDef *I2CA_Instance;
	
	if(I2CAx == I2CA0)	
	{
		I2CA_Instance = (IICA_TypeDef *)IICA0;
	}
#if defined(BAT32G1XX_80PIN) || defined(BAT32G1XX_100PIN)
	else if(I2CAx == I2CA1)
	{
		I2CA_Instance = (IICA_TypeDef *)IICA1;
	}
#endif
	return (I2CA_Instance->IICA);
}

/**
  * @brief  Write the data to IICA0 register.
  * @param  The data which to be write.
  * @retval None.
  */
void I2CA_WriteByte(I2CASelect_TypeDef I2CAx,uint8_t Data)
{
	IICA_TypeDef *I2CA_Instance;
	if(I2CAx == I2CA0)	
	{
		I2CA_Instance = (IICA_TypeDef *)IICA0;
	}
#if defined(BAT32G1XX_80PIN) || defined(BAT32G1XX_100PIN)
	else if(I2CAx == I2CA1)
	{
		I2CA_Instance = (IICA_TypeDef *)IICA1;
	}
#endif
	I2CA_Instance->IICA = Data;		

}

/**
  * @brief  Generates I2CA communication START condition.
  * @param  None
  * @retval None.
  */
void I2CA_GenerateSTART(I2CASelect_TypeDef I2CAx)
{
    /* Generate a START condition */
	I2CA_SET_REG(I2CAx,IICCTL00,I2CA_START_BIT);						
}

/**
  * @brief  Generates I2CA communication STOP condition.
  * @param  None
  * @retval None.
  */
void I2CA_GenerateSTOP(I2CASelect_TypeDef I2CAx)
{
    /* Generate a STOP condition */
	I2CA_SET_REG(I2CAx,IICCTL00,I2CA_STOP_BIT);						
}

/**
  * @brief  Checks whether the specified I2CA flag is set or not.
  * @param  I2CA_FLAG: specifies the flag to check.
  *          This parameter can be one of the following values:
  *            @arg I2CA_STATUS_SPD	: the STOP condition check status.
  *            @arg I2CA_STATUS_STD	: the START condition check status.
  *            @arg I2CA_STATUS_ACK	: the ACK condition check status.
  *            @arg I2CA_STATUS_TRC	: bus status in transmit(is 1) or receive(is 0) mode.
  *            @arg I2CA_STATUS_COI	: check the address is matched.
  *            @arg I2CA_STATUS_EXC	: the extend code received or not status.
  *            @arg I2CA_STATUS_ALD	: the arbitrate success(is 0) or fail(is 1).
  *            @arg I2CA_STATUS_MSTS: to display the communication role is master(is 1) or slave(is 0).
  *            @arg I2CA_STATUS_BUSBSY: I2C bus is busy or idle.
  *            @arg I2CA_STATUS_STCF: Generate START condition or not.
  * @retval The new state of I2CA_FLAG (SET or RESET).
  */
FlagStatus I2CA_GetFlagStaus(I2CASelect_TypeDef I2CAx,uint16_t I2CA_FLAG)
{
    FlagStatus bitstatus;
	uint8_t tmp_reg = 0;
    uint8_t tmp_flag = 0;

    /* Check the parameters */
    assert_param(IS_I2CA_STATUS(I2CA_FLAG));

	if(I2CAx == I2CA0)	
	{
		if (I2CA_FLAG == I2CA_STATUS_BUSBSY || I2CA_FLAG == I2CA_STATUS_STCF)
		{
			tmp_reg = IICA0->IICF0;
			tmp_flag = (uint8_t)(I2CA_FLAG >> 8);
		}
		else
		{
			tmp_reg = IICA0->IICS0;
			tmp_flag = (uint8_t)(I2CA_FLAG & 0xFF);
		}		
	}
#if defined(BAT32G1XX_80PIN) || defined(BAT32G1XX_100PIN)
	else if(I2CAx == I2CA1)
	{
		if (I2CA_FLAG == I2CA_STATUS_BUSBSY || I2CA_FLAG == I2CA_STATUS_STCF)
		{
			tmp_reg = IICA1->IICF0;
			tmp_flag = (uint8_t)(I2CA_FLAG >> 8);
		}
		else
		{
			tmp_reg = IICA1->IICS0;
			tmp_flag = (uint8_t)(I2CA_FLAG & 0xFF);
		}		
	}
#endif
    if ((tmp_reg & tmp_flag) != (uint16_t)RESET)
        bitstatus = SET;
    else
        bitstatus = RESET;

    return bitstatus;
}

/**
  * @brief  Send a data buffer through the I2CA peripheral to write.
  * @param  Address: Device address in the I2C bus.
  * @param  Reg: Register address in the Device.
  * @param  Data: Data buffer address.
  * @param  Len: Data buffer length need to transmit.
  * @retval None
  */
I2CA_Status I2CA_Master_WriteData(I2CASelect_TypeDef I2CAx,uint8_t Address, uint8_t Reg, uint8_t *Data, uint16_t Len)
{
	I2CA_Status status = OK;
    IRQn_Type irq;

    /* Get the IQR number depend on the I2Cx in SCI unit */
    irq = I2CA_IRQTable[I2CAx];
	
	if (I2CA_GetFlagStaus(I2CAx,I2CA_STATUS_BUSBSY) != RESET)
	{
		status = BUSY;
	}
	else
	{
		I2CA_GenerateSTART(I2CAx);

		I2CA_WriteByte(I2CAx,Address & 0xFE);
		
		while(!INTC_GetPendingIRQ(irq));
		INTC_ClearPendingIRQ(irq);
		
		I2CA_WriteByte(I2CAx,Reg);
		while(!INTC_GetPendingIRQ(irq));
		INTC_ClearPendingIRQ(irq);
		
		do
		{
			I2CA_WriteByte(I2CAx,*Data++);
			while(!INTC_GetPendingIRQ(irq));
			INTC_ClearPendingIRQ(irq);
			
		} while(--Len);
		
        /* stop condition is generated */
        I2CA_GenerateSTOP(I2CAx);
	}
	
	return status;
}

/**
  * @brief  Read a data buffer through the I2CA peripheral to device register address.
  * @param  Address: Device address in the I2C bus.
  * @param  Reg: Register address in the Device.
  * @param  Data: Data buffer address.
  * @param  Len: Data buffer length need to transmit read.
  * @retval None
  */
I2CA_Status I2CA_Master_ReadData(I2CASelect_TypeDef I2CAx,uint8_t Address, uint8_t Reg, uint8_t *Data, uint16_t Len)
{
	I2CA_Status status = OK;
    IRQn_Type irq;

    irq = I2CA_IRQTable[I2CAx];
	
	if (I2CA_GetFlagStaus(I2CAx,I2CA_STATUS_BUSBSY) != RESET)
	{
		status = BUSY;
	}
	else
	{
        /* Generate the START condition */
		I2CA_GenerateSTART(I2CAx);
		
		I2CA_WriteByte(I2CAx,Address & 0xFE);		
		
        /* Wait device address send succes */
        while(!INTC_GetPendingIRQ(irq));
        INTC_ClearPendingIRQ(irq);
		
		I2CA_WriteByte(I2CAx,Reg);
        /* Wait device register send succes */
        while(!INTC_GetPendingIRQ(irq));
        INTC_ClearPendingIRQ(irq);

        /* Generate the RESTART condition */
        I2CA_GenerateSTART(I2CAx);
		
		I2CA_WriteByte(I2CAx,Address | 0x01);		
		
        /* Wait address send succes */
        while(!INTC_GetPendingIRQ(irq));
        INTC_ClearPendingIRQ(irq);
		
		do {
            /* When the last byte to receive, we should not output ACK */
            if(Len == 1U)
            {
                /* acknowledgment disable in the last byte to receive */
				I2CA_CLEAR_REG(I2CAx,IICCTL00,I2CA_ACK_BIT);
            }

            /* Release the bus status for receive data */
			I2CA_SET_REG(I2CAx,IICCTL00,I2CA_WREL_BIT);
			
            /* Wait data receive succes */
			while(!INTC_GetPendingIRQ(irq));
			INTC_ClearPendingIRQ(irq);
			
            /* copy the receive data to target memory */
			*Data++ = I2CA_ReadByte(I2CAx);
		} while(--Len);
		
        /* STOP condition is generated */
        I2CA_GenerateSTOP(I2CAx);
	}
	
	return status;
}

/**
  * @brief  Receive a data buffer through the I2CA peripheral to device register address by slave mode.
  * @param  Data: Data buffer address.
  * @param  Size: Data size when the slave hope to recceive.
  * @param  Len: Data length in buffer received.
  * @retval status: This parameter can be a value of @ref I2CA_Status.
  */
I2CA_Status I2CA_Slave_ReceiveData(I2CASelect_TypeDef I2CAx,uint8_t *Data, uint16_t Size, uint16_t *Len)
{
	I2CA_Status status = OK;
    uint8_t Address_Match = 0;
    uint16_t Recv_Length = 0;
    IRQn_Type irq;

    irq = I2CA_IRQTable[I2CAx];

    while(1)
    {
        while(!INTC_GetPendingIRQ(irq));
        INTC_ClearPendingIRQ(irq);

        /* Check stop condition */
        if (I2CA_GetFlagStaus(I2CAx,I2CA_STATUS_SPD)) 
        {
            /* Disable I2CA stop condition interrupt*/
			I2CA_CLEAR_REG(I2CAx,IICCTL00,I2CA_SPIE_BIT);
            break;
        }
        else
        {
			/* When the address match is success */
            if (!Address_Match && I2CA_GetFlagStaus(I2CAx,I2CA_STATUS_COI))
            {
                Address_Match = 1;
				
				/* SPIE0 = 1: enable I2CA stop condition interrupt */
				I2CA_SET_REG(I2CAx,IICCTL00,I2CA_SPIE_BIT);
				
				/* interrupt request is generated at the eighth clock's falling edge */
				I2CA_CLEAR_REG(I2CAx,IICCTL00,I2CA_WTIM_BIT);
				/* cancel wait */
				I2CA_SET_REG(I2CAx,IICCTL00,I2CA_WREL_BIT);
            }
            else
            {
				if (Recv_Length < Size)
				{
					*Data = I2CA_ReadByte(I2CAx);
					Data++;
					Recv_Length++;

					/* WREL0 = 1U: cancel wait */
					I2CA_SET_REG(I2CAx,IICCTL00,I2CA_WREL_BIT);

					if(Size == Recv_Length)
					{
						/* WTIM0 = 1:  interrupt request is generated at the ninth clock's falling edge */
						I2CA_SET_REG(I2CAx,IICCTL00,I2CA_WTIM_BIT);

                        break;
					}
				}
				else
				{
					/* When the receive data length exceed the hope length,we shoule not write data to the receive buffer */
					/* WTIM0 = 1:  interrupt request is generated at the ninth clock's falling edge */
					I2CA_SET_REG(I2CAx,IICCTL00,I2CA_WTIM_BIT);
					/* WREL0 = 1U: cancel wait */
					I2CA_SET_REG(I2CAx,IICCTL00,I2CA_WREL_BIT);
				}
            }
        }
    }
	
	/* Set the receive data length */
	*Len = Recv_Length;
	
	return status;
}

/**
  * @brief  Transmit a data buffer through the I2CA peripheral to device register address by slave mode.
  * @param  Data: Data buffer address.
  * @param  Size: Data size when the slave hope to recceive.
  * @retval status: This parameter can be a value of @ref I2CA_Status.
  */
I2CA_Status I2CA_Slave_TransmitData(I2CASelect_TypeDef I2CAx,uint8_t *Reg, uint8_t *Data, uint16_t Size)
{
	I2CA_Status status = OK;
	uint8_t Stop_count = 0;
    uint8_t Address_Match = 0;
    uint16_t Send_Length = 0;
    IRQn_Type irq;
	IICA_TypeDef *I2CA_Instance;
	
    irq = I2CA_IRQTable[I2CAx];	
	if(I2CAx == I2CA0)	
	{
		I2CA_Instance = (IICA_TypeDef *)IICA0;
	}
#if defined(BAT32G1XX_80PIN) || defined(BAT32G1XX_100PIN)
	else if(I2CAx == I2CA1)
	{
		I2CA_Instance = (IICA_TypeDef *)IICA1;
	}
#endif
    while(1)
    {
        while(!INTC_GetPendingIRQ(irq));
        INTC_ClearPendingIRQ(irq);

        /* Check stop condition */
        if (I2CA_GetFlagStaus(I2CAx,I2CA_STATUS_SPD))
        {
			Stop_count++;
			
			/* To avoid the repeated stop condition */
			if (Stop_count == 2)
			{
				/* Disable I2CA */
				I2CA_CLEAR_REG(I2CAx,IICCTL00,I2CA_SPIE_BIT);				
				break;
			}
        }
        else
        {
			/* When the address match is success */
            if ((Address_Match < 2) && I2CA_GetFlagStaus(I2CAx,I2CA_STATUS_COI))  //接收地址和自身从机地址一致
            {
                Address_Match ++;
				
				if (Address_Match == 1)
				{
					I2CA_Instance->SVA|= 0x01;    //第一次先读		
				} 
				else if (Address_Match == 2)
				{
					*Reg = I2CA_ReadByte(I2CAx);
					I2CA_Instance->SVA &= ~ 0x01;
				}
				
				if (!(I2CA_Instance->IICCTL0 & I2CA_SPIE_ENABLE))
				{
					/* SPIE0 = 1: enable I2CA */
					I2CA_SET_REG(I2CAx,IICCTL00,I2CA_SPIE_BIT);
				}
				
				/* Sending or receiving */
				if (I2CA_GetFlagStaus(I2CAx,I2CA_STATUS_TRC))    //send status
				{
					I2CA_SET_REG(I2CAx,IICCTL00,I2CA_WTIM_BIT);//在输入9个时钟，将时钟置为低电平
					I2CA_WriteByte(I2CAx,*Data);
					Data++;
					Send_Length++;
				}
				else
				{
					I2CA_SET_REG(I2CAx,IICCTL00,I2CA_ACK_BIT); /* ACKE0 = 1U: enable acknowledgment */
					I2CA_CLEAR_REG(I2CAx,IICCTL00,I2CA_WTIM_BIT);// interrupt request is generated at the eighth clock's falling edge
					I2CA_SET_REG(I2CAx,IICCTL00,I2CA_WREL_BIT); // cancel wait 
				}
            }
            else
            {
				/* Sending status */
				if (I2CA_GetFlagStaus(I2CAx,I2CA_STATUS_TRC))
				{
					if (Send_Length == Size)
					{
						I2CA_WriteByte(I2CAx,0xFF);
					}
					else
					{
						/* Write data to IICA0 register to send it */
						I2CA_WriteByte(I2CAx,*Data);	
						Data++;
						Send_Length++;
					}
				}
            }
        }
	}
	
	return status;
}

/**
  * @brief  Transmit or receive the data which can be transmitted. This function only used by ISR.
  * @param  TxBuffer:   The transmit buffer pointer for application memory.
  * @param  TxLength:   The value pointer for transmit data length.
  * @param  RxBuffer:   The receive buffer pointer for application memory.
  * @param  RxLength:   The value pointer for received data length.
  * @param  RxTotLength:The total length of the receive buffer. 
  * @retval Event: This parameter can be a value of @ref I2CA_Slave_Event.
  */
I2CA_Slave_Event I2CA_Slave_TransmitReceive_IT(I2CASelect_TypeDef I2CAx, uint8_t *TxBuffer, uint16_t *TxLength, 
                                                uint8_t *RxBuffer, uint16_t *RxLength, uint16_t RxTotLength)
{
    I2CA_Slave_Event Event = I2C_SLAVE_EVENT_NONE;
    static uint8_t is_AddrMatch;
	static uint8_t TransmitDir;
    static uint8_t *TxData, *RxData;
    static uint16_t Length;

    /* Control for stop condition */
    if (is_AddrMatch && I2CA_GetFlagStaus(I2CAx,I2CA_STATUS_SPD))
    {
        is_AddrMatch = 0;
        if (TransmitDir == I2CA_DIR_TRANSMIT)
        {
            TransmitDir = 0;
            Event = I2C_SLAVE_EVENT_TRANSMIT_COMPLETED;
        }
        else if (TransmitDir == I2CA_DIR_RECEIVE)
        {
            TransmitDir = 0;
            *RxLength = Length;
            Event = I2C_SLAVE_EVENT_RECEIVE_COMPLETED;
        }
        
        /* I2CA periphal disable and ACK disable for the next transmission */
        I2CA_Acknowledge_Cmd(I2CAx,DISABLE);
        I2CA_Cmd(I2CAx,DISABLE);
    }
    else
    {
        /* Address to match and get the operate for master */
		if (!is_AddrMatch && I2CA_GetFlagStaus(I2CAx,I2CA_STATUS_COI))
		{
			is_AddrMatch = 1;
			
			I2CA_Cmd(I2CAx,ENABLE);
			
			if (I2CA_ReadByte(I2CAx) & 0x01)
			{
                /* Assign the TxBuffer pointer and the data length which to be transmit. */
				TxData = TxBuffer;
                Length = *TxLength;
				
                /* Config the interrupt position and write the first byte to register */
				I2CA_InterruptPosition_Config(I2CAx,I2CA_INTPOS_9CLK);
				I2CA_WriteByte(I2CAx,*TxData);
				TxData++;
				Length--;

				/* Slave transmit operate */
				TransmitDir = I2CA_DIR_TRANSMIT;
                Event = I2C_SLAVE_EVENT_READ_ADDRESS_MATCHED;
			}
			else
			{
                /* Assign the RxBuffer pointer. */
				RxData = RxBuffer;
				Length = 0;
				
                /* Config the interrupt position and enable ACK and I2CA periphal */
				I2CA_InterruptPosition_Config(I2CAx,I2CA_INTPOS_8CLK);
                I2CA_Acknowledge_Cmd(I2CAx,ENABLE);
				I2CA_WaitRelieve_Cmd(I2CAx,ENABLE);

				/* Slave receive operate */
				TransmitDir = I2CA_DIR_RECEIVE;
                Event = I2C_SLAVE_EVENT_WRITE_ADDRESS_MATCHED;
			}
		}
		else if (is_AddrMatch)
		{
			if (TransmitDir == I2CA_DIR_TRANSMIT)
			{
                /* The transmission direction is transmit. */
				if (Length > 0U)
				{
                    /* Write send data one by one byte to the register. */
					I2CA_WriteByte(I2CAx,*TxData);
					TxData++;
					Length--;
				}
				else
				{
                    /* When the data transmit is completed, we shoule write 0xFF to pull-up the bus. */
                    /* To wait the stop condition */
					I2CA_WriteByte(I2CAx,0xFF);
				}
                Event = I2C_SLAVE_EVENT_TRANSMIT_RUNNING;
			}
			else if (TransmitDir == I2CA_DIR_RECEIVE)
			{
                /* The transmission direction is receive. */
				I2CA_WaitRelieve_Cmd(I2CAx,ENABLE);
				if (Length < RxTotLength)
				{
					*RxData = I2CA_ReadByte(I2CAx);
					RxData++;
					Length++;
				}
				else
				{
                    /* Receive data operate finished and config the interrupt position to wait the stop condition. */
					I2CA_InterruptPosition_Config(I2CAx,I2CA_INTPOS_9CLK);
				}
                Event = I2C_SLAVE_EVENT_RECEIVE_RUNNING;
			}
		}
		else
		{
			I2CA_WaitRelieve_Cmd(I2CAx,ENABLE);
            Event = I2C_SLAVE_EVENT_ERROR;
		}
    }

    return Event;
}
