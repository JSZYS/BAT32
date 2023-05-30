#include "key.h"

/**
  * @brief  This function initializes the key interrupt module
  * @param  Key_InitStruct: pointer to a KEY_InitTypeDef structure that contains
  *         the configuration information for the specified KEY peripheral.
  * @retval None
  */
void KEY_Init(KEY_InitTypeDef* Key_InitStruct)
{
	volatile uint8_t w_count;
    /* Check the parameters */
    assert_param(IS_KEY_INT(Key_InitStruct->INT_Select));
	
	KEY->KRM = Key_InitStruct->INT_Select;

    for (w_count = 0U; w_count <= KEY_WAITTIME; w_count++)
    {   
        __NOP();
    }	
}

/**
  * @brief  This function clears KEY interrupt flag and enables interrupt.
  * @retval None
  */
void Key_Start(void)
{
    INTC_ClearPendingIRQ(KEY_IRQn); /* clear INTKR interrupt flag */
    INTC_EnableIRQ(KEY_IRQn);/* enable INTKR interrupt */
    NVIC_EnableIRQ(KEY_IRQn);/* enable INTKR interrupt */
}

/**
  * @brief  This function disables KEY interrupt and clears interrupt flag.
  * @retval None
  */
void Key_Stop(void)
{
    INTC_DisableIRQ(KEY_IRQn);      /* disable INTKR interrupt */
    INTC_ClearPendingIRQ(KEY_IRQn); /* clear INTKR interrupt flag */
}

