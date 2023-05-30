#ifndef __DMA_H__
#define __DMA_H__
#include <stdint.h>
#include "common.h"

typedef enum {
	DMA_VECTOR_INTP0 	 = 1U,  /* 外部中断INTP0 */
	DMA_VECTOR_INTP1 	 = 2U,  /* 外部中断INTP1 */
	DMA_VECTOR_INTP2 	 = 3U,  /* 外部中断INTP2 */
	DMA_VECTOR_INTP3 	 = 4U,  /* 外部中断INTP3 */
	DMA_VECTOR_INTP4 	 = 5U,  /* 外部中断INTP4 */
	DMA_VECTOR_INTP5 	 = 6U,  /* 外部中断INTP5 */
	DMA_VECTOR_INTP6 	 = 7U,  /* 外部中断INTP6 */
	DMA_VECTOR_INTP7 	 = 8U,  /* 外部中断INTP7 */
	DMA_VECTOR_KEY       = 9U,  /* 按键中断INTP3 */
	DMA_VECTOR_ADC   	 = 10U,  /* A/D 转换结束 */
	DMA_VECTOR_SR0       = 11U,  /* UART0接收传送结束/SPI01传送结束或者缓冲器空/IIC01传送结束 */
	DMA_VECTOR_SPI01     = 11U,  /* UART0接收传送结束/SPI01传送结束或者缓冲器空/IIC01传送结束 */
	DMA_VECTOR_IIC01     = 11U,  /* UART0接收传送结束/SPI01传送结束或者缓冲器空/IIC01传送结束 */
	DMA_VECTOR_ST0       = 12U,  /* UART0发送传送结束/SPI00传送结束或者缓冲器空/IIC00传送结束 */
	DMA_VECTOR_SPI00     = 12U,  /* UART0发送传送结束/SPI00传送结束或者缓冲器空/IIC00传送结束 */
	DMA_VECTOR_IIC00     = 12U,  /* UART0发送传送结束/SPI00传送结束或者缓冲器空/IIC00传送结束 */
	DMA_VECTOR_SR1       = 13U,  /* UART1接收传送结束/SPI11传送结束或者缓冲器空/IIC11传送结束 */
	DMA_VECTOR_SPI11     = 13U,  /* UART1接收传送结束/SPI11传送结束或者缓冲器空/IIC11传送结束 */
	DMA_VECTOR_IIC11     = 13U,  /* UART1接收传送结束/SPI11传送结束或者缓冲器空/IIC11传送结束 */
	DMA_VECTOR_ST1       = 14U,  /* UART1发送传送结束/SPI10传送结束或者缓冲器空/IIC10传送结束 */
	DMA_VECTOR_SPI10     = 14U,  /* UART1发送传送结束/SPI10传送结束或者缓冲器空/IIC10传送结束 */
	DMA_VECTOR_IIC10     = 14U,  /* UART1发送传送结束/SPI10传送结束或者缓冲器空/IIC10传送结束 */
	DMA_VECTOR_SR2       = 15U,  /* UART2接收传送结束/SPI21传送结束或者缓冲器空/IIC21传送结束 */
	DMA_VECTOR_SPI21     = 15U,  /* UART2接收传送结束/SPI21传送结束或者缓冲器空/IIC21传送结束 */
	DMA_VECTOR_IIC21     = 15U,  /* UART2接收传送结束/SPI21传送结束或者缓冲器空/IIC21传送结束 */
	DMA_VECTOR_ST2       = 16U,  /* UART2发送传送结束/SPI20传送结束或者缓冲器空/IIC20传送结束 */
	DMA_VECTOR_SPI20     = 16U,  /* UART2发送传送结束/SPI20传送结束或者缓冲器空/IIC20传送结束 */
	DMA_VECTOR_IIC20     = 16U,  /* UART2发送传送结束/SPI20传送结束或者缓冲器空/IIC20传送结束 */
	DMA_VECTOR_IICA      = 17U,  /* IICA通信结束 */
	DMA_VECTOR_TM40_CH0  = 19U,  /* 定时器阵列单元0的通道0的计数结束或者捕获结束 */
	DMA_VECTOR_TM40_CH1  = 20U,  /* 定时器阵列单元0的通道1的计数结束或者捕获结束 */ 
	DMA_VECTOR_TM40_CH2  = 21U,  /* 定时器阵列单元0的通道2的计数结束或者捕获结束 */
	DMA_VECTOR_TM40_CH3  = 22U,  /* 定时器阵列单元0的通道3的计数结束或者捕获结束 */
	DMA_VECTOR_IT        = 23U,  /* 间隔定时器中断 */
	DMA_VECTOR_FLASH     = 24U,
	DMA_VECTOR_TMC_OVF   = 26U,
	DMA_VECTOR_TMM_IMFA0 = 27U,
	DMA_VECTOR_TMM_IMFB0 = 28U,
	DMA_VECTOR_TMM_IMFC0 = 29U,
	DMA_VECTOR_TMM_IMFD0 = 30U,
	DMA_VECTOR_TMM_IMFA1 = 31U, 
	DMA_VECTOR_TMM_IMFB1 = 32U, 
	DMA_VECTOR_TMM_IMFC1 = 33U, 
	DMA_VECTOR_TMM_IMFD1 = 34U, 
	DMA_VECTOR_TMB_IMFA  = 35U,
	DMA_VECTOR_TMB_IMFB  = 36U, 
	DMA_VECTOR_TMA_UNF   = 37U,
	DMA_VECTOR_CMP0      = 38U,  /* 比较器0 */
	DMA_VECTOR_CMP1      = 39U,  /* 比较器1 */  
  
}DMA_VECTOR_t;

#define DMA_SrcInc_Enable      			((uint8_t)0x01)
#define DMA_SrcInc_Disable     			((uint8_t)0x00)

#define DMA_DstInc_Enable      			((uint8_t)0x01)
#define DMA_DstInc_Disable     			((uint8_t)0x00)

#define DMA_DataSize_Byte           	((uint16_t)0x0000) 
#define DMA_DataSize_HalfWord       	((uint16_t)0x0040) 
#define DMA_DataSize_Word           	((uint16_t)0x0080)

typedef enum
{
	DMA_Mode_Normal,
	DMA_Mode_Repeat	
}DMA_Mode_t;


#define DMA_RepeatAreaAddr_Src          ((uint8_t)0x02) 
#define DMA_RepeatAreaAddr_Dst 			((uint8_t)0x00)

#define DMA_RepeatInt_Enable            ((uint8_t)0x01) 
#define DMA_RepeatInt_Disable           ((uint8_t)0x00)


#define DMA_ChainTrans_None             ((uint8_t)0x00) 
#define DMA_ChainTrans_Head             ((uint8_t)0x01) 
#define DMA_ChainTrans_Node             ((uint8_t)0x02) 
#define DMA_ChainTrans_Tail             ((uint8_t)0x03) 
typedef struct
{
	DMA_VECTOR_t DMA_Vector;            /*!< Specifies the Vector used for the dma start source . 
	                                    This parameter can be a value of @ref DMA_VECTOR_t */
											
	uint16_t DMA_CtrlId;			/*!< Specifies the control data area id for the dma */
										  											
	uint32_t DMA_SrcAddr; /*!< Specifies the peripheral source base address for DMA */

	uint32_t DMA_DstAddr;    /*!< Specifies the peripheral destion base address for DMA*/
	                                    

	uint16_t DMA_BufferSize;         /*!< Specifies the buffer size, in data unit, of the specified Stream. */ 
	                                    
//	uint16_t DMA_BlockSize;         /*!< Specifies the DMA block size when transmittion,it can be 0x0000~0xFFFF. */ 
	                                    
	uint16_t DMA_SrcInc;      /*!< Specifies whether the Peripheral source address register should be incremented or not.
	                                   This parameter can be a value of @ref DMA_peripheral_incremented_mode */

	uint16_t DMA_DstInc;          /*!< Specifies whether the destnation address register should be incremented or not.
	                                    This parameter can be a value of @ref DMA_memory_incremented_mode */

	uint16_t DMA_DataSize;    /*!< Specifies the Peripheral data width.
	                                    This parameter can be a value of @ref DMA_peripheral_data_size */

	uint8_t DMA_Mode;               /*!< Specifies the operation mode of the DMA.
	                                    This parameter can be a value of @ref DMA_circular_normal_mode
	                                    @note The circular buffer mode cannot be used if the memory-to-memory
	                                          data transfer is configured on the selected Stream */


	uint8_t DMA_RepeatAreaAddr;          /*!< Specifies if the source or destnation address will be used for repeat area in DMA_Mode_Repeat
	                                    This parameter can be a value of @ref DMA_RepeatAddr
	                                    @note The DMA_RepeatAddr cannot be configured in Normal mode */
	                                            
	uint8_t DMA_RepeatInt;		  /*!< Specifies if interrupt is occured when transimission is finished in repeat area
										  This parameter can be a value of @ref DMA_RepeatInt
										  @note The DMA_RepeatInt is not effective in Normal mode */

	uint8_t DMA_ChainTrans;		  /*!< multi-control data transimission is triggered by one vector 
										  This parameter can be a value of @ref DMA_ChainTranStart*/
										 
}DMA_InitTypeDef;

void DMA_Start(DMA_VECTOR_t vect_num);
void DMA_Init(DMA_InitTypeDef * DMA_InitStruct);
void DMA_Stop(DMA_VECTOR_t vect_num);
void DMA_Trigger(DMA_VECTOR_t vect_num);

#endif
