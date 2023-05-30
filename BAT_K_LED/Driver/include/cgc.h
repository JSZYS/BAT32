/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __CGC_H_
#define __CGC_H_

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "common.h"
#define	SYSTYEM_CLOCK_CLOSE		0
#define	SYSTYEM_CLOCK_OPEN		1

#define IS_CGC_PER0_PERIPH(PERIPH) ((((PERIPH) & 0xFF) != 0x00) && ((PERIPH) != 0x00))

#define IS_CGC_PER1_PERIPH(PERIPH) ((((PERIPH) & 0xFF) != 0x00) && ((PERIPH) != 0x00))

#define IS_CGC_PER2_PERIPH(PERIPH) ((((PERIPH) & 0xFF) != 0x00) && ((PERIPH) != 0x00))

#define CGC_PER0Periph_TIM40            ((uint8_t)0x01) /*!< CGC PER0: TM40EN (Bit 0)       */
#define CGC_PER0Periph_CAN0             ((uint8_t)0x02) /*!< CGC PER0: CAN0EN (Bit 1)       */
#define CGC_PER0Periph_SCI0             ((uint8_t)0x04) /*!< CGC PER0: SCI0EN (Bit 2)       */
#define CGC_PER0Periph_SCI1             ((uint8_t)0x08) /*!< CGC PER0: SCI1EN (Bit 3)       */

#define CGC_PER0Periph_UART0            CGC_PER0Periph_SCI0
#define CGC_PER0Periph_UART1            CGC_PER0Periph_SCI0
#define CGC_PER0Periph_UART2            CGC_PER0Periph_SCI1
#define CGC_PER0Periph_I2C00            CGC_PER0Periph_SCI0
#define CGC_PER0Periph_I2C01            CGC_PER0Periph_SCI0
#define CGC_PER0Periph_I2C10            CGC_PER0Periph_SCI0
#define CGC_PER0Periph_I2C11            CGC_PER0Periph_SCI0
#define CGC_PER0Periph_I2C20            CGC_PER0Periph_SCI1
#define CGC_PER0Periph_I2C21            CGC_PER0Periph_SCI1
#define CGC_PER0Periph_SSPI00           CGC_PER0Periph_SCI0
#define CGC_PER0Periph_SSPI01           CGC_PER0Periph_SCI0
#define CGC_PER0Periph_SSPI10           CGC_PER0Periph_SCI0
#define CGC_PER0Periph_SSPI11           CGC_PER0Periph_SCI0
#define CGC_PER0Periph_SSPI20           CGC_PER0Periph_SCI1
#define CGC_PER0Periph_SSPI21           CGC_PER0Periph_SCI1

#define CGC_PER0Periph_I2CA0             ((uint8_t)0x10) /*!< CGC PER0: IICAEN (Bit 4)       */
#define CGC_PER0Periph_ADC              ((uint8_t)0x20) /*!< CGC PER0: ADCEN  (Bit 5)       */
#define CGC_PER0Periph_IRDA             ((uint8_t)0x40) /*!< CGC PER0: IRDAEN (Bit 6)       */
#define CGC_PER0Periph_RTC              ((uint8_t)0x80) /*!< CGC PER0: RTCEN  (Bit 7)       */

#define CGC_PER1Periph_TMA              ((uint8_t)0x01) /*!< CGC PER1: TMA     (Bit 0)     */
#define CGC_PER1Periph_TMC              ((uint8_t)0x02) /*!< CGC PER1: TMC     (Bit 1)     */
#define CGC_PER1Periph_PWMCUTOFF        ((uint8_t)0x04) /*!< CGC PER1: PWMCUTOFF(Bit 2) */
#define CGC_PER1Periph_DMA              ((uint8_t)0x08) /*!< CGC PER1: DMAEN    (Bit 3)     */
#define CGC_PER1Periph_TMM              ((uint8_t)0x10) /*!< CGC PER1: TMM     (Bit 4)     */
#define CGC_PER1Periph_PGACMP           ((uint8_t)0x20) /*!< CGC PER1: PGACMPEN (Bit 5)     */
#define CGC_PER1Periph_TMB              ((uint8_t)0x40) /*!< CGC PER1: TMB    (Bit 6)     */
#define CGC_PER1Periph_DAC              ((uint8_t)0x80) /*!< CGC PER1: DAC    (Bit 7)     */

#define CGC_PER2Periph_TM81				((uint8_t)0x01) /*!< CGC PER2: TM81     (Bit 0)     */
#define CGC_PER2Periph_CAN1				((uint8_t)0x02) /*!< CGC PER2: CAN1     (Bit 1)     */
#define CGC_PER2Periph_I2CA1			((uint8_t)0x04) /*!< CGC PER2: IICA1     (Bit 2)     */
#define CGC_PER2Periph_SCI2				((uint8_t)0x08) /*!< CGC PER2: SCI2     (Bit 3)     */
#define CGC_PER2Periph_OSDC				((uint8_t)0x10) /*!< CGC PER2: OSDC	     (Bit 4)     */
#define CGC_PER2Periph_CAN2				((uint8_t)0x20) /*!< CGC PER2: CAN2     (Bit 5)     */
#define CGC_PER2Periph_SPIHS0			((uint8_t)0x40) /*!< CGC PER2: SPIHS0     (Bit 6)     */
#define CGC_PER2Periph_SPIHS1			((uint8_t)0x80) /*!< CGC PER2: SPIHS1     (Bit 7)     */

void CGC_PER0PeriphClockCmd(uint8_t CGC_PER0Periph, FunctionalState NewState);
void CGC_PER1PeriphClockCmd(uint32_t CGC_PER1Periph, FunctionalState NewState);
void CGC_PER2PeriphClockCmd(uint32_t CGC_PER2Periph, FunctionalState NewState);

/***********************************************************************************************************************
Macro definitions (Register bit)
***********************************************************************************************************************/
/*
    Clock operation mode control register (CMC) 
*/
/* Control of X1 clock oscillation frequency (AMPH) */
#define _00_CGC_SYSOSC_UNDER10M                    (0x00U) /* 1MHz <= fX <= 10MHz */
#define _01_CGC_SYSOSC_OVER10M                     (0x01U) /* 10MHz < fX <= 20MHz */
/* XT1 oscillator oscillation mode selection (AMPHS1,AMPHS0) */
#define _00_CGC_LOW_OSCILLATION                    (0x00U) /* low power consumption oscillation */
#define _02_CGC_NORMAL_OSCILLATION                 (0x02U) /* normal oscillation */
#define _04_CGC_ULTRA_LOW_OSCILLATION              (0x04U) /* ultra-low power consumption oscillation */
/* Subsystem clock pin operation mode (EXCLKS,OSCSELS) */
#define _00_CGC_SUB_PORT                           (0x00U) /* XT1, XT2 as I/O port */
#define _10_CGC_SUB_OSC                            (0x10U) /* XT1, XT2 as crystal connection */
#define _20_CGC_SUB_PORT1                          (0x20U) /* XT1, XT2 as I/O port */
#define _30_CGC_SUB_EXT                            (0x30U) /* XT1 as I/O port, XT2 as external clock input */
#define _30_CGC_SUB_PIN                            (0x30U) /* XT1, XT2 pin setting */
/* High-speed system clock pin operation mode (EXCLK,OSCSEL) */
#define _00_CGC_HISYS_PORT                         (0x00U) /* X1, X2 as I/O port */
#define _40_CGC_HISYS_OSC                          (0x40U) /* X1, X2 as crystal/ceramic resonator connection */
#define _80_CGC_HISYS_PORT1                        (0x80U) /* X1, X2 as I/O port */
#define _C0_CGC_HISYS_EXT                          (0xC0U) /* X1 as I/O port, X2 as external clock input */
#define _C0_CGC_HISYS_PIN                          (0xC0U) /* X1, X2 pin setting */

/*
    System clock control register (CKC) 
*/
/* Main system clock (fMAIN) operation control (MCM0) */
#define _00_CGC_MAINCLK_SELFOCO                    (0x00U) /* selects the high-speed on-chip oscillator clock (fIH) as the main system clock (fMAIN) */
#define _10_CGC_MAINCLK_SELHISYS                   (0x10U) /* selects the high-speed system clock (fMX) as the main system clock (fMAIN) */
/* Status of Main system clock (fMAIN) (MCS) */
#define _00_CGC_MAINCLK_FOCO                       (0x00U) /* high-speed on-chip oscillator clock (fIH) */
#define _20_CGC_MAINCLK_HISYS                      (0x20U) /* high-speed system clock (fMX) */
/* Selection of CPU/peripheral hardware clock (fCLK) (CSS) */
#define _00_CGC_MAINCLK_SELECTED                   (0x00U) /* main system clock (fMAIN) */
#define _40_CGC_MAINCLK_FSUB                       (0x40U) /* subsystem clock (fSUB) */
/* Status of CPU/peripheral hardware clock (fCLK) (CLS) */
#define _00_CGC_MAINCLK_STATUS                     (0x00U) /* main system clock (fMAIN) */
#define _80_CGC_MAINCLK_STATUS                     (0x80U) /* subsystem clock (fSUB) */

/*
    Clock operation status control register (CSC) 
*/
/* High-speed on-chip oscillator clock operation control (HIOSTOP) */
#define _00_CGC_HIO_OPER                           (0x00U) /* high-speed on-chip oscillator operating */
#define _01_CGC_HIO_STOP                           (0x01U) /* high-speed on-chip oscillator stopped */
/* Subsystem clock operation control (XTSTOP) */
#define _00_CGC_FSUB_OPER                          (0x00U) /* XT1 oscillator/external clock operating */
#define _40_CGC_FSUB_STOP                          (0x40U) /* XT1 oscillator/external clock stopped */
/* High-speed system clock operation control (MSTOP) */
#define _00_CGC_HISYS_OPER                         (0x00U) /* X1 oscillator/external clock operating */
#define _80_CGC_HISYS_STOP                         (0x80U) /* X1 oscillator/external clock stopped */

/*
    Oscillation stabilization time counter status register (OSTC) 
*/
/* oscillation stabilization time status (MOST8,MOST9,MOST10,MOST11,MOST13,MOST15,MOST17,MOST18) */
#define _00_CGC_OSCSTAB_STA0                       (0x00U) /* 2^8/fX max. */
#define _80_CGC_OSCSTAB_STA8                       (0x80U) /* 2^8/fX */
#define _C0_CGC_OSCSTAB_STA9                       (0xC0U) /* 2^9/fX */
#define _E0_CGC_OSCSTAB_STA10                      (0xE0U) /* 2^10/fX */
#define _F0_CGC_OSCSTAB_STA11                      (0xF0U) /* 2^11/fX */
#define _F8_CGC_OSCSTAB_STA13                      (0xF8U) /* 2^13/fX */
#define _FC_CGC_OSCSTAB_STA15                      (0xFCU) /* 2^15/fX */
#define _FE_CGC_OSCSTAB_STA17                      (0xFEU) /* 2^17/fX */
#define _FF_CGC_OSCSTAB_STA18                      (0xFFU) /* 2^18/fX */

/*
    Oscillation stabilization time select register (OSTS) 
*/
/* oscillation stabilization time selection (OSTS2,OSTS1,OSTS0) */
#define _00_CGC_OSCSTAB_SEL8                       (0x00U) /* 2^8/fX */
#define _01_CGC_OSCSTAB_SEL9                       (0x01U) /* 2^9/fX */
#define _02_CGC_OSCSTAB_SEL10                      (0x02U) /* 2^10/fX */
#define _03_CGC_OSCSTAB_SEL11                      (0x03U) /* 2^11/fX */
#define _04_CGC_OSCSTAB_SEL13                      (0x04U) /* 2^13/fX */
#define _05_CGC_OSCSTAB_SEL15                      (0x05U) /* 2^15/fX */
#define _06_CGC_OSCSTAB_SEL17                      (0x06U) /* 2^17/fX */
#define _07_CGC_OSCSTAB_SEL18                      (0x07U) /* 2^18/fX */

/*
    Subsystem clock supply mode control register (OSMC) 
*/
/* Setting in STOP mode or HALT mode while subsystem clock is selected as CPU clock (RTCLPC) */
#define _00_CGC_CLK_ENABLE                         (0x00U) /* enables supply of subsystem clock to peripheral functions */
#define _80_CGC_CLK_STOP                           (0x80U) /* stops supply of subsystem clock to peripheral functions  */
/* Selection of operation clock for real-time clock, 12-bit interval timer, and timer RJ (WUTMMCK0) */
#define _00_CGC_TMRJ_IT_CLK_SUBSYSTEM_CLK          (0x00U) /* subsystem clock */
#define _10_CGC_TMRJ_IT_CLK_FIL                    (0x10U) /* low-speed on-chip oscillator clock  */

/*
    PLL Control Register (PLLCR)
*/
/* PLL Operation Control (PLLON[0:0]) */
#define _00_CGC_PLL_OPER            (0x00U) /* PLL operation */
#define _01_CGC_PLL_STOP            (0x01U) /* PLL Stop */
/* Frequency Multiplication Factor Select (PLLMUL[1:1]) */
#define _00_CGC_PLL_MUL_12_0        (0x00U) /* x12 */
#define _02_CGC_PLL_MUL_16_0        (0x02U) /* x16 */
/* PLL Input Frequency Division Ratio Select (PLIDIV[3:2]) */
#define _00_CGC_PLL_DIV_1           (0x00U) /* x1 */
#define _04_CGC_PLL_DIV_2           (0x04U) /* x1/2 */
#define _08_CGC_PLL_DIV_4           (0x08U) /* x1/4 */
/* PLL Source Select (PLLSRSEL) */
#define _00_CGC_PLLSR_fIH           (0x00U) /* fIH is selected for PLL source */
#define _80_CGC_PLLSR_fMX           (0x80U) /* fMX is selected for PLL source */
/*
    USB-dedicated PLL Control Register (UPLLCR)
*/
/* USB-dedicated PLL Operation Control (UPLLON[0:0]) */
#define _00_CGC_UPLL_OPER            (0x00U) /* UPLL operation */
#define _01_CGC_UPLL_STOP            (0x01U) /* UPLL Stop */
/* Frequency Multiplication Factor Select (UPLLMUL[1:1]) */
#define _00_CGC_UPLL_MUL_12_0        (0x00U) /* x12 */
#define _02_CGC_UPLL_MUL_16_0        (0x02U) /* x16 */
/* PLL Input Frequency Division Ratio Select (UPLIDIV[3:2]) */
#define _00_CGC_UPLL_DIV_1           (0x00U) /* x1 */
#define _04_CGC_UPLL_DIV_2           (0x04U) /* x1/2 */
#define _08_CGC_UPLL_DIV_4           (0x08U) /* x1/4 */
/* USB-Dedicated PLL Source Select (UPLLSRSEL) */
#define _00_CGC_UPLLSR_fIH           (0x00U) /* fIH is selected for UCLK PLL source */
#define _80_CGC_UPLLSR_fMX           (0x80U) /* fMX is selected for UCLK PLL source */


/***********************************************************************************************************************
Macro definitions
***********************************************************************************************************************/
#define CGC_SUBWAITTIME               (360U)   /* change the waiting time according to the system */

/***********************************************************************************************************************
Typedef definitions
***********************************************************************************************************************/

typedef enum
{
	OSC_PORT,	   //端口模式
	OSC_OSCILLATOR,//振荡模式
	OSC_EXCLK	   //外部时钟输入模式
} OSC_Pin_Mode_t;
#define IS_CGC_LSE_MODE(MODE)  (((MODE) == OSC_PORT)  || ((MODE) == OSC_OSCILLATOR) || ((MODE) == OSC_EXCLK))

typedef enum
{
	MAINCLK_FIH = 0, //高速内部振荡时钟
	MAINCLK_FMX = 1, //高速系统时钟
	MAINCLK_FSUB = 2, //副系统时钟
	MAINCLK_FIL = 3, //低速内部振荡时钟
	MAINCLK_FPLL = 4  //外部锁相环
} CLK_Select_t;

typedef enum
{ 
	OSC_LOW_POWER = 0,	//低功耗
	OSC_NORMAL_POWER,//普通功耗
	OSC_ULTRA_LOW_POWER //超低功耗
} OSC_Power_Mode_t;
#define IS_CGC_LSE_PWR_MODE(PWR)  (((PWR) == OSC_LOW_POWER)  || ((PWR) == OSC_NORMAL_POWER) || ((PWR) == OSC_ULTRA_LOW_POWER))

typedef enum
{
	OSC_UNDER_10M,
	OSC_OVER_10M,
} OSC_Speed_Mode_t;
#define IS_CGC_HSE_OSC_SPEED(SPEED)  (((SPEED) == OSC_UNDER_10M)  || ((SPEED) == OSC_OVER_10M))


typedef enum
{
    PLL_DIV_1,
    PLL_DIV_2,
    PLL_DIV_4
} PLL_Div_t;

typedef enum
{
    PLL_MUL_12,
    PLL_MUL_16,
} PLL_Mul_t;

typedef enum
{
    PLL_SR_fIH,
    PLL_SR_fMX,
} PLL_Src_t;


/***********************************************************************************************************************
Global functions
***********************************************************************************************************************/
void CGC_Osc_Setting(OSC_Pin_Mode_t main,OSC_Speed_Mode_t amph, OSC_Pin_Mode_t sub,OSC_Power_Mode_t amphs);
void CGC_MainOsc_Setting(OSC_Pin_Mode_t main, OSC_Speed_Mode_t amph);
void CGC_SubOsc_Setting(OSC_Pin_Mode_t sub, OSC_Power_Mode_t amphs);
void CGC_PLL_Setting(PLL_Src_t src,PLL_Div_t div, PLL_Mul_t mul);
void CGC_UPLL_Setting(PLL_Src_t src, PLL_Div_t div, PLL_Mul_t mul);
uint8_t CGC_Fclk_Select(CLK_Select_t cks);
void CGC_MainOsc_Stop(void);
void CGC_MainOsc_Start(void);
void CGC_SubOsc_Stop(void);
void CGC_SubOsc_Start(void);
void CGC_Hoco_Stop(void);
void CGC_Hoco_Start(void);

extern uint8_t USE_HSE_SYSTYEM_CLOCK;
void CGC_LSI_CFG_AS_FCLK(void);
void CGC_LSEConfig(OSC_Pin_Mode_t sub, OSC_Power_Mode_t amphs);
void CGC_LSE_CFG_AS_FCLK(void);
void CGC_HSEConfig(OSC_Pin_Mode_t main, OSC_Speed_Mode_t amph);
void CGC_HSE_CFG_AS_FCLK(void);
void CGC_HSI_CFG_AS_FCLK(void);

void CGC_HSE_CFG_AS_FMAIN(void);
void CGC_HSI_CFG_AS_FMAIN(void);
void CGC_PLL_Setting(PLL_Src_t src, PLL_Div_t div, PLL_Mul_t mul);
void CGC_PLL_CFG_AS_FCLK(void);


#endif

