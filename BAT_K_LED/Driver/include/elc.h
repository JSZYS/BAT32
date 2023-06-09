#ifndef __ELC_H_
#define __ELC_H_
#include <stdint.h>
#include "common.h"
#define _00_ELC_EVENT_INTP0           (0x00U) /* ELSELR00 */
#define _01_ELC_EVENT_INTP1           (0x01U) /* ELSELR01 */
#define _02_ELC_EVENT_INTP2           (0x02U) /* ELSELR02 */
#define _03_ELC_EVENT_INTP3           (0x03U) /* ELSELR03 */
#define _04_ELC_EVENT_INTRTC          (0x04U) /* ELSELR04 */
#define _05_ELC_EVENT_INTTM00         (0x05U) /* ELSELR05 */
#define _06_ELC_EVENT_INTTM01         (0x06U) /* ELSELR06 */
#define _07_ELC_EVENT_INTTM02         (0x07U) /* ELSELR07 */
#define _08_ELC_EVENT_INTTM03         (0x08U) /* ELSELR08 */
#define _09_ELC_EVENT_INTTM10         (0x09U) /* ELSELR09 */
#define _0A_ELC_EVENT_INTTM11         (0x0AU) /* ELSELR0A */
#define _0B_ELC_EVENT_INTTM12         (0x0BU) /* ELSELR0B */
#define _0C_ELC_EVENT_INTTM13         (0x0CU) /* ELSELR0C */
#define _0D_ELC_EVENT_INTCMP0         (0x0DU) /* ELSELR0D */
#define _0E_ELC_EVENT_INTCMP1         (0x0EU) /* ELSELR0E */

/*
    Event output destination select register n (ELSELRn) 
*/
/* Event output destination select register n (ELSELn1 - ELSELn0) */
#define _00_ELC_EVENT_LINK_OFF            (0x00U) /* event link disabled */
#define _01_ELC_EVENT_LINK_AD             (0x01U) /* select operation of peripheral function 1 to A/D converter */
#define _02_ELC_EVENT_LINK_TI00           (0x02U) /* select operation of peripheral function 2 to Timer input of timer 4 channel 0 */
#define _03_ELC_EVENT_LINK_TI01           (0x03U) /* select operation of peripheral function 3 to Timer input of timer 4 channel 1 */


/***********************************************************************************************************************
Macro definitions
***********************************************************************************************************************/
#define ELC_DESTINATION_COUNT              (0x0FU)

/***********************************************************************************************************************
Typedef definitions
***********************************************************************************************************************/

/***********************************************************************************************************************
Global functions
***********************************************************************************************************************/
void ELC_Start(uint32_t event_src, uint32_t event_dst);
void ELC_Stop(uint32_t event);
#endif
