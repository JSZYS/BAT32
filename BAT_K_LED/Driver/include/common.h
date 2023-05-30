#ifndef __COMMON_H__
#define __COMMON_H__

#include "BAT32A239.h"
#ifdef BAT32G139_100PIN
	#define BAT32G1XX_100PIN
#elif BAT32G139_80PIN
	#define BAT32G1XX_80PIN
#elif BAT32G139_64PIN
	#define BAT32G1XX_64PIN
#elif BAT32G139_48PIN
	#define BAT32G1XX_48PIN
#elif BAT32G139_32PIN
	#define BAT32G1XX_32PIN
#endif
typedef enum {RESET = 0, SET = !RESET} FlagStatus, ITStatus;
typedef enum {ERROR = 0, SUCCESS = !ERROR} ErrorStatus;

typedef enum {DISABLE = 0, ENABLE = !DISABLE} FunctionalState;
#define IS_FUNCTIONAL_STATE(STATE) (((STATE) == DISABLE) || ((STATE) == ENABLE))
//#define SPI_INTERFACE

#endif
