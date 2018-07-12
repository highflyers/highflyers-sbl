#ifndef DEBUG_H_
#define DEBUG_H_

#include <defines.h>
#include "stm32f4xx_hal.h"

#define DEBUG(msg)	transmit_info(msg,EMPTY)
#define DEBUG_V(msg, x)	transmit_info(msg, x)

void transmit_info(const char* data_type, float value);

#endif /* DEBUG_H_ */
