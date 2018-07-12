#ifndef DEBUG_H_
#define DEBUG_H_

#include <defines.h>
#include "stm32f4xx_hal.h"

#define DEBUG(msg)	transmit_info(msg,EMPTY)

void transmit_info(const char* data_type, float value);

#endif /* DEBUG_H_ */
