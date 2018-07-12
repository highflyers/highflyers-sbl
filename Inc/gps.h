/*
 * gps.h
 *
 *  Created on: 21.11.2017
 *      Author: Dell Latitude E5450
 */

#ifndef GPS_H_
#define GPS_H_

#include <Pos_check.h>
#include "tm_stm32_gps.h"
#include "tm_stm32_delay.h"
void GPS_init(void);
int GPS_update(Point_pos* actpos);

#endif /* GPS_H_ */
