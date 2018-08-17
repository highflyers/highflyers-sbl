#ifndef POS_CHECK_H_
#define POS_CHECK_H_

#ifndef Point_pos

#include "stm32f4xx_hal.h"

typedef struct
{
	float Latitude;
	float Longitude;
	float HDOP;
} Point_pos;

#endif
#ifndef BoundingBox

typedef struct
{
	float Latitude_min;
	float Latitude_max;
	float Longitude_min;
	float Longitude_max;
} BoundingBox;

#endif

int IsInPolygon(Point_pos* actpos, Point_pos *pointpos, BoundingBox* boxborders, int nop);

#endif /* POS_CHECK_H_ */
