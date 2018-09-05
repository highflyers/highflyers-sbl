#ifndef GPS_H_
#define GPS_H_

#include <Pos_check.h>
#include <minmea.h>

void gps_init(void);
float gps_update(Point_pos* actpos);

int gps_find_sequence_start(const char *data, const char *sequence);

#endif /* GPS_H_ */
