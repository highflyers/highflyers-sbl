/*
 * Pos_check.c
 *
 *  Created on: 20.11.2017
 *      Author: Dell Latitude E5450
 */
#include <Pos_check.h>

int IsInPolygon(Point_pos* actpos, Point_pos *pointpos, BoundingBox* boxborders,
		int nop) {
	float W, Wx, a, b, a1, b1 = -1, b2 = -1, c1, c2, x;
	int cutborders = 0, j = 0;
	if (actpos->Latitude > boxborders->Latitude_max
			|| actpos->Latitude < boxborders->Latitude_min
			|| actpos->Longitude > boxborders->Longitude_max
			|| actpos->Longitude < boxborders->Longitude_min) {
		return 0;
	}

	for (int i = 0; i < nop; i++) {
		if (i != (nop - 1))
			j = i + 1;
		else
			j = 0;
		a = (pointpos[j].Latitude - pointpos[i].Latitude)
				/ (pointpos[j].Longitude - pointpos[i].Longitude);
		b = pointpos[i].Latitude - (a * pointpos[i].Longitude);
		a1 = a;
		c1 = -b;
		c2 = -actpos->Latitude;
		W = (a1 * b2);
		Wx = (c1 * b2) - (c2 * b1);
		//proste pokrywaja sie
		if (W == 0 && Wx == 0) {
			if (actpos->Longitude >= pointpos[j].Longitude
					&& actpos->Longitude <= pointpos[i].Longitude)
				return 1;
		}
		//proste rownolegle
		else if (W == 0 && Wx != 0) {
			continue;
		}
		// pozostale przypadki
		else {
			x = Wx / W;
			if (pointpos[i].Longitude > pointpos[j].Longitude) {
				if (x >= pointpos[j].Longitude && x <= pointpos[i].Longitude
						&& x >= actpos->Longitude
						&& x <= boxborders->Longitude_max)
					cutborders++;
			} else {
				if (x >= pointpos[i].Longitude && x <= pointpos[j].Longitude
						&& x >= actpos->Longitude
						&& x <= boxborders->Longitude_max)
					cutborders++;
			}
		}
	}
	if (cutborders % 2 == 0)
		//znajduje sie poza dozwolonym obszarem
		return 0;
	else
		//znajduje sie w dozwolonym obszarze
		return 1;
}

