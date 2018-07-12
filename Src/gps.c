/*
 * gps.c
 *
 *  Created on: 21.11.2017
 *      Author: Dell Latitude E5450
 */

#include <gps.h>

TM_GPS_Data_t GPS_Data;
TM_GPS_Result_t result, current;
TM_GPS_Float_t GPS_Float;
TM_GPS_Distance_t GPS_Distance;
float temp;
int data_ret = 0;

void GPS_init() {
	TM_DELAY_Init();
	TM_GPS_Init(&GPS_Data, 9600);
	TM_DELAY_SetTime(0);
}

int GPS_update(Point_pos* actpos) {
	result = TM_GPS_Update(&GPS_Data);
	if (result == TM_GPS_Result_FirstDataWaiting && TM_DELAY_Time() > 3000) {
		TM_DELAY_SetTime(0);
	}
	if (result == TM_GPS_Result_NewData) {
		current = TM_GPS_Result_NewData;
		if (GPS_Data.Validity) {
			data_ret = 1;
			transmit_info("nowe dane", EMPTY);
		} else {
		}
	} else if (result == TM_GPS_Result_FirstDataWaiting
			&& current != TM_GPS_Result_FirstDataWaiting) {
		current = TM_GPS_Result_FirstDataWaiting;
		data_ret = 0;
	} else if (result == TM_GPS_Result_OldData
			&& current != TM_GPS_Result_OldData) {
		current = TM_GPS_Result_OldData;
		data_ret = 0;
	}

	actpos->Latitude = GPS_Data.Latitude;
	actpos->Longitude = GPS_Data.Longitude;
	actpos->HDOP = GPS_Data.HDOP;

	return data_ret;
}
