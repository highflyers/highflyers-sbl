/*
 * sd.c
 *
 *  Created on: 22.11.2017
 *      Author: Dell Latitude E5450
 */
#include <sd.h>
#include <fatfs.h>

static FATFS g_sFatFs;
FRESULT fresult;
FIL plik;
WORD odczytano;
int toread;
char* test;
char korektor[] = " ;\n\r";
char * dat_buff;
char key[1] = " ";
float dat_buff_f;

void SD_dataread(Point_pos **dst, BoundingBox* boxborders, int *np){
	//montiwanie, otwarcie pliku, sprawdzenie rozmiaru
		fresult = f_mount(&g_sFatFs, (TCHAR const*) "/", 1);
		fresult = f_open(&plik, "plik.txt", FA_READ);
		transmit_info("rozmiar pliku",(double)fresult);
		fresult = f_size(&plik);
		toread = fresult;

	//zaalokowanie pamiêci na odczytanie danych
		BYTE *SD_buff = (BYTE*) calloc(toread, sizeof(BYTE));

	//odczyt i zamkniêcie pliku
		fresult = f_read(&plik, SD_buff, toread, &odczytano);
		fresult = f_close(&plik);
		test = SD_buff;

	//sprawdzenie ilosci wierszy(punktow)
	for (int i = 0; i < 36; i++) {
		if (SD_buff[i] == key[0]) {
			*np = *np + 1;
		}
	}
	*np = *np + 4;
	Point_pos *pointpos = (Point_pos*) malloc(*np * sizeof(Point_pos));
	*dst = pointpos;

	//przepisanie danych z bufora do struktury
	for (int i = 0; i < *np; i++) {
		if (i == 0) {
			dat_buff = strtok(test, korektor);
		} else {
			dat_buff = strtok(NULL, korektor);
		}
		dat_buff_f = atof(dat_buff);
		pointpos[i].Latitude = dat_buff_f;
		transmit_info("pointpos[i].Latitude", pointpos[i].Latitude);
		dat_buff = strtok(NULL, korektor);
		dat_buff_f = atof(dat_buff);
		pointpos[i].Longitude = dat_buff_f;
		transmit_info("pointpos[i].Longitude", pointpos[i].Longitude);
	}
	free(SD_buff);

	//wyznaczenie granic pionowych i poziomych
	 boxborders->Latitude_max = pointpos[0].Latitude;
	 boxborders->Latitude_min = pointpos[0].Latitude;
	 boxborders->Longitude_max = pointpos[0].Longitude;
	 boxborders->Longitude_min = pointpos[0].Longitude;

	for (int i = 1; i < *np; i++) {
		if (pointpos[i].Latitude > boxborders->Latitude_max)
			boxborders->Latitude_max = pointpos[i].Latitude;
		if (pointpos[i].Latitude < boxborders->Latitude_min)
			boxborders->Latitude_min = pointpos[i].Latitude;
		if (pointpos[i].Longitude > boxborders->Longitude_max)
			boxborders->Longitude_max = pointpos[i].Longitude;
		if (pointpos[i].Longitude < boxborders->Longitude_min)
			boxborders->Longitude_min = pointpos[i].Longitude;
	}
	transmit_info("Latitude_max", boxborders->Latitude_max);
	transmit_info("Latitude_min", boxborders->Latitude_min);
	transmit_info("Longitude_max", boxborders->Longitude_max);
	transmit_info("Longitude_min", boxborders->Longitude_min);
}
