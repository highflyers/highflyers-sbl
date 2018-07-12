/*
 * sd.h
 *
 *  Created on: 22.11.2017
 *      Author: Dell Latitude E5450
 */

#ifndef SD_H_
#define SD_H_
//#include <main.h>
#include <Pos_check.h>
#define  MAX_BMP_FILES  25
#define  MAX_BMP_FILE_NAME 11

#define SD_CARD_NOT_FORMATTED                    0
#define SD_CARD_FILE_NOT_SUPPORTED               1
#define SD_CARD_OPEN_FAIL                        2
#define FATFS_NOT_MOUNTED                        3
#define TIMEOUT									3000

void SD_dataread( Point_pos **pointpos, BoundingBox* boxborders, int *np);

#endif /* SD_H_ */

