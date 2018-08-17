#include <gps.h>
#include <string.h>
#include <debug.h>
#include "stm32fxxx_hal.h"

#define gps_buffer_size		1024
char gps_buffer[gps_buffer_size];
uint32_t gps_buffer_index = 0;
UART_HandleTypeDef huart6;

void gps_init()
{
	huart6.Instance = USART6;
	huart6.Init.BaudRate = 9600;
	huart6.Init.WordLength = UART_WORDLENGTH_8B;
	huart6.Init.StopBits = UART_STOPBITS_1;
	huart6.Init.Parity = UART_PARITY_NONE;
	huart6.Init.Mode = UART_MODE_TX_RX;
	huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart6.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart6) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}
}

float gps_update(Point_pos* actpos)
{
	float ret = 0.0;

	gps_buffer_index = 0;
	while(1)
	{
		HAL_StatusTypeDef uart_ret = HAL_UART_Receive(&huart6, (uint8_t*) gps_buffer + gps_buffer_index, 1, 100);
		++gps_buffer_index;
		if ((uart_ret == HAL_TIMEOUT) || (gps_buffer_index > (gps_buffer_size - 2)))
		{
			gps_buffer[gps_buffer_index] = 0;
			break;
		}
	}

	struct minmea_sentence_rmc mininm_frame_rmc;
	struct minmea_sentence_gga mininm_frame_gga;
	int gprmc_index = gps_find_sequence_start(gps_buffer, "GPRMC");
	int gpgga_index = gps_find_sequence_start(gps_buffer, "GPGGA");
	if (gprmc_index >= 0 && gpgga_index >= 0)
	{
		bool minm_ret = minmea_parse_rmc(&mininm_frame_rmc, gps_buffer + gprmc_index);
		minm_ret |= minmea_parse_gga(&mininm_frame_gga, gps_buffer + gpgga_index);

		actpos->HDOP = minmea_tofloat(&mininm_frame_gga.hdop);
		actpos->Latitude = minmea_tofloat(&mininm_frame_rmc.latitude);
		actpos->Longitude = minmea_tofloat(&mininm_frame_rmc.longitude);

		DEBUG("%d %d %d", (int)(actpos->Latitude * 1000), (int)(actpos->Longitude * 1000), (int)(actpos->HDOP * 1000));
		ret = actpos->HDOP + 0.5;
	}
	return ret;
}

int gps_find_sequence_start(const char *data, const char *sequence)
{
	int i = 0;
	int len = strlen(data);
	for (i = 0; i < len - 6; ++i)
	{
		if (data[i] == '$')
		{
			int j = 0;
			for (j = 0; j < 5; ++j)
			{
				if (!(data[i + j + 1] == sequence[j]))
				{
					break;
				}
			}
			if (j == 5)
			{
				return i;
			}
		}
	}
	return -1;
}
