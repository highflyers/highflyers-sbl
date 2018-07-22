#include <debug.h>

char  __debug_buffer[256];

void transmit_info(const char* type, float value) {
	uint8_t data_type[50] = { 0 };
	uint8_t data_value[50] = { 0 };

	//wyslanie informacji o typie danych
	sprintf(data_type, "%s: ", type); // Stworzenie wiadomosci do wyslania oraz przypisanie ilosci wysylanych znakow do zmiennej size./
	HAL_UART_Transmit(&huart2, (uint8_t *) data_type, strlen(data_type),
	HAL_MAX_DELAY);

	//wyslanie danych tego typu
	sprintf(data_value, "%f\r\n", value);
	HAL_UART_Transmit(&huart2, (uint8_t *) data_value, strlen(data_value),
	HAL_MAX_DELAY);
}
