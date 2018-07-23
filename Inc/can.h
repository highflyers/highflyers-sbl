#ifndef CAN_H_
#define CAN_H_

#include <stm32f4xx_hal.h>
#include <stm32f4xx_hal_can.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define CAN_UAVCAN_ID_ESC_RAW_DATA		1030

typedef struct
{
	uint32_t rx, tx, rx_dropped;
} can_stats_t;

typedef struct
{
	uint32_t id;
	uint8_t data[8];
	uint8_t length;
} can_frame_t;

typedef struct
{
	uint32_t push_index, pop_index;
	uint32_t capacity;
	can_frame_t array[32];
	uint8_t locked;
} can_fifo_t;

void can_init();

void can_fifo_init(can_fifo_t *fifo);
uint32_t can_fifo_free_space(can_fifo_t *fifo);
uint8_t can_fifo_push(can_fifo_t *fifo, uint32_t id, uint8_t *data, uint8_t length);
can_frame_t *can_fifo_pop(can_fifo_t *fifo);

void can_spin();

#endif /* CAN_H_ */
