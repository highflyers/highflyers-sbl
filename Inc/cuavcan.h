//MIT License
//
//Copyright(c) 2018 High Flyers
//
//Permission is hereby granted, free of charge, to any person obtaining a copy
//of this software and associated documentation files(the "Software"), to deal
//in the Software without restriction, including without limitation the rights
//to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
//copies of the Software, and to permit persons to whom the Software is
//furnished to do so, subject to the following conditions :
//
//The above copyright notice and this permission notice shall be included in all
//copies or substantial portions of the Software.
//
//THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
//IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
//FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE
//AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
//LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
//OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
//SOFTWARE.

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <cuavcan_config.h>

#define CUAVCAN_MESSAGE_MAX_LENGTH		32

typedef struct
{
	uint16_t id;
	uint8_t source;
	uint8_t *payload;
	uint8_t length;
} cuavcan_message_t;

typedef struct
{
	cuavcan_message_t msg;
	bool is_initialized;
	bool is_complete;
	uint8_t transfer_id;
	bool expected_toggle;
} cuavcan_message_assembly_t;

typedef struct
{
	uint8_t data;
	bool is_start_of_transfer;
	bool is_end_of_transfer;
	bool toggle_bit;
	uint8_t transfer_id;
} cuavcan_tail_byte_t;

typedef struct
{
	uint16_t node_id;
	cuavcan_message_assembly_t msgs[CUAVCAN_MAX_SUBSCRIBED_MESSAGES];
	uint16_t * subscribed_ids;
	uint8_t subscribed_ids_length;
	void(*on_new_message)(cuavcan_message_t*);
} cuavcan_instance_t;

void cuavcan_init(cuavcan_instance_t *uavcan, uint16_t *subscribed_ids, uint8_t subscribed_ids_length, void (*on_new_message)(cuavcan_message_t*));

void cuavcan_message_assembly_reset(cuavcan_message_assembly_t *msg);

void cuavcan_handle_can_frame(cuavcan_instance_t *uavcan, uint32_t id, uint8_t *payload, uint8_t length);
uint32_t cuavcan_get_message_id(uint32_t frame_id);
uint8_t cuavcan_get_node_id(uint32_t frame_id);
void cuavcan_parse_tail_byte(uint8_t *payload, uint8_t length, cuavcan_tail_byte_t *dst);
bool cuavcan_is_transfer_single_frame(cuavcan_tail_byte_t *tail);
cuavcan_message_assembly_t *cuavcan_find_message_assembly(cuavcan_instance_t *uavcan, uint16_t msg_id);
void cuavcan_copy(uint8_t *from, uint8_t *to, uint8_t count);
