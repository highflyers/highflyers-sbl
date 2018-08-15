#include <can.h>
#include <debug.h>
#include <errno.h>
#include <cuavcan.h>

extern CAN_HandleTypeDef CanHandle;
extern CanTxMsgTypeDef TxMessage;
extern CanRxMsgTypeDef RxMessage;
extern CanRxMsgTypeDef RxMessage1;
extern can_stats_t can_stats;
extern int32_t *id1010_data;
extern int32_t *id1030_data;
extern can_fifo_t can_fifo;
extern cuavcan_instance_t uavcan;

void can_init()
{
	GPIO_InitTypeDef GPIO_InitStruct;
	__HAL_RCC_CAN1_CLK_ENABLE()
	;
	__HAL_RCC_GPIOD_CLK_ENABLE()
	;
	GPIO_InitStruct.Pin = GPIO_PIN_1;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Alternate = GPIO_AF9_CAN1;
	HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = GPIO_PIN_0;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Alternate = GPIO_AF9_CAN1;
	HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

	CAN_FilterConfTypeDef sFilterConfig;

	CanHandle.Instance = CAN1;
	CanHandle.pTxMsg = &TxMessage;
	CanHandle.pRxMsg = &RxMessage;
	CanHandle.pRx1Msg = &RxMessage1;

	CanHandle.Init.TTCM = DISABLE;
	CanHandle.Init.ABOM = DISABLE;
	CanHandle.Init.AWUM = DISABLE;
	CanHandle.Init.NART = DISABLE;
	CanHandle.Init.RFLM = DISABLE;
	CanHandle.Init.TXFP = DISABLE;
	CanHandle.Init.Mode = CAN_MODE_NORMAL;
	CanHandle.Init.SJW = CAN_SJW_1TQ;
	CanHandle.Init.BS1 = CAN_BS1_6TQ;
	CanHandle.Init.BS2 = CAN_BS2_1TQ;
	CanHandle.Init.Prescaler = 9;

	if (HAL_CAN_Init(&CanHandle) != HAL_OK)
	{
		Error_Handler();
	}

	sFilterConfig.FilterNumber = 0;
	sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
	sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
	sFilterConfig.FilterIdHigh = 0x0000;
	sFilterConfig.FilterIdLow = 0x0000;
	sFilterConfig.FilterMaskIdHigh = 0x0000;
	sFilterConfig.FilterMaskIdLow = 0x0000;
	sFilterConfig.FilterFIFOAssignment = 0;
	sFilterConfig.FilterActivation = ENABLE;
	sFilterConfig.BankNumber = 14;

	if (HAL_CAN_ConfigFilter(&CanHandle, &sFilterConfig) != HAL_OK)
	{
		/* Filter configuration Error */
		Error_Handler();
	}

	CanHandle.pTxMsg->ExtId = 130;
	CanHandle.pTxMsg->RTR = CAN_RTR_DATA;
	CanHandle.pTxMsg->IDE = CAN_ID_EXT;
	CanHandle.pTxMsg->DLC = 2;
	CanHandle.pTxMsg->Data[0] = 156;
	CanHandle.pTxMsg->Data[1] = 86;

	HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
	HAL_NVIC_SetPriority(CAN1_RX1_IRQn, 0, 1);
	HAL_NVIC_EnableIRQ(CAN1_RX1_IRQn);

	HAL_CAN_Receive_IT(&CanHandle, CAN_FIFO0);
	return;
}

void can_fifo_init(can_fifo_t *fifo)
{
	fifo->pop_index = 0;
	fifo->push_index = 0;
	fifo->capacity = 32;
	fifo->locked = 0;
}

uint32_t can_fifo_free_space(can_fifo_t *fifo)
{
	int32_t used_space = fifo->push_index - fifo->pop_index;
	uint32_t free_space = 0;
	if (used_space >= 0)
	{
		free_space = fifo->capacity - used_space;
	}
	else
	{
		free_space = -used_space;
	}
	return free_space;
}

uint8_t can_fifo_push(can_fifo_t *fifo, uint32_t id, uint8_t *data, uint8_t length)
{
	uint8_t ret = 0;
	if (!fifo->locked)
	{
		fifo->locked = 1;
		if (can_fifo_free_space(fifo) > 1)
		{
			fifo->array[fifo->push_index].id = id;
			fifo->array[fifo->push_index].length = length;
			*(uint64_t*)fifo->array[fifo->push_index].data = *(uint64_t *)data;
//			*(uint64_t*)fifo->array[fifo->push_index].data = *(uint64_t *)data;
//			*(uint64_t*)fifo->array[fifo->push_index].data = 0x5634 + id;
//			for(unsigned i = 0; i < length; ++i)
//			{
////				fifo->array[fifo->push_index].data[i] = data[i];
////				fifo->array[fifo->push_index].data[i] = id % i;
//				id += data[i];
//			}
//			fifo->array[fifo->push_index].data[length-1] = 0x9a;

			uint32_t new_push_idnex = fifo->push_index + 1;
			new_push_idnex %= fifo->capacity;
			if(new_push_idnex != fifo->pop_index)
			{
				fifo->push_index = new_push_idnex;
			}

			ret = 1;
		}
		fifo->locked = 0;
	}
	return ret;
}

can_frame_t *can_fifo_pop(can_fifo_t *fifo)
{
	can_frame_t *ret = NULL;
	if (!fifo->locked)
	{
		fifo->locked = 1;
		if (can_fifo_free_space(fifo) < fifo->capacity)
		{
			ret = &fifo->array[fifo->pop_index];
			++fifo->pop_index;
			fifo->pop_index %= fifo->capacity;
		}
		fifo->locked = 0;
	}
	return ret;
}

void can_spin()
{
	can_frame_t *frame;
	while((frame = can_fifo_pop(&can_fifo)) != NULL)
	{
		cuavcan_handle_can_frame(&uavcan, frame->id, frame->data, frame->length);
	}
	update_pwm(id1030_data);
}

void HAL_CAN_RxCpltCallback(CAN_HandleTypeDef* hcan)
{CanRxMsgTypeDef *can_active_buffer = hcan->pRxMsg;
	if (can_fifo_push(&can_fifo, can_active_buffer->ExtId,
			can_active_buffer->Data, can_active_buffer->DLC) == 0)
	{
		++can_stats.rx_dropped;
	}
	++can_stats.rx;
	HAL_CAN_Receive_IT(&CanHandle, CAN_FIFO0);
}

