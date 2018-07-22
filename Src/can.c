#include <can.h>
#include <debug.h>
#include <errno.h>

extern CAN_HandleTypeDef CanHandle;

static uint8_t node_health = UAVCAN_NODE_HEALTH_OK;
static uint8_t node_mode   = UAVCAN_NODE_MODE_INITIALIZATION;
#define CANARD_MEMORY_SIZE		1024
static CanardInstance canard;                       ///< The library instance
static uint8_t canard_memory_pool[CANARD_MEMORY_SIZE];            ///< Arena for memory allocation, used by the library

void can_init(CAN_HandleTypeDef *can)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	char str[128];
	__HAL_RCC_CAN1_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();
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
	static CanTxMsgTypeDef TxMessage;
	static CanRxMsgTypeDef RxMessage;
	static CanRxMsgTypeDef RxMessage1;

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

	  HAL_CAN_Receive_IT(&CanHandle, CAN_FIFO0);

	  return;
}

int can_decode_frame(const CanRxMsgTypeDef *rx, can_uavcan_frame_t *dst)
{
	dst->id = ((rx->ExtId) >> 8) & 0xFFFF;
	return 0;
}

int can_handle_esc_command(const can_uavcan_frame_t *frame, int32_t *esc_array)
{
	int ret = 0;
	if(frame->id == CAN_UAVCAN_ID_ESC_RAW_DATA)
	{

	}
	else
	{
		ret = -EINVAL;
	}
	return ret;
}

void can_uavcan_handle_node_info_request(CanardInstance *ins,
		CanardRxTransfer *transfer)
{
	printf("GetNodeInfo request from %d\n", transfer->source_node_id);

	uint8_t buffer[UAVCAN_GET_NODE_INFO_RESPONSE_MAX_SIZE];
	memset(buffer, 0, UAVCAN_GET_NODE_INFO_RESPONSE_MAX_SIZE);

	// NodeStatus
	makeNodeStatusMessage(buffer);

	// SoftwareVersion
	buffer[7] = APP_VERSION_MAJOR;
	buffer[8] = APP_VERSION_MINOR;
	buffer[9] = 1;                    // Optional field flags, VCS commit is set
	uint32_t u32 = GIT_HASH;
	canardEncodeScalar(buffer, 80, 32, &u32);
	// Image CRC skipped

	// HardwareVersion
	// Major skipped
	// Minor skipped
	readUniqueID(&buffer[24]);
	// Certificate of authenticity skipped

	// Name
	const size_t name_len = strlen(APP_NODE_NAME);
	memcpy(&buffer[41], APP_NODE_NAME, name_len);

	const size_t total_size = 41 + name_len;

	/*
	 * Transmitting; in this case we don't have to release the payload because it's empty anyway.
	 */
	const int16_t resp_res = canardRequestOrRespond(ins,
			transfer->source_node_id,
			UAVCAN_GET_NODE_INFO_DATA_TYPE_SIGNATURE,
			UAVCAN_GET_NODE_INFO_DATA_TYPE_ID, &transfer->transfer_id,
			transfer->priority, CanardResponse, &buffer[0],
			(uint16_t) total_size);
	if (resp_res <= 0)
	{
		(void) fprintf(stderr, "Could not respond to GetNodeInfo; error %d\n",
				resp_res);
	}
}

void can_uavcan_handle_esc_raw_data(CanardInstance *ins,
		CanardRxTransfer *transfer)
{
	DEBUG("RAW DATA");
	char str[128];
	unsigned str_n = 0;
	str_n += snprintf(str+str_n, 127-str_n, "RX: %d: [", transfer->data_type_id);
	for(int i = 0; i < transfer->payload_len; ++i)
	{
		str_n += snprintf(str+str_n, 127-str_n, "%02x ", transfer->payload_head[i]);
	}
	str_n += snprintf(str+str_n, 127-str_n, "]");
	DEBUG(str);
}

static void can_on_transfer_received(CanardInstance* ins,
                               CanardRxTransfer* transfer)
{
	DEBUG("can_on_transfer_received: %d", transfer->data_type_id);
    if ((transfer->transfer_type == CanardTransferTypeRequest) &&
        (transfer->data_type_id == UAVCAN_GET_NODE_INFO_DATA_TYPE_ID))
    {
    	can_uavcan_handle_node_info_request(ins, transfer);
    }

	if(transfer->data_type_id == CAN_UAVCAN_ID_ESC_RAW_DATA)
	{
		can_uavcan_handle_esc_raw_data(ins, transfer);
	}
}

static bool shouldAcceptTransfer(const CanardInstance* ins,
		uint64_t* out_data_type_signature, uint16_t data_type_id,
		CanardTransferType transfer_type, uint8_t source_node_id)
{
	bool ret = false;
	(void) source_node_id;
	if ((transfer_type == CanardTransferTypeRequest)
			&& (data_type_id == UAVCAN_GET_NODE_INFO_DATA_TYPE_ID))
	{
		*out_data_type_signature = UAVCAN_GET_NODE_INFO_DATA_TYPE_SIGNATURE;
		ret = true;
	}
	if(data_type_id == CAN_UAVCAN_ID_ESC_RAW_DATA)
	{
		ret = true;
	}
	if(data_type_id == 1010)
	{
		ret = true;
	}
	if(ret == false)
	{
		DEBUG("Unknown data type: %d", data_type_id);
	}
	return true;
}

static uint64_t getMonotonicTimestampUSec(void)
{
    return HAL_GetTick() * 1000;
}

static void readUniqueID(uint8_t* out_uid)
{
	for (uint8_t i = 0; i < UNIQUE_ID_LENGTH_BYTES; i++)
	{
		out_uid[i] = i;
	}
}

static void makeNodeStatusMessage(uint8_t buffer[UAVCAN_NODE_STATUS_MESSAGE_SIZE])
{
    memset(buffer, 0, UAVCAN_NODE_STATUS_MESSAGE_SIZE);

    static uint32_t started_at_sec = 0;
    if (started_at_sec == 0)
    {
        started_at_sec = (uint32_t)(getMonotonicTimestampUSec() / 1000000U);
    }

    const uint32_t uptime_sec = (uint32_t)((getMonotonicTimestampUSec() / 1000000U) - started_at_sec);

    /*
     * Here we're using the helper for demonstrational purposes; in this simple case it could be preferred to
     * encode the values manually.
     */
    canardEncodeScalar(buffer,  0, 32, &uptime_sec);
    canardEncodeScalar(buffer, 32,  2, &node_health);
    canardEncodeScalar(buffer, 34,  3, &node_mode);
}

static void processTxRxOnce(int32_t timeout_msec)
{
	HAL_StatusTypeDef status;
    char str[128];
    // Transmitting
    for (const CanardCANFrame* txf = NULL; (txf = canardPeekTxQueue(&canard)) != NULL;)
    {
    	CanHandle.pTxMsg->IDE = CAN_ID_EXT;
    	CanHandle.pTxMsg->DLC = txf->data_len;
    	CanHandle.pTxMsg->ExtId = txf->id;
    	CanHandle.pTxMsg->RTR = CAN_RTR_DATA;
    	for(unsigned i = 0; i < 8; ++i)
    	{
    		CanHandle.pTxMsg->Data[i] = txf->data[i];
    	}
    	status = HAL_CAN_Transmit(&CanHandle, timeout_msec);
        const int16_t tx_res = status == HAL_OK;
        if (tx_res < 0)         // Failure - drop the frame and report
        {
            canardPopTxQueue(&canard);
            DEBUG("Transmit error ??, frame dropped");
        }
        else if (tx_res > 0)    // Success - just drop the frame
        {
    		snprintf(str, 127, "TX: %x %lu [%lu %lu %lu %lu %lu %lu %lu %lu]", CanHandle.pTxMsg->ExtId,
    				CanHandle.pTxMsg->IDE,
					CanHandle.pTxMsg->Data[0],
					CanHandle.pTxMsg->Data[1],
					CanHandle.pTxMsg->Data[2],
					CanHandle.pTxMsg->Data[3],
					CanHandle.pTxMsg->Data[4],
					CanHandle.pTxMsg->Data[5],
					CanHandle.pTxMsg->Data[6],
					CanHandle.pTxMsg->Data[7]);
//    		DEBUG(str);
            canardPopTxQueue(&canard);
        }
        else                    // Timeout - just exit and try again later
        {
            break;
        }
    }

    CanardCANFrame rx_frame;
    const uint64_t timestamp = getMonotonicTimestampUSec();

	status = HAL_CAN_Receive(&CanHandle, CAN_FIFO0, 1000);
    const int16_t rx_res = status == HAL_OK;
    if (rx_res < 0)             // Failure - report
    {
        DEBUG("RECEIVE error ??, frame dropped");
    }
    else if (rx_res > 0)        // Success - process the frame
    {
    	for(unsigned i = 0; i < 8; ++i)
    	{
    		rx_frame.data[i] = CanHandle.pRxMsg->Data[i];
    	}
    	rx_frame.data_len = CanHandle.pRxMsg->DLC;
    	if(CanHandle.pRxMsg->IDE == CAN_ID_EXT)
    	{
    		rx_frame.id = CanHandle.pRxMsg->ExtId;
    		rx_frame.id |= CANARD_CAN_FRAME_EFF;
    	}
    	else
    	{
    		rx_frame.id = CanHandle.pRxMsg->StdId;
    	}
    	if(CanHandle.pRxMsg->RTR)
    	{
    		rx_frame.id |= CANARD_CAN_FRAME_RTR;
    	}

//        canardHandleRxFrame(&canard, &rx_frame, timestamp);

        DEBUG_NO_NEWLINE("%08x ", CanHandle.pRxMsg->ExtId);
        for(int i = 0; i < CanHandle.pRxMsg->DLC; ++i)
        {
        	DEBUG_NO_NEWLINE("%02x ", CanHandle.pRxMsg->Data[i]);
        }
        DEBUG("");

    }
    else
    {
        DEBUG("RECEIVE timeout");                       // Timeout - nothing to do
    }
}

void sbl_canardInit()
{
	canardInit(&canard, canard_memory_pool, sizeof(canard_memory_pool),
			can_on_transfer_received, shouldAcceptTransfer, NULL);

	canardSetLocalNodeID(&canard, 54);


	uint64_t next_1hz_service_at = getMonotonicTimestampUSec();

	for (;;)
	{
		processTxRxOnce(10);

		const uint64_t ts = getMonotonicTimestampUSec();

		if (ts >= next_1hz_service_at)
		{
			next_1hz_service_at += 1000000;
			process1HzTasks(ts);
		}
	}
}

static void process1HzTasks(uint64_t timestamp_usec)
{
    /*
     * Purging transfers that are no longer transmitted. This will occasionally free up some memory.
     */
    canardCleanupStaleTransfers(&canard, timestamp_usec);

    /*
     * Printing the memory usage statistics.
     */
    {
        const CanardPoolAllocatorStatistics stats = canardGetPoolAllocatorStatistics(&canard);
        const uint16_t peak_percent = (uint16_t)(100U * stats.peak_usage_blocks / stats.capacity_blocks);

        printf("Memory pool stats: capacity %u blocks, usage %u blocks, peak usage %u blocks (%u%%)\n",
               stats.capacity_blocks, stats.current_usage_blocks, stats.peak_usage_blocks, peak_percent);

        /*
         * The recommended way to establish the minimal size of the memory pool is to stress-test the application and
         * record the worst case memory usage.
         */
        if (peak_percent > 70)
        {
            puts("WARNING: ENLARGE MEMORY POOL");
        }
    }

    /*
     * Transmitting the node status message periodically.
     */
    {
        uint8_t buffer[UAVCAN_NODE_STATUS_MESSAGE_SIZE];
        makeNodeStatusMessage(buffer);

        static uint8_t transfer_id;  // Note that the transfer ID variable MUST BE STATIC (or heap-allocated)!

        const int16_t bc_res = canardBroadcast(&canard,
                                               UAVCAN_NODE_STATUS_DATA_TYPE_SIGNATURE,
                                               UAVCAN_NODE_STATUS_DATA_TYPE_ID,
                                               &transfer_id,
                                               CANARD_TRANSFER_PRIORITY_LOW,
                                               buffer,
                                               UAVCAN_NODE_STATUS_MESSAGE_SIZE);
        if (bc_res <= 0)
        {
            (void)fprintf(stderr, "Could not broadcast node status; error %d\n", bc_res);
        }
    }

    node_mode = UAVCAN_NODE_MODE_OPERATIONAL;
}

