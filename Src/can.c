#include <can.h>
#include <debug.h>

extern CAN_HandleTypeDef CanHandle;

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
	while (1)
	{
		HAL_CAN_Receive(&CanHandle, CAN_FIFO0, 1000);
			snprintf(str, 127, "rec: %x %lu [%lu %lu %lu %lu %lu %lu %lu %lu]", CanHandle.pRxMsg->ExtId,
					CanHandle.pRxMsg->IDE, CanHandle.pRxMsg->Data[0], CanHandle.pRxMsg->Data[1], CanHandle.pRxMsg->Data[2], CanHandle.pRxMsg->Data[3], CanHandle.pRxMsg->Data[4], CanHandle.pRxMsg->Data[5], CanHandle.pRxMsg->Data[6], CanHandle.pRxMsg->Data[7]);
			DEBUG(str);

	}

}
