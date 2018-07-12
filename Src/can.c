#include <can.h>
#include <debug.h>

void can_init(CAN_HandleTypeDef *can)
{
	HAL_StatusTypeDef status;
	char str[128];
	DEBUG(__FUNCTION__);
	__HAL_RCC_CAN1_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();
	GPIO_InitTypeDef g;
	g.Alternate = GPIO_AF9_CAN1;
	g.Mode = GPIO_MODE_AF_OD;
	g.Pin = GPIO_PIN_0;
	g.Pull = GPIO_PULLUP;
	g.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	HAL_GPIO_Init(GPIOD, &g);
	g.Pin = GPIO_PIN_1;
	g.Mode = GPIO_MODE_AF_PP;
	g.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOD, &g);

	can->Instance = CAN1;
	can->Init.ABOM = DISABLE;
	can->Init.AWUM = DISABLE;
	can->Init.BS1 = CAN_BS1_6TQ;
	can->Init.BS2 = CAN_BS2_1TQ;
	can->Init.Mode = CAN_MODE_NORMAL;
	can->Init.NART = DISABLE;
	can->Init.Prescaler = 9;
	can->Init.RFLM = DISABLE;
	can->Init.SJW = CAN_SJW_1TQ;
	can->Init.TTCM = DISABLE;
	can->Init.TXFP = DISABLE;
	status = HAL_CAN_Init(can);
//	CAN_FilterConfTypeDef filterConf;
//	filterConf.BankNumber = 0;
//	filterConf.FilterActivation = ENABLE;
//	filterConf.FilterFIFOAssignment = CAN_FIFO0;
//	filterConf.FilterIdHigh = 0xFFFF;
//	filterConf.FilterIdLow = 0xFFFF;
//	filterConf.FilterMaskIdHigh = 0xFFFF;
//	filterConf.FilterMaskIdLow = 0xFFFF;
//	filterConf.FilterMode = CAN_FILTERMODE_IDMASK;
//	filterConf.FilterNumber = 0;
//	filterConf.FilterScale = CAN_FILTERSCALE_32BIT;
//	HAL_CAN_ConfigFilter(can, &filterConf);
	snprintf(str, 127, "init: %d", status);
	DEBUG(str);
	while (1)
	{
		status = HAL_CAN_Receive(can, CAN_FIFO0, 500);
		snprintf(str, 127, "Received: %d %lu %lu %lu", status, can->pRxMsg->StdId, can->pRxMsg->DLC, can->pRxMsg->IDE);
		DEBUG(str);
	}
}
