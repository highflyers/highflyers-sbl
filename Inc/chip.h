/*
 * defines.h
 *
 *  Created on: 03.01.2018
 *      Author: Dell Latitude E5450
 */

#ifndef CHIP_H_
#define CHIP_H_

#include "stm32f4xx.h"  // HAL header file

#define STM32F4XX
#define STM32_PCLK1           (36000000ul)          // 36 MHz
#define STM32_TIMCLK1         (72000000ul)          // 72 MHz
#define CAN1_TX_IRQHandler    CAN1_TX_IRQHandler
#define CAN1_RX0_IRQHandler   CAN1_RX0_IRQHandler
#define CAN1_RX1_IRQHandler   CAN1_RX1_IRQHandler

#define UAVCAN_STM32_BAREMETAL 1
#define UAVCAN_STM32_TIMER_NUMBER 5
#define UAVCAN_STM32_NUM_IFACES 1

#endif /* CHIP_H_ */
