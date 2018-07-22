/**
 ******************************************************************************
 * File Name          : main.c
 * Description        : Main program body
 ******************************************************************************
 * This notice applies to any and all portions of this file
 * that are not between comment pairs USER CODE BEGIN and
 * USER CODE END. Other portions of this file, whether
 * inserted by the user or by software development tools
 * are owned by their respective copyright owners.
 *
 * Copyright (c) 2018 STMicroelectronics International N.V.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted, provided that the following conditions are met:
 *
 * 1. Redistribution of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of STMicroelectronics nor the names of other
 *    contributors to this software may be used to endorse or promote products
 *    derived from this software without specific written permission.
 * 4. This software, including modifications and/or derivative works of this
 *    software, must execute solely and exclusively on microcontroller or
 *    microprocessor devices manufactured by or for STMicroelectronics.
 * 5. Redistribution and use of this software other than as permitted under
 *    this license is void and will automatically terminate your rights under
 *    this license.
 *
 * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
 * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
 * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT
 * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"
#include "fatfs.h"

/* USER CODE BEGIN Includes */
#include <gps.h>
#include <Pos_check.h>
#include <sd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <can.h>
#include <debug.h>
#include "defines.h"
#include "stm32fxxx_hal.h"
#include "tm_stm32_gps.h"
#include "tm_stm32_delay.h"

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
SD_HandleTypeDef hsd;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim9;
TIM_HandleTypeDef htim12;

CAN_HandleTypeDef CanHandle;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
typedef struct {
	TIM_HandleTypeDef *timer;
	uint32_t channel;
	uint16_t pulse;
	uint16_t pulse_min;
	uint16_t pulse_max;
	uint16_t pulse_fs_value;
} Pwm_output;

uint8_t Received[38];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SDIO_SD_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM12_Init(void);
static void MX_TIM9_Init(void);
static void MX_TIM1_Init(void);
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
static void initOUTPUTS(void);
static void change_PWM(Pwm_output output, uint16_t pulse);
static void setRGB(uint8_t state);
void termination(void);
void update_inputs(void);

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
int operatorAL = 5;
int operatorAR = -5;
int operatorE = 2;
int operatorR = -3;
int operatorM = 1;
int operatorJ = -6;
int pulse_print;

Pwm_output LED_R = { &htim4, TIM_CHANNEL_2, 0, 0, 999, 0 };
Pwm_output LED_G = { &htim4, TIM_CHANNEL_3, 0, 0, 999, 0 };
Pwm_output LED_B = { &htim4, TIM_CHANNEL_4, 0, 0, 999, 0 };

Pwm_output Aileron_R = { &htim3, TIM_CHANNEL_3, 75, 70, 90, 50 }; //PWM7
Pwm_output Aileron_L = { &htim3, TIM_CHANNEL_4, 75, 70, 90, 50 };
Pwm_output Elevator = { &htim3, TIM_CHANNEL_2, 75, 60, 100, 50 };
Pwm_output Rudder = { &htim12, TIM_CHANNEL_1, 75, 60, 90, 50 };
Pwm_output Motor1 = { &htim12, TIM_CHANNEL_2, 75, 70, 100, 50 };
Pwm_output Motor2 = { &htim4, TIM_CHANNEL_1, 75, 70, 100, 50 };
Pwm_output Motor3 = { &htim9, TIM_CHANNEL_1, 75, 70, 100, 50 };
Pwm_output Motor4 = { &htim9, TIM_CHANNEL_2, 75, 70, 100, 50 };
Pwm_output Joint1 = { &htim1, TIM_CHANNEL_2, 75, 50, 100, 50 };
Pwm_output Joint2 = { &htim1, TIM_CHANNEL_3, 75, 50, 100, 50 };
/* USER CODE END 0 */

int main(void) {

	/* USER CODE BEGIN 1 */
	Point_pos actpos;
	Point_pos *pointpos;
	BoundingBox boxborders;
	int nop = 0, ret = 2, hdop_counter = 0, n = 0;
	float hdop = 0;

	/* USER CODE END 1 */

	/* MCU Configuration----------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_SDIO_SD_Init();
	MX_TIM1_Init();
	MX_TIM3_Init();
	MX_TIM4_Init();
	MX_TIM9_Init();
	MX_TIM12_Init();
	MX_USART2_UART_Init();
	MX_FATFS_Init();

	can_init(&hCan);
	sbl_canardInit();

	HAL_UART_Receive_IT(&huart2, Received, 38);
	/* USER CODE BEGIN 2 */
	setRGB(INIT);
	transmit_info("wartosci poczatkowe na wyjsciach", EMPTY);
	initOUTPUTS();
	GPS_init();
	HAL_Delay(1000);
	transmit_info("odczyt z karty", EMPTY);
	SD_dataread(&pointpos, &boxborders, &nop);
	transmit_info("oczekiwanie na GPS", EMPTY);
	while (hdop <= HDOP_MIN || hdop >= HDOP_MAX) {
		HAL_Delay(10);
		hdop = GPS_update(&actpos);
		transmit_info("hdop", hdop);
	}
	TM_GENERAL_DisableInterrupts();
	ret = IsInPolygon(&actpos, pointpos, &boxborders, nop);
	TM_GENERAL_EnableInterrupts();
	transmit_info("Sprawdzenie pozycji poczatkowej, wynik = ", (float) ret);
	if (ret == 0) {
		transmit_info("Poza strefa", EMPTY);
		setRGB(FAULT);
		termination();
	}else{
		transmit_info("W strefie", EMPTY);
	}
	setRGB(OK);
	transmit_info("OK, main loop", EMPTY);
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		if(GPS_update(&actpos) == 1){
		transmit_info("hdop", actpos.HDOP);
		if ((actpos.HDOP <= HDOP_MIN || actpos.HDOP >= HDOP_MAX)
				& (hdop_counter >= HDOP_COUNT)) {
			transmit_info("funkcja terminacji - brak gps", EMPTY);
			setRGB(FAULT);
			termination();
		} else if ((actpos.HDOP <= HDOP_MIN || actpos.HDOP >= HDOP_MAX)
				& (hdop_counter < HDOP_COUNT)) {
			hdop_counter++;
			transmit_info("chwilowy brak sygnalu", (float) hdop_counter);
		} else {
			hdop_counter = 0;
			TM_GENERAL_DisableInterrupts();
			ret = IsInPolygon(&actpos, pointpos, &boxborders, nop);
			TM_GENERAL_EnableInterrupts();
			transmit_info("sprawdzenie pozycji", (float) ret);
			if (ret == 0) {
				transmit_info("funkcja terminacji - poza strefa", EMPTY);
				setRGB(FAULT);
				termination();
			}
		}
		TM_GENERAL_DisableInterrupts();
		update_inputs();
		TM_GENERAL_EnableInterrupts();
		}
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
		HAL_Delay(10);
	}
	/* USER CODE END 3 */

}






/** System Clock Configuration
 */
void SystemClock_Config(void) {

	RCC_OscInitTypeDef RCC_OscInitStruct;
	RCC_ClkInitTypeDef RCC_ClkInitStruct;

	/**Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE()
	;

	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

	/**Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 12;
	RCC_OscInitStruct.PLL.PLLN = 72;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 3;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	/**Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	/**Configure the Systick interrupt time
	 */
	HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000);

	/**Configure the Systick
	 */
	HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

	/* SysTick_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* SDIO init function */
static void MX_SDIO_SD_Init(void) {

	GPIO_InitTypeDef g;
	g.Mode = GPIO_MODE_AF_PP;
	g.Pull = GPIO_PULLUP;
	g.Speed = GPIO_SPEED_FREQ_HIGH;

	g.Alternate = GPIO_AF12_SDIO;
	g.Pin = GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12;
	HAL_GPIO_Init(GPIOC, &g);

	g.Alternate = GPIO_AF12_SDIO;
	g.Pin = GPIO_PIN_2;
	HAL_GPIO_Init(GPIOD, &g);

	hsd.Instance = SDIO;
	hsd.Init.BusWide = SDIO_BUS_WIDE_1B;
	hsd.Init.ClockBypass = SDIO_CLOCK_BYPASS_DISABLE;
	hsd.Init.ClockDiv = SDIO_INIT_CLK_DIV;
	hsd.Init.ClockEdge = SDIO_CLOCK_EDGE_RISING;
	hsd.Init.ClockPowerSave = SDIO_CLOCK_POWER_SAVE_DISABLE;
	hsd.Init.HardwareFlowControl = SDIO_HARDWARE_FLOW_CONTROL_DISABLE;

	HAL_SD_Init(&hsd);
}

/* TIM1 init function */
static void MX_TIM1_Init(void) {

	TIM_MasterConfigTypeDef sMasterConfig;
	TIM_OC_InitTypeDef sConfigOC;
	TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig;

	htim1.Instance = TIM1;
	htim1.Init.Prescaler = PWM_PRESCALER;
	htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim1.Init.Period = PWM_PERIOD;
	htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim1.Init.RepetitionCounter = 0;
	if (HAL_TIM_PWM_Init(&htim1) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig)
			!= HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
	sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
	if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2)
			!= HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3)
			!= HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
	sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
	sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
	sBreakDeadTimeConfig.DeadTime = 0;
	sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
	sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
	sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
	if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig)
			!= HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	HAL_TIM_MspPostInit(&htim1);

}

/* TIM3 init function */
static void MX_TIM3_Init(void) {

	TIM_MasterConfigTypeDef sMasterConfig;
	TIM_OC_InitTypeDef sConfigOC;

	htim3.Instance = TIM3;
	htim3.Init.Prescaler = PWM_PRESCALER;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = PWM_PERIOD;
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	if (HAL_TIM_PWM_Init(&htim3) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig)
			!= HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 75;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3)
			!= HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	HAL_TIM_MspPostInit(&htim3);

}

/* TIM4 init function */
static void MX_TIM4_Init(void) {

	TIM_MasterConfigTypeDef sMasterConfig;
	TIM_OC_InitTypeDef sConfigOC;

	htim4.Instance = TIM4;
	htim4.Init.Prescaler = PWM_PRESCALER;
	htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim4.Init.Period = PWM_PERIOD;
	htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	if (HAL_TIM_PWM_Init(&htim4) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig)
			!= HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2)
			!= HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3)
			!= HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4)
			!= HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	HAL_TIM_MspPostInit(&htim4);

}

/* TIM9 init function */
static void MX_TIM9_Init(void) {

	TIM_OC_InitTypeDef sConfigOC;

	htim9.Instance = TIM9;
	htim9.Init.Prescaler = PWM_PRESCALER;
	htim9.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim9.Init.Period = PWM_PERIOD;
	htim9.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	if (HAL_TIM_PWM_Init(&htim9) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim9, &sConfigOC, TIM_CHANNEL_1)
			!= HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	if (HAL_TIM_PWM_ConfigChannel(&htim9, &sConfigOC, TIM_CHANNEL_2)
			!= HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	HAL_TIM_MspPostInit(&htim9);

}

/* TIM12 init function */
static void MX_TIM12_Init(void) {

	TIM_OC_InitTypeDef sConfigOC;

	htim12.Instance = TIM12;
	htim12.Init.Prescaler = PWM_PRESCALER;
	htim12.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim12.Init.Period = PWM_PERIOD;
	htim12.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	if (HAL_TIM_PWM_Init(&htim12) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim12, &sConfigOC, TIM_CHANNEL_1)
			!= HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	if (HAL_TIM_PWM_ConfigChannel(&htim12, &sConfigOC, TIM_CHANNEL_2)
			!= HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	HAL_TIM_MspPostInit(&htim12);

}

/* USART2 init function */
static void MX_USART2_UART_Init(void) {

	huart2.Instance = USART2;
	huart2.Init.BaudRate = 115200;
	huart2.Init.WordLength = UART_WORDLENGTH_8B;
	huart2.Init.StopBits = UART_STOPBITS_1;
	huart2.Init.Parity = UART_PARITY_NONE;
	huart2.Init.Mode = UART_MODE_TX_RX;
	huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart2.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart2) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}
}

/** Configure pins as 
 * Analog
 * Input
 * Output
 * EVENT_OUT
 * EXTI
 */
static void MX_GPIO_Init(void) {

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOH_CLK_ENABLE()
	;
	__HAL_RCC_GPIOA_CLK_ENABLE()
	;
	__HAL_RCC_GPIOB_CLK_ENABLE()
	;
	__HAL_RCC_GPIOD_CLK_ENABLE()
	;
	__HAL_RCC_GPIOC_CLK_ENABLE()
	;
	__HAL_RCC_GPIOE_CLK_ENABLE()
	;
}

/* USER CODE BEGIN 4 */
void initOUTPUTS(void) {
	change_PWM(Aileron_L, Aileron_L.pulse);
	change_PWM(Aileron_R, Aileron_R.pulse);
	change_PWM(Elevator, Elevator.pulse);
	change_PWM(Rudder, Rudder.pulse);
	change_PWM(Motor1, Motor1.pulse);
	change_PWM(Motor2, Motor2.pulse);
	change_PWM(Motor3, Motor3.pulse);
	change_PWM(Motor4, Motor4.pulse);
	change_PWM(Joint1, Joint1.pulse);
	change_PWM(Joint2, Joint2.pulse);
}

void update_inputs(void) {
	if (Aileron_L.pulse >= 100) {
		operatorAL = -5;
	} else if (Aileron_L.pulse <= 50) {
		operatorAL = 5;
	}
	Aileron_L.pulse = Aileron_L.pulse + operatorAL;

	if (Aileron_R.pulse >= 100) {
		operatorAR = -5;
	} else if (Aileron_R.pulse <= 50) {
		operatorAR = 5;
	}
	Aileron_R.pulse = Aileron_R.pulse + operatorAR;

	if (Elevator.pulse >= 100) {
		operatorE = -2;
	} else if (Elevator.pulse <= 50) {
		operatorE = 2;
	} else
		Elevator.pulse = Elevator.pulse + operatorE;

	if (Rudder.pulse >= 100) {
		operatorR = -3;
	} else if (Rudder.pulse <= 50) {
		operatorR = 3;
	}
	Rudder.pulse = Rudder.pulse + operatorR;

	if (Motor1.pulse >= 100) {
		operatorM = -1;
	} else if (Motor1.pulse <= 50) {
		operatorM = 1;
	}
	Motor1.pulse = Motor1.pulse + operatorM;

	if (Motor2.pulse >= 100) {
		operatorM = -1;
	} else if (Motor2.pulse <= 50) {
		operatorM = 1;
	}
	Motor2.pulse = Motor2.pulse + operatorM;

	if (Motor3.pulse >= 100) {
		operatorM = -1;
	} else if (Motor3.pulse <= 50) {
		operatorM = 1;
	}
	Motor3.pulse = Motor3.pulse + operatorM;

	if (Motor4.pulse >= 100) {
		operatorM = -1;
	} else if (Motor4.pulse <= 50) {
		operatorM = 1;
	}
	Motor4.pulse = Motor4.pulse + operatorM;

	if (Joint1.pulse >= 100) {
		operatorJ = -6;
	} else if (Joint1.pulse <= 50) {
		operatorJ = 6;
	}
	Joint1.pulse = Joint1.pulse + operatorJ;

	if (Joint2.pulse >= 100) {
		operatorJ = -6;
	} else if (Joint2.pulse <= 50) {
		operatorJ = 6;
	}
	Joint2.pulse = Joint2.pulse + operatorJ;

	transmit_info("WEJSCIA", EMPTY);
	transmit_info("Aileron_L.pulse", Aileron_L.pulse);
	transmit_info("Aileron_R.pulse", Aileron_R.pulse);
	transmit_info("Elevator.pulse", Elevator.pulse);
	transmit_info("Rudder.pulse", Rudder.pulse);
	transmit_info("Motor1.pulse", Motor1.pulse);
	transmit_info("Motor2.pulse", Motor2.pulse);
	transmit_info("Motor3.pulse", Motor3.pulse);
	transmit_info("Motor4.pulse", Motor4.pulse);
	transmit_info("Joint1.pulse", Joint1.pulse);
	transmit_info("Joint2.pulse", Joint2.pulse);

	transmit_info("WYJSCIA", EMPTY);
	change_PWM(Aileron_L, Aileron_L.pulse);
	transmit_info("Aileron_L.pulse", pulse_print);
	change_PWM(Aileron_R, Aileron_R.pulse);
	transmit_info("Aileron_R.pulse", pulse_print);
	change_PWM(Elevator, Elevator.pulse);
	transmit_info("Elevator.pulse", pulse_print);
	change_PWM(Rudder, Rudder.pulse);
	transmit_info("Rudder.pulse", pulse_print);
	change_PWM(Motor1, Motor1.pulse);
	transmit_info("Motor1.pulse", pulse_print);
	change_PWM(Motor2, Motor2.pulse);
	transmit_info("Motor2.pulse", pulse_print);
	change_PWM(Motor3, Motor3.pulse);
	transmit_info("Motor3.pulse", pulse_print);
	change_PWM(Motor4, Motor4.pulse);
	transmit_info("Motor4.pulse", pulse_print);
	change_PWM(Joint1, Joint1.pulse);
	transmit_info("Joint1.pulse", pulse_print);
	change_PWM(Joint2, Joint2.pulse);
	transmit_info("Joint2.pulse", pulse_print);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
//	uint8_t Data[40];
	char korektor[] = " ;\n\r";
	char * dat_buff;
	float dat_buff_f[10];

	//przepisanie danych z bufora do struktury
	for (int i = 0; i < 10; i++) {
		if (i == 0) {
			dat_buff = strtok(&Received, korektor);
		} else {
			dat_buff = strtok(NULL, korektor);
		}
		dat_buff_f[i] = atof(dat_buff);
	}

	//Porownanie nowych wartoœci z poprzednimi i zmiana
	if (dat_buff_f[0] != Aileron_L.pulse) {
		Aileron_L.pulse = dat_buff_f[0];
		change_PWM(Aileron_L, Aileron_L.pulse);
	}
	if (dat_buff_f[1] != Aileron_R.pulse) {
		Aileron_R.pulse = dat_buff_f[1];
		change_PWM(Aileron_R, Aileron_R.pulse);
	}
	if (dat_buff_f[2] != Elevator.pulse) {
		Elevator.pulse = dat_buff_f[2];
		change_PWM(Elevator, Elevator.pulse);
	}
	if (dat_buff_f[3] != Rudder.pulse) {
		Rudder.pulse = dat_buff_f[3];
		change_PWM(Rudder, Rudder.pulse);
	}
	if (dat_buff_f[4] != Motor1.pulse) {
		Motor1.pulse = dat_buff_f[4];
		change_PWM(Motor1, Motor1.pulse);
	}
	if (dat_buff_f[5] != Motor2.pulse) {
		Motor2.pulse = dat_buff_f[5];
		change_PWM(Motor2, Motor2.pulse);
	}
	if (dat_buff_f[6] != Motor3.pulse) {
		Motor3.pulse = dat_buff_f[6];
		change_PWM(Motor3, Motor3.pulse);
	}
	if (dat_buff_f[7] != Motor4.pulse) {
		Motor4.pulse = dat_buff_f[7];
		change_PWM(Motor4, Motor4.pulse);
	}
	if (dat_buff_f[8] != Joint1.pulse) {
		Joint1.pulse = dat_buff_f[8];
		change_PWM(Joint1, Joint1.pulse);
	}
	if (dat_buff_f[9] != Joint2.pulse) {
		Joint2.pulse = dat_buff_f[9];
		change_PWM(Joint2, Joint2.pulse);
	}
	transmit_info("Aileron_L.pulse", Aileron_L.pulse);
	transmit_info("Aileron_R.pulse", Aileron_R.pulse);
	transmit_info("Elevator.pulse", Elevator.pulse);
	transmit_info("Rudder.pulse", Rudder.pulse);
	transmit_info("Motor1.pulse", Motor1.pulse);
	transmit_info("Motor2.pulse", Motor2.pulse);
	transmit_info("Motor3.pulse", Motor3.pulse);
	transmit_info("Motor4.pulse", Motor4.pulse);
	transmit_info("Joint1.pulse", Joint1.pulse);
	transmit_info("Joint2.pulse", Joint2.pulse);
	// Ponowne w³¹czenie nas³uchiwania
	HAL_UART_Receive_IT(&huart2, Received, 38);
}

//zmiana parametrów sygna³u PWM
void change_PWM(Pwm_output output, uint16_t pulse) {
	HAL_TIM_PWM_Stop(output.timer, output.channel);
	TIM_OC_InitTypeDef sConfigOC;
	(*output.timer).Init.Period = PWM_PERIOD;
	HAL_TIM_PWM_Init(output.timer);
	sConfigOC.OCMode = TIM_OCMODE_PWM1;

	//sprawdzenie ograniczeñ kana³u i korekcja
	if (pulse > output.pulse_max) {
		pulse = output.pulse_max;
	} else if (pulse < output.pulse_min) {
		pulse = output.pulse_min;
	}

	//do wysy³ania danych testowych
	pulse_print = pulse;
	sConfigOC.Pulse = pulse;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	HAL_TIM_PWM_ConfigChannel(output.timer, &sConfigOC, output.channel);
	HAL_TIM_PWM_Start(output.timer, output.channel);
}

//sygnalizacja diod¹ RGB stanu BSP
void setRGB(uint8_t state) {
	switch (state) {
	case INIT:
		change_PWM(LED_R, R_RGB_INIT);
		change_PWM(LED_G, G_RGB_INIT);
		change_PWM(LED_B, B_RGB_INIT);
		break;
	case OK:
		change_PWM(LED_R, R_RGB_OK);
		change_PWM(LED_G, G_RGB_OK);
		change_PWM(LED_B, B_RGB_OK);
		break;
	case FAULT:
		change_PWM(LED_R, R_RGB_FAULT);
		change_PWM(LED_G, G_RGB_FAULT);
		change_PWM(LED_B, B_RGB_FAULT);
		break;
	}
}

//ustawienie wartoœci sterów w pozycjach failsafe
void termination(void) {
	change_PWM(Aileron_L, Aileron_L.pulse_fs_value);
	change_PWM(Aileron_R, Aileron_R.pulse_fs_value);
	change_PWM(Elevator, Elevator.pulse_fs_value);
	change_PWM(Rudder, Rudder.pulse_fs_value);
	change_PWM(Motor1, Motor1.pulse_fs_value);
	change_PWM(Motor2, Motor2.pulse_fs_value);
	change_PWM(Motor3, Motor3.pulse_fs_value);
	change_PWM(Motor4, Motor4.pulse_fs_value);
	change_PWM(Joint1, Joint1.pulse_fs_value);
	change_PWM(Joint2, Joint2.pulse_fs_value);
	while (1) {
		HAL_Delay(1000);
	}
}
/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @param  None
 * @retval None
 */
void _Error_Handler(char * file, int line) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	while (1) {
	}
	/* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT

/**
 * @brief Reports the name of the source file and the source line number
 * where the assert_param error has occurred.
 * @param file: pointer to the source file name
 * @param line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t* file, uint32_t line)
{
	/* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
	 ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/* USER CODE END 6 */

}

#endif

/**
 * @}
 */

/**
 * @}
 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
