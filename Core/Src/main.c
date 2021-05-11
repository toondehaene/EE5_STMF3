/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define FIRST_DAMAGE_TH 10 //this is percent
#define SECOND_DAMAGE_TH 50
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
CAN_FilterTypeDef sFilterConfig;
CAN_RxHeaderTypeDef RxHeader;
peripheral my_ID = DASHBOARD;
uint8_t Rbuffer[8], DEBUGbuffer[8];
car_telemetry_packet my_car_telemetry_packet;
lap_data_packet my_lap_data_packet;
session_packet my_session_packet;
car_status_packet my_car_status_packet;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
uint8_t getdamageColorRG(int damage);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

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
	MX_CAN_Init();
	MX_USART1_UART_Init();
	/* USER CODE BEGIN 2 */

	sFilterConfig.FilterActivation = ENABLE;
	sFilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
	sFilterConfig.FilterIdHigh = 0x0; //line up the higher ID for a compare (11 bits and 16 bits give 5 bit shift)
//  sFilterConfig.FilterIdHigh = (car_telemetryID<<5|DASHBOARD)<<5;	//line up the higher ID for a compare (11 bits and 16 bits give 5 bit shift)
	sFilterConfig.FilterIdLow = 0x0;
//	sFilterConfig.FilterMaskIdHigh = 0xFFFF<<5;	//0000001111100000
	sFilterConfig.FilterMaskIdHigh = 0x0000;	//0000001111100000
	sFilterConfig.FilterMaskIdLow = 0x00;
	sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
	sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
	sFilterConfig.FilterBank = 0;

	HAL_CAN_ConfigFilter(&hcan, &sFilterConfig);
	HAL_CAN_Start(&hcan);
	HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING); //last argument is mask of activated ITs just OR| them together

	// UART DASHBOARD TEMPLATES:
	uint8_t speed[8] = { 0x5A, 0xA5, 0x05, 0x82, 0x00, 0x01, 0x00, 0x00 };
	uint8_t throttle[20] = { 0x5A, 0xA5, 0x17, 0x82, 0x00, 0x10, 0x00, 0x04,
			0x00, 0x01, 0x00, 0x4C, 0x00, 0x1E, 0x00, 0x6F, 0x00, 0xAA, 0x00,
			0x00 };
	uint8_t brake[20] = { 0x5A, 0xA5, 0x17, 0x82, 0x00, 0x11, 0x00, 0x04, 0x00,
			0x01, 0x00, 0x0E, 0x00, 0x18, 0x00, 0x33, 0x00, 0xAB, 0x00, 0x00 };
	uint8_t gear[8] = { 0x5A, 0xA5, 0x05, 0x82, 0x00, 0x03, 0x00, 0x00 };
	uint8_t currentLapTime[8] =
			{ 0x5A, 0xA5, 0x05, 0x82, 0x00, 0x02, 0x00, 0x00 };
	uint8_t carPosition[8] = { 0x5A, 0xA5, 0x05, 0x82, 0x00, 0x06, 0x00, 0x00 };
	uint8_t currentLapNum[8] =
			{ 0x5A, 0xA5, 0x05, 0x82, 0x00, 0x06, 0x00, 0x00 };
	uint8_t totalLaps[8] = { 0x5A, 0xA5, 0x05, 0x82, 0x00, 0x05, 0x00, 0x00 };
	uint8_t frontLeftWingDamage[20] = { 0x5A, 0xA5, 0x17, 0x82, 0x00, 0x13,
			0x00, 0x04, 0x00, 0x01, 0x00, 0x88, 0x00, 0xE7, 0x00, 0x9D, 0x01,
			0x04, 0xF0, 0x00 };
	uint8_t frontRightWingDamage[20] = { 0x5A, 0xA5, 0x17, 0x82, 0x00, 0x12,
			0x00, 0x04, 0x00, 0x01, 0x00, 0x88, 0x00, 0xB6, 0x00, 0x9D, 0x00,
			0xD3, 0xF0, 0x00 };
	uint8_t rearWingDamage[20] = { 0x5A, 0xA5, 0x17, 0x82, 0x00, 0x14, 0x00,
			0x04, 0x00, 0x01, 0x01, 0x55, 0x00, 0xCB, 0x01, 0x6A, 0x00, 0xEA,
			0xF0, 0x00 }
	；
	uint8_t engineDamage[20] = { 0x5A, 0xA5, 0x17, 0x82, 0x00, 0x15, 0x00, 0x04,
			0x00, 0x01, 0x00, 0xEE, 0x00, 0xC9, 0x01, 0x2A, 0x00, 0xF0, 0xF0,
			0x00 };
	uint8_t FRTyre[20] = { 0x5A, 0xA5, 0x17, 0x82, 0x00, 0x16, 0x00, 0x04, 0x00,
			0x01, 0x00, 0xA9, 0x00, 0xAE, 0x00, 0xC9, 0x00, 0xC4, 0xF0, 0x00 };
	uint8_t FLTyre[20] = { 0x5A, 0xA5, 0x17, 0x82, 0x00, 0x17, 0x00, 0x04, 0x00,
			0x01, 0x00, 0xAA, 0x00, 0xF8, 0x00, 0xC9, 0x01, 0x0C, 0xF0, 0x00 };
	uint8_t BRTyre[20] = { 0x5A, 0xA5, 0x17, 0x82, 0x00, 0x18, 0x00, 0x04, 0x00,
			0x01, 0x01, 0x46, 0x00, 0xAE, 0x01, 0x65, 0x00, 0xC4, 0xF0, 0x00 };
	uint8_t BLTyre[20] = { 0x5A, 0xA5, 0x17, 0x82, 0x00, 0x19, 0x00, 0x04, 0x00,
			0x01, 0x01, 0x46, 0x00, 0xF2, 0x01, 0x66, 0x01, 0x07, 0xF0, 0x00 };
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		/* USER CODE END WHILE */
		switch (my_ID) {

		case DASHBOARD: {
			//car_telemetry
			speed[6] = ((uint16_t) my_car_telemetry_packet.m_speed * 180 / 400)
					>> 8; //highest byte of a degree rotation where 400 kmh is 180 degrees
			speed[7] = ((uint16_t) my_car_telemetry_packet.m_speed * 180 / 400)
					& 0xFF; //lowest byte of a degree rotation where 400 kmh is 180 degrees
			HAL_UART_Transmit(&huart1, speed, sizeof(speed), 1000);

			throttle[17] = 30 + (my_car_telemetry_packet.throttle * 140 / 100);
			HAL_UART_Transmit(&huart1, throttle, sizeof(throttle), 1000);

			brake[17] = 30 + (my_car_telemetry_packet.brake * 140 / 100);
			HAL_UART_Transmit(&huart1, brake, sizeof(brake), 1000);

			gear[6] = ((uint16_t) my_car_telemetry_packet.gear) >> 8;
			gear[7] = ((uint16_t) my_car_telemetry_packet.gear) & 0xFF;
			HAL_UART_Transmit(&huart1, gear, sizeof(gear), 1000);

			//lap_data

			currentLapTime[6] = ((uint16_t) (my_lap_data_packet.m_currentLapTime
					* 100)) >> 8;
			currentLapTime[7] = ((uint16_t) (my_lap_data_packet.m_currentLapTime
					* 100)) & 0xFF;
			HAL_UART_Transmit(&huart1, currentLapTime, sizeof(currentLapTime),
					1000);

			carPosition[6] = (uint16_t) (my_lap_data_packet.m_carPosition) >> 8;
			carPosition[7] = (uint16_t) (my_lap_data_packet.m_carPosition)
					& 0xFF;
			HAL_UART_Transmit(&huart1, carPosition, sizeof(carPosition), 1000);

			currentLapNum[6] = (uint16_t) (my_lap_data_packet.m_currentLapNum)
					>> 8;
			currentLapNum[7] = (uint16_t) (my_lap_data_packet.m_currentLapNum)
					& 0xFF;
			HAL_UART_Transmit(&huart1, currentLapNum, sizeof(currentLapNum),
					1000);

			//session

			totalLaps[6] = (uint16_t) (my_session_packet.m_totalLaps) >> 8;
			totalLaps[7] = (uint16_t) (my_session_packet.m_totalLaps) & 0xFF;
			HAL_UART_Transmit(&huart1, totalLaps, sizeof(totalLaps), 1000);

			//car_status
			frontLeftWingDamage[18] = getdamageColorRG(
					my_car_status_packet.m_frontLeftWingDamage);
			HAL_UART_Transmit(&huart1, frontLeftWingDamage,
					sizeof(frontLeftWingDamage), 1000);

			frontRightWingDamage[18] = getdamageColorRG(
					my_car_status_packet.m_frontRightWingDamage);
			HAL_UART_Transmit(&huart1, frontRightWingDamage,
					sizeof(frontRightWingDamage), 1000);

			rearWingDamage[18] = getdamageColorRG(
					my_car_status_packet.m_rearWingDamage);
			HAL_UART_Transmit(&huart1, rearWingDamage, sizeof(rearWingDamage),
					1000);

			engineDamage[18] = getdamageColorRG(
					my_car_status_packet.m_engineDamage);
			HAL_UART_Transmit(&huart1, engineDamage, sizeof(engineDamage),
					1000);
			//tyres: TODO: check which index corresponds with wich tyre
			FRTyre[18] = getdamageColorRG(my_car_status_packet.m_tyresWear[0]);
			HAL_UART_Transmit(&huart1, FRTyre, sizeof(FRTyre), 1000);

			FLTyre[18] = getdamageColorRG(my_car_status_packet.m_tyresWear[1]);
			HAL_UART_Transmit(&huart1, FLTyre, sizeof(FLTyre), 1000);

			BRTyre[18] = getdamageColorRG(my_car_status_packet.m_tyresWear[2]);
			HAL_UART_Transmit(&huart1, BRTyre, sizeof(BRTyre), 1000);

			BLTyre[18] = getdamageColorRG(my_car_status_packet.m_tyresWear[3]);
			HAL_UART_Transmit(&huart1, BLTyre, sizeof(BLTyre), 1000);

		}
			break;
		case MIST_FAN:
			break;
		case COLOR_FLAGS:
			break;
		case BROADCAST:
			break;
		case LEDS:
			break;
//		HAL_Delay(100);
//		if(HAL_CAN_GetRxFifoFillLevel(&hcan, CAN_RX_FIFO0)!=0){
//			HAL_CAN_GetRxMessage(&hcan, CAN_RX_FIFO0, &RxHeader, DEBUGbuffer);
//		}
			//show the current speed in chunks of 10 on the output leds in binary form
			// example: speed is 125 kmh -> 12 % 16 -> 12 in binary on the leds
			/* USER CODE END 3 */
		}
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };
	RCC_PeriphCLKInitTypeDef PeriphClkInit = { 0 };

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
		Error_Handler();
	}
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
	PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief CAN Initialization Function
 * @param None
 * @retval None
 */
static void MX_CAN_Init(void) {

	/* USER CODE BEGIN CAN_Init 0 */

	/* USER CODE END CAN_Init 0 */

	/* USER CODE BEGIN CAN_Init 1 */

	/* USER CODE END CAN_Init 1 */
	hcan.Instance = CAN;
	hcan.Init.Prescaler = 21;
	hcan.Init.Mode = CAN_MODE_NORMAL;
	hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
	hcan.Init.TimeSeg1 = CAN_BS1_12TQ;
	hcan.Init.TimeSeg2 = CAN_BS2_4TQ;
	hcan.Init.TimeTriggeredMode = DISABLE;
	hcan.Init.AutoBusOff = DISABLE;
	hcan.Init.AutoWakeUp = DISABLE;
	hcan.Init.AutoRetransmission = DISABLE;
	hcan.Init.ReceiveFifoLocked = DISABLE;
	hcan.Init.TransmitFifoPriority = DISABLE;
	if (HAL_CAN_Init(&hcan) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN CAN_Init 2 */

	/* USER CODE END CAN_Init 2 */

}

/**
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART1_UART_Init(void) {

	/* USER CODE BEGIN USART1_Init 0 */

	/* USER CODE END USART1_Init 0 */

	/* USER CODE BEGIN USART1_Init 1 */

	/* USER CODE END USART1_Init 1 */
	huart1.Instance = USART1;
	huart1.Init.BaudRate = 115200;
	huart1.Init.WordLength = UART_WORDLENGTH_8B;
	huart1.Init.StopBits = UART_STOPBITS_1;
	huart1.Init.Parity = UART_PARITY_NONE;
	huart1.Init.Mode = UART_MODE_TX_RX;
	huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart1.Init.OverSampling = UART_OVERSAMPLING_16;
	huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_UART_Init(&huart1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART1_Init 2 */

	/* USER CODE END USART1_Init 2 */

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOF_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB,
	GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15, GPIO_PIN_RESET);

	/*Configure GPIO pin : PC13 */
	GPIO_InitStruct.Pin = GPIO_PIN_13;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pins : PB12 PB13 PB14 PB15 */
	GPIO_InitStruct.Pin = GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
uint8_t getdamageColorRG(int damage) {
	if (damage < FIRST_DAMAGE_TH) {
		return 0x0F; //full green
	} else if (damage < SECOND_DAMAGE_TH) { //almost no damage
		return 0xFA; //orange
	} else if (damage > SECOND_DAMAGE_TH) { //a lot of damage
		return 0xF0; //full red
	}
	return 0;
}
/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
	}
	/* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
