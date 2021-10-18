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
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;


/* USER CODE BEGIN PV */
uint8_t TxData[8];
uint8_t RxData[8];
uint8_t count = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
/* USER CODE BEGIN PFP */
void CAN1_Tx(uint32_t IDE, uint32_t StdId, uint32_t ExtId, uint32_t RTR, uint32_t DLC, uint8_t TxData[8]);
void CAN_Start(CAN_HandleTypeDef *hcan);
void CAN1_Rx(uint32_t RxFifo);
void CAN1_FilterConfig(uint32_t FilterActivation, uint32_t FilterBank, uint32_t FilterFIFOAssignment, uint32_t FilterMode, uint32_t FilterScale, uint32_t SlaveStartFilterBank, uint32_t FilterIdHigh, uint32_t FilterIdLow, uint32_t FilterMaskIdHigh, uint32_t FilterMaskIdLow);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
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
  MX_CAN1_Init();
  /* USER CODE BEGIN 2 */
  CAN1_FilterConfig(CAN_FILTER_ENABLE, 0, CAN_FILTER_FIFO0, CAN_FILTERMODE_IDLIST, CAN_FILTERSCALE_32BIT, 14, 0x123, 0x0000, 0x156, 0x0000);
  if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
  {
	  Error_Handler();
  }
  CAN_Start(&hcan1);
  //CAN1_Tx(CAN_ID_STD, 0x123, 0, CAN_RTR_DATA, 1, TxData);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 20;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_15TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = ENABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = ENABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_14|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB14 PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_14|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */

/**
  * @brief  CAN1 Transmission Function.
  * @param  The type of identifier for the message that will be transmitted.
  * 	    This parameter must be CAN_ID_STD or CAN_ID_EXT.
  * @param  Standard Id of the message that will be transmitted.(11 bits)
  * 	    This parameter must be a number between Min_Data = 0 and Max_Data = 0x7FF.
  * 	    If the IDE is selected as CAN_ID_EXT, it can be negligible.
  * @param  Extended Id of the message that will be transmitted.(29 bits)
  * 		This parameter must be a number between Min_Data = 0 and Max_Data = 0x1FFFFFFF.
  * 		If the IDE is selected as CAN_ID_STD, it can be negligible.
  * @param	The type of frame for the message that will be transmitted.
  * 		This parameter must be CAN_RTR_DATA (Sending a Frame) or CAN_RTR_REMOTE (Requesting a Frame).
  * @param  The length of the frame that will be transmitted (0-8 Bytes).
  * 		This parameter must be a number between Min_Data = 0 and Max_Data = 8.
  * @param  8-Bytes Frame that will be transmitted.
  * 		If the RTR is selected as CAN_RTR_REMOTE, it can be negligible.
  * @retval None
  */
void CAN1_Tx(uint32_t IDE, uint32_t StdId, uint32_t ExtId, uint32_t RTR, uint32_t DLC, uint8_t TxData[8]){

	CAN_TxHeaderTypeDef TxHeader;
	uint32_t pTxMailbox;

	TxHeader.IDE = IDE;
	TxHeader.RTR = RTR;
	TxHeader.DLC = DLC;
	if(TxHeader.IDE == CAN_ID_STD){
		TxHeader.StdId = StdId;
		TxHeader.ExtId = 0;
	}
	else if(TxHeader.IDE == CAN_ID_EXT){
		TxHeader.StdId = 0;
		TxHeader.ExtId = ExtId;
	}

    if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &pTxMailbox) != HAL_OK)
    {
      Error_Handler();
    }
}

/**
  * @brief Start the CAN modules.
  * @param None
  * @retval None
  */
void CAN_Start(CAN_HandleTypeDef *hcan){

	if (HAL_CAN_Start(hcan) != HAL_OK)
	{
	  Error_Handler();
	}

}

/**
  * @brief CAN1 Received Frame Function.
  * @param This parameter can be CAN Receive FIFO Number.
  * @retval None
  */
void CAN1_Rx(uint32_t RxFifo){

	CAN_RxHeaderTypeDef RxHeader;

	if (HAL_CAN_GetRxMessage(&hcan1, RxFifo, &RxHeader, RxData) != HAL_OK)
	{
	  Error_Handler();
	}

	  if ((RxHeader.DLC == 1))
	  {
		  if(RxData[0] %3 == 1){
			  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
			  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7 | GPIO_PIN_14, GPIO_PIN_RESET);
		  }
		  else if(RxData[0] %3 == 2){
			  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);
			  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0 | GPIO_PIN_14, GPIO_PIN_RESET);
		  }
		  else if(RxData[0] %3 == 0){
			  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
			  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0 | GPIO_PIN_7, GPIO_PIN_RESET);
		  }
	  }

}

/**
  * @brief  CAN1 Filter Configuration Function.
  * @param  Enable or disable the filter.
  * 	    This parameter must be CAN_FILTER_ENABLE or CAN_FILTER_DISABLE.
  * @param  Select the filter bank which will be initialized.
  *         For single CAN instance(14 dedicated filter banks),
  * 		this parameter must be a number between Min_Data = 0 and Max_Data = 13.
  * 		For dual CAN instances(28 filter banks shared),
  * 		this parameter must be a number between Min_Data = 0 and Max_Data = 27.
  * @param  Select the FIFO (0 or 1) which will be assigned to the filter.
  * 		This parameter must be CAN_FILTER_FIFO0 or CAN_FILTER_FIFO1.
  * @param	Specifies the filter mode to be initialized.
  * 		This parameter must be CAN_FILTERMODE_IDMASK or CAN_FILTERMODE_IDLIST.
  * @param  Specifies the filter scale 16-bit or 32-bit.
  * 		This parameter must be CAN_FILTERSCALE_16BIT or CAN_FILTERSCALE_32BIT
  * @param  Select the start filter bank for the slave CAN instance.
  * 		For single CAN instances, this parameter is meaningless.
  * 		For dual CAN instances, all filter banks with lower index are assigned to master
  * 		CAN instance, whereas all filter banks with greater index are assigned to slave CAN instance.
  * 		This parameter must be a number between Min_Data = 0 and Max_Data = 27.
  * @param  Specifies the filter identification number (MSBs for a 32-bit
  * 		configuration, first one for a 16-bit configuration).
  * 		This parameter must be a number between Min_Data = 0x0000 and Max_Data = 0xFFFF.
  * @param	Specifies the filter identification number (LSBs for a 32-bit.
  * 		configuration, second one for a 16-bit configuration).
  * 		This parameter must be a number between Min_Data = 0x0000 and Max_Data = 0xFFFF.
  * @param  Specifies the filter mask number or identification number,
  * 		according to the mode (MSBs for a 32-bit configuration,
  * 		first one for a 16-bit configuration).
  * 		This parameter must be a number between Min_Data = 0x0000 and Max_Data = 0xFFFF.
  * @param	Specifies the filter mask number or identification number,
  * 		according to the mode (LSBs for a 32-bit configuration,
  * 		second one for a 16-bit configuration).
  * 		This parameter must be a number between Min_Data = 0x0000 and Max_Data = 0xFFFF.
  * @retval None
  */
void CAN1_FilterConfig(uint32_t FilterActivation, uint32_t FilterBank, uint32_t FilterFIFOAssignment, uint32_t FilterMode, uint32_t FilterScale, uint32_t SlaveStartFilterBank, uint32_t FilterIdHigh, uint32_t FilterIdLow, uint32_t FilterMaskIdHigh, uint32_t FilterMaskIdLow){

	CAN_FilterTypeDef sFilterConfig;

	sFilterConfig.FilterActivation = FilterActivation;
	sFilterConfig.FilterBank = FilterBank;
	sFilterConfig.FilterFIFOAssignment = FilterFIFOAssignment;
	sFilterConfig.FilterMode = FilterMode;
	sFilterConfig.FilterScale = FilterScale;
	sFilterConfig.SlaveStartFilterBank = SlaveStartFilterBank;

	sFilterConfig.FilterIdHigh = FilterIdHigh << 5;
	sFilterConfig.FilterIdLow = FilterIdLow << 5;
	sFilterConfig.FilterMaskIdHigh = FilterMaskIdHigh << 5;
	sFilterConfig.FilterMaskIdLow = FilterMaskIdLow << 5;


	if (HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig) != HAL_OK)
	{
	  Error_Handler();
	}

}

void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcan){


}
void HAL_CAN_TxMailbox1CompleteCallback(CAN_HandleTypeDef *hcan){



}
void HAL_CAN_TxMailbox2CompleteCallback(CAN_HandleTypeDef *hcan){


}
void HAL_CAN_TxMailbox0AbortCallback(CAN_HandleTypeDef *hcan){


}
void HAL_CAN_TxMailbox1AbortCallback(CAN_HandleTypeDef *hcan){


}
void HAL_CAN_TxMailbox2AbortCallback(CAN_HandleTypeDef *hcan){


}
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan){

	CAN1_Rx(CAN_RX_FIFO0);

}
void HAL_CAN_RxFifo0FullCallback(CAN_HandleTypeDef *hcan){


}
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan){


}
void HAL_CAN_RxFifo1FullCallback(CAN_HandleTypeDef *hcan){


}
void HAL_CAN_SleepCallback(CAN_HandleTypeDef *hcan){


}
void HAL_CAN_WakeUpFromRxMsgCallback(CAN_HandleTypeDef *hcan){


}
void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan){


}
/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM14 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM14) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
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
