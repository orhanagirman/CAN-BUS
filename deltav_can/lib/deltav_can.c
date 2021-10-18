/**
  ******************************************************************************
  * @file    deltav_can.c
  * @Author  Orhan AÐIRMAN
  * @Created on 11 Aðu 2021
  * @brief   DeltaV Can Bus Routines.
  ******************************************************************************
  * @attention
  *
  * @verbatim
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"
#include "deltav_can.h"

/******************************************************************************/
/*                          DELTAV CAN-BUS FUNCTIONS                          */
/******************************************************************************/

/**
  * @brief  CAN Transmission Function.
  * @param  hcan pointer to a CAN_HandleTypeDef structure that contains
  *         the configuration information for the specified CAN.
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
void CAN_Tx(CAN_HandleTypeDef *hcan, uint32_t IDE, uint32_t StdId, uint32_t ExtId, uint32_t RTR, uint32_t DLC, uint8_t TxData[8]){

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

    if (HAL_CAN_AddTxMessage(hcan, &TxHeader, TxData, &pTxMailbox) != HAL_OK)
    {
      Error_Handler();
    }
}

/**
  * @brief Start the CAN modules.
  * @param hcan pointer to a CAN_HandleTypeDef structure that contains
  *        the configuration information for the specified CAN.
  * @retval None
  */
void CAN_Start(CAN_HandleTypeDef *hcan){

	if (HAL_CAN_Start(hcan) != HAL_OK)
	{
	  Error_Handler();
	}

}

/**
  * @brief CAN Received Frame Function.
  * @param hcan pointer to a CAN_HandleTypeDef structure that contains
  *        the configuration information for the specified CAN.
  * @param This parameter can be CAN Receive FIFO Number.
  * @retval None
  */
void CAN_Rx(CAN_HandleTypeDef *hcan, uint32_t RxFifo){

	if (HAL_CAN_GetRxMessage(hcan, RxFifo, &RxHeader, RxData) != HAL_OK)
	{
	  Error_Handler();
	}

}

/**
  * @brief  CAN Filter Configuration Function.
  * @param  hcan pointer to a CAN_HandleTypeDef structure that contains
  *         the configuration information for the specified CAN.
  * @param  The type of identifier for the message that will be filtered.
  * 	    This parameter must be CAN_ID_STD or CAN_ID_EXT.
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
void CAN_FilterConfig(CAN_HandleTypeDef *hcan, uint32_t FilterIDE, uint32_t FilterActivation, uint32_t FilterBank, uint32_t FilterFIFOAssignment, uint32_t FilterMode, uint32_t FilterScale,
		uint32_t SlaveStartFilterBank, uint32_t FilterIdHigh, uint32_t FilterIdLow, uint32_t FilterMaskIdHigh, uint32_t FilterMaskIdLow){

	sFilterConfig.FilterActivation = FilterActivation;
	sFilterConfig.FilterBank = FilterBank;
	sFilterConfig.FilterFIFOAssignment = FilterFIFOAssignment;
	sFilterConfig.FilterMode = FilterMode;
	sFilterConfig.FilterScale = FilterScale;
	sFilterConfig.SlaveStartFilterBank = SlaveStartFilterBank;

	if(FilterIDE == CAN_ID_STD){
		sFilterConfig.FilterIdHigh = FilterIdHigh << 5;
		sFilterConfig.FilterIdLow = FilterIdLow << 5;
		sFilterConfig.FilterMaskIdHigh = FilterMaskIdHigh << 5;
		sFilterConfig.FilterMaskIdLow = FilterMaskIdLow << 5;
	}
	else if(FilterIDE == CAN_ID_EXT && FilterScale == CAN_FILTERSCALE_32BIT){
		sFilterConfig.FilterIdHigh = FilterIdHigh >> 13;
		sFilterConfig.FilterIdLow = FilterIdLow << 3 | (0x0004);
		sFilterConfig.FilterMaskIdHigh = FilterMaskIdHigh >> 13;
		sFilterConfig.FilterMaskIdLow = FilterMaskIdLow << 3 | (0x0004);
	}

	if (HAL_CAN_ConfigFilter(hcan, &sFilterConfig) != HAL_OK)
	{
	  Error_Handler();
	}

}

/************************ (C) COPYRIGHT DeltaV Uzay Teknolojileri A.S. *****END OF FILE****/
