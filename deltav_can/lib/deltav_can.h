/**
  ******************************************************************************
  * @file    deltac_can.h
  * @Author  Orhan AÐIRMAN
  * @Created on 11 Aðu 2021
  * @brief   This file contains the headers of the deltav_can handlers.
  ******************************************************************************
  * @attention
  *
  * @verbatim
 ******************************************************************************
  */

#ifndef INC_DELTAV_CAN_H_
#define INC_DELTAV_CAN_H_

#ifdef __cplusplus
 extern "C" {
#endif

 /* Private variables ---------------------------------------------------------*/
CAN_TxHeaderTypeDef TxHeader;
CAN_RxHeaderTypeDef RxHeader;
CAN_FilterTypeDef sFilterConfig;
uint32_t pTxMailbox;
uint8_t RxData[8];

/* Exported functions prototypes ---------------------------------------------*/
void CAN_Tx(CAN_HandleTypeDef *hcan, uint32_t IDE, uint32_t StdId, uint32_t ExtId, uint32_t RTR, uint32_t DLC, uint8_t TxData[8]);
void CAN_Start(CAN_HandleTypeDef *hcan);
void CAN_Rx(CAN_HandleTypeDef *hcan, uint32_t RxFifo);
void CAN_FilterConfig(CAN_HandleTypeDef *hcan, uint32_t FilterIDE, uint32_t FilterActivation, uint32_t FilterBank, uint32_t FilterFIFOAssignment, uint32_t FilterMode, uint32_t FilterScale, uint32_t SlaveStartFilterBank, uint32_t FilterIdHigh, uint32_t FilterIdLow, uint32_t FilterMaskIdHigh, uint32_t FilterMaskIdLow);

#ifdef __cplusplus
}
#endif

#endif /* INC_DELTAV_CAN_H_ */

/************************ (C) COPYRIGHT DeltaV Uzay Teknolojileri A.S. *****END OF FILE****/
