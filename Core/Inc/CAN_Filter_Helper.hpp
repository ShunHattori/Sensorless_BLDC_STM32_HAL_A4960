#pragma once
#include "main.h"

// setting up can filter mode and IDs or mask
// https://hsdev.co.jp/stm32-can/
void CAN_Filter_Init(CAN_HandleTypeDef *hcan_obj, uint32_t fId1, uint32_t fId2,
		uint32_t fId3, uint32_t fId4) {
	CAN_FilterTypeDef sFilterConfig;
	sFilterConfig.FilterIdHigh = fId1 << 5;
	sFilterConfig.FilterIdLow = fId2 << 5;
	sFilterConfig.FilterMaskIdHigh = fId3 << 5;
	sFilterConfig.FilterMaskIdLow = fId4 << 5;
	sFilterConfig.FilterScale = CAN_FILTERSCALE_16BIT;
	sFilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
	sFilterConfig.FilterBank = 0;
	sFilterConfig.FilterMode = CAN_FILTERMODE_IDLIST;
	sFilterConfig.SlaveStartFilterBank = 14;
	sFilterConfig.FilterActivation = ENABLE;

	if (HAL_CAN_ConfigFilter(hcan_obj, &sFilterConfig) != HAL_OK) {
		/* Filter configuration Error */
		Error_Handler();
	}
}
