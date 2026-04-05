/* USER CODE BEGIN Header */
/**
	******************************************************************************
	* @file           : encoder.h
	* @brief          : Generic 4-channel encoder driver interface.
	******************************************************************************
	* @attention
	*
	* Copyright (c) 2026 STMicroelectronics.
	* All rights reserved.
	*
	* This software is licensed under terms that can be found in the LICENSE file
	* in the root directory of this software component.
	* If no LICENSE file comes with this software, it is provided AS-IS.
	*
	******************************************************************************
	*/
/* USER CODE END Header */

#ifndef __ENCODER_H__
#define __ENCODER_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "tim.h"

#include <stdint.h>

typedef enum
{
	ENCODER_ID_1 = 0U,
	ENCODER_ID_2 = 1U,
	ENCODER_ID_3 = 2U,
	ENCODER_ID_4 = 3U,
	ENCODER_COUNT = 4U
} EncoderId_t;

typedef struct
{
	TIM_HandleTypeDef *timer;
	uint8_t reversed;
} EncoderConfig_t;

void Encoder_GetDefaultConfig(EncoderConfig_t config[ENCODER_COUNT]);
void Encoder_Init(const EncoderConfig_t config[ENCODER_COUNT]);
void Encoder_DeInit(void);
int32_t Encoder_GetCount(EncoderId_t encoder);
int32_t Encoder_GetDelta(EncoderId_t encoder);
void Encoder_Reset(EncoderId_t encoder);
void Encoder_ResetAll(void);

#ifdef __cplusplus
}
#endif

#endif /* __ENCODER_H__ */
