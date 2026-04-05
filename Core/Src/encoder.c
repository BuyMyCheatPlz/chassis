/* USER CODE BEGIN Header */
/**
	******************************************************************************
	* @file           : encoder.c
	* @brief          : Generic 4-channel encoder driver implementation.
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

#include "encoder.h"

#define ENCODER_MAX_COUNT  4U

typedef struct
{
	EncoderConfig_t config;
	uint16_t last_raw;
	int32_t total_count;
	int16_t last_delta;
} EncoderState_t;

static EncoderState_t s_encoders[ENCODER_MAX_COUNT];
static uint8_t s_initialized;

static uint8_t Encoder_IsValid(EncoderId_t encoder)
{
	return (encoder < ENCODER_COUNT) ? 1U : 0U;
}

static int16_t Encoder_ApplyPolarity(const EncoderConfig_t *config, int16_t delta)
{
	if ((config != NULL) && (config->reversed != 0U))
	{
		delta = (int16_t)-delta;
	}

	return delta;
}

static void Encoder_UpdateOne(EncoderId_t encoder)
{
	EncoderState_t *state = &s_encoders[encoder];

	if (state->config.timer == NULL)
	{
		state->last_delta = 0;
		return;
	}

	uint16_t raw = (uint16_t)__HAL_TIM_GET_COUNTER(state->config.timer);
	int16_t delta = (int16_t)(raw - state->last_raw);

	state->last_raw = raw;
	delta = Encoder_ApplyPolarity(&state->config, delta);
	state->last_delta = delta;
	state->total_count += delta;
}

void Encoder_GetDefaultConfig(EncoderConfig_t config[ENCODER_COUNT])
{
	if (config == NULL)
	{
		return;
	}

	config[ENCODER_ID_1].timer = &htim3;
	config[ENCODER_ID_1].reversed = 0U;

	config[ENCODER_ID_2].timer = &htim4;
	config[ENCODER_ID_2].reversed = 0U;

	config[ENCODER_ID_3].timer = &htim5;
	config[ENCODER_ID_3].reversed = 0U;

	config[ENCODER_ID_4].timer = &htim8;
	config[ENCODER_ID_4].reversed = 0U;
}

void Encoder_Init(const EncoderConfig_t config[ENCODER_COUNT])
{
	EncoderConfig_t default_config[ENCODER_COUNT];
	const EncoderConfig_t *source = config;

	if (source == NULL)
	{
		Encoder_GetDefaultConfig(default_config);
		source = default_config;
	}

	for (uint8_t i = 0U; i < ENCODER_COUNT; i++)
	{
		s_encoders[i].config = source[i];
		s_encoders[i].last_raw = 0U;
		s_encoders[i].total_count = 0;
		s_encoders[i].last_delta = 0;

		if (s_encoders[i].config.timer != NULL)
		{
			if (HAL_TIM_Encoder_Start(s_encoders[i].config.timer, TIM_CHANNEL_ALL) != HAL_OK)
			{
				Error_Handler();
			}

			__HAL_TIM_SET_COUNTER(s_encoders[i].config.timer, 0U);
		}
	}

	s_initialized = 1U;
}

void Encoder_DeInit(void)
{
	for (uint8_t i = 0U; i < ENCODER_COUNT; i++)
	{
		if (s_encoders[i].config.timer != NULL)
		{
			(void)HAL_TIM_Encoder_Stop(s_encoders[i].config.timer, TIM_CHANNEL_ALL);
		}
	}

	s_initialized = 0U;
}

int32_t Encoder_GetCount(EncoderId_t encoder)
{
	if (Encoder_IsValid(encoder) == 0U)
	{
		return 0;
	}

	if (s_initialized == 0U)
	{
		return 0;
	}

	Encoder_UpdateOne(encoder);
	return s_encoders[encoder].total_count;
}

int32_t Encoder_GetDelta(EncoderId_t encoder)
{
	if (Encoder_IsValid(encoder) == 0U)
	{
		return 0;
	}

	if (s_initialized == 0U)
	{
		return 0;
	}

	Encoder_UpdateOne(encoder);
	return s_encoders[encoder].last_delta;
}

void Encoder_Reset(EncoderId_t encoder)
{
	if (Encoder_IsValid(encoder) == 0U)
	{
		return;
	}

	s_encoders[encoder].total_count = 0;
	s_encoders[encoder].last_delta = 0;
	s_encoders[encoder].last_raw = 0U;

	if (s_encoders[encoder].config.timer != NULL)
	{
		__HAL_TIM_SET_COUNTER(s_encoders[encoder].config.timer, 0U);
	}
}

void Encoder_ResetAll(void)
{
	for (uint8_t i = 0U; i < ENCODER_COUNT; i++)
	{
		Encoder_Reset((EncoderId_t)i);
	}
}
