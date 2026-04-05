/* USER CODE BEGIN Header */
/**
	******************************************************************************
	* @file           : motor.c
	* @brief          : Generic 4-motor driver implementation.
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

#include "motor.h"
#include "encoder.h"

#define MOTOR_MAX_COUNT      4U
#define MOTOR_SPEED_SCALE    1000L

typedef struct
{
	MotorConfig_t config;
	int16_t target_speed;
} MotorState_t;

static MotorState_t s_motors[MOTOR_MAX_COUNT];

static uint16_t Motor_AbsSpeed(int16_t speed_permille)
{
	if (speed_permille < 0)
	{
		speed_permille = (int16_t)-speed_permille;
	}

	if (speed_permille > MOTOR_SPEED_SCALE)
	{
		speed_permille = MOTOR_SPEED_SCALE;
	}

	return (uint16_t)speed_permille;
}

static uint8_t Motor_IsValid(MotorId_t motor)
{
	return (motor < MOTOR_COUNT) ? 1U : 0U;
}

static GPIO_PinState Motor_GetDirPinState(uint8_t asserted, uint8_t active_high)
{
	if (active_high == 0U)
	{
		asserted ^= 1U;
	}

	return (asserted != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET;
}

static void Motor_ApplyDirection(const MotorConfig_t *config, int16_t speed_permille)
{
	uint8_t dir_a_assert = 0U;
	uint8_t dir_b_assert = 0U;

	if (speed_permille != 0)
	{
		uint8_t forward = (speed_permille > 0) ? 1U : 0U;

		if (config->direction_reversed != 0U)
		{
			forward ^= 1U;
		}

		if (config->dir_active_high == 0U)
		{
			/* Physical level conversion is handled per pin later. */
		}

		dir_a_assert = (forward != 0U) ? 1U : 0U;
		dir_b_assert = (forward != 0U) ? 0U : 1U;
	}

	if (config->dir_a_port != NULL)
	{
		HAL_GPIO_WritePin(config->dir_a_port,
		                 config->dir_a_pin,
		                 Motor_GetDirPinState(dir_a_assert, config->dir_active_high));
	}

	if (config->dir_b_port != NULL)
	{
		HAL_GPIO_WritePin(config->dir_b_port,
		                 config->dir_b_pin,
		                 Motor_GetDirPinState(dir_b_assert, config->dir_active_high));
	}
}

static uint32_t Motor_GetTimerPeriod(const MotorConfig_t *config)
{
	if (config->pwm_timer == NULL)
	{
		return 0U;
	}

	return __HAL_TIM_GET_AUTORELOAD(config->pwm_timer);
}

void Motor_GetDefaultConfig(MotorConfig_t config[MOTOR_COUNT])
{
	if (config == NULL)
	{
		return;
	}

	config[MOTOR_ID_1].pwm_timer = &htim2;
	config[MOTOR_ID_1].pwm_channel = TIM_CHANNEL_1;
	config[MOTOR_ID_1].encoder_timer = &htim3;
	config[MOTOR_ID_1].dir_a_port = GPIOC;
	config[MOTOR_ID_1].dir_a_pin = GPIO_PIN_0;
	config[MOTOR_ID_1].dir_b_port = GPIOC;
	config[MOTOR_ID_1].dir_b_pin = GPIO_PIN_4;
	config[MOTOR_ID_1].dir_active_high = 1U;
	config[MOTOR_ID_1].direction_reversed = 0U;

	config[MOTOR_ID_2].pwm_timer = &htim2;
	config[MOTOR_ID_2].pwm_channel = TIM_CHANNEL_2;
	config[MOTOR_ID_2].encoder_timer = &htim4;
	config[MOTOR_ID_2].dir_a_port = GPIOC;
	config[MOTOR_ID_2].dir_a_pin = GPIO_PIN_1;
	config[MOTOR_ID_2].dir_b_port = GPIOC;
	config[MOTOR_ID_2].dir_b_pin = GPIO_PIN_5;
	config[MOTOR_ID_2].dir_active_high = 1U;
	config[MOTOR_ID_2].direction_reversed = 0U;

	config[MOTOR_ID_3].pwm_timer = &htim2;
	config[MOTOR_ID_3].pwm_channel = TIM_CHANNEL_3;
	config[MOTOR_ID_3].encoder_timer = &htim5;
	config[MOTOR_ID_3].dir_a_port = GPIOC;
	config[MOTOR_ID_3].dir_a_pin = GPIO_PIN_2;
	config[MOTOR_ID_3].dir_b_port = GPIOA;
	config[MOTOR_ID_3].dir_b_pin = GPIO_PIN_4;
	config[MOTOR_ID_3].dir_active_high = 1U;
	config[MOTOR_ID_3].direction_reversed = 0U;

	config[MOTOR_ID_4].pwm_timer = &htim2;
	config[MOTOR_ID_4].pwm_channel = TIM_CHANNEL_4;
	config[MOTOR_ID_4].encoder_timer = &htim8;
	config[MOTOR_ID_4].dir_a_port = GPIOC;
	config[MOTOR_ID_4].dir_a_pin = GPIO_PIN_3;
	config[MOTOR_ID_4].dir_b_port = GPIOA;
	config[MOTOR_ID_4].dir_b_pin = GPIO_PIN_5;
	config[MOTOR_ID_4].dir_active_high = 1U;
	config[MOTOR_ID_4].direction_reversed = 0U;
}

void Motor_Init(const MotorConfig_t config[MOTOR_COUNT])
{
	MotorConfig_t default_config[MOTOR_COUNT];
	const MotorConfig_t *source = config;

	if (source == NULL)
	{
		Motor_GetDefaultConfig(default_config);
		source = default_config;
	}

	for (uint8_t i = 0U; i < MOTOR_COUNT; i++)
	{
		s_motors[i].config = source[i];
		s_motors[i].target_speed = 0;
		if (s_motors[i].config.dir_a_port != NULL)
		{
			HAL_GPIO_WritePin(s_motors[i].config.dir_a_port,
			                 s_motors[i].config.dir_a_pin,
			                 Motor_GetDirPinState(0U, s_motors[i].config.dir_active_high));
		}

		if (s_motors[i].config.dir_b_port != NULL)
		{
			HAL_GPIO_WritePin(s_motors[i].config.dir_b_port,
			                 s_motors[i].config.dir_b_pin,
			                 Motor_GetDirPinState(0U, s_motors[i].config.dir_active_high));
		}

		if (s_motors[i].config.pwm_timer != NULL)
		{
			if (HAL_TIM_PWM_Start(s_motors[i].config.pwm_timer, s_motors[i].config.pwm_channel) != HAL_OK)
			{
				Error_Handler();
			}

			__HAL_TIM_SET_COMPARE(s_motors[i].config.pwm_timer, s_motors[i].config.pwm_channel, 0U);
		}

	}
}

void Motor_SetSpeed(MotorId_t motor, int16_t speed_permille)
{
	if (Motor_IsValid(motor) == 0U)
	{
		return;
	}

	MotorState_t *state = &s_motors[motor];
	state->target_speed = speed_permille;

	Motor_ApplyDirection(&state->config, speed_permille);

	if (state->config.pwm_timer == NULL)
	{
		return;
	}

	if (speed_permille == 0)
	{
		__HAL_TIM_SET_COMPARE(state->config.pwm_timer, state->config.pwm_channel, 0U);
		return;
	}

	uint16_t magnitude = Motor_AbsSpeed(speed_permille);
	uint32_t period = Motor_GetTimerPeriod(&state->config);
	uint32_t compare = (period * magnitude) / MOTOR_SPEED_SCALE;

	if (compare > period)
	{
		compare = period;
	}

	__HAL_TIM_SET_COMPARE(state->config.pwm_timer, state->config.pwm_channel, compare);
}

void Motor_SetSpeedAll(const int16_t speed_permille[MOTOR_COUNT])
{
	if (speed_permille == NULL)
	{
		return;
	}

	for (uint8_t i = 0U; i < MOTOR_COUNT; i++)
	{
		Motor_SetSpeed((MotorId_t)i, speed_permille[i]);
	}
}

void Motor_AllStop(void)
{
	for (uint8_t i = 0U; i < MOTOR_COUNT; i++)
	{
		Motor_SetSpeed((MotorId_t)i, 0);
	}
}

int32_t Motor_GetEncoderCount(MotorId_t motor)
{
	if (Motor_IsValid(motor) == 0U)
	{
		return 0;
	}

	return Encoder_GetCount((EncoderId_t)motor);
}

void Motor_ResetEncoderCount(MotorId_t motor)
{
	if (Motor_IsValid(motor) == 0U)
	{
		return;
	}

	MotorState_t *state = &s_motors[motor];
	(void)state;
	Encoder_Reset((EncoderId_t)motor);
}
