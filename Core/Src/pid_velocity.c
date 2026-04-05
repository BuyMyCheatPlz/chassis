/* USER CODE BEGIN Header */
/**
	******************************************************************************
	* @file           : pid_velocity.c
	* @brief          : Generic 4-channel PID velocity control implementation.
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

#include "pid_velocity.h"

#define PID_VELOCITY_MAX_COUNT  4U
#define PID_VELOCITY_ENCODER_QUADRATURE_FACTOR  4L
#define PID_VELOCITY_REF_LINE_COUNT             12L
#define PID_VELOCITY_REF_GEAR_RATIO             500L

static int32_t PIDVelocity_GetCountsPerOutputRev(const PIDVelocityConfig_t *config)
{
	int32_t line_count;
	int32_t gear_ratio;

	if (config == NULL)
	{
		return 1;
	}

	line_count = (config->encoder_line_count > 0U) ? (int32_t)config->encoder_line_count : 1;
	gear_ratio = (config->gear_ratio > 0U) ? (int32_t)config->gear_ratio : 1;

	return line_count * PID_VELOCITY_ENCODER_QUADRATURE_FACTOR * gear_ratio;
}

typedef struct
{
	PIDVelocityConfig_t config;
	PIDVelocityState_t state;
} PIDVelocityChannel_t;

static PIDVelocityChannel_t s_channels[PID_VELOCITY_MAX_COUNT];
static uint8_t s_initialized;

static uint8_t PIDVelocity_IsValid(PIDVelocityId_t id)
{
	return (id < PID_VELOCITY_COUNT) ? 1U : 0U;
}

static int32_t PIDVelocity_ClampInt32(int32_t value, int32_t minimum, int32_t maximum)
{
	if (value < minimum)
	{
		return minimum;
	}

	if (value > maximum)
	{
		return maximum;
	}

	return value;
}

static int16_t PIDVelocity_ClampOutput(int32_t value, int32_t limit)
{
	value = PIDVelocity_ClampInt32(value, -limit, limit);
	return (int16_t)value;
}

void PIDVelocity_GetDefaultConfig(PIDVelocityConfig_t config[PID_VELOCITY_COUNT])
{
	if (config == NULL)
	{
		return;
	}

	config[PID_VELOCITY_ID_1].encoder_id = ENCODER_ID_1;
	config[PID_VELOCITY_ID_1].motor_id = MOTOR_ID_1;
	config[PID_VELOCITY_ID_1].encoder_line_count = 12U;
	config[PID_VELOCITY_ID_1].gear_ratio = 500U;
	config[PID_VELOCITY_ID_1].kp = 0.40f;
	config[PID_VELOCITY_ID_1].ki = 0.03f;
	config[PID_VELOCITY_ID_1].kd = 0.00f;
	config[PID_VELOCITY_ID_1].sample_time_ms = 10U;
	config[PID_VELOCITY_ID_1].output_limit_permille = 1000;
	config[PID_VELOCITY_ID_1].integral_limit = 20000;
	config[PID_VELOCITY_ID_1].max_speed_cps = 2000;
	config[PID_VELOCITY_ID_1].reverse_feedback = 0U;

	config[PID_VELOCITY_ID_2] = config[PID_VELOCITY_ID_1];
	config[PID_VELOCITY_ID_2].encoder_id = ENCODER_ID_2;
	config[PID_VELOCITY_ID_2].motor_id = MOTOR_ID_2;

	config[PID_VELOCITY_ID_3] = config[PID_VELOCITY_ID_1];
	config[PID_VELOCITY_ID_3].encoder_id = ENCODER_ID_3;
	config[PID_VELOCITY_ID_3].motor_id = MOTOR_ID_3;

	config[PID_VELOCITY_ID_4] = config[PID_VELOCITY_ID_1];
	config[PID_VELOCITY_ID_4].encoder_id = ENCODER_ID_4;
	config[PID_VELOCITY_ID_4].motor_id = MOTOR_ID_4;
}

void PIDVelocity_Init(const PIDVelocityConfig_t config[PID_VELOCITY_COUNT])
{
	PIDVelocityConfig_t default_config[PID_VELOCITY_COUNT];
	const PIDVelocityConfig_t *source = config;

	if (source == NULL)
	{
		PIDVelocity_GetDefaultConfig(default_config);
		source = default_config;
	}

	for (uint8_t i = 0U; i < PID_VELOCITY_COUNT; i++)
	{
		s_channels[i].config = source[i];
		s_channels[i].state.target_speed_cps = 0;
		s_channels[i].state.measured_speed_cps = 0;
		s_channels[i].state.output_permille = 0;
		s_channels[i].state.integral = 0.0f;
		s_channels[i].state.last_error = 0.0f;
	}

	s_initialized = 1U;
}

void PIDVelocity_SetTarget(PIDVelocityId_t id, int32_t target_speed_cps)
{
	if (PIDVelocity_IsValid(id) == 0U)
	{
		return;
	}

	s_channels[id].state.target_speed_cps = target_speed_cps;
}

void PIDVelocity_SetTargetAll(const int32_t target_speed_cps[PID_VELOCITY_COUNT])
{
	if (target_speed_cps == NULL)
	{
		return;
	}

	for (uint8_t i = 0U; i < PID_VELOCITY_COUNT; i++)
	{
		PIDVelocity_SetTarget((PIDVelocityId_t)i, target_speed_cps[i]);
	}
}

void PIDVelocity_SetChassisCommand(int32_t lx_permille, int32_t y_permille, int32_t w_permille)
{
	int32_t wheel_command[PID_VELOCITY_COUNT];
	int32_t max_abs_command = 0;

	lx_permille = PIDVelocity_ClampInt32(lx_permille, -1000, 1000);
	y_permille = PIDVelocity_ClampInt32(y_permille, -1000, 1000);
	w_permille = PIDVelocity_ClampInt32(w_permille, -1000, 1000);

	/* Wheel order: FL, FR, RL, RR */
	wheel_command[PID_VELOCITY_ID_1] = y_permille + lx_permille + w_permille;
	wheel_command[PID_VELOCITY_ID_2] = y_permille - lx_permille - w_permille;
	wheel_command[PID_VELOCITY_ID_3] = y_permille - lx_permille + w_permille;
	wheel_command[PID_VELOCITY_ID_4] = y_permille + lx_permille - w_permille;

	for (uint8_t i = 0U; i < PID_VELOCITY_COUNT; i++)
	{
		int32_t abs_command = wheel_command[i] < 0 ? -wheel_command[i] : wheel_command[i];

		if (abs_command > max_abs_command)
		{
			max_abs_command = abs_command;
		}
	}

	if (max_abs_command > 1000)
	{
		for (uint8_t i = 0U; i < PID_VELOCITY_COUNT; i++)
		{
			wheel_command[i] = (wheel_command[i] * 1000) / max_abs_command;
		}
	}

	for (uint8_t i = 0U; i < PID_VELOCITY_COUNT; i++)
	{
		const PIDVelocityConfig_t *config = &s_channels[i].config;
		int32_t target_speed_cps = (wheel_command[i] * config->max_speed_cps) / 1000;

		s_channels[i].state.target_speed_cps = target_speed_cps;
	}
}

void PIDVelocity_Reset(PIDVelocityId_t id)
{
	if (PIDVelocity_IsValid(id) == 0U)
	{
		return;
	}

	s_channels[id].state.measured_speed_cps = 0;
	s_channels[id].state.output_permille = 0;
	s_channels[id].state.integral = 0.0f;
	s_channels[id].state.last_error = 0.0f;
	Encoder_Reset(s_channels[id].config.encoder_id);
	Motor_SetSpeed(s_channels[id].config.motor_id, 0);
}

void PIDVelocity_ResetAll(void)
{
	for (uint8_t i = 0U; i < PID_VELOCITY_COUNT; i++)
	{
		PIDVelocity_Reset((PIDVelocityId_t)i);
	}
}

const PIDVelocityState_t *PIDVelocity_GetState(PIDVelocityId_t id)
{
	if (PIDVelocity_IsValid(id) == 0U)
	{
		return NULL;
	}

	return &s_channels[id].state;
}

void PIDVelocity_RunStep(void)
{
	if (s_initialized == 0U)
	{
		return;
	}

	for (uint8_t i = 0U; i < PID_VELOCITY_COUNT; i++)
	{
		PIDVelocityChannel_t *channel = &s_channels[i];
		const PIDVelocityConfig_t *config = &channel->config;
		PIDVelocityState_t *state = &channel->state;
		const int32_t ref_counts_per_rev = PID_VELOCITY_REF_LINE_COUNT
									 * PID_VELOCITY_ENCODER_QUADRATURE_FACTOR
									 * PID_VELOCITY_REF_GEAR_RATIO;
		int32_t counts_per_rev;

		int32_t delta_count = Encoder_GetDelta(config->encoder_id);

		if (config->reverse_feedback != 0U)
		{
			delta_count = -delta_count;
		}

		if (config->sample_time_ms == 0U)
		{
			state->measured_speed_cps = 0;
			state->output_permille = 0;
			Motor_SetSpeed(config->motor_id, 0);
			continue;
		}

		state->measured_speed_cps = (int32_t)((delta_count * 1000L) / (int32_t)config->sample_time_ms);
		counts_per_rev = PIDVelocity_GetCountsPerOutputRev(config);

		/*
		 * Normalize measured speed to a 12-line, 500-ratio reference drivetrain,
		 * so existing PID gains/max_speed_cps tuning remains comparable.
		 */
		if (counts_per_rev > 0)
		{
			state->measured_speed_cps = (state->measured_speed_cps * ref_counts_per_rev) / counts_per_rev;
		}

		float error = (float)(state->target_speed_cps - state->measured_speed_cps);
		float dt = (float)config->sample_time_ms / 1000.0f;

		state->integral += error * dt;
		if (config->integral_limit > 0)
		{
			float integral_limit = (float)config->integral_limit;
			if (state->integral > integral_limit)
			{
				state->integral = integral_limit;
			}
			else if (state->integral < -integral_limit)
			{
				state->integral = -integral_limit;
			}
		}

		float derivative = (dt > 0.0f) ? ((error - state->last_error) / dt) : 0.0f;
		float output = (config->kp * error) + (config->ki * state->integral) + (config->kd * derivative);
		int16_t output_permille = PIDVelocity_ClampOutput((int32_t)output, config->output_limit_permille);

		state->output_permille = output_permille;
		state->last_error = error;

		Motor_SetSpeed(config->motor_id, output_permille);
	}
}
