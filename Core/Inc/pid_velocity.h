/* USER CODE BEGIN Header */
/**
	******************************************************************************
	* @file           : pid_velocity.h
	* @brief          : Generic 4-channel PID velocity control interface.
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

#ifndef __PID_VELOCITY_H__
#define __PID_VELOCITY_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "encoder.h"
#include "motor.h"

#include <stdint.h>

typedef enum
{
	PID_VELOCITY_ID_1 = 0U,
	PID_VELOCITY_ID_2 = 1U,
	PID_VELOCITY_ID_3 = 2U,
	PID_VELOCITY_ID_4 = 3U,
	PID_VELOCITY_COUNT = 4U
} PIDVelocityId_t;

typedef struct
{
	EncoderId_t encoder_id;
	MotorId_t motor_id;
	float kp;
	float ki;
	float kd;
	uint32_t sample_time_ms;
	int32_t output_limit_permille;
	int32_t integral_limit;
	int32_t max_speed_cps;
	uint8_t reverse_feedback;
} PIDVelocityConfig_t;

typedef struct
{
	int32_t target_speed_cps;
	int32_t measured_speed_cps;
	int32_t output_permille;
	float integral;
	float last_error;
} PIDVelocityState_t;

void PIDVelocity_GetDefaultConfig(PIDVelocityConfig_t config[PID_VELOCITY_COUNT]);
void PIDVelocity_Init(const PIDVelocityConfig_t config[PID_VELOCITY_COUNT]);
void PIDVelocity_SetTarget(PIDVelocityId_t id, int32_t target_speed_cps);
void PIDVelocity_SetTargetAll(const int32_t target_speed_cps[PID_VELOCITY_COUNT]);
void PIDVelocity_SetChassisCommand(int32_t lx_permille, int32_t y_permille, int32_t w_permille);
void PIDVelocity_Reset(PIDVelocityId_t id);
void PIDVelocity_ResetAll(void);
void PIDVelocity_RunStep(void);
const PIDVelocityState_t *PIDVelocity_GetState(PIDVelocityId_t id);

#ifdef __cplusplus
}
#endif

#endif /* __PID_VELOCITY_H__ */
