/* USER CODE BEGIN Header */
/**
	******************************************************************************
	* @file           : motor.h
	* @brief          : Generic 4-motor driver interface.
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

#ifndef __MOTOR_H__
#define __MOTOR_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "tim.h"

#include <stdint.h>

typedef enum
{
	MOTOR_ID_1 = 0U,
	MOTOR_ID_2 = 1U,
	MOTOR_ID_3 = 2U,
	MOTOR_ID_4 = 3U,
	MOTOR_COUNT = 4U
} MotorId_t;

typedef struct
{
	TIM_HandleTypeDef *pwm_timer;
	uint32_t pwm_channel;
	TIM_HandleTypeDef *encoder_timer;
	GPIO_TypeDef *dir_a_port;
	uint16_t dir_a_pin;
	GPIO_TypeDef *dir_b_port;
	uint16_t dir_b_pin;
	uint8_t dir_active_high;
	uint8_t direction_reversed;
} MotorConfig_t;

void Motor_GetDefaultConfig(MotorConfig_t config[MOTOR_COUNT]);
void Motor_Init(const MotorConfig_t config[MOTOR_COUNT]);
void Motor_SetSpeed(MotorId_t motor, int16_t speed_permille);
void Motor_SetSpeedAll(const int16_t speed_permille[MOTOR_COUNT]);
void Motor_AllStop(void);
int32_t Motor_GetEncoderCount(MotorId_t motor);
void Motor_ResetEncoderCount(MotorId_t motor);

#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_H__ */
