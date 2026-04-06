/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.c
  * @brief   This file provides code for the configuration
  *          of the USART instances.
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
/* Includes ------------------------------------------------------------------*/
#include "usart.h"

/* USER CODE BEGIN 0 */

#include "cmsis_os.h"
#include "pid_velocity.h"

#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define USART1_RX_BUFFER_SIZE  64U

static uint8_t s_usart1_rx_buffer[USART1_RX_BUFFER_SIZE];
static uint8_t s_usart1_pending_buffer[USART1_RX_BUFFER_SIZE];
static volatile uint16_t s_usart1_pending_length = 0U;
static volatile uint8_t s_usart1_pending_ready = 0U;

static void USART1_RxRestart(void);
static void USART1_StorePendingFrame(const uint8_t *buffer, uint16_t length);
static void USART1_ProcessFrame(const uint8_t *buffer, uint16_t length);
static void USART1_QueueText(const char *text);
static void USART1_ExtractAndQueueForwardFrames(const char *frame);
static uint8_t USART1_ParseAngleSpeedDeviceId(const char *cursor,
                                              float *angle,
                                              long *speed,
                                              long *device_id,
                                              int *consumed);

void USART1_IdleReceiveIRQHandler(void);

extern osMessageQueueId_t Usart_SendingHandle;

/* USER CODE END 0 */

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart2_tx;

/* USART1 init function */

void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  USART1_RxRestart();

  /* USER CODE END USART1_Init 2 */

}
/* USART2 init function */

void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspInit 0 */

  /* USER CODE END USART1_MspInit 0 */
    /* USART1 clock enable */
    __HAL_RCC_USART1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**USART1 GPIO Configuration
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* USART1 DMA Init */
    /* USART1_RX Init */
    hdma_usart1_rx.Instance = DMA1_Channel5;
    hdma_usart1_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_usart1_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart1_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart1_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart1_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart1_rx.Init.Mode = DMA_NORMAL;
    hdma_usart1_rx.Init.Priority = DMA_PRIORITY_LOW;
    if (HAL_DMA_Init(&hdma_usart1_rx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(uartHandle,hdmarx,hdma_usart1_rx);

    /* USART1 interrupt Init */
    HAL_NVIC_SetPriority(USART1_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
  /* USER CODE BEGIN USART1_MspInit 1 */

  /* USER CODE END USART1_MspInit 1 */
  }
  else if(uartHandle->Instance==USART2)
  {
  /* USER CODE BEGIN USART2_MspInit 0 */

  /* USER CODE END USART2_MspInit 0 */
    /* USART2 clock enable */
    __HAL_RCC_USART2_CLK_ENABLE();

    __HAL_RCC_GPIOD_CLK_ENABLE();
    /**USART2 GPIO Configuration
    PD5     ------> USART2_TX
    PD6     ------> USART2_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_5;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    __HAL_AFIO_REMAP_USART2_ENABLE();

    /* USART2 DMA Init */
    /* USART2_TX Init */
    hdma_usart2_tx.Instance = DMA1_Channel7;
    hdma_usart2_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_usart2_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart2_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart2_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart2_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart2_tx.Init.Mode = DMA_NORMAL;
    hdma_usart2_tx.Init.Priority = DMA_PRIORITY_LOW;
    if (HAL_DMA_Init(&hdma_usart2_tx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(uartHandle,hdmatx,hdma_usart2_tx);

    /* USART2 interrupt Init */
    HAL_NVIC_SetPriority(USART2_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(USART2_IRQn);
  /* USER CODE BEGIN USART2_MspInit 1 */

  /* USER CODE END USART2_MspInit 1 */
  }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle)
{

  if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspDeInit 0 */

  /* USER CODE END USART1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART1_CLK_DISABLE();

    /**USART1 GPIO Configuration
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_9|GPIO_PIN_10);

    /* USART1 DMA DeInit */
    HAL_DMA_DeInit(uartHandle->hdmarx);

    /* USART1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART1_IRQn);
  /* USER CODE BEGIN USART1_MspDeInit 1 */

  /* USER CODE END USART1_MspDeInit 1 */
  }
  else if(uartHandle->Instance==USART2)
  {
  /* USER CODE BEGIN USART2_MspDeInit 0 */

  /* USER CODE END USART2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART2_CLK_DISABLE();

    /**USART2 GPIO Configuration
    PD5     ------> USART2_TX
    PD6     ------> USART2_RX
    */
    HAL_GPIO_DeInit(GPIOD, GPIO_PIN_5|GPIO_PIN_6);

    /* USART2 DMA DeInit */
    HAL_DMA_DeInit(uartHandle->hdmatx);

    /* USART2 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART2_IRQn);
  /* USER CODE BEGIN USART2_MspDeInit 1 */

  /* USER CODE END USART2_MspDeInit 1 */
  }
}

static void USART1_RxRestart(void)
{
  if (HAL_UART_Receive_DMA(&huart1, s_usart1_rx_buffer, USART1_RX_BUFFER_SIZE) != HAL_OK)
  {
    Error_Handler();
  }

  __HAL_UART_CLEAR_IDLEFLAG(&huart1);
  __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
}

static void USART1_StorePendingFrame(const uint8_t *buffer, uint16_t length)
{
  if ((buffer == NULL) || (length == 0U))
  {
    return;
  }

  if (length > USART1_RX_BUFFER_SIZE)
  {
    length = USART1_RX_BUFFER_SIZE;
  }

  memcpy(s_usart1_pending_buffer, buffer, length);
  s_usart1_pending_length = length;
  s_usart1_pending_ready = 1U;
}

static void USART1_ProcessFrame(const uint8_t *buffer, uint16_t length)
{
  char frame[USART1_RX_BUFFER_SIZE + 1U] = {0};
  char *lx_command;
  char *rx_command;
  long lx = 0;
  long y = 0;
  long rx = 0;
  long y_dummy = 0;
  uint8_t has_lx = 0U;
  uint8_t has_rx = 0U;

  if ((buffer == NULL) || (length == 0U))
  {
    return;
  }

  if (length > USART1_RX_BUFFER_SIZE)
  {
    length = USART1_RX_BUFFER_SIZE;
  }

  memcpy(frame, buffer, length);
  frame[length] = '\0';

  lx_command = strstr(frame, "LX");
  if (lx_command != NULL)
  {
    if (sscanf(lx_command, "LX%ld,Y%ld", &lx, &y) == 2)
    {
      has_lx = 1U;
    }
    else if (sscanf(lx_command, "LX%ld Y%ld", &lx, &y) == 2)
    {
      has_lx = 1U;
    }
  }

  rx_command = strstr(frame, "RX");
  if (rx_command != NULL)
  {
    if (sscanf(rx_command, "RX%ld,Y%ld", &rx, &y_dummy) == 2)
    {
      has_rx = 1U;
    }
    else if (sscanf(rx_command, "RX%ld Y%ld", &rx, &y_dummy) == 2)
    {
      has_rx = 1U;
    }
  }

  if ((has_lx != 0U) && (has_rx != 0U))
  {
    /* Combined command in one frame, order-independent. */
    PIDVelocity_SetChassisCommand((int32_t)lx, (int32_t)y, (int32_t)rx);
    return;
  }

  if (has_lx != 0U)
  {
    /* LX,Y command drives translation only and does not wait for W. */
    PIDVelocity_SetChassisCommand((int32_t)lx, (int32_t)y, 0);
    return;
  }

  if (has_rx != 0U)
  {
    /* RX,Y command maps RX to W and performs pure rotation. */
    PIDVelocity_SetChassisCommand(0, 0, (int32_t)rx);
    return;
  }

  /* If not a mecanum command, forward raw frame to USART2. */
  {
    size_t frame_len = strnlen(frame, sizeof(frame) - 1U);
    if (frame_len > 0U)
    {
      USART1_QueueText(frame);
    }
  }
}

static void USART1_QueueText(const char *text)
{
  UsartCommandMessage_t message;
  size_t copy_len;

  if ((text == NULL) || (Usart_SendingHandle == NULL))
  {
    return;
  }

  copy_len = strnlen(text, sizeof(message.text) - 1U);
  memcpy(message.text, text, copy_len);
  message.text[copy_len] = '\0';

  (void)osMessageQueuePut(Usart_SendingHandle, &message, 0U, 0U);
}

static uint8_t USART1_ParseAngleSpeedDeviceId(const char *cursor,
                                              float *angle,
                                              long *speed,
                                              long *device_id,
                                              int *consumed)
{
  char *endptr;
  const char *speed_pos;
  const char *id_pos;
  const char *value_pos;

  if ((cursor == NULL) || (angle == NULL) || (speed == NULL) || (device_id == NULL) || (consumed == NULL))
  {
    return 0U;
  }

  endptr = NULL;
  *angle = strtof(cursor + 6, &endptr);
  if ((endptr == (cursor + 6)) || (endptr == NULL))
  {
    return 0U;
  }

  speed_pos = strstr(endptr, "Speed");
  if (speed_pos == NULL)
  {
    speed_pos = strstr(endptr, "SPEED");
  }
  if (speed_pos == NULL)
  {
    speed_pos = strstr(endptr, "speed");
  }
  if (speed_pos == NULL)
  {
    return 0U;
  }

  value_pos = speed_pos + 5;
  while ((*value_pos != '\0') && ((*value_pos == ':') || (*value_pos == '=') || (*value_pos == ',') || isspace((unsigned char)*value_pos)))
  {
    value_pos++;
  }

  endptr = NULL;
  *speed = strtol(value_pos, &endptr, 10);
  if ((endptr == value_pos) || (endptr == NULL))
  {
    return 0U;
  }

  id_pos = strstr(endptr, "Device_Id");
  if (id_pos == NULL)
  {
    id_pos = strstr(endptr, "Device_ID");
  }
  if (id_pos == NULL)
  {
    id_pos = strstr(endptr, "DeviceId");
  }
  if (id_pos == NULL)
  {
    return 0U;
  }

  value_pos = id_pos;
  while ((*value_pos != '\0') && (*value_pos != ':') && (*value_pos != '=') && !isspace((unsigned char)*value_pos))
  {
    value_pos++;
  }
  while ((*value_pos != '\0') && ((*value_pos == ':') || (*value_pos == '=') || (*value_pos == ',') || isspace((unsigned char)*value_pos)))
  {
    value_pos++;
  }

  endptr = NULL;
  *device_id = strtol(value_pos, &endptr, 10);
  if ((endptr == value_pos) || (endptr == NULL))
  {
    return 0U;
  }

  *consumed = (int)(endptr - cursor);
  return 1U;
}

static void USART1_ExtractAndQueueForwardFrames(const char *frame)
{
  /* Placeholder: forwarding logic moved to USART1_ProcessFrame. */
  (void)frame;
}

void USART1_IdleReceiveIRQHandler(void)
{
  uint16_t received_length;

  if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_IDLE) == RESET)
  {
    return;
  }

  __HAL_UART_CLEAR_IDLEFLAG(&huart1);
  HAL_UART_DMAStop(&huart1);

  received_length = (uint16_t)(USART1_RX_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(huart1.hdmarx));
  USART1_StorePendingFrame(s_usart1_rx_buffer, received_length);
  USART1_RxRestart();
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1)
  {
    USART1_StorePendingFrame(s_usart1_rx_buffer, USART1_RX_BUFFER_SIZE);
    USART1_RxRestart();
  }
}

void USART1_PollAndProcessPendingFrame(void)
{
  uint16_t pending_length;
  uint8_t local_buffer[USART1_RX_BUFFER_SIZE];

  if (s_usart1_pending_ready == 0U)
  {
    return;
  }

  __disable_irq();
  if (s_usart1_pending_ready == 0U)
  {
    __enable_irq();
    return;
  }

  pending_length = s_usart1_pending_length;
  if (pending_length > USART1_RX_BUFFER_SIZE)
  {
    pending_length = USART1_RX_BUFFER_SIZE;
  }

  if (pending_length > 0U)
  {
    memcpy(local_buffer, s_usart1_pending_buffer, pending_length);
  }
  s_usart1_pending_ready = 0U;
  __enable_irq();

  if (pending_length > 0U)
  {
    USART1_ProcessFrame(local_buffer, pending_length);
  }
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

