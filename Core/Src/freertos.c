/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "usart.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "pid_velocity.h"

#include <stdio.h>
#include <string.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Motor_Run */
osThreadId_t Motor_RunHandle;
const osThreadAttr_t Motor_Run_attributes = {
  .name = "Motor_Run",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for Usart_Send */
osThreadId_t Usart_SendHandle;
const osThreadAttr_t Usart_Send_attributes = {
  .name = "Usart_Send",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for Motor_Running */
osMessageQueueId_t Motor_RunningHandle;
const osMessageQueueAttr_t Motor_Running_attributes = {
  .name = "Motor_Running"
};
/* Definitions for Usart_Sending */
osMessageQueueId_t Usart_SendingHandle;
const osMessageQueueAttr_t Usart_Sending_attributes = {
  .name = "Usart_Sending"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void StartTask02(void *argument);
void StartTask03(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of Motor_Running */
  Motor_RunningHandle = osMessageQueueNew (16, sizeof(uint16_t), &Motor_Running_attributes);

  /* creation of Usart_Sending */
  Usart_SendingHandle = osMessageQueueNew (16, sizeof(UsartCommandMessage_t), &Usart_Sending_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of Motor_Run */
  Motor_RunHandle = osThreadNew(StartTask02, NULL, &Motor_Run_attributes);

  /* creation of Usart_Send */
  Usart_SendHandle = osThreadNew(StartTask03, NULL, &Usart_Send_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartTask02 */
/**
* @brief Function implementing the Motor_Run thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask02 */
void StartTask02(void *argument)
{
  /* USER CODE BEGIN StartTask02 */
  PIDVelocityConfig_t pidConfig[PID_VELOCITY_COUNT];
  PIDVelocity_GetDefaultConfig(pidConfig);
  PIDVelocity_Init(pidConfig);
  PIDVelocity_ResetAll();

  const uint32_t controlPeriodMs = 10U;

  /* Infinite loop */
  for(;;)
  {
    PIDVelocity_RunStep();
    osDelay(controlPeriodMs);
  }
  /* USER CODE END StartTask02 */
}

/* USER CODE BEGIN Header_StartTask03 */
/**
* @brief Function implementing the Usart_Send thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask03 */
void StartTask03(void *argument)
{
  /* USER CODE BEGIN StartTask03 */
  UsartCommandMessage_t rx_msg;

  /* Infinite loop */
  for(;;)
  {
    if (osMessageQueueGet(Usart_SendingHandle, &rx_msg, NULL, osWaitForever) == osOK)
    {
      size_t tx_len = strnlen(rx_msg.text, sizeof(rx_msg.text));

      if (tx_len > 0U)
      {
        (void)HAL_UART_Transmit(&huart2, (uint8_t *)rx_msg.text, (uint16_t)tx_len, 100U);
      }
    }
  }
  /* USER CODE END StartTask03 */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

