/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
osThreadId defaultTaskHandle;
osThreadId DJI_Motor_TaskHandle;
osThreadId Mec_Arm_TaskHandle;
osThreadId Chassis_TaskHandle;
osThreadId RC_TaskHandle;
osThreadId Navigation_TaskHandle;
osThreadId Task_FlowHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void DJI_Motor_Ctrl_Task_Entry(void const * argument);
void Mec_Arm_Ctrl_Task_Entry(void const * argument);
void Chassis_Task_Entry(void const * argument);
void RC_Task_Entry(void const * argument);
void Navigation_Task_Entry(void const * argument);
void Task_Flow_Entry(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

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

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of DJI_Motor_Task */
  osThreadDef(DJI_Motor_Task, DJI_Motor_Ctrl_Task_Entry, osPriorityLow, 0, 128);
  DJI_Motor_TaskHandle = osThreadCreate(osThread(DJI_Motor_Task), NULL);

  /* definition and creation of Mec_Arm_Task */
  osThreadDef(Mec_Arm_Task, Mec_Arm_Ctrl_Task_Entry, osPriorityNormal, 0, 128);
  Mec_Arm_TaskHandle = osThreadCreate(osThread(Mec_Arm_Task), NULL);

  /* definition and creation of Chassis_Task */
  osThreadDef(Chassis_Task, Chassis_Task_Entry, osPriorityNormal, 0, 128);
  Chassis_TaskHandle = osThreadCreate(osThread(Chassis_Task), NULL);

  /* definition and creation of RC_Task */
  osThreadDef(RC_Task, RC_Task_Entry, osPriorityHigh, 0, 128);
  RC_TaskHandle = osThreadCreate(osThread(RC_Task), NULL);

  /* definition and creation of Navigation_Task */
  osThreadDef(Navigation_Task, Navigation_Task_Entry, osPriorityNormal, 0, 128);
  Navigation_TaskHandle = osThreadCreate(osThread(Navigation_Task), NULL);

  /* definition and creation of Task_Flow */
  osThreadDef(Task_Flow, Task_Flow_Entry, osPriorityHigh, 0, 128);
  Task_FlowHandle = osThreadCreate(osThread(Task_Flow), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
		HAL_GPIO_TogglePin(LED1_GPIO_Port,LED1_Pin); //因为使用3S供电 板上红色POWER灯不亮 以此来判断是否开机
    osDelay(500);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_DJI_Motor_Ctrl_Task_Entry */
/**
* @brief Function implementing the DJI_Motor_Ctrl_ thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_DJI_Motor_Ctrl_Task_Entry */
__weak void DJI_Motor_Ctrl_Task_Entry(void const * argument)
{
  /* USER CODE BEGIN DJI_Motor_Ctrl_Task_Entry */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END DJI_Motor_Ctrl_Task_Entry */
}

/* USER CODE BEGIN Header_Mec_Arm_Ctrl_Task_Entry */
/**
* @brief Function implementing the Mec_Arm_Ctrl_Ta thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Mec_Arm_Ctrl_Task_Entry */
__weak void Mec_Arm_Ctrl_Task_Entry(void const * argument)
{
  /* USER CODE BEGIN Mec_Arm_Ctrl_Task_Entry */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Mec_Arm_Ctrl_Task_Entry */
}

/* USER CODE BEGIN Header_Chassis_Task_Entry */
/**
* @brief Function implementing the Chassis_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Chassis_Task_Entry */
__weak void Chassis_Task_Entry(void const * argument)
{
  /* USER CODE BEGIN Chassis_Task_Entry */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Chassis_Task_Entry */
}

/* USER CODE BEGIN Header_RC_Task_Entry */
/**
* @brief Function implementing the RC_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_RC_Task_Entry */
__weak void RC_Task_Entry(void const * argument)
{
  /* USER CODE BEGIN RC_Task_Entry */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END RC_Task_Entry */
}

/* USER CODE BEGIN Header_Navigation_Task_Entry */
/**
* @brief Function implementing the Navigation_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Navigation_Task_Entry */
__weak void Navigation_Task_Entry(void const * argument)
{
  /* USER CODE BEGIN Navigation_Task_Entry */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Navigation_Task_Entry */
}

/* USER CODE BEGIN Header_Task_Flow_Entry */
/**
* @brief Function implementing the Task_Flow thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Task_Flow_Entry */
__weak void Task_Flow_Entry(void const * argument)
{
  /* USER CODE BEGIN Task_Flow_Entry */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Task_Flow_Entry */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
