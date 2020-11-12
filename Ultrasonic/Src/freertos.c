/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
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
osThreadId SerialHandle;
osThreadId LEDHandle;
extern uint8_t Rx_Buffer1[128];
extern uint8_t Rx_Buffer2[128];
/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartSerial(void const *argument);
void StartLED(void const *argument);

extern void serial_Read(uint8_t uart, uint8_t size);
extern int serial_Available(int uart);
extern void serial_write(int port, uint8_t *text);
extern void analogWrite(uint8_t pwm);
extern GPIO_PinState digitalRead(char pin[2]);
extern void digitalWrite(char LedPin[3], GPIO_PinState Value);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */
/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize);

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize)
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
void MX_FREERTOS_Init(void)
{
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
  /* definition and creation of Serial */
  osThreadDef(Serial, StartSerial, osPriorityNormal, 0, 128);
  SerialHandle = osThreadCreate(osThread(Serial), NULL);

  /* definition and creation of LED */
  osThreadDef(LED, StartLED, osPriorityRealtime, 0, 128);
  LEDHandle = osThreadCreate(osThread(LED), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */
}

/* USER CODE BEGIN Header_StartSerial */
/**
  * @brief  Function implementing the Serial thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartSerial */
void StartSerial(void const *argument)
{
  /* USER CODE BEGIN StartSerial */
  /* Infinite loop */
  digitalWrite("A8", GPIO_PIN_RESET);
  osDelay(100);
  serial_write(2, (uint8_t *)"HELLO UART 111112!!!");
  osDelay(100);
  serial_write(2, (uint8_t *)"123");
  osDelay(100);
  digitalWrite("A8", GPIO_PIN_SET);
  osDelay(100);
  for (;;)
  {
    serial_Read(2, 9);
    osDelay(100);
  }
  /* USER CODE END StartSerial */
}

/* USER CODE BEGIN Header_StartLED */
/**
* @brief Function implementing the LED thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartLED */
void StartLED(void const *argument)
{
  /* USER CODE BEGIN StartLED */
  /* Infinite loop */
  for (;;)
  {
    //analogWrite(50);
    HAL_GPIO_TogglePin(BLUE_LED_GPIO_Port, BLUE_LED_Pin);
    osDelay(1000);
    HAL_GPIO_TogglePin(BLUE_LED_GPIO_Port, GREEN_LED_Pin);
    osDelay(1000);
    HAL_GPIO_TogglePin(BLUE_LED_GPIO_Port, RED_LED_Pin);
    osDelay(1000);
  }
  /* USER CODE END StartLED */
}
/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
