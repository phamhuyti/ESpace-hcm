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
extern TIM_HandleTypeDef htim17;
extern ADC_HandleTypeDef hadc;
/* USER CODE END Variables */
osThreadId Ultrasonic_Calculate_Handle;
osThreadId LEDHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

extern void Pwm_Start(void);
extern void Pwm_Stop(void);
extern uint64_t micros(void);
extern GPIO_PinState digitalRead(char pin[2]);
extern void digitalWrite(char LedPin[3], GPIO_PinState Value);
extern void serial_write(int port, uint8_t *text);
extern void serial_Read(uint8_t uart, uint8_t size);
extern uint8_t serial_Available(int uart);
extern void delay_us(uint64_t time);
extern float map(float x, float in_min, float in_max, float out_min, float out_max);
uint32_t buffer[3];
float Temperature;
float Temperature_Ext;
uint64_t a = 0, b = 0, pre = 0;
float s = 0;
/* USER CODE END FunctionPrototypes */

void Ultrasonic_Calculate(void const *argument);
void StartLED(void const *argument);

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
  osThreadDef(Serial, Ultrasonic_Calculate, osPriorityNormal, 0, 128);
  Ultrasonic_Calculate_Handle = osThreadCreate(osThread(Serial), NULL);

  /* definition and creation of LED */
  osThreadDef(LED, StartLED, osPriorityIdle, 0, 128);
  LEDHandle = osThreadCreate(osThread(LED), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  // HAL_ADC_Start(&hadc);
  HAL_ADC_Start_DMA(&hadc, buffer, 3);
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
void Ultrasonic_Calculate(void const *argument)
{
  /* USER CODE BEGIN StartSerial */
  /* Infinite loop */
  uint64_t i;
  for (;;)
  {
    i = 0;
    pre = micros();
    Pwm_Start();
    delay_us(500);
    Pwm_Stop();
    delay_us(2500);
    do
    {
      i++;
      delay_us(1);
      // HAL_ADC_PollForConversion(&hadc,1);
      // buffer[0] = HAL_ADC_GetValue(&hadc);
      // HAL_ADC_PollForConversion(&hadc,1);
      // buffer[1] = HAL_ADC_GetValue(&hadc);
      // HAL_ADC_PollForConversion(&hadc,1);
      // buffer[2] = HAL_ADC_GetValue(&hadc);
    } while (buffer[1] < 3500 && i < 3000);
    if (i < 3000)
    {
      // b = micros();
      a = micros() - pre;
      s = (float)(a / 2000.0) * 0.320;
    }
    else
      s = 9999999;
    delay(100);
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
    float vsense = 3.3 / 4095;
    Temperature = ((buffer[2] * vsense - 1.43) / 4.3) + 25;
    float tem = buffer[0] * 3.3 / 4095;
    Temperature_Ext = map(tem, 1.6, 0, -30, 125);
  }
  /* USER CODE END StartLED */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

// Called when first half of buffer is filled
// void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc)
// {
//   HAL_GPIO_WritePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin, GPIO_PIN_SET);
// }

// // Called when buffer is completely filled
// void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
// {
//   HAL_GPIO_WritePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin, GPIO_PIN_RESET);
//   for (uint8_t i = 0; i < 3; i++)
//   {
//     adcBuffer[i]=buffer[i];
//   }
//   float vsense = 3.3/4095;
//   Temperature = ((buffer[2]*vsense-1.43)/4.3)+25;
//   Temperature_Ext = (float)buffer[0]*3.3/4095;
// }

/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
