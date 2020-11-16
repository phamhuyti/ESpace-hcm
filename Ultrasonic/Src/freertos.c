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
#include "distance.h"
#include "gpio.h"
#include "usart.h"
#include "median.h"
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
extern ADC_HandleTypeDef hadc;
/* USER CODE END Variables */
osThreadId Task_UltrasonicHandle;
osThreadId Task_IdleHandle;
osThreadId Task_SerialHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
float map(float x, float in_min, float in_max, float out_min, float out_max);
uint8_t Temperature;
uint8_t Temperature_Ext;
float resuft = 0;
uint16_t TimeStart = 500,
         TimeStop = 2600,
         AmpMax = 2500,
         AmpMin = 1450;
median_filter_t filter;
float values_bfr[5];
float sort_bfr[5];
/* USER CODE END FunctionPrototypes */

void Start_Ultrasonic_Calculate(void const *argument);
void Start_Idle(void const *argument);
void Start_Task_Serial(void const *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize);

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer,
                                   StackType_t **ppxIdleTaskStackBuffer,
                                   uint32_t *pulIdleTaskStackSize)
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
  /* definition and creation of Task_Ultrasonic */
  osThreadDef(Task_Ultrasonic, Start_Ultrasonic_Calculate, osPriorityNormal, 0, 128);
  Task_UltrasonicHandle = osThreadCreate(osThread(Task_Ultrasonic), NULL);

  /* definition and creation of Task_Idle */
  osThreadDef(Task_Idle, Start_Idle, osPriorityIdle, 0, 128);
  Task_IdleHandle = osThreadCreate(osThread(Task_Idle), NULL);

  /* definition and creation of Task_Serial */
  osThreadDef(Task_Serial, Start_Task_Serial, osPriorityNormal, 0, 128);
  Task_SerialHandle = osThreadCreate(osThread(Task_Serial), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  median_filter_init(&filter, 5, values_bfr, sort_bfr);
  /* USER CODE END RTOS_THREADS */
}

/* USER CODE BEGIN Header_Start_Ultrasonic_Calculate */
/**
  * @brief  Function implementing the Task_Ultrasonic thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_Start_Ultrasonic_Calculate */
void Start_Ultrasonic_Calculate(void const *argument)
{
  /* USER CODE BEGIN Start_Ultrasonic_Calculate */
  /* Infinite loop */
  for (;;)
  {
    resuft = median_filter_add_new_value(&filter, (Distance_Caculate(TimeStart, TimeStop, AmpMax, AmpMin)));
    // Pwm_Start();
    // delay_us(TimeStart);
    // Pwm_Stop();
    // delay_us(TimeStop);
    // delay(50);
  }
  /* USER CODE END Start_Ultrasonic_Calculate */
}

/* USER CODE BEGIN Header_Start_Idle */
/**
* @brief Function implementing the Task_Idle thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_Idle */
void Start_Idle(void const *argument)
{
  /* USER CODE BEGIN Start_Idle */
  /* Infinite loop */
  for (;;)
  {
    float vsense = 3.3 / 4095;
    Temperature = ((Adc_buffer[2] * vsense - 1.43) / 4.3) + 25;
    float tem = Adc_buffer[0] * 3.3 / 4095;
    Temperature_Ext = map(tem, 1.6, 0, -30, 125);
    digitalWrite(GREEN_LED, HIGH);
    delay(100);
    digitalWrite(GREEN_LED, LOW);
    delay(100);
  }
  /* USER CODE END Start_Idle */
}

/* USER CODE BEGIN Header_Start_Task_Serial */
/**
* @brief Function implementing the Task_Serial thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_Task_Serial */
void Start_Task_Serial(void const *argument)
{
  /* USER CODE BEGIN Start_Task_Serial */
  /* Infinite loop */
  for (;;)
  {
    uint8_t buff[5];
    sprintf(buff, "%.2f\n", resuft);
    serial_write(1, buff);
    serial_Read(1, 5);
    switch (Rx_Buffer1[0])
    {
    case '1':
      TimeStart = ((Rx_Buffer1[1] - 48) * 10 +
                   (Rx_Buffer1[2] - 48)) *
                  100;
      Rx_Buffer1[0] = NULL;
      break;
    case '2':
      TimeStop = ((Rx_Buffer1[1] - 48) * 10 + (Rx_Buffer1[2] - 48)) * 100;
      Rx_Buffer1[0] = NULL;
      break;
    case '3':
      AmpMax = ((Rx_Buffer1[1] - 48) * 1000 +
                (Rx_Buffer1[2] - 48) * 100 +
                (Rx_Buffer1[3] - 48) * 10 +
                (Rx_Buffer1[4] - 48));
      Rx_Buffer1[0] = NULL;
      break;
    case '4':
      SetDutyCycle_PWM((((Rx_Buffer1[1] - 48) * 10 + (Rx_Buffer1[2] - 48))));
      Rx_Buffer1[0] = NULL;
      break;

    default:
      break;
    }
    osDelay(100);
  }
  /* USER CODE END Start_Task_Serial */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

float map(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
