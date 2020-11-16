/* USER CODE BEGIN Header */
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
#include "FLASH_PAGE.h"
/* USER CODE END Includes */

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
float resuft = 0, tem;
distance_value_t data = {
    100, 2600, 2600, 1600};

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
 uint8_t data2 = 'H';

__IO uint32_t Rx_Data[4];
void MX_FREERTOS_Init(void)
{
  Flash_Write_Data(0x0801FC00, (uint32_t*)data2);

  Flash_Read_Data(0x0801FC00, Rx_Data);
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
  Init_filter();

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
    resuft = Distance_Caculate(data);
    delay(100);
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
    // uint8_t Temperature;
    // uint8_t Temperature_Ext;
    // float vsense = 3.3 / 4095;
    // Temperature = ((Adc_buffer[2] * vsense - 1.43) / 4.3) + 25;
    // float tem = Adc_buffer[0] * 3.3 / 4095;
    // Temperature_Ext = map(tem, 1.6, 0, -30, 125);
    digitalWrite(GREEN_LED, HIGH);
    delay(500);
    digitalWrite(GREEN_LED, LOW);
    delay(500);
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
      data.start = ((Rx_Buffer1[1] - 48) * 1000 +
                    (Rx_Buffer1[2] - 48) * 100 +
                    (Rx_Buffer1[3] - 48) * 10 +
                    (Rx_Buffer1[4] - 48));
      Rx_Buffer1[0] = NULL;
      break;
    case '2':
      data.stop = ((Rx_Buffer1[1] - 48) * 1000 +
                   (Rx_Buffer1[2] - 48) * 100 +
                   (Rx_Buffer1[3] - 48) * 10 +
                   (Rx_Buffer1[4] - 48));
      Rx_Buffer1[0] = NULL;
      break;
    case '3':
      data.ampmax = ((Rx_Buffer1[1] - 48) * 1000 +
                     (Rx_Buffer1[2] - 48) * 100 +
                     (Rx_Buffer1[3] - 48) * 10 +
                     (Rx_Buffer1[4] - 48));
      Rx_Buffer1[0] = NULL;
      break;
    case '4':
      SetDutyCycle_PWM(((Rx_Buffer1[1] - 48) * 1000 +
                        (Rx_Buffer1[2] - 48) * 100 +
                        (Rx_Buffer1[3] - 48) * 10 +
                        (Rx_Buffer1[4] - 48)));
      Rx_Buffer1[0] = NULL;
      break;

    default:
      break;
    }
    osDelay(10000);
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
