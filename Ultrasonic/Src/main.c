/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"
#include "cmsis_os.h"
#include "adc.h"
#include "can.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

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

/* USER CODE BEGIN PV */
uint64_t tick_milis = 0; // micros value
uint8_t Rx_Buffer1[128]; // Uart RX Message 1
uint8_t Rx_Buffer2[28];  // Uart RX Message 2
extern CAN_HandleTypeDef hcan;
CAN_RxHeaderTypeDef rxHeader;                //CAN Bus Receive Header
CAN_TxHeaderTypeDef txHeader;                //CAN Bus Transmit Header
uint8_t canRX[8] = {0, 1, 2, 3, 4, 5, 6, 7}; //CAN Bus Receive Buffer
CAN_FilterTypeDef canfil;                    //CAN Bus Filter
uint32_t canMailbox;                         //CAN Bus Mail box variable
uint8_t isCanAvailable = 0;
uint8_t stt = 0;
uint64_t counter = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// Setup GPIO
void digitalWrite(char LedPin[3], GPIO_PinState Value)
{
  HAL_GPIO_WritePin(((LedPin[0] == 'A') ? GPIOA : GPIOB), (uint16_t)(1 << (LedPin[1]) - 48), Value);
}
// Read GPIO State
GPIO_PinState digitalRead(char pin[2])
{
  if (pin[0] == 'B')
    HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = (uint16_t)(1 << (pin[1]) - 48);
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(((pin[0] == 'A') ? GPIOA : GPIOB), &GPIO_InitStruct);
  return HAL_GPIO_ReadPin(((pin[0] == 'A') ? GPIOA : GPIOB), (uint16_t)(1 << (pin[1]) - 48));
}
// Start PWM
void Pwm_Start(void)
{
  stt = 1;
  htim3.Instance->CCR1 = 50;
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
}

// Stop PWM
void Pwm_Stop(void)
{
  stt = 0;
  HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
}
// Set PWM 0 - 100
void SetDutyCycle_PWM(uint8_t pwm)
{
  TIM_OC_InitTypeDef sConfigOC;
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = pwm;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1);
  // HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
}
// Return micros from chip start
uint64_t micros(void)
{
  return (tick_milis * 1000 + __HAL_TIM_GET_COUNTER(&htim17));
}
//Delay time microsecond
void delay_us(uint64_t time)
{
  counter = time+ micros();
  while (counter>micros())
    ;
}
// Return millis from chip start
uint32_t millis(void)
{
  return HAL_GetTick();
}
// Write Serial port
void serial_write(int port, uint8_t *text)
{
  if (port == 1)
  {
    HAL_UART_Transmit_DMA(&huart1, text, (uint16_t)(strlen(text))); //check strlen variables main.h
  }
  else
  {
    digitalWrite(DATARX, LOW);
    HAL_UART_Transmit_DMA(&huart2, text, (uint16_t)strlen(text));
  }
}
// Read Serial data buffer
void serial_Read(uint8_t uart, uint8_t size)
{
  if (uart == 1)
  {
    HAL_UART_Receive_DMA(&huart1, Rx_Buffer1, size);
  }
  else
  {
    HAL_UART_AbortReceive(&huart2);
    digitalWrite(DATARX, HIGH);
    HAL_UART_Receive_DMA(&huart2, Rx_Buffer2, size);
  }
}
// Check data ready to read
uint8_t serial_Available(int uart)
{
  if (uart == 1)
  {
    if (hdma_usart1_rx.Lock == HAL_UNLOCKED)
    {
      return 1;
    }
    else
    {
      return 0;
    }
  }
  else
  {
    if (hdma_usart2_rx.Lock == HAL_UNLOCKED)
    {
      return 1;
    }
    else
    {
      return 0;
    }
  }
}
// CAN filter
void can_init()
{
  canfil.FilterBank = 0;
  canfil.FilterMode = CAN_FILTERMODE_IDMASK;
  canfil.FilterFIFOAssignment = CAN_RX_FIFO0;
  canfil.FilterIdHigh = 0;
  canfil.FilterIdLow = 0;
  canfil.FilterMaskIdHigh = 0;
  canfil.FilterMaskIdLow = 0;
  canfil.FilterScale = CAN_FILTERSCALE_32BIT;
  canfil.FilterActivation = ENABLE;
  canfil.SlaveStartFilterBank = 14;
  HAL_CAN_ConfigFilter(&hcan, &canfil);                             //Initialize CAN Filter
  HAL_CAN_Start(&hcan);                                             //Initialize CAN Bus
  HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING); // Initialize CAN Bus Rx Interrupt
}
// Send CAN message
void can_send(uint32_t id, uint8_t *data, uint32_t len)
{
  CAN_TxHeaderTypeDef sendmssg;
  sendmssg.DLC = len;
  sendmssg.IDE = CAN_ID_STD;
  sendmssg.RTR = CAN_RTR_DATA;
  sendmssg.StdId = id;
  sendmssg.TransmitGlobalTime = DISABLE;
  HAL_CAN_AddTxMessage(&hcan, &sendmssg, data, &canMailbox); // Send Message
}
// Recieve CAN message
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan1)
{
  HAL_CAN_GetRxMessage(hcan1, CAN_RX_FIFO0, &rxHeader, canRX); //Receive CAN bus message to canRX buffer
  isCanAvailable = 1;
}
// Check CAN available
uint8_t can_Available()
{
  return isCanAvailable;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC_Init();
  MX_CAN_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_TIM17_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim17);
  // SetDutyCycle_PWM(100);
  /* USER CODE END 2 */

  /* Call init function for freertos objects (in freertos.c) */
  MX_FREERTOS_Init();
  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI14|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSI14State = RCC_HSI14_ON;
  RCC_OscInitStruct.HSI14CalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
    if(counter>0) counter--;
  }
  /* USER CODE BEGIN Callback 1 */
  if (htim->Instance == TIM17) //check if the interrupt comes from TIM17 for 1000 micros
  {
    tick_milis++;
  }
  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
