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
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "can.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define DEVICE_USTICKER 1
#define delay(...) HAL_Delay(__VA_ARGS__)
#define GREEN_LED "B7"
#define RED_LED "B6"
#define BLUE_LED "B5"
#define BUTTON1 "A1"
#define HIGH GPIO_PIN_SET
#define LOW GPIO_PIN_RESET
#define TRUE 1
#define FALSE 0
#define TIMEOUT_UART1 100
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint32_t tick_micros = 0;                                    // micros value
char Rx_indx1, Rx_data1[2], Rx_Buffer1[100], Transfer_cplt1; // Uart RX Message 1
char Rx_indx2, Rx_data2[2], Rx_Buffer2[100], Transfer_cplt2; // Uart RX Message 2
uint32_t last_uart1 = 0, last_uart2 = 0;                     // Last time recieve uart
extern CAN_HandleTypeDef hcan;
CAN_RxHeaderTypeDef rxHeader;                //CAN Bus Receive Header
CAN_TxHeaderTypeDef txHeader;                //CAN Bus Transmit Header
uint8_t canRX[8] = {0, 1, 2, 3, 4, 5, 6, 7}; //CAN Bus Receive Buffer
CAN_FilterTypeDef canfil;                    //CAN Bus Filter
uint32_t canMailbox;                         //CAN Bus Mail box variable
char isCanAvailable = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// Setup GPIO
void digitalWrite(char LedPin[3], int Value)
{
  if (LedPin[0] == 'A')
  {
    HAL_GPIO_WritePin(GPIOA, LedPin[1] - 32, Value);
  }
  if (LedPin[0] == 'B')
  {
    HAL_GPIO_WritePin(GPIOB, LedPin[1] - 32, Value);
  }
}
// Read GPIO State
char digitalRead(char pin[3])
{
  if (pin[0] == 'A')
  {
    return HAL_GPIO_ReadPin(GPIOA, pin[1] - 32);
  }
  if (pin[0] == 'B')
  {
    return HAL_GPIO_ReadPin(GPIOB, pin[1] - 32);
  }
}
// Set PWM 0 - 100
void analogWrite(uint8_t pwm)
{
  TIM_OC_InitTypeDef sConfigOC;
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = pwm * 12;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
}
// Timer Interrupt
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM14) //check if the interrupt comes from TIM1 for 1 micros
  {
    tick_micros++;
  }
}
// Return micros from chip start
uint32_t micros()
{
  return tick_micros;
}
// Return millis from chip start
uint32_t millis()
{
  return HAL_GetTick();
}
// Write Serial port
void serial_write(int port, char *text)
{
  if (port == 1)
  {
    HAL_UART_Transmit(&huart1, text, (uint16_t)strlen(text), 1000);
  }
  else
  {
    HAL_UART_Transmit(&huart2, text, (uint16_t)strlen(text), 1000);
  }
}
// Uart Recive Interrupt
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  uint8_t i;

  // Uart 1
  if (huart->Instance == USART1) //current UART1
  {
    if (Rx_indx1 == 0)
    {
      for (i = 0; i < 100; i++)
        Rx_Buffer1[i] = 0;
    } //clear Rx_Buffer before receiving new data

    //if (Rx_data[0]!=13) //if received data different from ascii 13 (enter)
    if (millis() - last_uart1 < TIMEOUT_UART1 || Rx_indx1 == 0)
    {
      Rx_Buffer1[Rx_indx1++] = Rx_data1[0]; //add data to Rx_Buffer
    }
    else //if received data = 13
    {
      Rx_indx1 = 0;
      Transfer_cplt1 = 1; //transfer complete, data is ready to read
    }
    last_uart1 = millis();
    HAL_UART_Receive_IT(&huart1, Rx_data1, 1); //activate UART receive interrupt every time
  }

  // Uart 2
  if (huart->Instance == USART2) //current UART1
  {
    if (Rx_indx2 == 0)
    {
      for (i = 0; i < 100; i++)
        Rx_Buffer2[i] = 0;
    } //clear Rx_Buffer before receiving new data

    //if (Rx_data[0]!=13) //if received data different from ascii 13 (enter)
    if (millis() - last_uart2 < TIMEOUT_UART1 || Rx_indx2 == 0)
    {
      Rx_Buffer2[Rx_indx2++] = Rx_data2[0]; //add data to Rx_Buffer
    }
    else //if received data = 13
    {
      Rx_indx2 = 0;
      Transfer_cplt2 = 1; //transfer complete, data is ready to read
    }
    last_uart2 = millis();
    HAL_UART_Receive_IT(&huart2, Rx_data2, 1); //activate UART receive interrupt every time
  }
}
// Read Serial data buffer
char *serial_Read(int uart)
{
  if (uart == 1)
  {
    return Rx_Buffer1;
  }
  else
  {
    return Rx_Buffer2;
  }
}
// Check data ready to read
int serial_Available(int uart)
{
  if (uart == 1)
  {
    if (Transfer_cplt1 > 0)
    {
      Transfer_cplt1 = 0;
      return 1;
    }
    else
    {
      return 0;
    }
  }
  else
  {
    if (Transfer_cplt2 > 0)
    {
      Transfer_cplt2 = 0;
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
char can_Available()
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
  MX_ADC_Init();
  MX_CAN_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_TIM14_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */
	HAL_UART_Receive_IT(&huart2, Rx_data2, 1);
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
    HAL_GPIO_WritePin(GPIOA,GPIO_PIN_8,GPIO_PIN_RESET);
    delay(10);
    serial_write(2, "123");
    HAL_GPIO_WritePin(GPIOA,GPIO_PIN_8,GPIO_PIN_SET);
		delay(10);
      serial_Read(2);
    delay(100);
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI14 | RCC_OSCILLATORTYPE_HSE;
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
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1;
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
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
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
