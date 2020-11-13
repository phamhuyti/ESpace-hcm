/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C"
{
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx_hal.h"

  /* Private includes ----------------------------------------------------------*/
  /* USER CODE BEGIN Includes */

  /* USER CODE END Includes */

  /* Exported types ------------------------------------------------------------*/
  /* USER CODE BEGIN ET */

  /* USER CODE END ET */

  /* Exported constants --------------------------------------------------------*/
  /* USER CODE BEGIN EC */

  /* USER CODE END EC */

  /* Exported macro ------------------------------------------------------------*/
  /* USER CODE BEGIN EM */

  /* USER CODE END EM */

  /* Exported functions prototypes ---------------------------------------------*/
  void Error_Handler(void);

  /* USER CODE BEGIN EFP */
  extern uint8_t Rx_Buffer1[128]; // Uart RX Message 1
  extern uint8_t Rx_Buffer2[128]; // Uart RX Message 2
  extern uint32_t Adc_buffer[3];

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define BUTTON_1_Pin GPIO_PIN_1
#define BUTTON_1_GPIO_Port GPIOA
#define BUTTON_2_Pin GPIO_PIN_0
#define BUTTON_2_GPIO_Port GPIOB
#define DATA_IN_OUT_Pin GPIO_PIN_8
#define DATA_IN_OUT_GPIO_Port GPIOA
#define BLUE_LED_Pin GPIO_PIN_5
#define BLUE_LED_GPIO_Port GPIOB
#define GREEN_LED_Pin GPIO_PIN_6
#define GREEN_LED_GPIO_Port GPIOB
#define RED_LED_Pin GPIO_PIN_7
#define RED_LED_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */
#define delay(...) osDelay(__VA_ARGS__)
#define GREEN_LED "B7"
#define RED_LED "B6"
#define BLUE_LED "B5"
#define BUTTON1 "A1"
#define BUTTON2 "B0"
#define ULTRASONIC_IN "A7"
#define DATARX_PIN "A8"
#define HIGH GPIO_PIN_SET
#define LOW GPIO_PIN_RESET
#define TRUE 1
#define FALSE 0
  extern int strlen(uint8_t *);

  /* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
