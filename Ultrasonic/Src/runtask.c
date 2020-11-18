#include "runtask.h"
#include "main.h"
#include "distance.h"
#include "usart.h"
#include "tim.h"
#include "cmsis_os.h"
uint16_t resuft = 0;
distance_value_t data = {
    100, 2600, 2600, 1600};
extern osStatus osDelay (uint32_t millisec);
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
    for (uint8_t i = 0; i < 10; i++)
    {
        resuft = Distance_Caculate(data);
        delay(100);
    }
    for (;;)
    {
        resuft = Distance_Caculate(data);
        delay(1000);
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
        uint8_t buff[5], Buffer[5];
        sprintf(buff, "%d\n", resuft);
        serial_write(1, buff, 3);
        serial_Read(1, 5, Buffer);
        switch (Buffer[0])
        {
        case '1':
            data.start = ((Buffer[1] - 48) * 1000 +
                          (Buffer[2] - 48) * 100 +
                          (Buffer[3] - 48) * 10 +
                          (Buffer[4] - 48));
            Buffer[0] = NULL;
            break;
        case '2':
            data.stop = ((Buffer[1] - 48) * 1000 +
                         (Buffer[2] - 48) * 100 +
                         (Buffer[3] - 48) * 10 +
                         (Buffer[4] - 48));
            Buffer[0] = NULL;
            break;
        case '3':
            data.ampmax = ((Buffer[1] - 48) * 1000 +
                           (Buffer[2] - 48) * 100 +
                           (Buffer[3] - 48) * 10 +
                           (Buffer[4] - 48));
            Buffer[0] = NULL;
            break;
        case '4':
            SetDutyCycle_PWM(((Buffer[1] - 48) * 1000 +
                              (Buffer[2] - 48) * 100 +
                              (Buffer[3] - 48) * 10 +
                              (Buffer[4] - 48)));
            Buffer[0] = NULL;
            break;

        default:
            break;
        }
        osDelay(1000);
    }
    /* USER CODE END Start_Task_Serial */
}

/* USER CODE BEGIN Header_Start_Task_RS485 */
/**
* @brief Function implementing the Task_RS485 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_Task_RS485 */
void Start_Task_RS485(void const *argument)
{
    /* USER CODE BEGIN Start_Task_RS485 */
    /* Infinite loop */

    uint8_t Buffer_index[11];
    uint32_t pre = millis();
    uint8_t yes[4] = {0x06, 0x01, 0x01, 0x10}, no[4] = {0x06, 0x01, 0x01, 0x00};
    serial_write(2, "UART 2!!!\n", 10);
    for (;;)
    {
        if (millis() - pre > 30000)
        {
            for (uint8_t i = 0; i < 3; i++)
            {
                pre = millis();
                if (55 < resuft < 100)
                    serial_write(2, yes, 4);
                else
                    serial_write(2, no, 4);
                if (serial_Available(2))
                    serial_Read(2, 11, Buffer_index);
                delay(2000);
                if (Buffer_index[3] == 0x02)
                    break;
            }
        }
        for (uint8_t i = 3; i < 11; i++)
        {
            if (Buffer_index[i] != 0 && Buffer_index[2] == i - 2)
            {
                for (uint8_t i = 0; i < 11; i++)
                    Buffer_index[i] = NULL;
                break;
            }
        }
        delay(1);
    }
    /* USER CODE END Start_Task_RS485 */
}
