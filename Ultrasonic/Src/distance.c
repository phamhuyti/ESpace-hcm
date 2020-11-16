#include "distance.h"
#include "gpio.h"
#include "tim.h"
float s = 0;
uint32_t tem;
uint16_t Delta, Pre;
float Distance_Caculate(uint16_t start, uint16_t stop, uint16_t ampmax, uint16_t ampmin)
{
    Delta = 0, Pre = 0;
    float resuft = 0;
    uint8_t div = 0;
    for (uint8_t i = 0; i < 5; i++)
    {
        Pre = micros();
        Pwm_Start();
        delay_us(start);
        Pwm_Stop();
        delay_us(stop);
        uint32_t Timeout = millis();
        do
        {
            tem = Adc_buffer[1];
        } while (((tem < ampmax)&&(tem > ampmin)) && millis() - Timeout < 100);//1613
        if (((tem > ampmax)||(tem < ampmin)) && millis() - Timeout < 100)
        {
            Delta = micros() - Pre;
            s = (float)(Delta / 2000.0) * 0.330;
            if ( s>3.5 )
            {
                s = 99999999;
                digitalWrite(BLUE_LED, HIGH);
            }
            else
            {
                div++;
                resuft += s;
            }
        }
        else
            digitalWrite(RED_LED, HIGH);
        delay(50);
    }
    digitalWrite(RED_LED, LOW);
    digitalWrite(BLUE_LED, LOW);
    return (resuft / div);
}
