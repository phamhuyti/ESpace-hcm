#include "distance.h"

uint16_t values_bfr[10];
uint16_t sort_bfr[10];
median_filter_t filter;
void Init_filter(void)
{
    median_filter_init(&filter, 10, values_bfr, sort_bfr);
}

uint16_t Distance_Caculate(distance_value_t value)
{
    float s = 0;
    uint16_t Delta, Pre;
    Delta = 0, Pre = 0;
    float resuft = 0;
    uint32_t tem;
    Pre = micros();
    Pwm_Start();
    delay_us(value.start);
    Pwm_Stop();
    delay_us(value.stop);
    uint32_t Timeout = millis();
    do
    {
        tem = Adc_buffer[1];
    } while (((tem < value.ampmax) && (tem > value.ampmin)) && millis() - Timeout < 100);
    if (((tem > value.ampmax) || (tem < value.ampmin)) && millis() - Timeout < 100)
    {
        Delta = micros() - Pre;
        Delta = median_filter_add_new_value(&filter, (uint16_t)(Delta));
        s = (float)(Delta / 2000.0) * 0.34902;
        if (s > 3.5)
        {
            s = 99999999;
            digitalWrite(BLUE_LED, HIGH);
        }
        else
        {
            resuft += s;
        }
    }
    else
        digitalWrite(RED_LED, HIGH);
    delay(50);
    digitalWrite(RED_LED, LOW);
    digitalWrite(BLUE_LED, LOW);
    return (resuft*100);
}
