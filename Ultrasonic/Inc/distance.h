#ifndef DISTANCE_H_
#define DISTANCE_H_

#include "main.h"
#include "gpio.h"
#include "tim.h"
#include "median.h"
typedef struct _distance_value
{
    uint16_t start;
    uint16_t stop;
    uint16_t ampmax;
    uint16_t ampmin;
} distance_value_t;
void Init_filter(void);
uint16_t Distance_Caculate(distance_value_t value);
#endif /* MEDIAN_H_ */
