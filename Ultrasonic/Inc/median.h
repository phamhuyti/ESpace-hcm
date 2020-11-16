/*
 * median.h
 *
 * Created: 18.10.2016 15:25:10
 *  Author: M43888
 */

#ifndef MEDIAN_H_
#define MEDIAN_H_

#include <stdint.h>

typedef struct _median_filter {
	float  length;
	float  median;
	uint8_t   counter;
	float *value_bfr;
	float *sort_bfr;
} median_filter_t;

void median_filter_init(median_filter_t *filter, float lenght, float *bfr, float *sort_bfr);

float median_filter_add_new_value(median_filter_t *filter, float new_value);

#endif /* MEDIAN_H_ */
