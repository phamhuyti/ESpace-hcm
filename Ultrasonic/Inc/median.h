#ifndef MEDIAN_H_
#define MEDIAN_H_

#include <stdint.h>

typedef struct _median_filter {
	uint16_t  length;
	uint16_t  median;
	uint8_t   counter;
	uint16_t *value_bfr;
	uint16_t *sort_bfr;
} median_filter_t;

void median_filter_init(median_filter_t *filter, uint16_t lenght, uint16_t *bfr, uint16_t *sort_bfr);

uint16_t median_filter_add_new_value(median_filter_t *filter, uint16_t new_value);

#endif /* MEDIAN_H_ */
