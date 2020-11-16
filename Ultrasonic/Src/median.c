
#include "median.h"

/*
 * Init filter. Only works for filter of length 2n-1 n=0,1,2,3,4,5...
 */
void median_filter_init(median_filter_t *filter, uint16_t lenght, uint16_t *bfr, uint16_t *sort_bfr)
{
	for (uint16_t i = 0; i < lenght; i++) {
		bfr[i]      = i;
		sort_bfr[i] = i;
	}

	filter->length    = lenght;
	filter->value_bfr = bfr;
	filter->sort_bfr  = sort_bfr;
	filter->median    = lenght / 2;
	filter->counter   = 0;
}

uint16_t median_filter_add_new_value(median_filter_t *filter, uint16_t new_value)
{
	uint16_t old_value = filter->value_bfr[filter->counter];
	if (old_value == new_value) {
		filter->counter++;
		return filter->median;
	}
	filter->value_bfr[filter->counter] = new_value;
	uint8_t counter;
	if (new_value > old_value) {
		counter = 0;
		for (uint8_t i = 0; i < filter->length; i++) {
			if (i == filter->counter)
				continue;

			if (new_value > filter->value_bfr[i]) {
				counter++;
				if (filter->sort_bfr[i] > 0) {
					if (filter->value_bfr[i] > old_value
					    || ((filter->value_bfr[i] == old_value)
					        && (filter->sort_bfr[i] > filter->sort_bfr[filter->counter]))) {
						filter->sort_bfr[i]--;
					}
				}
			}
			if (filter->sort_bfr[i] == filter->length / 2)
				filter->median = filter->value_bfr[i];
		}
	} else {
		counter = filter->length - 1;
		for (uint8_t i = 0; i < filter->length; i++) {
			if (i == filter->counter)
				continue;

			if (new_value < filter->value_bfr[i]) {
				counter--;
				if (filter->sort_bfr[i] < filter->length - 1) {
					if (filter->value_bfr[i] < old_value
					    || ((filter->value_bfr[i] == old_value)
					        && (filter->sort_bfr[i] < filter->sort_bfr[filter->counter]))) {
						filter->sort_bfr[i]++;
					}
				}
			}
			if (filter->sort_bfr[i] == filter->length / 2)
				filter->median = filter->value_bfr[i];
		}
	}

	if (counter == filter->length / 2) {
		filter->median = new_value;
	}
	filter->sort_bfr[filter->counter++] = counter;
	if (filter->counter >= filter->length)
		filter->counter = 0;

	return filter->median;
}
