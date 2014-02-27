#ifndef __IMAGE__H__
#define __IMAGE__H__
#include "common/image_util.h"
#include <math.h>
double color_dist(uint32_t p1, uint32_t p2);

void RGBtoHSV( uint32_t r, uint32_t g, uint32_t b, 
	double *h, double *s, double *v);

#endif
