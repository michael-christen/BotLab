#ifndef BARREL_DISTORTION_H
#define BARREL_DISTORTION_H

#include <math.h>
#include "common/math_util.h"
#include "common/image_u32.h"
#include "pixel.h"
#include <stdio.h>

#define DISTORT_A 0.6712
#define DISTORT_C -.000722

pixel_t* getLookupTable(int width, int height);

void destroyLookupTable(pixel_t* table);

void correctDistortion(image_u32_t* im, pixel_t* lookupTable);

#endif
