#ifndef BARREL_DISTORTION_H
#define BARREL_DISTORTION_H

#include <math.h>
#include "common/math_util.h"
#include "common/image_u32.h"
#include <stdio.h>

#define C -.00048

typedef struct pixel pixel_t;
struct pixel{
	int x,y;
};

pixel_t* getLookupTable(int width, int height);

void destroyLookupTable(pixel_t* table);

void correctDistortion(image_u32_t* im, pixel_t* lookupTable);

#endif
