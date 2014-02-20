#ifndef BARREL_DISTORTION_H
#define BARREL_DISTORTION_H

#include <math.h>
#include "common/math_util.h"
#include "common/image_u32.h"

void correctDistortion(image_u32_t* im, int C);

#endif
