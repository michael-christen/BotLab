#ifndef __BLOB_DETECTION__H__
#define __BLOB_DETECTION__H__
#include "common/image_util.h"
#include "pixel.h"
#include "image.h"
#include <math.h>
#include <stdlib.h>
#include <stdio.h>

#define COLOR_THRESHOLD 37.0
//ABGR
//Blue
#define TEMPLATE_PX 0xff514430
#define SHOW_PX 0xff0127ff
#define MAX_NUM_POINTS 1000

unsigned int is_tape(uint32_t px);


int line_detection(image_u32_t *im, pixel_t *points);


#endif
