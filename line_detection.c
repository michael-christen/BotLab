#include "line_detection.h"

unsigned int is_tape(uint32_t px) {
    return color_dist(TEMPLATE_PX, px) < COLOR_THRESHOLD;
}

int line_detection(image_u32_t *im, pixel_t *points) {
    int num_points = 0;
    int y, x, id;
    uint32_t px;
    pixel_t point;
    //1st pass
    for(x = 0; x < im->width; ++x) {
	for(y = 0; y < im->height; ++y) {
	    id = im->stride*y + x;
            px = im->buf[id];
	    if(is_tape(px)) {
		im->buf[id] = SHOW_PX;
		point.x = x;
		point.y = y;
		points[num_points++] = point;
		break;
	    }
	}
    }
    return num_points;
}
