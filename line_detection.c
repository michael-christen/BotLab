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
        //look bottom to top
	for(y = im->height-1; y >= 0; --y) {
	    id = im->stride*y + x;
            px = im->buf[id];
	    if(is_tape(px)) {
		//Make sure above is blue too
		int good = 1;
		for(int chk = y-1; chk >= 0 &&
		    chk >= y - SEARCH_DIST; -- chk) {
		    int c_id = im->stride * chk + x;
		    uint32_t c_px = im->buf[c_id];
		    if(!is_tape(c_px)) {
			good = 0;
			break;
		    } else {
			//im->buf[c_id] = SHOW_PX;
		    }
		}
		if(good) {
		    im->buf[id] = SHOW_PX;
		    point.x = x;
		    point.y = y;
		    if(num_points < MAX_NUM_POINTS) {
			points[num_points++] = point;
		    }
		    break;
		}
	    }
	}
    }
    return num_points;
}
