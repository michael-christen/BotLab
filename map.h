#ifndef __MAP__H__
#define __MAP__H__
#include <assert.h>
#include "maebot_app.h"
typedef struct map map_t;
typedef struct map_tile map_tile_t;
#define MAX_MAP_WIDTH  300
#define MAX_MAP_HEIGHT 300
#define MAP_RES        5

struct map_tile {
	int seen;
	int repulse_val;
	int wf_num;
};

struct map {
	image_u32_t *image;
	int width, height;
	map_tile_t tiles[MAX_MAP_WIDTH * MAX_MAP_HEIGHT];
};

#endif
