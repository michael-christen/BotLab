#ifndef WORLD_MAP_H
#define WORLD_MAP_H

#include "common/image_u32.h"
#include "path.h"
#include "haz_map.h"

#define WORLD_MAP_MAX_WIDTH 251
#define WORLD_MAP_MAX_HEIGHT 251
#define WORLD_MAP_UNSEEN 0
#define WORLD_MAP_SEEN 1
#define WORLD_MAP_RES 10

typedef struct world_map_t world_map_t;
typedef struct world_map_tile_t world_map_tile_t;

struct world_map_tile_t {
	uint8_t seen;
	int shot_diamond; //1 true, 0 false
};

struct world_map_t {
	int width, height;
	int top, bottom, left, right; //highest, lowest, leftest and rightest seen so far
	world_map_tile_t worldMap[WORLD_MAP_MAX_WIDTH * WORLD_MAP_MAX_HEIGHT];
	image_u32_t *image;
};

void world_map_init(world_map_t *wm, int w, int h);
void world_map_set(world_map_t *wm, double x, double y, int8_t type);
void world_map_destroy(world_map_t *wm);

#endif
