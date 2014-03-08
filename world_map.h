#ifndef WORLD_MAP_H
#define WORLD_MAP_H

#include "common/image_u32.h"
#include "path.h"
#include "haz_map.h"

#define WORLD_MAP_MAX_WIDTH 1000
#define WORLD_MAP_MAX_HEIGHT 1000
#define WORLD_MAP_UNKNOWN 0
#define WORLD_MAP_FREE 1
#define WORLD_MAP_OBSTACLE 2
#define WORLD_MAP_UNIT_TO_CM 10

typedef struct world_map_t world_map_t;
typedef struct world_map_tile_t world_map_tile_t;

struct world_map_tile_t {
	int count, type;
};

struct world_map_t {
	int width, height;
	world_map_tile_t worldMap[WORLD_MAP_MAX_WIDTH * WORLD_MAP_MAX_HEIGHT];
	image_u32_t *image;
};

void world_map_init(world_map_t *wm, int w, int h);
void world_map_set(world_map_t *wm, int x, int y, int type);
void world_map_update(world_map_t *wm, int x, int y, haz_map_t *hm);
void world_map_getPath(world_map_t *wm, path_t *path);
void world_map_destroy(world_map_t *wm);

#endif