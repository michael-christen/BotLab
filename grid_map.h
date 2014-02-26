#include "common/image_u32.h"

#define GRID_MAP_MAX_WIDTH 500
#define GRID_MAP_MAX_HEIGHT 500
#define GRID_MAP_FREE 0
#define GRID_MAP_CLOSECALL 1
#define GRID_MAP_OBSTACLE 2

typedef struct grid_map_t grid_map_t;

struct grid_map_t {
	int width, height;
	int gridMap[GRID_MAP_MAX_WIDTH * GRID_MAP_MAX_HEIGHT];
	image_u32_t *image;
};

void grid_map_init(grid_map_t *gm, int w, int h);
void grid_map_set(grid_map_t *gm, int x, int y, int type);
int grid_map_get(grid_map_t *gm, int x, int y);
void grid_map_destroy(grid_map_t *gm);