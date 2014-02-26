#include "grid_map.h"

void grid_map_init(grid_map_t *gm, int w, int h) {
	gm->width = w;
	gm->height = h;

	int i, j;
	for (i = 0; i < h; i++) {
		for (j = 0; j < w; i++) {
			grid_map_set(gm, i, j, GRID_MAP_FREE);
		}
	}
}

void grid_map_set(grid_map_t *gm, int x, int y, int type) {
	gm->gridMap[y*gm->width + x] = type;
}

int grid_map_get(grid_map_t *gm, int x, int y) {
	return gm->gridMap[y*gm->width + x];
}