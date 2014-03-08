#include "grid_map.h"
#include <stdio.h>

void grid_map_init(grid_map_t *gm, int w, int h) {
	gm->image = image_u32_create(w, h);
	gm->width = w;
	gm->height = h;

	int i, j;
	for (i = 0; i < h; i++) {
		for (j = 0; j < w; j++) {
			grid_map_set(gm, j, i, GRID_MAP_FREE);
		}
	}
}

void grid_map_set(grid_map_t *gm, int x, int y, int type) {
	int color;

	gm->gridMap[y*gm->width + x] = type;

	switch (type) {
		case GRID_MAP_CLOSECALL:
			color = 0xFF00FFFF;
		break;
		case GRID_MAP_OBSTACLE:
			color = 0xFFFFFF00;
		break;
		default:
			color = 0xFF00FF00;
		break;
	}
	gm->image->buf[y*gm->image->width + x] = color;
}

int grid_map_get(grid_map_t *gm, int x, int y) {
	return gm->gridMap[y*gm->width + x];
}

void grid_map_destroy(grid_map_t *gm) {
	image_u32_destroy(gm->image);
}