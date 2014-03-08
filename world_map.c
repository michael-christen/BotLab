#include "world_map.h"

void world_map_init(world_map_t *wm, int w, int h) {
	wm->image = image_u32_create(w, h);
	wm->width = w;
	wm->height = h;

	int x, y;
	for (y = 0; y < h; y++) {
		for (x = 0; x < w; x++) {
			wm->worldMap[y*wm->width + x].count = 0;
			wm->worldMap[y*wm->width + x].type = WORLD_MAP_UNKNOWN;
			wm->image->buf[y*wm->image->width + x] = 0xFFCCCCCC;
		}
	}
}

void world_map_set(world_map_t *wm, int x, int y, int type) {
	int color;

	wm->worldMap[y*wm->width + x].type = type;

	switch (type) {
		case WORLD_MAP_UNKNOWN:
			color = 0xFFCCCCCC;
		break;
		case WORLD_MAP_FREE:
			color = 0xFFFFFFFF;
		break;
		case WORLD_MAP_OBSTACLE:
			color = 0xFFFF0000;
		break;
		default:
			color = 0xFF000000;
		break;
	}
	wm->image->buf[y*wm->image->width + x] = color;
}

void world_map_update(world_map_t *wm, int x, int y, haz_map_t *hm) {
	return;
}

void world_map_getPath(world_map_t *wm, path_t *path) {
	return;
}

void world_map_destroy(world_map_t *wm) {
	image_u32_destroy(wm->image);
}