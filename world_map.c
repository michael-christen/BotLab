#include "world_map.h"

void world_map_init(world_map_t *wm, int w, int h) {
	wm->image = image_u32_create(w, h);
	wm->width = w;
	wm->height = h;
	wm->top = wm->height / 2;
	wm->bottom = wm->height / 2;
	wm->left = wm->width / 2;
	wm->right = wm->width / 2;

	int x, y;
	for (y = 0; y < h; y++) {
		for (x = 0; x < w; x++) {
			world_map_set(wm, x, y, WORLD_MAP_UNSEEN);
		}
	}
}

void world_map_set(world_map_t *wm, int x, int y, int8_t type) {
	int adjX = x / WORLD_MAP_RES + wm->width / 2;
	int adjY = y / WORLD_MAP_RES + wm->height / 2;
	int color;

	wm->worldMap[adjY*wm->width + adjX].seen = type;

	switch (type) {
		case WORLD_MAP_SEEN:
			color = 0xFF8AE051;
		break;
		default: // unseen
			color = 0xFFBBBBBB;
		break;
	}
	wm->image->buf[adjY*wm->image->width + adjX] = color;

//	printf("seen x: %d, y: %f \n", adjX, adjY);
	//reset t/b/l/r if necessary
	if(adjX < wm->left){
		wm->left = adjX;
//		printf("went lefter\n");
	}
	if(adjX > wm->right){
		wm->right = adjX;
//		printf("went righter\n");
	}
	if(adjY < wm->bottom){
		wm->bottom = adjY;
//		printf("went lower\n");
	}
	if(adjY > wm->top){
		wm->top = adjY;
//		printf("went higher\n");
	}
}

void world_map_destroy(world_map_t *wm) {
	image_u32_destroy(wm->image);
}
