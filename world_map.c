#include "world_map.h"

void data_set(world_map_t *wm, int adjX, int adjY, int8_t type){

	if(adjX < 0 || adjX >= WORLD_MAP_MAX_WIDTH
		|| adjY < 0 || adjY >= WORLD_MAP_MAX_HEIGHT){
		return;
	}
	int color;

	if(wm->worldMap[adjY*wm->width + adjX].visited == WORLD_MAP_UNVISITED){
		wm->worldMap[adjY*wm->width + adjX].timestamp = clock();
	}

	wm->worldMap[adjY*wm->width + adjX].visited = type;

	switch (type) {
		case WORLD_MAP_VISITED:
			//printf ("setting x: %d, y: %d \n", adjX, adjY);
			color = 0xFF8AE051;
		break;
		default: // unvisited
			color = 0xFFBBBBBB;
		break;
	}
	wm->image->buf[adjY*wm->image->width + adjX] = color;
	return;
}

int world_map_in_bounds(world_map_t *wm, int x, int y) {
	if (x >= 0 && x < wm->width && y >= 0 && y < wm->height) {
		return 1;
	} else {
		return 0;
	}
}


void world_map_init(world_map_t *wm, int w, int h) {
	wm->image = image_u32_create(w, h);
	wm->width = w;
	wm->height = h;
	wm->top = wm->height / 2;
	wm->bottom = wm->height / 2;
	wm->left = wm->width / 2;
	wm->right = wm->width / 2;
	position_t o[8] = {{-1,-1},{0,-1},{1,-1},{1,0},{1,1},{0,1},{-1,1},{-1,0}};

	int x, y, z;
	for (y = 0; y < h; y++) {
		for (x = 0; x < w; x++) {
			data_set(wm, x, y, WORLD_MAP_UNVISITED);
			wm->worldMap[y*w + x].shot_diamond = 0;
			wm->worldMap[y*w + x].visited = 0;
			wm->worldMap[y*w + x].timestamp = CLOCKS_PER_SEC * 250 ;
			wm->worldMap[y*w + x].distance = 0 ;
			for(z = 0; z < 8; z++){
				int neX = x + o[z].x;
				int neY = y + o[z].y;
				if (world_map_in_bounds(wm, neX, neY)) {
					wm->worldMap[y*w + x].neighbors[z] = &wm->worldMap[neY*w + neX];
				}
			}
		}
	}



}

void world_map_set(world_map_t *wm, double x, double y, int8_t type) {
	int adjX = x / WORLD_MAP_RES + wm->width / 2;
	int adjY = y / WORLD_MAP_RES + wm->height / 2;

	data_set(wm, adjX, adjY, type);

//	printf("in map cell x: %d y: %d\n", adjX, adjY);
	//reset t/b/l/r if necessary
	if(adjX < wm->left){
		wm->left = adjX;
		//printf("new max left\n");
	}
	if(adjX > wm->right){
		wm->right = adjX;
		//printf("new max right\n");
	}
	if(adjY < wm->bottom){
		wm->bottom = adjY;
		//printf("new max low\n");
	}
	if(adjY > wm->top){
		wm->top = adjY;
		//printf("new max high\n");

	}
}

void world_map_destroy(world_map_t *wm) {
	image_u32_destroy(wm->image);
}
