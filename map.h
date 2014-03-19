#ifndef __MAP__H__
#define __MAP__H__
typedef struct map map_t;
typedef struct map_tile map_tile_t;
#define MAX_MAP_WIDTH  		300
#define MAX_MAP_HEIGHT 		300
#define MAP_RES        		5
#define MAP_FREE	   		0
#define MAP_OBSTACLE   		1
#define MAP_MAX_REPULSE 	200
#define MAP_MIN_REPULSE 	1
#define MAP_OBSTACLE_RADIUS 1
#define MAP_REPULSE_RADIUS 	4

struct map_tile {
	int seen;
	int type;
	int repulse_val;
	int wf_num;
};

struct map {
	int width, height, minX, maxX, minY, maxY;
	map_tile_t tiles[MAX_MAP_WIDTH * MAX_MAP_HEIGHT];
	image_u32_t *image;
};

void map_set(map_t *hm, int x, int y, uint8_t type);
void map_get(map_t *hm, map_tile_t *tile, int x, int y);
void map_destroy(map_t *hm);
void map_world_to_map();
int map_in_bounds(map_t *hm, int x, int y);
void map_cleanup(map_t *hm);
void map_compute_config(map_t *hm);

#endif
