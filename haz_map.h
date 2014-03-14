#ifndef HAZ_MAP_H
#define HAZ_MAP_H

#include "common/image_u32.h"
#include "path.h"
#include "time.h"

#define HAZ_MAP_MAX_WIDTH  100
#define HAZ_MAP_MAX_HEIGHT 100
#define HAZ_MAP_UNKNOWN 0
#define HAZ_MAP_FREE 1
#define HAZ_MAP_OBSTACLE 2
#define GRID_RES 5
#define HAZ_MAP_OBSTACLE_RADIUS 5
#define HAZ_MAP_CONFIG_RAIDUS 15
#define HAZ_MAP_HUGE_DIST 999
#define HAZ_MAP_TTL 5
#define HAZ_MAP_REPULSE_FACTOR 50

typedef struct haz_map_t haz_map_t;
typedef struct haz_map_tile_t haz_map_tile_t;
typedef struct dijkstra_data_t dijkstra_data_t;
typedef struct dijkstra_dists_t dijkstra_dists_t;
typedef struct haz_map_neighbor_t haz_map_neighbor_t;

struct dijkstra_data_t {
	uint8_t shortestPathKnown;
	uint32_t parentIndex, dist, pathCount;
};

struct dijkstra_dists_t {
	uint32_t tileIndex;
	double dist;
};

struct haz_map_neighbor_t {
	double distFactor;
	haz_map_tile_t *tile;
};

struct haz_map_tile_t {
	clock_t timestamp;
	int x, y, numNeighbors;
	uint8_t type;
	double val;
	haz_map_neighbor_t neighbors[8];
};

struct haz_map_t {
	uint32_t width, height;
	double x, y, max_free_val, diffX, diffY;
	haz_map_tile_t hazMap[HAZ_MAP_MAX_WIDTH * HAZ_MAP_MAX_HEIGHT];
	image_u32_t *image;
};

void haz_map_init(haz_map_t *hm, int w, int h);
void haz_map_set(haz_map_t *hm, int x, int y, uint8_t type);
// Don't call this function, its "private"
void haz_map_set_data(haz_map_t *hm, int x, int y, haz_map_tile_t *data);
void haz_map_get(haz_map_t *hm, haz_map_tile_t *tile, int x, int y);
void haz_map_translate(haz_map_t *hm, double newX, double newY, double oldX, double oldY);
void haz_map_destroy(haz_map_t *hm);
int haz_map_in_bounds(haz_map_t *hm, int x, int y);
path_t* haz_map_get_path(haz_map_t *hm, double endX, double endY);
void haz_map_cleanup(haz_map_t *hm);
void haz_map_compute_config(haz_map_t *hm);

#endif
