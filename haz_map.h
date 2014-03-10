#ifndef HAZ_MAP_H
#define HAZ_MAP_H

#include "common/image_u32.h"

#define HAZ_MAP_MAX_WIDTH 300
#define HAZ_MAP_MAX_HEIGHT 300
#define HAZ_MAP_UNKNOWN 0
#define HAZ_MAP_FREE 1
#define HAZ_MAP_OBSTACLE 2
#define HAZ_MAP_IMPASSABLE 3
#define HAZ_MAP_UNIT_TO_CM 1
#define HAZ_MAP_CONFIG_RAIDUS 15

typedef struct haz_map_t haz_map_t;

struct haz_map_t {
	int width, height;
	int hazMap[HAZ_MAP_MAX_WIDTH * HAZ_MAP_MAX_HEIGHT];
	image_u32_t *image;
};

void haz_map_init(haz_map_t *hm, int w, int h);
void haz_map_set(haz_map_t *hm, int x, int y, int type);
void haz_map_translate(haz_map_t *hm, int newX, int newY, int oldX, int oldY);
int haz_map_get(haz_map_t *hm, int x, int y);
void haz_map_destroy(haz_map_t *hm);

#endif