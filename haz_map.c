#include "haz_map.h"
#include <math.h>
#include <stdio.h>

// Haz Map origin at bottom left corner

double map(double x, double in_min, double in_max, double out_min, double out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void haz_map_init(haz_map_t *hm, int w, int h) {
	hm->image = image_u32_create(w, h);
	hm->width = w;
	hm->height = h;

	int i, j;
	for (i = 0; i < h; i++) {
		for (j = 0; j < w; j++) {
			haz_map_set(hm, j, i, HAZ_MAP_UNKNOWN);
		}
	}
}

void haz_map_set(haz_map_t *hm, int x, int y, int type) {

	//Bounds checking
	if(y*hm->image->stride + x >= hm->image->height*hm->image->stride) {
		return;
	}

	int color;

	hm->hazMap[y*hm->width + x] = type;

	// Set Circle around to impassible or free or unknown
	int maxU, minU;
	double mapAngle;
	int maxV = fmin(y + HAZ_MAP_CONFIG_RAIDUS, hm->height - 1);
	int minV = fmax(y - HAZ_MAP_CONFIG_RAIDUS, 0);


	int u, v, tile;
	if (type == HAZ_MAP_OBSTACLE) {
		for (v = minV; v <= maxV; v++) {
			mapAngle = map(v, minV, maxV, 0, M_PI);
			maxU = fmin(x + HAZ_MAP_CONFIG_RAIDUS * sin(mapAngle), hm->width - 1);
			minU = fmax(x - HAZ_MAP_CONFIG_RAIDUS * sin(mapAngle), 0);
			for (u = minU; u <= maxU; u++) {
				tile = haz_map_get(hm, u, v);
				if (tile == HAZ_MAP_UNKNOWN || tile == HAZ_MAP_FREE) {
					haz_map_set(hm, u, v, HAZ_MAP_IMPASSABLE);
				}
			}
		}
	}

	switch (type) {
		case HAZ_MAP_UNKNOWN:
			color = 0xFF777777;
		break;
		case HAZ_MAP_FREE:
			color = 0xFF5EFF9C;
		break;
		case HAZ_MAP_OBSTACLE:
			color = 0xFFFF0044;
		break;
		case HAZ_MAP_IMPASSABLE:
			color = 0xFFFF00BB;
		break;
		default:
			color = 0xFF000000;
		break;
	}
	hm->image->buf[y*hm->image->stride + x] = color;
}

void haz_map_translate(haz_map_t *hm, int newX, int newY, int oldX, int oldY) {
	int diffX = newX - oldX;
	int diffY = newY - oldY;
	int lowX = 0;
	int highX = hm->width;
	int lowY = 0;
	int highY = hm->height;

	if (diffY >= 0) {
		lowY = diffY;
	} else {
		highY = hm->height - diffY;
	}

	if (diffX >= 0) {
		lowX = diffX;
	} else {
		highX = hm->width - diffX;
	}

	for (int y = 0; y < hm->height; y++) {
		for (int x = 0; x < hm->width; x++) {
			if (x < lowX || x > highX || y < lowY || y > highY) {
				haz_map_set(hm, x, y, HAZ_MAP_UNKNOWN);
			} else {
				haz_map_set(hm, x - diffX, y - diffY, haz_map_get(hm, x, y));
			}
		}
	}

}

int haz_map_get(haz_map_t *hm, int x, int y) {
	return hm->hazMap[y*hm->width + x];
}

void haz_map_destroy(haz_map_t *hm) {
	image_u32_destroy(hm->image);
}
