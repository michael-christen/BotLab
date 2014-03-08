#include "haz_map.h"

// Haz Map origin at bottom left corner

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
	int color;

	hm->hazMap[y*hm->width + x] = type;

	switch (type) {
		case HAZ_MAP_UNKNOWN:
			color = 0xFF777777;
		break;
		case HAZ_MAP_FREE:
			color = 0xFF5EFF9C;
		break;
		case HAZ_MAP_OBSTACLE:
			color = 0xFF9C5EFF;
		break;
		case HAZ_MAP_IMPASSABLE:
			color = 0xFF4400FF;
		break;
		default:
			color = 0xFF000000;
		break;
	}
	hm->image->buf[y*hm->image->width + x] = color;
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
			if (x < lowX || x > highX || y < lowX || y > highY) {
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