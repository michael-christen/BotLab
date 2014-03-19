#include "map.h"

// Private helper functions

double map_map_value(double x, double in_min, double in_max, double out_min, double out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


// Public functions
void map_init(map_t *map, int w, int h) {
	map->width = w;
	map->height = h;
	map->minX = map->maxX = w / 2;
	map->minY = map->maxY = h / 2;
}

void map_set(map_t *map, double wX, double wY, uint8_t type) {
	int x, y;

	map_world_to_map(wX, wY, &x, &y);
	map_tile_t *tile;
	tile = map_get(map, x, y);
	tile->type = type;
	tile->seen = 1;

	//Extend max check dimensions
	if (x < map->minX) {
		map->minX = x;
	} else if (x > map->maxX) {
		map->maxX = x;
	}

	if (y < map->minY) {
		map->minY = y;
	} else if (y > map->maxY) {
		map->maxY = y;
	}
}

// X and Y correspond to map x and y, not world x and y
map_tile_t* map_get(map_t *map, map_tile_t *tile, int x, int y) {
	return &(map->tiles[y*map->width + x]);
}

void map_compute_config(map_t *map, ) {
	map_tile_t *tile;
	int x, y, rX, rY;
	int rMinX, rMaxX, rMinY, rMaxY;
	int tempVal;
	double dist;

	for (y = map->minX; y < map->maxY; y++) {
		for (x = map->minX; x < map->maxX; x++) {
			tile = map_get(map, x, y);
			if (tile->type == MAP_FREE) {
				tile->repulse_val = MAP_MIN_REPULSE;
			}
		}
	}

	for (y = map->minX; y < map->maxY; y++) {
		for (x = map->minX; x < map->maxX; x++) {
			tile = map_get(map, x, y);
			if (tile->seen) {
				if (tile->type == MAP_OBSTACLE) {
					rMinX = max(x - MAP_REPULSE_RADIUS, 0);
					rMaxX = min(x + MAP_REPULSE_RADIUS, map->width - 1);
					rMinY = max(y - MAP_REPULSE_RADIUS, 0);
					rMaxY = min(y + MAP_REPULSE_RADIUS, map->height - 1);

					for (rX = rMinX; rX <= rMaxX; rX++) {
						for (rY = rMinY; rX <= rMaxY; rY++) {
							tile = map_get(map, x, y);
							if (tile->seen) {
								dist = sqrt(pow(rX - x, 2) + pow(rY - y, 2));

								if (dist <= MAP_OBSTACLE_RADIUS) {
									tile->type = MAP_OBSTACLE;
									tile->repulse_val = MAP_MAX_REPULSE;
									tile->image->buf[rY*map->width + X] = 0xFFFFFF00 | (tile->repulse_val - 1);
								} else if (dist <= MAP_REPULSE_RADIUS) {
									tempVal = map_map_value(dist, 0, MAP_REPULSE_RADIUS,
													MAP_MIN_REPULSE, MAP_MAX_REPULSE);
									if (tempVal > tile->repulse_val) {
										tile->type = MAP_FREE;
										tile->repulse_val = tempVal;
										tile->image->buf[rY*map->width + X] = 0xFFFFFF00 | (tile->repulse_val - 1);
									}
								}
							}
						}
					}
				} else {
					tile->image->buf[rY*map->width + X] = 0xFFFFFFFF;
				}
			} else {
				tile->image->buf[rY*map->width + X] = 0xFF777777;
			}
		}	
	}
}