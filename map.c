#include "map.h"

// Private helper functions

double map_map_value(double x, double in_min, double in_max, double out_min, double out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

int min(int a, int b) {
	if (a < b) {
		return a;
	} else {
		return b;
	}
}

int max(int a, int b) {
	if (a > b) {
		return a;
	} else {
		return b;
	}
}

// Public functions
void map_init(map_t *map, int w, int h) {
	map->width = w;
	map->height = h;
	map->minX = map->maxX = w / 2;
	map->minY = map->maxY = h / 2;
	map->image = image_u32_create(w, h);
}

void map_set(map_t *map, double wX, double wY, uint8_t type) {
	int x, y;
	map_tile_t *tile;

	map_worldcoords_to_map_tile(map, wX, wY, &x, &y);
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
map_tile_t* map_get(map_t *map, int x, int y) {
	return &(map->tiles[y*map->width + x]);
}

void map_compute_config(map_t *map) {
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
			printf("x: %d, y: %d\n", x, y);
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
									map->image->buf[rY*map->image->stride + rX] = 0xFFFFFF00 | (tile->repulse_val - 1);
								} else if (dist <= MAP_REPULSE_RADIUS) {
									tempVal = map_map_value(dist, 0, MAP_REPULSE_RADIUS,
													MAP_MIN_REPULSE, MAP_MAX_REPULSE);
									if (tempVal > tile->repulse_val) {
										tile->type = MAP_FREE;
										tile->repulse_val = tempVal;
										map->image->buf[rY*map->image->stride + rX] = 0xFFFFFF00 | (tile->repulse_val - 1);
									}
								}
							}
						}
					}
				}
			} else {
				map->image->buf[y*map->image->stride + x] = 0xFF777777;
			}
		}	
	}
}
int id_to_x(map_t *map, int id) {
	return id % map->width;
}
int id_to_y(map_t *map, int id) {
	return ((id - id_to_x(map,id)) / map->width);
}
int min_neighbor_id(map_t * map, int * visited, int x, int y) {
	int min_val = -1;
	int min_id  = -1;
	//Searching *'s
	//***
	//*.*
	//***
	for(int j = -1; j <= 1; ++j) {
		//edge
		if(j + y < 0 || j + y > map->height) {
			continue;
		}
		for(int i = -1; i <= 1; ++i) {
			//edge
			if(i + x < 0 || i + x > map->width) {
				continue;
			}
			//origin
			if(i == 0 && j == 0) {
				continue;
			}
			int id = map->width*(y+j) + (x+i);
			int cur_val = map->tiles[id].wf_num;
			if(visited[id] && (min_val == -1 || cur_val < min_val)) {
				min_val = cur_val;
				min_id = id;
			}
		}
	}
	return min_id;
}


void map_worldcoords_to_map_tile(map_t * m, double x, double y, int * tileX, int * tileY){
	*tileX = x / MAP_RES + MAX_MAP_WIDTH / 2;
	*tileY = y / MAP_RES + MAX_MAP_HEIGHT / 2;
}

void map_tile_to_worldcoords(map_t * m, double * x, double * y, int tileX, int tileY){
	*x = (tileX  - MAX_MAP_WIDTH / 2) * MAP_RES;
	*y = (tileY  - MAX_MAP_HEIGHT / 2) * MAP_RES;
}

//-1 means no visited neighbors
int min_neighbor_val(map_t * map, int * visited, int x, int y) {
	return map->tiles[min_neighbor_id(map, visited, x, y)].wf_num;
}

path_t * compute_wavefront(map_t * map, int cur_x, int cur_y) {
	int visited [MAX_MAP_WIDTH * MAX_MAP_HEIGHT] = {0};
	int beginId = cur_x + cur_y*map->width;
	visited[beginId] = 1;
	int num_visited = 1;
	while(num_visited < map->width * map->height) {
		for(int i = 0; i < map->width; ++i) {
			for(int j = 0; j < map->height; ++j) {
				int id = i + j * map->width;
				//Might need to allow even if visited
				if(visited[id]) {
					continue;
				}
				else {
					int val = min_neighbor_val(map, visited, i, j);
					//-1 means no visited neighbors
					if(val != -1) {
						map->tiles[id].wf_num = map->tiles[id].repulse_val 
							+ val;
						visited[id] = 1;
						num_visited ++;
					}
				}
			}
		}
	}
	//Find min val unseen in map
	int min_unseen = -1;
	int min_id     = -1;
	position_t temp_pos;
	for(int i = 0; i < map->width; ++i) {
		for(int j = 0; j < map->height; ++j) {
			int id = i + j * map->width;
			map_tile_t curTile = map->tiles[id];
			if(!curTile.seen && 
					(min_unseen == -1 || 
					 curTile.wf_num < min_unseen)
			) {
				min_unseen = curTile.wf_num;
				min_id     = id;
				temp_pos.x = i;
				temp_pos.y = j;
			}
		}
	}
	//Find path back, not sure if valid idea
	//Brian approved
	position_t  temp_way_points[MAX_MAP_WIDTH * MAX_MAP_HEIGHT];
	int num_way_points = 0;
	while(temp_pos.x != cur_x && temp_pos.y != cur_y) {
		min_id = min_neighbor_id(map, visited, temp_pos.x, temp_pos.y);
		temp_way_points[num_way_points ++] = temp_pos;
		temp_pos.x = id_to_x(map, min_id);
		temp_pos.y = id_to_y(map, min_id);
	}
	path_t     *return_path  = malloc(sizeof(path_t));
	position_t *final_points = malloc(num_way_points * sizeof(position_t));
	return_path->waypoints = final_points;
	return_path->position  = 0;
	return_path->length    = num_way_points;
	for(int i = 0; i < num_way_points; ++i) {
		final_points[i] = temp_way_points[num_way_points - 1 - i];
	}
	return return_path;
}

map_destroy(map_t *map) {
	image_u32_destroy(map->image);
}