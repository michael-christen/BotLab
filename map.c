#include "map.h"
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



