#include "map.h"
//-1 means no visited neighbors
int min_neighbor_val(map_t * map, int * visited, int x, int y) {
	int min_val = -1;
	unsigned int len = 0;
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
			}
		}
	}
	return min_val;
}

void compute_wavefront(map_t * map, int cur_x, int cur_y) {
	int visited [MAX_MAP_WIDTH * MAX_MAP_HEIGHT] = {0};
	int beginId = cur_x + cur_y*map->width;
	visited[beginId] = 1;
	for(int i = 0; i < map->width; ++i) {
		for(int j = 0; j < map->height; ++j) {
			int id = i + j * map->width;
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
				}
			}
		}
	}
	//Check all visited
	for(int i = 0; i < map->width; ++i) {
		for(int j = 0; j < map->height; ++j) {
			int id = i + j * map->width;
			assert(visited[id]);

		}
	}
}

