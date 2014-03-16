#include "explorer.h"
#include "common/matd.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

int movement_compare(const void *elt1, const void *elt2){
	//compares tiles, moving higher priority tiles to front of array
	world_map_tile_t tile1 = *((world_map_tile_t*)elt1);
	world_map_tile_t tile2 = *((world_map_tile_t*)elt2);

	if(tile1.visited == WORLD_MAP_VISITED){
		if(tile2.visited == WORLD_MAP_VISITED){
			if(tile1.distance > tile2.distance) { return 1;} 	//both visited: tile 2 closer [tile2 | tile1]
			if(tile1.distance < tile2.distance) { return -1;} 	//both visited: tile 1 closer [tile1 | tile2]
			return(tile1.timestamp - tile2.timestamp); 			// if tile 2 older -> [tile2 | tile1]
		}
		else{
			return 1; //1 visited, 2 unvisited [tile2 | tile1]
		}
	}
	else{
		if(tile2.visited == WORLD_MAP_VISITED){
			return -1; //1 unvisited, 2 visited [tile1 | tile2]
		}
		else{
			return(tile1.distance - tile2.distance); 	//both unvisited: tile 2 closer -> [tile2 | tile1]
		}
	}
}





void cm_to_world_cell(int x, int y, int *gridx, int *gridy){
		gridx = x / WORLD_MAP_RES + WORLD_MAP_MAX_WIDTH / 2;
		gridy = y / WORLD_MAP_RES + WORLD_MAP_MAX_HEIGHT / 2;
}







path_t * sort_neighbors(world_map_tile_t *curr_tile, haz_map_t *hm, int x, int y){

	int max_x = (WORLD_MAP_RES * WORLD_MAP_MAX_WIDTH) / 2;
	int max_y = (WORLD_MAP_RES * WORLD_MAP_MAX_HEIGHT) / 2;

	int gridx, gridy;
	cm_to_world_cell(x, y, &gridx, &gridy);

	//find coordinates for center of all neighboring grid cells
	int up, down, right, left;
	up = gridy * WORLD_MAP_RES + WORLD_MAP_RES/2;
	y = gridy * WORLD_MAP_RES - WORLD_MAP_RES/2;
	down = gridy * WORLD_MAP_RES - 3 * WORLD_MAP_RES/2;
	right = gridx * WORLD_MAP_RES + WORLD_MAP_RES/2;
 	x = gridx * WORLD_MAP_RES - WORLD_MAP_RES/2;
	left = gridx * WORLD_MAP_RES - 3 * WORLD_MAP_RES/2;
	

	int num_neighbors = 0;
	//bounds check before calling to get path
	if( up <= max_y ) {
		curr_tile->neighbors[num_neighbors]->path_to = haz_map_get_path(hm, up, x);
		num_neighbors++;
		if( left >= -max_x ){
			curr_tile->neighbors[num_neighbors]->path_to = haz_map_get_path(hm, up, left);
			curr_tile->neighbors[num_neighbors]->path_to = haz_map_get_path(hm, y, left);
			num_neighbors += 2;
		}
		if( right <= max_x ){
			curr_tile->neighbors[num_neighbors]->path_to = haz_map_get_path(hm, up, right);
			curr_tile->neighbors[num_neighbors]->path_to = haz_map_get_path(hm, y, right);
			num_neighbors += 2;
		}
	}
	if( down >= -max_y ) {
		curr_tile->neighbors[num_neighbors]->path_to = haz_map_get_path(hm, down, x);
		num_neighbors ++;
		if( left >= -max_x ){
			curr_tile->neighbors[num_neighbors]->path_to = haz_map_get_path(hm, down, left);
			num_neighbors ++;
		}
		if( right <= max_x ){
			curr_tile->neighbors[num_neighbors]->path_to = haz_map_get_path(hm, down, right);
			num_neighbors ++;	
		}
	}

	//evaluate grid cell distance for all neighbors
	for (int i = 0; i < num_neighbors; i++){
		double distance = curr_tile->neighbors[i]->path_to->distance;
		int grid_dist = (distance + WORLD_MAP_RES/2)  / WORLD_MAP_RES;
		curr_tile->neighbors[i]->distance = grid_dist;	
	}

	//qsort
	qsort(curr_tile->neighbors, num_neighbors, sizeof(world_map_tile_t), movement_compare);

	return curr_tile->neighbors[0]->path_to;
}


