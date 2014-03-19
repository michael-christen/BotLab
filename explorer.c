#include "explorer.h"
#include "common/matd.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

/*int movement_compare(const void *elt1, const void *elt2){
	//compares tiles, moving higher priority tiles to front of array
	const world_map_tile_t *tile1 = *((const world_map_tile_t**) elt1);
	const world_map_tile_t *tile2 = *((const world_map_tile_t**) elt2);
	if((tile1->path_to->length != 0 && tile2->path_to->length != 0)){
		if(tile1->visited == WORLD_MAP_VISITED){
			if(tile2->visited == WORLD_MAP_VISITED){
				if(tile1->distance > tile2->distance) { return 1;} 	//both visited: tile 2 closer [tile2 | tile1]
				if(tile1->distance < tile2->distance) { return -1;} 	//both visited: tile 1 closer [tile1 | tile2]
				return(tile1->timestamp - tile2->timestamp); 			// if tile 2 older -> [tile2 | tile1]
			}
			else{
				return 1; //1 visited, 2 unvisited [tile2 | tile1]
			}
		}
		else{
			if(tile2->visited == WORLD_MAP_VISITED){
				return -1; //1 unvisited, 2 visited [tile1 | tile2]
			}
			else{
				return(tile1->distance - tile2->distance); 	//both unvisited: tile 2 closer -> [tile2 | tile1]
			}
		}
	}
	else{
		if (tile1->path_to->length > 0){ return -1;} //tile 1 has path [tile1 | tile2]
		else { return 1;} //tile 2 has path [tile2 | tile1]
	}


}





void cm_to_world_cell(int x, int y, int *gridx, int *gridy){
		*gridx = x / WORLD_MAP_RES + WORLD_MAP_MAX_WIDTH / 2;
		*gridy = -(y / WORLD_MAP_RES) + WORLD_MAP_MAX_HEIGHT / 2;
}



path_t * dumb_explore(void * data, double pre_analyze_theta){

	state_t * state = data;

	double theta = pre_analyze_theta; //state->pos_theta;
	double bruce_x = state->pos_x;
	double bruce_y = state->pos_y;
	haz_map_t * hm = &state->hazMap;

		theta = -theta;
		theta = theta + M_PI/2;


	path_t * dumb_path;
	uint8_t valid = 0;
	while(!valid){
		double x = 5 * cos(theta);
		double y = 5 * sin(theta);

		dumb_path = haz_map_get_path(hm, bruce_y + y, bruce_x + x);
		if (dumb_path->length != 0) {
			valid = 1;
		}
		theta = theta + M_PI/4;
	}

	return dumb_path;

}



path_t * choose_path(void * data, double pre_analyze_theta){
	//printf("in choose path");
	//set max for bound checking
	int max_x = (WORLD_MAP_RES * WORLD_MAP_MAX_WIDTH) / 2;
	int max_y = (WORLD_MAP_RES * WORLD_MAP_MAX_HEIGHT) / 2;

	//grab values
	state_t * state = data;
	world_map_t * wm = &state->world_map;
	haz_map_t * hm = &state->hazMap;
	double x = state->pos_x;
	double y = state->pos_y;

	//find grid map cell bruce is currently in
	int gridx, gridy;
	cm_to_world_cell(x, y, &gridx, &gridy);
	world_map_tile_t * curr_tile = &wm->worldMap[gridy*(wm->width) + gridx];

	//printf("gridx: %d gridy: %d x: %f y: %f\n", gridx, gridy, x, y);
	//find coordinates for center of all neighboring grid cells

	double up, down, right, left;
	y = gridy*WORLD_MAP_RES + WORLD_MAP_RES/2 - WORLD_MAP_MAX_WIDTH*WORLD_MAP_RES/2;
	x = gridx*WORLD_MAP_RES + WORLD_MAP_RES/2 - WORLD_MAP_MAX_HEIGHT*WORLD_MAP_RES/2;
	up =  y - WORLD_MAP_RES;
	down = y + WORLD_MAP_RES;
	right = x + WORLD_MAP_RES;
	left = x - WORLD_MAP_RES;

	//printf("x: %f, y: %f, up: %f down: %f right %f left %f)\n", x, y, up, down, left, right);

	int num_neighbors = 0;
	//bounds check before calling to get path
	if( up >= -max_y ) {
		curr_tile->neighbors[num_neighbors] = &wm->worldMap[(gridy - 1)*wm->width + (gridx)];
		curr_tile->neighbors[num_neighbors]->path_to = haz_map_get_path(hm, up, x);
		num_neighbors++;
		if( left >= -max_x ){
			curr_tile->neighbors[num_neighbors] = &wm->worldMap[(gridy - 1)*wm->width + (gridx - 1)];
			curr_tile->neighbors[num_neighbors]->path_to = haz_map_get_path(hm, up, left);
			num_neighbors++;
			curr_tile->neighbors[num_neighbors] = &wm->worldMap[(gridy)*wm->width + (gridx - 1)];
			curr_tile->neighbors[num_neighbors]->path_to = haz_map_get_path(hm, y, left);
			num_neighbors++;
		}
		if( right < max_x ){
			curr_tile->neighbors[num_neighbors] = &wm->worldMap[(gridy - 1)*wm->width + (gridx + 1)];
			curr_tile->neighbors[num_neighbors]->path_to = haz_map_get_path(hm, up, right);
			num_neighbors++;
			curr_tile->neighbors[num_neighbors] = &wm->worldMap[(gridy)*wm->width + (gridx + 1)];
			curr_tile->neighbors[num_neighbors]->path_to = haz_map_get_path(hm, y, right);
			num_neighbors++;
		}
	}
	if( down < max_y ) {
		curr_tile->neighbors[num_neighbors] = &wm->worldMap[(gridy + 1)*wm->width + (gridx)];
		curr_tile->neighbors[num_neighbors]->path_to = haz_map_get_path(hm, down, x);
		num_neighbors++;
		if( left >= -max_x ){
			curr_tile->neighbors[num_neighbors] = &wm->worldMap[(gridy + 1)*wm->width + (gridx - 1)];
			curr_tile->neighbors[num_neighbors]->path_to = haz_map_get_path(hm, down, left);
			num_neighbors++;
		}
		if( right < max_x ){
			curr_tile->neighbors[num_neighbors] = &wm->worldMap[(gridy + 1)*wm->width + (gridx + 1)];
			curr_tile->neighbors[num_neighbors]->path_to = haz_map_get_path(hm, down, right);
			num_neighbors++;
		}
	}

	//printf("num_neighbors: %d\n", num_neighbors);


	//evaluate grid cell distance for all neighbors
	int i;
	double distance = 0;

	for (i = 0; i < num_neighbors; i++){

		distance = 0;
		if (curr_tile->neighbors[i]->path_to->length != 0){
			distance = curr_tile->neighbors[i]->path_to->distance;
		}
		int grid_dist = ((int)distance + WORLD_MAP_RES/2)  / WORLD_MAP_RES;
		//printf("neighbor %d has distance %f and grid_dist %d\n", i, distance, grid_dist);
		//printf("neighbor %d visited: %d, timestamp %d\n", i, curr_tile->neighbors[i]->visited,  curr_tile->neighbors[i]->timestamp);
		curr_tile->neighbors[i]->distance = grid_dist;
	}



	//qsort
	qsort(curr_tile->neighbors, num_neighbors, sizeof(curr_tile->neighbors[0]), movement_compare);

	int use_path = 0;

	//printf("target path length: %d \n", curr_tile->neighbors[0]->path_to->length);
	while(use_path < num_neighbors){
		if (curr_tile->neighbors[use_path]->path_to->length == 0 ){
			use_path++;
		} else {
			break;
		}
	}

	path_t * best_path;
	if(use_path >= num_neighbors){
		best_path = malloc(sizeof(path_t));
		best_path->length = best_path->distance = 0;

	}
	else{
		best_path = curr_tile->neighbors[use_path]->path_to;
	}

	for(i = 0; i < num_neighbors; i++){
		if(i != use_path){
			path_destroy(curr_tile->neighbors[i]->path_to);
		}
	}

	return best_path;
}*/



















//
//
//
//
//
//
//
//old stuff because i'm not sure how to make it compile without
//
//
//
//
//
//
//
void explorer_init(explorer_t *ex) {
	ex->goHome = 0;
}


explorer_state_t explorer_run(explorer_t *ex, map_t *map, double x, double y, double theta) {
	/*int forwardDist = explorer_check_region(ex, hm, EXPLORER_REGION_FORWARD, theta);
	int leftDist = explorer_check_region(ex, hm, EXPLORER_REGION_LEFT, theta);
	printf("forwardDist: %d\n", forwardDist);
	printf("leftDist: %d\n", leftDist);*/
	return EX_MOVE;


}

path_t* explorer_get_move(explorer_t *ex) {
	return ex->path;
}


