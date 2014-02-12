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
		*gridx = x / WORLD_MAP_RES + WORLD_MAP_MAX_WIDTH / 2;
		*gridy = y / WORLD_MAP_RES + WORLD_MAP_MAX_HEIGHT / 2;
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
	int mid_x, mid_y;
	int up, down, right, left;


	mid_y = WORLD_MAP_RES/2 + WORLD_MAP_RES*gridy;
 	mid_x = WORLD_MAP_RES/2 + WORLD_MAP_RES*gridx;
	up =  mid_y - WORLD_MAP_RES;
	down = mid_y + WORLD_MAP_RES;
	right = mid_x + WORLD_MAP_RES;
	left = mid_x - WORLD_MAP_RES;

	printf("midx, midy: (%d, %d) up: %d down: %d right %d left %d)\n", mid_x, mid_y, up, down, left, right);

	int num_neighbors = 0;
	//bounds check before calling to get path
	if( up <= max_y ) {
		curr_tile->neighbors[num_neighbors] = &wm->worldMap[(gridy - 1)*(wm->width) + (gridx)];
		curr_tile->neighbors[num_neighbors]->path_to = haz_map_get_path(hm, up, x);
		num_neighbors++;
		if( left >= -max_x ){
			curr_tile->neighbors[num_neighbors] = &wm->worldMap[(gridy - 1)*(wm->width) + (gridx - 1)];
			curr_tile->neighbors[num_neighbors]->path_to = haz_map_get_path(hm, up, left);
			num_neighbors++;
			curr_tile->neighbors[num_neighbors] = &wm->worldMap[(gridy)*(wm->width) + (gridx - 1)];
			curr_tile->neighbors[num_neighbors]->path_to = haz_map_get_path(hm, y, left);
			num_neighbors++;
		}
		if( right <= max_x ){
			curr_tile->neighbors[num_neighbors] = &wm->worldMap[(gridy - 1)*(wm->width) + (gridx + 1)];
			curr_tile->neighbors[num_neighbors]->path_to = haz_map_get_path(hm, up, right);
			num_neighbors++;
			curr_tile->neighbors[num_neighbors] = &wm->worldMap[(gridy)*(wm->width) + (gridx + 1)];
			curr_tile->neighbors[num_neighbors]->path_to = haz_map_get_path(hm, y, right);
			num_neighbors++;
		}
	}
	if( down >= -max_y ) {
		curr_tile->neighbors[num_neighbors] = &wm->worldMap[(gridy + 1)*(wm->width) + (gridx)];
		curr_tile->neighbors[num_neighbors]->path_to = haz_map_get_path(hm, down, x);
		num_neighbors++;
		if( left >= -max_x ){
			curr_tile->neighbors[num_neighbors] = &wm->worldMap[(gridy + 1)*(wm->width) + (gridx - 1)];
			curr_tile->neighbors[num_neighbors]->path_to = haz_map_get_path(hm, down, left);
			num_neighbors++;
		}
		if( right <= max_x ){
			curr_tile->neighbors[num_neighbors] = &wm->worldMap[(gridy + 1)*(wm->width) + (gridx + 1)];
			curr_tile->neighbors[num_neighbors]->path_to = haz_map_get_path(hm, down, right);
			num_neighbors++;
		}
	}

	//printf("num_neighbors: %d", num_neighbors);

	//evaluate grid cell distance for all neighbors
	for (int i = 0; i < num_neighbors; i++){
		double distance = curr_tile->neighbors[i]->path_to->distance;
		int grid_dist = (distance + WORLD_MAP_RES/2)  / WORLD_MAP_RES;
		printf("neighbor %d has distance %f and grid_dist %d\n", i, distance, grid_dist);
		curr_tile->neighbors[i]->distance = grid_dist;

	}

	//qsort
	qsort(curr_tile->neighbors, num_neighbors, sizeof(world_map_tile_t), movement_compare);

	printf("target path length: %d \n", curr_tile->neighbors[0]->path_to->length);
    state->targetPath = curr_tile->neighbors[0]->path_to;
    state->targetPathValid  = 1;


	return curr_tile->neighbors[0]->path_to;
}



















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



int explorer_check_region(explorer_t *ex, haz_map_t *hm, int region, double theta) {
	matd_t *forwardRot, *checkPosTrans, *checkPosRot, *checkPos, *origin, *pos, *trans;
	double originD[3] = {0, 0, 1};
	double fRotD[9] = {cos(theta), -sin(theta), 0, sin(theta), cos(theta), 0, 0, 0, 1};

	origin = matd_create_data(3, 1, originD);
	pos = matd_identity(3);
	matd_put(pos, 0, 2, hm->width/2);
	matd_put(pos, 1, 2, hm->height/2);
	forwardRot = matd_create_data(3, 3, fRotD);
	trans = matd_identity(3);
	int count = 1;
	int u, v;
	int hmMaxY = hm->height;
	int hmMinY = 0;
	int hmMaxX = hm->width;
	int hmMinX = 0;
	int cont = 1;
	int dist = -1;
	haz_map_tile_t tile;

	while (cont == 1) {
		if (region == EXPLORER_REGION_FORWARD) {
			matd_put(trans, 1, 2, count * EXPLORER_TRACE_DIST);
		} else {
			matd_put(trans, 0, 2, -count * EXPLORER_TRACE_DIST);
		}
		checkPosTrans = matd_multiply(pos, forwardRot);
		checkPosRot = matd_multiply(checkPosTrans, trans);
		checkPos = matd_multiply(checkPosRot, origin);
		u = matd_get(checkPos, 0, 0);
		v = matd_get(checkPos, 1, 0);
		matd_destroy(checkPosTrans);
		matd_destroy(checkPosRot);
		matd_destroy(checkPos);
		//printf("u: %d, v: %d\n", u, v);

		if (u > hmMinX && u < hmMaxX && v > hmMinY && v < hmMaxY) {
			haz_map_get(hm, &tile, u, v);

			switch (tile.type) {
				case HAZ_MAP_OBSTACLE:
					dist = count;
					cont = 0;
				break;
				default:
					count++;
				break;
			}
		} else {
			// wall not found
			cont = 0;
			break;
		}
	}

	matd_destroy(forwardRot);
	matd_destroy(pos);
	matd_destroy(trans);
	return dist;
}


explorer_state_t explorer_run(explorer_t *ex, haz_map_t *hm, double x, double y, double theta) {
	/*int forwardDist = explorer_check_region(ex, hm, EXPLORER_REGION_FORWARD, theta);
	int leftDist = explorer_check_region(ex, hm, EXPLORER_REGION_LEFT, theta);
	printf("forwardDist: %d\n", forwardDist);
<<<<<<< HEAD
	printf("leftDist: %d\n", leftDist);*/
	return EX_MOVE;


}

path_t* explorer_get_move(explorer_t *ex) {
	return ex->path;
}


