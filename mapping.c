#include "mapping.h"
#include "stdio.h"
#include "haz_map.h"


void add_obstacles_to_map(double x_rel, double y_rel, void * data){
	//data is state
	state_t * state = data;

	//get obstacles into map coords with a rotation based on bruce's rotation, translation based on bruce's location
	double rot_theta = 0; //need to get from gyro sensors. gyro[0/1/2?]
	double bruce_x = state->pos_x, bruce_y = state->pos_y;

	matd_t *rel_coords = matd_create_data(3, 1, (double[]) {	x_rel,
																y_rel,
																0});
	matd_t *R = matd_create_data(3, 3, (double[]) {	cos(rot_theta),		sin(rot_theta),	0,
													-sin(rot_theta),	cos(rot_theta),	0,
													0,					0,				1});
	matd_t *T = matd_create_data(3, 3, (double[]) {	1,	0,  bruce_x,
													0,	1,	bruce_y,
													0,	0,	1});
	matd_t * rel_to_real = matd_multiply(T, R);

	matd_t * real = matd_multiply(rel_to_real, rel_coords);


	int x = matd_get(real, 0, 0);
	int y = matd_get(real, 0, 1);

	int scale_x = (x+5)/10; //add 5 to make it round, not truncate
	int scale_y = (y+5)/10; //add 5 to make it round, not truncate

	state->obstacle_map[scale_x][scale_y].status = OCCUPIED;
	state->obstacle_map[scale_x][scale_y].created = clock();

/*
	int bruce_grid_x = (bruce_x + 5)/10; //add 5 to make it round, not truncate
	bruce_grid_x++; //bruce can't see 1 grid cell in front of him

	for(bruce_grid_x; bruce_grid_x < scale_x; bruce_grid_x++){
			state->obstacle_map[bruce_grid_x][scale_y].status = UNOCCUPIED;
			state->obstacle_map[bruce_grid_x][scale_y].created = clock();
	}
*/ //can't do this unless i take theta into account
	return;

}

void add_obstacles_to_haz_map( double x_rel, double y_rel, void * data, haz_map_t *hm){
	double rot_theta = 0; //need to get from gyro sensors. gyro[0/1/2?]

	matd_t *rel_coords = matd_create_data(3, 1, (double[]) {	x_rel,
																y_rel,
																0});

	matd_t *R = matd_create_data(3, 3, (double[]) {	cos(rot_theta),		sin(rot_theta),	0,
													-sin(rot_theta),	cos(rot_theta),	0,
													0,					0,				1});

	matd_t * real = matd_multiply(R, rel_coords);


	x_rel = matd_get(real, 0, 0);
	y_rel = matd_get(real, 0, 1);

	haz_map_set(hm, (x_rel + HAZ_MAP_MAX_WIDTH/2), y_rel, HAZ_MAP_OBSTACLE);
	y_rel--;
	for(y_rel; y_rel > 10; y_rel--){
		haz_map_set(hm, (x_rel + HAZ_MAP_MAX_WIDTH/2), y_rel, HAZ_MAP_FREE);
	}
}

void find_point_pos( void * data, int x_px, int y_px, haz_map_t *hm){

	state_t * state = data;
	matd_t * px_coords = matd_create_data(3, 1, (double[]) {	x_px,
																y_px,
																1});
	matd_t * H = matd_create_data(3, 3, (double[]) {	0,	0,  210,
																0,	0,	-2472,
																0,	0,	-52});
	matd_t * rel_coords = matd_multiply(H, px_coords);

//determine x and y coordinates, relative to bruce
	double x_rel, y_rel;
	x_rel = matd_get(rel_coords, 0, 0);
	y_rel = matd_get(rel_coords, 0, 1);

	add_obstacles_to_map( x_rel, y_rel, data);
	add_obstacles_to_haz_map( x_rel, y_rel, data, hm);

	return;

}






void find_H_matrix(){
	//click_array is x and y of pixels clicked, 4 for now
	zarray_t * rw_coords = zarray_create(sizeof(float[2]));
	float real[2] = {0, 0};
	real[0] = -30;
	real[1] = 60;
	zarray_add(rw_coords, real);
	real[0] = 0;
	real[1] = 90;
	zarray_add(rw_coords, real);
	real[0] = 0;
	real[1] = 60;
	zarray_add(rw_coords, real);
	real[0] = 60;
	real[1] = 90;
	zarray_add(rw_coords, real);

	zarray_t * click_array = zarray_create(sizeof(float[2]));
	float pix[2] = {0, 0};
	pix[0] = 80;
	pix[1] = 144;
	zarray_add(click_array, pix);
	pix[0] = 315;
	pix[1] = 162;
	zarray_add(click_array, pix);
	pix[0] = 315;
	pix[1] = 141;
	zarray_add(click_array, pix);
	pix[0] = 615;
	pix[1] = 159;
	zarray_add(click_array, pix);

	zarray_t * correspondences = zarray_create(sizeof(float[4]));

	float coordinates[4];
	int click[2];

	for(int i = 0; i < 4; i++){
		zarray_get(click_array, i, click);
		zarray_get(rw_coords, i, real);
		coordinates[0] = click[0];
		coordinates[1] = click[1];
		coordinates[2] = real[0];
		coordinates[3] = real[1];
		zarray_add(correspondences, coordinates);
	}

	//matd_t * H = homography_compute(correspondences);
	matd_t * H = matd_create(3,3);

	int H00 = matd_get(H, 0, 0);
	int H01 = matd_get(H, 0, 1);
	int H02 = matd_get(H, 0, 2);
	int H10 = matd_get(H, 1, 0);
	int H11 = matd_get(H, 1, 1);
	int H12 = matd_get(H, 1, 2);
	int H20 = matd_get(H, 2, 0);
	int H21 = matd_get(H, 2, 1);
	int H22 = matd_get(H, 2, 2);

	matd_destroy(H);

	printf(" %d, %d, %d, \n %d, %d, %d, \n %d, %d, %d, \n", H00, H01, H02, H10, H11, H12, H20, H21, H22);
}






