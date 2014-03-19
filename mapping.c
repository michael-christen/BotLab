#include "mapping.h"
#include "stdio.h"
#include "map.h"
#include "pixel.h"

void add_obstacles_to_map( double x_rel, double y_rel, double bruce_x, double bruce_y, double theta, map_t *map, int obstacle){

	//matrices for position realtive to bruce, rotation, and multiplication of the 2
	matd_t *rel_coords = matd_create_data(3, 1, (double[]) {	x_rel,
																y_rel,
																1});

	//printf("original coords x: %f, y: %f\n", x_rel, y_rel);
	matd_t *R = matd_create_data(3, 3, (double[]) {	cos(theta),	-sin(theta),	0,
													sin(theta),	 cos(theta),	0,
													0,					0,				1});

	matd_t * real = matd_multiply(R, rel_coords);

	x_rel = matd_get(real, 0, 0);
	y_rel = matd_get(real, 1, 0);


	//constant left bias
	double xbias = 1.5; //cm


	double map_x = x_rel + xbias;
	double map_y = y_rel;

	int map_x_scaled = (map_x/MAP_RES) +(map->width/2) + bruce_x;
	int map_y_scaled = (map_y/MAP_RES) +(map->height/2) + bruce_y;

	if ( map_x_scaled < 0 ||  map_x_scaled > map->width){
		return;
	}
	if ( map_y_scaled < 0 ||  map_y_scaled > map->height){
		return;
	}

	//place point on map
	if(obstacle == 1){
		map_set(map,  map_x_scaled,  map_y_scaled, MAP_OBSTACLE);
	}
	else{
		//if (checkTile.type != HAZ_MAP_INVALID && checkTile.type != HAZ_MAP_OBSTACLE) {
			map_set(map,  map_x_scaled,  map_y_scaled, MAP_FREE);
		//}
	}

	matd_destroy(rel_coords);
	matd_destroy(R);
	matd_destroy(real);

	return;
}




void homography_project(const matd_t *H, double x, double y, double *ox, double *oy)
{
    double xx = MATD_EL(H, 0, 0)*x + MATD_EL(H, 0, 1)*y + MATD_EL(H, 0, 2);
    double yy = MATD_EL(H, 1, 0)*x + MATD_EL(H, 1, 1)*y + MATD_EL(H, 1, 2);
    double zz = MATD_EL(H, 2, 0)*x + MATD_EL(H, 2, 1)*y + MATD_EL(H, 2, 2);

    *ox = xx / zz;
    *oy = yy / zz;
}







void find_point_pos( void * data, double theta, double bruce_x, double bruce_y, int obstacle){
	/*
		int obstacle: 0 if free space, 1 if obstacle
	*/
	printf("top of find point pos\n");
	state_t * state = data;
	map_t *map = &state->map;

	matd_t * H = matd_create_data(3, 3, (double[]) { 0.014442,       0.002133,      -6.026192,
      															-0.001299,      -0.000377,       5.889305,
    																-0.000036,       0.001629,      -0.385430});

	//homography_project(H, x_px, y_px, &x_rel, &y_rel);

	int points = state->num_pts_tape;
	int i;
	double x_cm, y_cm;

	for(i = 0; i < points; i++){
		obstacle = 1;
		double x_px = state->tape[i].x;
		double y_px = state->tape[i].y;
		homography_project(H, x_px, y_px, &x_cm, &y_cm);
		add_obstacles_to_map( x_cm, y_cm, bruce_x, bruce_y, theta, map, obstacle);
		obstacle = 0;
		for(y_px = y_px + 10; y_px < 480; y_px += 10){
			homography_project(H, x_px, y_px, &x_cm, &y_cm);
			add_obstacles_to_map(x_cm, y_cm, bruce_x, bruce_y, theta, map, obstacle);
		}
	}
	printf("above compute config\n");
	map_compute_config(map);
	printf("after compute config\n");

	matd_destroy(H);
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
	real[1] = 30;
	zarray_add(rw_coords, real);
	real[0]= 0;
	real[1] = 60;
	zarray_add(rw_coords, real);
	real[0] = 0;
	real[1] = 90;
	zarray_add(rw_coords, real);
	real[0] = 30;
	real[1] = 60;
	zarray_add(rw_coords, real);
	real[0] = -15;
	real[1] = 30;
	zarray_add(rw_coords, real);
	real[0] = 15;
	real[1] = 30;
	zarray_add(rw_coords, real);

	zarray_t * click_array = zarray_create(sizeof(float[2]));
	float pix[2] = {0, 0};
	pix[0] = 190;
	pix[1] = 298;
	zarray_add(click_array, pix);
	pix[0] = 372;
	pix[1] = 352;
	zarray_add(click_array, pix);
	pix[0] = 373;
	pix[1] = 301;
	zarray_add(click_array, pix);
	pix[0] = 376;
	pix[1] = 281;
	zarray_add(click_array, pix);
	pix[0] = 559;
	pix[1] = 300;
	zarray_add(click_array, pix);
	pix[0] = 156;
	pix[1] = 357;
	zarray_add(click_array, pix);
	pix[0] = 525;
	pix[1] = 349;
	zarray_add(click_array, pix);

	zarray_t * correspondences = zarray_create(sizeof(float[4]));

	float coordinates[4];
	float click[2];

	for(int i = 0; i < 7; i++){
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

	matd_print(H, "%15f");

	//from barrel distortion fixed image


}






