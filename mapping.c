#include "mapping.h"
#include "stdio.h"
#include "haz_map.h"

//not using add_obstacles_to_map right now, no global map
/*
void add_obstacles_to_map(double x_rel, double y_rel, void * data){
	
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

	int scale_x = (x+GRID_RES/2)/GRID_RES; //add half res to make it round, not truncate
	int scale_y = (y+GRID_RES/2)/GRID_RES; //add half res to make it round, not truncate

	state->obstacle_map[scale_x][scale_y].status = OCCUPIED;
	state->obstacle_map[scale_x][scale_y].created = clock();



	return;

}
*/

void add_obstacles_to_haz_map( double x_rel, double y_rel, void * data, haz_map_t *hm, int obstacle){
	state_t * state = data;
	double rot_theta = /*state->gyro[0]*/ 0; //need to get from gyro sensors. gyro[0/1/2?]

	matd_t *rel_coords = matd_create_data(3, 1, (double[]) {	x_rel,		
																y_rel,	
																1});


	matd_t *R = matd_create_data(3, 3, (double[]) {	cos(rot_theta),		sin(rot_theta),	0,
													-sin(rot_theta),	cos(rot_theta),	0,
													0,					0,				1});

	matd_t * real = matd_multiply(R, rel_coords);


	x_rel = matd_get(real, 0, 0);
	y_rel = matd_get(real, 1, 0);

	double xbias = 1.5;

	printf("adding obstacle at x: %f, y: %f\n", x_rel + xbias, y_rel);
	


	if(obstacle == 1){
		haz_map_set(hm, (x_rel + HAZ_MAP_MAX_WIDTH/2 + xbias), y_rel, HAZ_MAP_OBSTACLE);
	}
	else{
		haz_map_set(hm, (x_rel + HAZ_MAP_MAX_WIDTH/2 + xbias), y_rel, HAZ_MAP_FREE);
	}
}

void find_point_pos( void * data, int x_px, int y_px, haz_map_t *hm, int obstacle){
	/*
		pixel location, defined by its x,y coords, 
		reference to hazard map,
		int obstacle: 0 if free space, 1 if obstacle
	*/
	state_t * state = data;


	matd_t * H = matd_create_data(3, 3, (double[]) { 0.014442,       0.002133,      -6.026192,
      															-0.001299,      -0.000377,       5.889305,
    																-0.000036,       0.001629,      -0.385430});
	

	//determine x and y coordinates, relative to bruce, using homography project fcn

//determine x and y coordinates, relative to bruce

	double x_rel = 0, y_rel = 0;
	homography_project(H, x_px, y_px, &x_rel, &y_rel);


	//add_obstacles_to_map( x_rel, y_rel, data);
	add_obstacles_to_haz_map( x_rel, y_rel, data, hm, obstacle);

	return;
}






void find_H_matrix(void * data){
	state_t * state = data;
	//click_array is x and y of pixels clicked, 4 for now
	zarray_t * rw_coords = zarray_create(sizeof(float[2]));
	float real[2] = {0, 0};
	real[0] = -30;
	real[1] = 60;
	zarray_add(rw_coords, real);
	real[0] = 0;
	real[1] = 30;
	zarray_add(rw_coords, real);
	real[0] = 0;
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

	matd_t * H = homography_compute(correspondences);

	matd_print(H, "%15f");

	//from barrel distortion fixed image


}






