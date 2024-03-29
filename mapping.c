#include "mapping.h"
#include "stdio.h"
#include "haz_map.h"
#include "pixel.h"

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

void add_obstacles_to_haz_map( double x_rel, double y_rel, double bruce_x, double bruce_y, double theta, haz_map_t *hm, int obstacle){

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

	int map_x_scaled = (map_x/GRID_RES) +(HAZ_MAP_MAX_WIDTH/2);
	int map_y_scaled = (map_y/GRID_RES) +(HAZ_MAP_MAX_HEIGHT/2);

	if ( map_x_scaled < 0 ||  map_x_scaled > HAZ_MAP_MAX_WIDTH){
		return;
	}
	if ( map_y_scaled < 0 ||  map_y_scaled > HAZ_MAP_MAX_HEIGHT){
		return;
	}

	//place point on haz_map
	if(obstacle == 1){
		haz_map_set(hm,  map_x_scaled,  map_y_scaled, HAZ_MAP_OBSTACLE);
	}
	else{
		//haz_map_tile_t checkTile;
		//haz_map_get(hm, &checkTile, map_x_scaled, map_y_scaled);
		//if (checkTile.type != HAZ_MAP_INVALID && checkTile.type != HAZ_MAP_OBSTACLE) {
			haz_map_set(hm,  map_x_scaled,  map_y_scaled, HAZ_MAP_FREE);
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
	state_t * state = data;
	haz_map_t *hm = &state->hazMap;

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
		add_obstacles_to_haz_map( x_cm, y_cm, bruce_x, bruce_y, theta, hm, obstacle);
		obstacle = 0;
		for(y_px = y_px + 10; y_px < 480; y_px += 10){
			homography_project(H, x_px, y_px, &x_cm, &y_cm);
			add_obstacles_to_haz_map( x_cm, y_cm, bruce_x, bruce_y, theta, hm, obstacle);
		}
	}

	haz_map_cleanup(hm);
	haz_map_compute_config(hm);

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






