#include "mapping.h"
#include "stdio.h"


void add_obstacles_to_map(double x_rel, double y_rel, void * data){
	//data is state
	state_t * state = data;

	//get obstacles into map coords with a rotation based on bruce's rotation, translation based on bruce's location
	double rot_theta = 0; //need to get from gyro sensors, not hardcode
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


	//a little stuck on how to fill these in along the diagonal from left side to right side
	//find slope...for x = left_x to right_x, fill in appropriate y? and up one, down 1?
	//remember right and left are meaningless now


	int x = matd_get(real, 0, 0);
	int y = matd_get(real, 0, 1);

	int scale_x = (x+5)/10; //add 5 to make it round, not truncate
	int scale_y = (y+5)/10; //add 5 to make it round, not truncate

	state->obstacle_map[scale_x][scale_y].status = OCCUPIED;
	state->obstacle_map[scale_x][scale_y].created = clock();
	return;

}



void find_point_pos( void * data, int x_px, int y_px){
	//void pointer to state, can state hold blob?
	state_t * state = data;
	matd_t * px_coords = matd_create_data(3, 1, (double[]) {	x_px,
																y_px,
																1});
	matd_t * H = matd_create_data(3, 3, (double[]) {	1,	0,  0,
																0,	1,	0,
																0,	0,	1});
	matd_t * rel_coords = matd_multiply(H, px_coords);
	
//determine x and y coordinates of blob, relative to bruce
	double x_rel, y_rel;
	x_rel = matd_get(rel_coords, 0, 0);
	y_rel = matd_get(rel_coords, 0, 1);


	add_obstacles_to_map( x_rel, y_rel, data);

}






void find_H_matrix(zarray_t * click_array, int za_size){
	//click_array is x and y of pixels clicked, 4 for now
	zarray_t * rw_coords = zarray_create(sizeof(float[2]));
	float real[2] = {0, 0};
	//real[0] = x1;
	//real[1] = y1;
	zarray_add(rw_coords, real);
	//real[0] = x2;
	//real[1] = y2;
	zarray_add(rw_coords, real);
	//real[0] = x3;
	//real[1] = y3;
	zarray_add(rw_coords, real);
	//real[0] = x4;
	//real[1] = y4;
	zarray_add(rw_coords, real);

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

	matd_t * H = homography_compute(correspondences);

	int H00 = matd_get(H, 0, 0);
	int H01 = matd_get(H, 0, 1);
	int H02 = matd_get(H, 0, 2);
	int H10 = matd_get(H, 1, 0);
	int H11 = matd_get(H, 1, 1);
	int H12 = matd_get(H, 1, 2);
	int H20 = matd_get(H, 2, 0);
	int H21 = matd_get(H, 2, 1);
	int H22 = matd_get(H, 2, 2);

	printf(" %d, %d, %d, \n %d, %d, %d, \n %d, %d, %d, \n", H00, H01, H02, H10, H11, H12, H20, H21, H22);
}






