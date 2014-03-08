#ifndef MAP_H
#define MAP_H

#include <math.h>
#include "common/matd.h"
#include "common/zarray.h"
#include "common/homography.h"
#include <stdio.h>
#include "maebot_app.h"
#include "time.h"


void add_obstacles_to_map(double x_rel, double y_rel, void * data);

void find_point_pos( void * data, int x_px, int y_px);

void find_H_matrix(zarray_t * click_array, int za_size);



#endif