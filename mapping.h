#ifndef MAP_H
#define MAP_H

#include <math.h>
#include "common/matd.h"
#include "common/zarray.h"
#include "common/homography.h"
#include <stdio.h>
#include "maebot_app.h"
#include "time.h"


//void add_obstacles_to_map(double x_rel, double y_rel, void * data);

void homography_project(const matd_t *H, double x, double y, double *ox, double *oy);

void find_point_pos( void * data, double theta, double bruce_x, double bruce_y, int obstacle);

void find_H_matrix();



#endif
