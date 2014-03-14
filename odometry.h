#ifndef __ODOMETRY__H__
#define __ODOMETRY__H__
#include "lcmtypes/maebot_motor_feedback_t.h"
#include "common/matd.h"
#include <math.h>

#define DIST_BETWEEN_WHEELS 8.1
#define WHEEL_CIRCUMFERENCE 10
#define TICKS_PER_CM        45.3 
#define LONG_ERR            0.05
#define LAT_ERR             0.02

typedef struct od odometry_t;
struct od{ 
	double x, y, theta;
};

void odometry_handler (
	const lcm_recv_buf_t *rbuf,
	const char * channel,
	const maebot_motor_feedback_t * msg,
	void * data
);

odometry_t get_odometry_data(int32_t diff_left, int32_t diff_right); 

/*Get 3 x 3 transformation matrix from given odometry
  [ cos(t) | -sin(t) | x ]
  [ sin(t) |  cos(t) | y ]
  [   0    |    0    | 1 ]
*/
matd_t * get_transform_mat(odometry_t od);

//Get transform matrices from odometries, multiply into
//each other, and get the updated x, y, and theta
odometry_t get_updated_od(odometry_t prev, odometry_t cur);

/*Get 3 x 6 jacobian from combining these 2 odometries
  [ 1 | 0 | g | cos(a_theta) | -sin(a_theta) | 0 ]
  [ 0 | 1 | h | sin(a_theta) | cos(a_theta)  | 0 ]
  [ 0 | 0 | 1 |       0      |      0        | 1 ]
g: 0, could be -sin(a_theta)*bx - cos(a_theta)*by
h: 0, could be  cos(a_theta)*bx - sin(a_theta)*by
*/
matd_t * get_jacobian(odometry_t prev, odometry_t cur);

/*Returns 3 x 3 Matrix that represents variance of x,y,theta
  [ (sr^2+sl^2)/4  |   0  | (sr^2-sl^2)/2b  ]
  [      0         | ss^2 |       0         ]
  [ (sr^2-sl^2)/2b |   0  | (sr^2+sl^2)/b^2 ]
*/
matd_t * get_motor_variance(int32_t diff_left, int32_t diff_right);

//Returns 3 x 3 Matrix of updated variance by combining cur variance
//with previous variances
matd_t * get_updated_variance(
		matd_t *prev_variance, odometry_t prev_od, 
		matd_t *cur_variance,  odometry_t cur_od);

double getDistFromTicks(int32_t ticks);


#endif
