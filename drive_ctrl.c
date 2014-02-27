#include "drive_ctrl.h"

void driveRad(state_t * state, double radius, double speed) {
    double b        = DIST_BETWEEN_WHEELS;
    double rc       = abs(radius);
    double l_speed  = rc - b/2;
    double r_speed  = rc + b/2;
    //Invert ratio to go in opposite direction
    if(radius < 0) {
	double temp = l_speed;
	l_speed  = r_speed ;
	r_speed  = temp;
    }
    //Normalize so greatest = 1
    double max = (l_speed > r_speed ) ?
		    l_speed : r_speed ;
    l_speed  /= max;
    r_speed  /= max;

    //Factor by speed
    l_speed  *= speed;
    r_speed  *= speed;

    pthread_mutex_lock(&state->cmd_mutex);
    state->cmd.motor_left_speed  = l_speed;
    state->cmd.motor_right_speed = r_speed;
    pthread_mutex_unlock(&state->cmd_mutex);
}

void driveStraight(state_t * state, double speed) {
    driveRad(state, DBL_MAX, speed);
}

void driveRot(state_t * state, double omega) {
    driveRad(state, 0, omega);
}

void driveToTheta(state_t * state, double theta) {
	//Rotate to theta
}

void driveStop(state_t * state) {
    driveRad(state, 0, 0);
}

