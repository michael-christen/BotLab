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

void driveStop(state_t * state) {
    driveRad(state, 0, 0);
}

void driveToTheta(state_t * state, double theta) {
	double thresh = 0.1;

	state->goal_theta = theta;

	while(abs(state->goal_theta - state->pos_theta) > thresh){
		//Won't quite work yet, I have some left overs
		//from green targeting pid
		//
		double difference = state->pos_theta - state->goal_theta;
		if(difference > M_PI){
			state->goal_theta -= 2 * M_PI;
		}else if(difference < -M_PI){
			state->goal_theta += 2 * M_PI;
		}

		/*if(state->pos_theta > 3*M_PI/2 && state->goal_theta < M_PI/2){
			difference -= 2*M_PI;
		}else if(state->pos_theta < M_PI/2 && state->goal_theta > 3*M_PI/2){
			difference += 2*M_PI;
		}*/
		double pid_out = pid_get_output(state->theta_pid,
				state->pos_theta - state->goal_theta);
		double motor_val = pid_to_rot(state->theta_pid, pid_out);


		driveRot(state, motor_val);
		usleep(10000);
	}
	driveStop(state);
}

void rotateTheta(state_t * state, double theta) {
	driveToTheta(state, fmod(state->pos_theta + theta, 2*M_PI));
}

void driveToPosition(state_t * state, position_t position){
	double thresh = 0.3;

	state->goal_x = position.x;
	state->goal_y = position.y;

	if(fabs(state->goal_x - state->pos_x) > thresh ||
		fabs(state->goal_y - state->pos_y) > thresh){
		double dx = position.x - state->pos_x;
		double dy = position.y - state->pos_y;
		double dtheta = atan2(dy, dx);
		driveToTheta(state, dtheta);

		state->waiting_on_pos = 1;
		state->waiting_on_theta = 0;
		//drive to position
		driveStraight(state, 1);

		pthread_mutex_lock(&state->drive_mutex);
		pthread_cond_wait(&state->drive_cond, &state->drive_mutex);
		pthread_mutex_unlock(&state->drive_mutex);

		state->waiting_on_pos = 0;
		driveStop(state);
	}
}
