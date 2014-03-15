#include "drive_ctrl.h"

//+ Radius goes to left
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

double getThetaDist(double from, double to) {

	from = fmod(from, 2*M_PI);
	to   = fmod(to, 2*M_PI);
	double difference = fmod(to - from, 2*M_PI);
	double sn = sign(difference);
	if(fabs(difference) > M_PI) {
		difference = -sn*M_PI + fmod(difference, M_PI);
	}
	return difference;
}

void driveToTheta(state_t * state, double theta) {
	double thresh = 0.01;

	state->goal_theta = theta;
	state->gyro_int[2] = 0;
	int64_t beginningInt = state->gyro_int[2];
	double beginningTheta = state->pos_theta;

	while(abs(getThetaDist(state->pos_theta,state->goal_theta)) > thresh){
		//Won't quite work yet, I have some left overs
		//from green targeting pid
		//
		double difference = getThetaDist(state->pos_theta, state->goal_theta);

		/*if(difference > M_PI){
			state->goal_theta += 2 * M_PI;
		}else if(difference < -M_PI){
			state->goal_theta -= 2 * M_PI;
		}*/

		//printf("difference: %f\n",difference);
		double pid_out = pid_get_output(state->theta_pid, difference);
		// / by 2 to decrease speed
		double motor_val = pid_to_rot(state->theta_pid, pid_out)/2;

		driveRot(state, motor_val);
		usleep(5000);
	}
	int64_t endInt = state->gyro_int[2];
	double endTheta = state->pos_theta;
	double gyroTheta = (endInt - beginningInt)/state->gyro_ticks_per_theta;
	double stateTheta = endTheta - beginningTheta;
	gyroTheta = gyroTheta/M_PI * 180.0;
	stateTheta = stateTheta/M_PI * 180.0;
	printf("stopping pid with diff: %f\n",
		   getThetaDist(state->pos_theta, state->goal_theta));
	printf("Theta measured by gyro: %g\n", gyroTheta);
	printf("Theta measured by tick: %g\n", stateTheta);
	driveStop(state);
}

void rotateTheta(state_t * state, double theta) {
	theta *= -1;
	driveToTheta(state, fmod(state->pos_theta + theta, 2*M_PI));
}

void driveToPosition(state_t * state, position_t position){
	double thresh = 5;

	state->goal_x = position.x;
	state->goal_y = position.y;
	double dist = getDist(state->pos_x, state->pos_y,
			state->goal_x, state->goal_y);
	double theta= getDiffTraj(state);

	if(dist > thresh) {
		printf("theta chosen: %f\n",theta);
		//rotateTheta(state, theta);
	}
	pid_ctrl_t *pos_pid = malloc(sizeof(pid_ctrl_t));
	pid_init(pos_pid, 0.03, 0.0, 0.0, 0.0, 0, 0.3);
	pid_ctrl_t *angle_pid = malloc(sizeof(pid_ctrl_t));
	pid_init(angle_pid, 1.0, 0.0, 0.0, 0.0, 0, M_PI);
	while(dist > thresh && state->goToMouseCoords){
		dist = getDist(state->pos_x, state->pos_y,
			state->goal_x, state->goal_y);
		theta= getDiffTraj(state);
		printf("dist: %f, theta: %f\n",dist, theta);

		//Negative is to the right
		//Positive is to the left
		double base   = 2;
		double norm   = 1000;
		double ang_f  = -pid_get_output(angle_pid, theta);
		int    dir    = sign(ang_f);
		double mag_ang= fabs(ang_f);
		double radius = dir*norm*pow(base,-mag_ang);
		//double radius = dir*norm / mag_ang;

		double speed_f= pid_get_output(pos_pid, dist);
		double speed  = fabs(speed_f);
		printf("radius: %f, speed: %f\n",radius, speed);

		double thresh_rad = M_PI/2;
		/*
		if(fabs(radius) >= thresh_rad) {
			rotateTheta(theta);
		} else {
			driveRad(state, radius, speed);
		}
		*/

		usleep(10000);
	}
	driveStop(state);
	free(pos_pid);
	free(angle_pid);
}

double getDist(double cur_x, double cur_y,
						double new_x, double new_y) {
	return sqrt(pow(cur_x-new_x,2) + pow(cur_y-new_y,2));
}

double getTheta(double x, double y) {
	return atan2(y, x);
}

//- is to the right
double getDiffTraj(state_t *state) {
	double cur_theta = fmod(-state->pos_theta + M_PI/2, 2*M_PI);
	double goal_theta = getTheta(state->goal_x - state->pos_x,
					      state->goal_y - state->pos_y);
	double diff_theta = fmod( getThetaDist(
										   cur_theta,
										   goal_theta
										  ),
							  M_PI
							);
	/*
	printf("cur: %f, goal: %f, diff: %f\n",
		   cur_theta, goal_theta, diff_theta
		  );
		  */
	return diff_theta;
}
