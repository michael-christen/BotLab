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

	//l_speed  += state->left_offset;

	//Normalize so greatest = 1
	double max = (l_speed > r_speed ) ?
		l_speed : r_speed ;
	l_speed  /= max;
	r_speed  /= max;

	//Factor by speed
	l_speed  *= speed;
	r_speed  *= speed;

	//printf("L: %f, R: %f, offset: %f\n",l_speed, r_speed, state->left_offset);


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
	//0.1 ~= 6 degrees
	double thresh = state->theta_pid->min_output;

	//Need to do this to clear out previous integral and stuff
	pid_update_goal(state->theta_pid, 0);
	state->goal_theta = theta;
	state->gyro_int[2] = 0;
	int64_t beginningInt = state->gyro_int[2];
	double beginningTheta = state->pos_theta;
	int    num_zeros      = 0;
	int    min_zeros      = 5;

	while(num_zeros < min_zeros){
		//Won't quite work yet, I have some left overs
		//from green targeting pid
		//
		double difference = getThetaDist(state->pos_theta, state->goal_theta);
		if(fabs(difference) < thresh) {
			num_zeros ++;
		} else {
			num_zeros = 0;
		}

		/*if(difference > M_PI){
			state->goal_theta += 2 * M_PI;
		}else if(difference < -M_PI){
			state->goal_theta -= 2 * M_PI;
		}*/

		double pid_out = -pid_get_output(state->theta_pid, difference);
		// / by 2 to decrease speed
		double motor_val = pid_to_rot(state->theta_pid, pid_out)/2;
		//offset to linearize
	    motor_val += sign(motor_val)*0.1;
		//printf("difference: %f, pid: %f, motor_val: %f\n",difference, pid_out, motor_val);

		driveRot(state, motor_val);
		//Sampling speed is important,
		//if you oversample D will lose relevance
		usleep(50000);
		/*
		usleep(100000);
		driveStop(state);
		usleep(100000);
		*/
	}
	int64_t endInt = state->gyro_int[2];
	double endTheta = state->pos_theta;
	double gyroTheta = (endInt - beginningInt)/state->gyro_ticks_per_theta;
	printf("Gyro angle in radians: %g\n", gyroTheta);
	double stateTheta = endTheta - beginningTheta;
	gyroTheta = gyroTheta/M_PI * 180.0;
	stateTheta = stateTheta/M_PI * 180.0;
	//printf("stopping pid with diff: %f\n",
		 //  getThetaDist(state->pos_theta, state->goal_theta));
	printf("Theta measured by gyro: %g\n", gyroTheta);
	printf("Theta measured by tick: %g\n", stateTheta);
	driveStop(state);
}

void rotateTheta(state_t * state, double theta) {
	driveToTheta(state, fmod(state->pos_theta + theta, 2*M_PI));
}

void driveToPosition(state_t * state, position_t position){
	double thresh = 5;

	state->goal_x = position.x;
	state->goal_y = position.y;
	double dist = getDist(state->pos_x, state->pos_y,
			state->goal_x, state->goal_y);
	double theta= getDiffTraj(state);

	state->doing_pid_theta = 1;

	if(dist > thresh) {
		printf("theta chosen: %f\n",theta);
		rotateTheta(state,-theta);
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
		double radius = -dir*norm*pow(base,-mag_ang);
		//radius        = -norm;
		//double radius = dir*norm / mag_ang;

		double speed_f= pid_get_output(pos_pid, dist);
		double speed  = fabs(speed_f);
		printf("radius: %f, speed: %f\n",radius, speed);

		double thresh_ang = M_PI/2;
		if(fabs(theta) >= thresh_ang) {
			printf("rotating theta\n");
			rotateTheta(state, -theta);
		} else {
			driveRad(state, radius, speed);
		}

		usleep(50000);
	}
	printf("You have arrived at your destination!\n");
	state->doing_pid_theta = 0;
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
