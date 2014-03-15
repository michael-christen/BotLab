#include "drive_ctrl.h"
#include <stdlib.h>
#include <math.h>
#define EPSILON 0.0001

int equiv(double x, double y) {
	return (fabs(x-y) < EPSILON);
}
int check(double x, double y) {
	int val = equiv(x, y);
	if(!val) {
		printf("FAILURE x: %f, y: %f\n", 
				x, y);
	}
	return val;
}
void set_goal(state_t *state, double x, double y) {
	state->goal_x = x;
	state->goal_y = y;
}

void set_pos(state_t *state, double x, double y) {
	state->pos_x = x;
	state->pos_y = y;
}

void set_theta(state_t *state, double theta) {
	state->pos_theta = theta;
}

void test_diff_traj() {
	state_t * state = malloc(sizeof(state_t));

	set_pos(state, 0, 0);
	set_goal(state, 1, 1);
	set_theta(state, 0);
	assert(check(getDiffTraj(state), -M_PI/4));

	set_pos(state, 1, 1);
	set_goal(state, 2, 2);
	set_theta(state, 0);
	assert(check(getDiffTraj(state), -M_PI/4));

	set_pos(state, 1, 1);
	set_goal(state, 0, 2);
	set_theta(state, 0);
	assert(check(getDiffTraj(state), M_PI/4));

	set_pos(state, 1, 1);
	set_goal(state, 0, 2);
	set_theta(state, M_PI/4);
	assert(check(getDiffTraj(state), 0));
}

void test_theta_dist() {
}

int main() {
	test_diff_traj();
	test_theta_dist();
	return 0;
}
