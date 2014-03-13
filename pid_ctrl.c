#include "pid_ctrl.h"

double sign(double val) {
	if(val < 0) return -1;
	if(val > 0) return 1;
	return 0;
}

void pid_init(pid_ctrl_t *pid, double P, double I, double D, double goal) {
    pid_update_pid(pid, P, I, D);
    pid_update_goal(pid, goal);
}

void pid_update_pid(pid_ctrl_t *pid, double P, double I, double D) {
    pid->P = P;
    pid->I = I;
    pid->D = D;
}

void pid_update_goal(pid_ctrl_t *pid, double goal) {
    pid->integral   = 0;
    pid->first_meas = 1;
    pid->goal       = goal;
    //Shouldn't be used first time
    pid->prev_clk   = clock();
    pid->prev_err   = 0;
}

double pid_get_output(pid_ctrl_t *pid, double meas) {
    clock_t cur_clock   = clock();
    double dt           = (cur_clock - pid->prev_clk + 0.0)/CLOCKS_PER_SEC;
    double err          = pid->goal - meas;
	/*
	if(err < 0) {
		printf("ERROR is < 0\n");
	}
	else if(err > 0) {
		printf("ERROR is > 0\n");
	}
	*/
    double derivative   = 0;

    if(pid->first_meas) {
		pid->first_meas = 0;
    } else {
		pid->integral   += err*dt;
		derivative      = (err - pid->prev_err)/dt;
    }

	//If passes, get rid of integral
	if(sign(pid->prev_err) != sign(err)) {
		printf("SWITCH");
		pid->integral = 0;
	}

	double proportion   = pid->P * err;
	double integral     = pid->I * pid->integral;
	double deriv_out    = pid->D * -derivative;
	printf("proportion: %f\n",proportion);
	//printf("integral  : %f\n",integral);
	printf("derivative: %f\n",deriv_out);
    double output       = proportion +
		                  integral   +
						  deriv_out;

	if(fabs(output) > MAX_VAL) {
		output = sign(output) * MAX_VAL;
	}
	//STOP when within error bounds
    if(fabs(err) < MIN_OUTPUT) {
		pid->integral   = 0;
		output          = 0;
	}

    pid->prev_err       = err;
    pid->prev_clk       = cur_clock;
    return output;
}

//Map (-max_val, +max_val) -> (-1,1), ~actually want (-.14,.14)
//what to do when < min_movable, I will catch up, but will it
//take too long?
double pid_to_rot(double pid_out) {
    //There
    if(fabs(pid_out) < MIN_OUTPUT) {
		printf("there\n");
		return 0;
    }
	//Scale to (-1,1)
	pid_out /= MAX_VAL;

	//Scale to factor
	pid_out *= -MAPPING_FACTOR;

    printf("speed: %f\n",pid_out);
    return pid_out;
}

