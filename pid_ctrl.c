#include "pid_ctrl.h"

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
    clock_t cur_clock = clock();
    double dt         = (cur_clock - pid->prev_clk + 0.0)/CLOCKS_PER_SEC;
    double err      = pid->goal - meas;
    double derivative = 0;

    if(pid->first_meas) {
	pid->first_meas = 0;
    } else {
	pid->integral += err*dt;
	derivative     = (err - pid->prev_err)/dt;
    }

    double output     = pid->P*err +
	                pid->I*pid->integral +
			pid->D*derivative;

    pid->prev_err     = err;
    pid->prev_clk     = cur_clock;
    return output;
}

double pid_to_rot(double pid_out) {
    //There
    if(fabs(pid_out) < MIN_OUTPUT) {
	printf("there\n");
	return 0;
    }
    if(fabs(pid_out) > MAX_OUTPUT) {
	//get sign
	pid_out /= fabs(pid_out);
	//Set to max_output
	pid_out *= MAX_OUTPUT;
    }
    pid_out /= -MAX_OUTPUT;
    printf("speed: %f\n",pid_out);
    return pid_out;
}
