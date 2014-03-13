#ifndef __PID_CTRL__H__
#define __PID_CTRL__H__
#include<time.h>
#include<stdio.h>
#include<math.h>
#include<float.h>

//#define MAX_VAL 100
//#define MIN_OUTPUT 16
#define MAPPING_FACTOR  0.4
typedef struct pid pid_ctrl_t;
struct pid {
    double goal;
    double P, I, D;

    double integral;
    clock_t prev_clk;
    double  prev_err;

    //Boolean to not evaluate integral or derivative on first attempt
    int first_meas;

	int min_output, max_val;
};

void   pid_init(pid_ctrl_t *pid, double P, double I, double D, double goal, int min_output, int max_val);
void   pid_update_pid(pid_ctrl_t *pid, double P, double I, double D);
void   pid_update_goal(pid_ctrl_t *pid, double goal);
double pid_get_output(pid_ctrl_t *pid, double meas);
double pid_to_rot(pid_ctrl_t *pid, double pid_out);
#endif
