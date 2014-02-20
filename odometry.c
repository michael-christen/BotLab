#include "odometry.h"
#include "maebot_app.h" //I don't like this either
#include <unistd.h>

void odometry_handler (
	const lcm_recv_buf_t *rbuf,
	const char * channel,
	const maebot_motor_feedback_t * msg,
	void * data
) {
    state_t * state = data;
    //x += (dL + dR)/2     --> cos(theta) * that
    //y += 0               --> sin(theta) * that
    //theta += (dR - dL)/b --> same
    //printf("left: %d\tright: %d\n",msg->encoder_left_ticks, msg->encoder_right_ticks);
    if(state->odometry_seen) {
	int32_t diff_left = msg->encoder_left_ticks - state->prev_left_ticks;
	int32_t diff_right = msg->encoder_right_ticks - state->prev_right_ticks;
	
	double dL       = getDistFromTicks(diff_left);
	double dR       = getDistFromTicks(diff_right);
	double avg      = (dL + dR)/2.0;
	//msg->motor_left_actual_speed;
	state->pos_x    += avg * sin(state->pos_theta);
	state->pos_y    += avg * cos(state->pos_theta);
	state->pos_theta+= (dR - dL) / DIST_BETWEEN_WHEELS;
	printf("x: %f\ty: %f\ttheta: %f\n",state->pos_x, state->pos_y, state->pos_theta);
    } else {
	state->odometry_seen = 1;
    }
    state->prev_left_ticks = msg->encoder_left_ticks;
    state->prev_right_ticks = msg->encoder_right_ticks;
}

double getDistFromTicks(int32_t ticks) {
    return (ticks + 0.0) / (TICKS_PER_CM + 0.0);
}
