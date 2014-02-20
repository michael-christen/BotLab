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
    double dL       = getDistFromTicks(msg->encoder_left_ticks);
    double dR       = getDistFromTicks(msg->encoder_right_ticks);
    double avg      = (dL + dR)/2.0;
    //msg->motor_left_actual_speed;
    state->pos_x    += avg * cos(state->pos_theta);
    state->pos_y    += avg * sin(state->pos_theta);
    state->pos_theta+= (dR - dL) / DIST_BETWEEN_WHEELS;
}

double getDistFromTicks(int32_t ticks) {
    return (ticks + 0.0) / (TICKS_PER_CM + 0.0);
}
