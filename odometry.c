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
	if(state->odometry_seen){
		state->last_x = state->pos_x;
		state->last_y = state->last_y;
		state->last_theta = state->pos_theta;

		int32_t diff_left = msg->encoder_left_ticks - state->prev_left_ticks;
		int32_t diff_right = msg->encoder_right_ticks - state->prev_right_ticks;

		double dL        = getDistFromTicks(diff_left);
		double dR        = getDistFromTicks(diff_right);
		double avg       = (dL + dR)/2.0;
		matd_t *temp_var = matd_identity(2);
		matd_put(temp_var,0,0,state->alpha*avg);
		matd_put(temp_var,1,1,state->beta*avg);
		matd_t *mult_result = matd_multiply(
				state->var_matrix,temp_var);
		matd_destroy(temp_var);
		matd_destroy(state->var_matrix);
		state->var_matrix = mult_result;
		//msg->motor_left_actual_speed;
		state->pos_x     += avg * sin(state->pos_theta);
		state->pos_y     += avg * cos(state->pos_theta);
		state->pos_theta += (dL - dR) / DIST_BETWEEN_WHEELS;

		double posthesh = 0.3;
		double mthreshold = 1.0;
		double rthreshold = 0.1;

		if(state->waiting_on_pos && 
				abs(state->pos_x - state->goal_x) < mthreshold &&
				abs(state->pos_y - state->goal_y) < mthreshold){
			pthread_mutex_lock(&state->drive_mutex);
			pthread_cond_broadcast(&state->drive_cond);
			pthread_mutex_unlock(&state->drive_mutex);
		}

		if(state->waiting_on_theta &&
				abs(state->pos_theta - state->goal_theta) < rthreshold){
			pthread_mutex_lock(&state->drive_mutex);
			pthread_cond_broadcast(&state->drive_cond);
			pthread_mutex_unlock(&state->drive_mutex);
		}

		if(abs(state->pos_x - state->last_x) > mthreshold ||
				abs(state->pos_y - state->last_y) > mthreshold){
			state->translating = 1;
		} else{
			state->translating = 0;
		}

		if(abs(state->pos_theta - state->last_theta) > rthreshold){
			state->rotating = 1;
		}else{
			state->rotating = 0;
		}

		state->moving = (state->translating | state->rotating);

	} else {
		state->odometry_seen = 1;
	}
	state->prev_left_ticks = msg->encoder_left_ticks;
	state->prev_right_ticks = msg->encoder_right_ticks;
}

double getDistFromTicks(int32_t ticks) {
	return (ticks + 0.0) / (TICKS_PER_CM + 0.0);
}
