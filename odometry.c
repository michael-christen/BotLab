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
		state->last_x     = state->pos_x;
		state->last_y     = state->last_y;
		state->last_theta = state->pos_theta;
		state->last_pos   = state->pos;
		matd_destroy(state->last_var);
		state->last_var   = state->cur_var;

		int32_t diff_left     = msg->encoder_left_ticks - state->prev_left_ticks;
		int32_t diff_right    = msg->encoder_right_ticks - state->prev_right_ticks;
		odometry_t pos_update = get_odometry_data(diff_left, diff_right);
		matd_t *var_update    = get_motor_variance(diff_left, diff_right);

		state->pos            = get_updated_od(state->last_pos,pos_update);
		state->cur_var        = get_updated_variance(
				state->last_var, state->last_pos,
				var_update, pos_update
		);
		matd_destroy(var_update);

		MATD_EL(state->var_matrix,0,0) = MATD_EL(state->cur_var,0,0);
		MATD_EL(state->var_matrix,1,1) = MATD_EL(state->cur_var,1,1);
		//msg->motor_left_actual_speed;
		state->dist      += pos_update.x;
		state->pos_x     += pos_update.x * sin(state->pos_theta);
		state->pos_y     += pos_update.x * cos(state->pos_theta);
		state->pos_theta += pos_update.theta;

		if(state->stored_mat_num < MAX_NUM_ELLIPSES && fabs(state->dist) > 10.0) {
			//printf("Adding Ellipse\n");
			state->stored_pos[state->stored_mat_num].x = state->pos_x;
			state->stored_pos[state->stored_mat_num].y = state->pos_y;
			state->stored_pos[state->stored_mat_num].theta = state->pos_theta;
			/*
			printf("x: %f, y: %f, theta: %f",
				   state->stored_pos[state->stored_mat_num].x,
				   state->stored_pos[state->stored_mat_num].y,
				   state->stored_pos[state->stored_mat_num].theta
		    );
			*/

			state->stored_matrices[state->stored_mat_num ++] =
				matd_copy(state->var_matrix);
			state->dist = 0;
		}

		double mthreshold = 1.0;
		double rthreshold = 0.1;
		/*if(state->waiting_on_pos &&
		  fabs(state->pos_x - state->goal_x) < mthreshold &&
		  fabs(state->pos_y - state->goal_y) < mthreshold){
		  pthread_mutex_lock(&state->drive_mutex);
		  pthread_cond_broadcast(&state->drive_cond);
		  pthread_mutex_unlock(&state->drive_mutex);
		  }

		  if(state->waiting_on_theta &&
		  fabs(state->pos_theta - state->goal_theta) < rthreshold){
		  pthread_mutex_lock(&state->drive_mutex);
		  pthread_cond_broadcast(&state->drive_cond);
		  pthread_mutex_unlock(&state->drive_mutex);
		  }*/

		if(fabs(state->pos_x - state->last_x) > mthreshold ||
				fabs(state->pos_y - state->last_y) > mthreshold){
			state->translating = 1;
		} else{
			state->translating = 0;
		}

		if(fabs(state->pos_theta - state->last_theta) > rthreshold){
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

odometry_t get_odometry_data(int32_t diff_left, int32_t diff_right) {
	double dL        = getDistFromTicks(diff_left);
	double dR        = getDistFromTicks(diff_right);
	odometry_t output;
	output.x     = (dL + dR)/2.0;
	output.y     = 0;
	output.theta = (dL - dR)/DIST_BETWEEN_WHEELS;
	return output;
}

matd_t * get_transform_mat(odometry_t od) {
	matd_t * output = matd_identity(3);

	MATD_EL(output,0,0) = cos(od.theta);
	MATD_EL(output,0,1) = -sin(od.theta);
	MATD_EL(output,0,2) = od.x;

	MATD_EL(output,1,0) = sin(od.theta);
	MATD_EL(output,1,1) = cos(od.theta);
	MATD_EL(output,1,2) = od.y;

	MATD_EL(output,2,0) = 0;
	MATD_EL(output,2,1) = 0;
	MATD_EL(output,2,2) = 1;
	return output;
}

odometry_t get_updated_od(odometry_t prev, odometry_t cur) {
	matd_t *prev_transform = get_transform_mat(prev);
	matd_t *cur_transform  = get_transform_mat(cur);
	matd_t *mult_result    = matd_multiply(
			prev_transform, cur_transform);
	odometry_t output;
	output.x	           = MATD_EL(mult_result,0,2);
	output.y	           = MATD_EL(mult_result,1,2);
	output.theta           = prev.theta + cur.theta;
	matd_destroy(prev_transform);
	matd_destroy(cur_transform);
	matd_destroy(mult_result);
	return output;
}

matd_t * get_jacobian(odometry_t prev, odometry_t cur) {
	matd_t *jacob      = matd_create(3,6);
	double g = 0;
	double h = 0;
	//double g = -sin(prev.theta)*cur.x - cos(prev.theta)*cur.y;
	//double h = cos(prev.theta)*cur.x  - sin(prev.theta)*cur.y;

	MATD_EL(jacob,0,0) = 1;
	MATD_EL(jacob,0,1) = 0;
	MATD_EL(jacob,0,2) = g;
	MATD_EL(jacob,0,3) = cos(prev.theta);
	MATD_EL(jacob,0,4) = -sin(prev.theta);
	MATD_EL(jacob,0,5) = 0;

	MATD_EL(jacob,1,0) = 0;
	MATD_EL(jacob,1,1) = 1;
	MATD_EL(jacob,1,2) = h;
	MATD_EL(jacob,1,3) = sin(prev.theta);
	MATD_EL(jacob,1,4) = cos(prev.theta);
	MATD_EL(jacob,1,5) = 0;

	MATD_EL(jacob,2,0) = 0;
	MATD_EL(jacob,2,1) = 0;
	MATD_EL(jacob,2,2) = 1;
	MATD_EL(jacob,2,3) = 0;
	MATD_EL(jacob,2,4) = 0;
	MATD_EL(jacob,2,5) = 1;
	return jacob;
}

matd_t * get_motor_variance(int32_t diff_left, int32_t diff_right) {
	double dL           = getDistFromTicks(diff_left);
	double dR           = getDistFromTicks(diff_right);
	double sigR         = LONG_ERR*dR;
	double sigR2        = pow(sigR,2);
	double sigL         = LONG_ERR*dL;
	double sigL2        = pow(sigL,2);
	double sigS         = LAT_ERR*(dL+dR);
	double sigS2        = pow(sigS,2);
	double b            = DIST_BETWEEN_WHEELS;
	double b2           = pow(DIST_BETWEEN_WHEELS,2);
	matd_t *output      = matd_identity(3);

	MATD_EL(output,0,0) = (sigR2 + sigL2)/4;
	MATD_EL(output,0,2) = (sigR2 - sigL2)/(2*b);

	MATD_EL(output,1,1) = sigS2;

	MATD_EL(output,2,0) = (sigR2 - sigL2)/(2*b);
	MATD_EL(output,2,2) = (sigR2 + sigL2)/b2;
	return output;
}

matd_t * get_updated_variance(
		matd_t *prev_variance, odometry_t prev_od,
		matd_t *cur_variance,  odometry_t cur_od) {
	matd_t *jacob   = get_jacobian(prev_od, cur_od);
	matd_t *jacob_a = matd_select(jacob,0,2,0,2);
	matd_t *jacob_b = matd_select(jacob,0,2,3,5);
	//sig_total = J_a*sig_a*J_a' + J_b*sig_b*J_b'
	matd_t *update  = matd_op("M*M*(M') + M*M*(M')",
			jacob_a,prev_variance,jacob_a,
			jacob_b,cur_variance,jacob_b
	);

	matd_destroy(jacob);
	matd_destroy(jacob_a);
	matd_destroy(jacob_b);
	return update;
}

double getDistFromTicks(int32_t ticks) {
	return (ticks + 0.0) / (TICKS_PER_CM + 0.0);
}
