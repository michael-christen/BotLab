#include "maebot_app.h"
#include "led.h"
#include "calibration.h"
#include "odometry.h"
#include "pid_ctrl.h"
#include "drive_ctrl.h"
#include "barrel_distortion.h"
#include "mapping.h"
#include <stdio.h>
#include <unistd.h>
#include <pthread.h>
#include <stdlib.h>
#include <lcm/lcm.h>
#include <signal.h>

#include "vx/vx.h"
#include "vx/vxo_drawables.h"
#include "vx/vx_remote_display_source.h"

#include "common/getopt.h"
#include "common/image_util.h"
#include "common/timestamp.h"
#include "imagesource/image_source.h"
#include "imagesource/image_convert.h"

#include "lcmtypes/maebot_diff_drive_t.h"
#include "lcmtypes/maebot_motor_feedback_t.h"
#include "lcmtypes/maebot_laser_t.h"
#include "lcmtypes/maebot_leds_t.h"
#include "lcmtypes/maebot_sensor_data_t.h"

#define NUM_BLINKS 3
#define STOP 0
//1-hot encoding
#define FORWARD 1
#define BACKWARD 2
#define LEFT 4
#define RIGHT 8
#define PID 16

#define LONG_SPEED 0.3
#define ROT_SPEED 0.2

#define FBDIV 100000.0
#define LRDIV 100000.0

#define MOUSE_MOVE_THRESHOLD 10

#define GYRO_OFF_2 -280.5897772	//From 3815 samples

int count;

void setLaser(state_t* state, int lsr_val){
	//lsr_val == 1 ? laser is on : laser is off
	pthread_mutex_lock(&state->lsr_mutex);
	state->lsr.laser_power = lsr_val;
	pthread_mutex_unlock(&state->lsr_mutex);
}

void fireLaser(state_t* state){
	//printf("pew pew\n");
	setLaser(state, 1);
	LEDStatus(state, LASER_ON);
	usleep(200000);

	int i;
	for(i = 1; i < NUM_BLINKS; i++){
		setLaser(state, 0);
		LEDStatus(state, NONE);
		usleep(150000);

		setLaser(state, 1);
		LEDStatus(state, LASER_ON);
		usleep(200000);
	}

	setLaser(state, 0);
	LEDStatus(state, NONE);
}

void moveBot(state_t* state){
	//double arc_val = 5;
	if(state->doing_pid_theta) {
		return;
	}
	if(((state->cmd_val & PID) | state->doing_pid) != 0) {
		//uint32_t color_detect = state->red | state->green << 8 | state->blue << 16 | 0xff << 24;
		//printf("color: %x\n",color_detect);
		//printf("thresh: %f\n",state->thresh);
		pthread_mutex_lock(&state->image_mutex);
		{
			//Wait for fresh image to arrive
			pthread_cond_wait(&state->image_cv, &state->image_mutex);
			state->num_balls = blob_detection(state->im, state->balls,
											  state->hue, 0xff039dfd, state->thresh);
			//printf("num_balls: %d\n",state->num_balls);
			if(state->num_balls == 1) {
				state->diff_x = state->im->width/2.0 - state->balls[0].x;
				printf("diff_x = %f\n",state->diff_x);
				//printf("x: %f\n", diff_x);
				state->im->buf[(int) (state->im->stride*state->balls[0].y + state->balls[0].x)] = 0xffff0000;

				state->diamond_seen  = 1;
			} else {
				state->diamond_seen = 0;
			}
		}
		pthread_mutex_unlock(&state->image_mutex);
		if(state->doing_pid) {
			double pid_out = pid_get_output( state->green_pid,state->diff_x);
			if(pid_out == 0 ) {
				if(state->diamond_seen) {
					printf("done with pid\n");
					state->num_pid_zeros ++;
					if(state->num_pid_zeros >= 5) {
						state->doing_pid = 0;
					}
				} else {
					//Give it a little kick in the pants to go
					//looking
					state->diff_x = 0.1;
				}
			}
			state->green_pid_out = pid_out;
			//printf("pid_out: %f\n",pid_out);
		}


		if(!state->diamond_seen) {
			printf("not seen\n");
		}
		double rot = pid_to_rot(state->green_pid, state->green_pid_out);
		driveRot(state, rot);
		usleep(50000);
		driveStop(state);
	} else if (state->cmd_val & FORWARD) {
		driveRad(state, -1000, LONG_SPEED);
		//printf("ARC: %f\n", state->left_offset);
	} else if(state->cmd_val & BACKWARD) {
		driveStraight(state, -LONG_SPEED);
	} else if(state->cmd_val & RIGHT) {
		state->save_gyro = state->gyro_int[2];
		state->save_theta = state->pos_theta;
		driveRot(state, -ROT_SPEED);
	} else if(state->cmd_val & LEFT) {
		state->save_gyro = state->gyro_int[2];
		state->save_theta = state->pos_theta;
		driveRot(state, ROT_SPEED);
	}  else {
		driveStop(state);
		/*if(state->rotating){
			double gyroDif = (state->gyro_int[2] - state->save_gyro) / state->gyro_ticks_per_theta;
			gyroDif = gyroDif / M_PI * 180;

			double tickDif = state->pos_theta - state->save_theta;
			tickDif = tickDif / M_PI * 180;

			printf("Rotation stopped...\n");
			printf("Change in gyro theta: %g\n", gyroDif);
			printf("Change in tick theta: %g\n", tickDif);
		}*/
	}
}

static int touch_event (vx_event_handler_t * vh, vx_layer_t * vl, vx_camera_pos_t * pos, vx_touch_event_t * mouse)
{
	return 0;
}
static int mouse_event (vx_event_handler_t * vh, vx_layer_t * vl, vx_camera_pos_t * pos, vx_mouse_event_t * mouse)
{
	state_t * state = vh->impl;
	vx_mouse_event_t last_mouse;
	if (state->init_last_mouse) {
		memcpy(&last_mouse, &state->last_mouse, sizeof(vx_mouse_event_t));
		memcpy(&state->last_mouse, mouse, sizeof(vx_mouse_event_t));
	} else {
		memcpy(&state->last_mouse, mouse, sizeof(vx_mouse_event_t));
		state->init_last_mouse = 1;
		return 0;
	}

	int diff_button = mouse->button_mask ^ last_mouse.button_mask;
	int button_down = diff_button & (!last_mouse.button_mask);
	int button_up = diff_button & last_mouse.button_mask;
	if (button_down) {
		state->mouseDownX = mouse->x;
		state->mouseDownY = mouse->y;
	} else if (button_up) {
		int mouseMoveDist = sqrt(pow(state->mouseDownX - mouse->x, 2) + pow(state->mouseDownY - mouse->y, 2));
		if (mouseMoveDist < MOUSE_MOVE_THRESHOLD) {
			double man_point[3];
			vx_ray3_t ray;
			vx_camera_pos_compute_ray(pos, mouse->x, mouse->y, &ray);
			vx_ray3_intersect_xy(&ray, 0, man_point);
			// Add state machine flag here
			state->goalMouseX = man_point[0] / CM_TO_VX;
			state->goalMouseY = man_point[1] / CM_TO_VX;
			printf("Clicked: %f cm's x, %f cm's y\n",
					state->goalMouseX, state->goalMouseY);
			pthread_mutex_lock(&state->haz_map_mutex);
			if (state->targetPathValid == 1) {
				path_destroy(state->targetPath);
			}
			state->targetPath = haz_map_get_path(&state->hazMap,
					state->goalMouseX, state->goalMouseY);
			if (state->targetPath->length > 0) {
				state->targetPathValid = 1;
			} else {
				free(state->targetPath);
				state->targetPathValid = 0;
			}
			pthread_mutex_unlock(&state->haz_map_mutex);
			position_t pos;
			pos.x = state->goalMouseX;
			pos.y = state->goalMouseY;
			if(state->goToMouseCoords) {
				driveToPosition(state,pos);
			}
		}
	}


	return 0;
}

static int key_event (vx_event_handler_t * vh, vx_layer_t * vl, vx_key_event_t * key)
{
	state_t *state = vh->impl;

	state->cmd_val = 0;
	if (!key->released) {
		// forward
		if(key->key_code == 'p' || state->doing_pid) {
			if(!state->doing_pid) {
				state->num_pid_zeros = 0;
				pid_update_goal(state->green_pid, 0);
			}
			state->cmd_val = PID;
			state->doing_pid = 1;
			if(key->key_code == 'o') {
				state->doing_pid = 0;
				state->cmd_val |= ~PID;
			}
		} else if (key->key_code == '1') {
			state->goToMouseCoords = !state->goToMouseCoords;
			printf("%d going to Mouse coords\n",state->goToMouseCoords);
		}else if (key->key_code == 'w' || key->key_code == 'W') {
			state->cmd_val |= FORWARD;
		} else if (key->key_code == 'a' || key->key_code == 'A' ) {
			state->cmd_val |= LEFT;
		} else if (key->key_code == 's' || key->key_code == 'S') {
			state->cmd_val |= BACKWARD;
		} else if (key->key_code == 'd' || key->key_code == 'D') {
			state->cmd_val |= RIGHT;
		} else if(key->key_code == 'l' || key->key_code == 'L') {
			// fire laser
			fireLaser(state);
		} else if(key->key_code == '0') {
			printf("%f, \n",state->dist);
			state->dist = 0;
		}/* else if(key->key_code == 'r') {
			state->red ++;
		} else if(key->key_code == 'g') {
			state->green ++;
		} else if(key->key_code == 'b') {
			state->blue ++;
		} else if(key->key_code == 't') {
			state->red --;
		} else if(key->key_code == 'h') {
			state->green --;
		} else if(key->key_code == 'n') {
			state->blue --;
		} else if(key->key_code == 'y') {
			state->thresh ++;
		} else if(key->key_code == 'u') {
			state->thresh --;
		} else if(key->key_code == 'z') {
			state->hue ++;
		} else if(key->key_code == 'x') {
			state->hue --;
		}*/
		/*   <  >
		   P v  g
		   I b  b
		   D n  j
		 */
		else if(key->key_code == 'g'){
			pid_update_pid(state->theta_pid, state->theta_pid->P + 0.1, state->theta_pid->I, state->theta_pid->D);
			printf("P updated to %g\n", state->theta_pid->P);
			/*state->gyro_ticks_per_theta += 500;
			printf("Gyro ticks per theta updated to %f\n", state->gyro_ticks_per_theta);*/
		}else if(key->key_code == 'v'){
			pid_update_pid(state->theta_pid, state->theta_pid->P - 0.1, state->theta_pid->I, state->theta_pid->D);
			printf("P updated to %g\n", state->theta_pid->P);
			/*state->gyro_ticks_per_theta -= 500;
			printf("Gyro ticks per theta updated to %f\n", state->gyro_ticks_per_theta);*/
		}else if(key->key_code == 'h'){
			pid_update_pid(state->theta_pid, state->theta_pid->P, state->theta_pid->I + 0.1, state->theta_pid->D);
			printf("I updated to %g\n", state->theta_pid->I);
		}else if(key->key_code == 'b'){
			pid_update_pid(state->theta_pid, state->theta_pid->P, state->theta_pid->I - 0.1, state->theta_pid->D);
			printf("I updated to %g\n", state->theta_pid->I);
		}else if(key->key_code == 'j'){
			pid_update_pid(state->theta_pid, state->theta_pid->P, state->theta_pid->I, state->theta_pid->D + 0.1);
			printf("D updated to %g\n", state->theta_pid->D);
		}else if(key->key_code == 'n'){
			pid_update_pid(state->theta_pid, state->theta_pid->P, state->theta_pid->I, state->theta_pid->D - 0.1);
			printf("D updated to %g\n", state->theta_pid->D);
		}else if(key->key_code == 'm') {
			//left, some inconsistency
			state->doing_pid_theta = 1;
			rotateTheta(state, -M_PI/2.0);
			state->doing_pid_theta = 0;
		}else if(key->key_code == 'k') {
			//right
			state->doing_pid_theta = 1;
			rotateTheta(state, M_PI/2.0);
			state->doing_pid_theta = 0;
		}else if(key->key_code == 'c') {
			if(!state->calibrate && !state->calibrating){
				state->calibrate = 1;
			}else if(state->calibrating){
				state->calibrating = 0;
			}
		} else if(key->key_code ==';') {
			state->left_offset += 5;
		} else if(key->key_code =='\'') {
			state->left_offset -= 5;
		}

		state->red &= 0xff;
		state->green &= 0xff;
		state->blue &= 0xff;
	}
	if (state->cmd_val & FORWARD) {
		LEDStatus(state, MOVE_FORWARD);
	} else if(state->cmd_val & BACKWARD) {
		LEDStatus(state, MOVE_BACKWARD);
	} else if(state->cmd_val & RIGHT) {
		LEDStatus(state, TURN_RIGHT);
	} else if(state->cmd_val & LEFT) {
		LEDStatus(state, TURN_LEFT);
	} else {
		LEDStatus(state, NONE);
	}
	return 0;
}

static void nodestroy (vx_event_handler_t * vh)
{
	// do nothing, since this event handler is statically allocated.
}

static state_t * global_state;
static void handler(int signum)
{
	switch (signum)
	{
		case SIGINT:
		case SIGQUIT:
			pthread_mutex_lock(&global_state->running_mutex);
			//printf("setting running to 0\n");
			global_state->running = 0;
			pthread_mutex_unlock(&global_state->running_mutex);
			break;
		default:
			break;
	}
}

static void * motionFxn(void * data){

	state_t * state = data;
	while (state->running) {
		moveBot(state);
		usleep(20000);
	}
	return NULL;
}

static void * send_cmds(void * data)
{
	state_t * state = data;

	while (state->running) {

		pthread_mutex_lock(&state->lcm_mutex);
		pthread_mutex_lock(&state->cmd_mutex);
		{
			//state->cmd.timestamp = utime_now();
			maebot_diff_drive_t_publish(state->lcm,  "MAEBOT_DIFF_DRIVE", &(state->cmd));
		}
		pthread_mutex_unlock(&state->cmd_mutex);
		pthread_mutex_unlock(&state->lcm_mutex);

		state->motor_count = (state->motor_count+1)%20;
		usleep(50000); // send at 20 hz
	}
	return NULL;
}

static void * send_lsr(void * data){
	state_t * state = data;

	while(state->running) {
		pthread_mutex_lock(&state->lcm_mutex);
		pthread_mutex_lock(&state->lsr_mutex);
		{
			maebot_laser_t_publish(state->lcm, "MAEBOT_LASER", &(state->lsr));
		}
		pthread_mutex_unlock(&state->lsr_mutex);
		pthread_mutex_unlock(&state->lcm_mutex);

		usleep(50000);
	}

	return NULL;
}

static void * send_led(void * data){
	state_t * state = data;

	while(state->running){
		pthread_mutex_lock(&state->lcm_mutex);
		pthread_mutex_lock(&state->led_mutex);
		{
			maebot_leds_t_publish(state->lcm, "MAEBOT_LEDS", &state->led);
		}
		pthread_mutex_unlock(&state->led_mutex);
		pthread_mutex_unlock(&state->lcm_mutex);

		usleep(50000);
	}

	return NULL;
}

void * camera_analyze(void * data)
{
	state_t * state = data;

	zarray_t *urls = image_source_enumerate();

	//printf("Cameras:\n");
	for (int i = 0; i < zarray_size(urls); i++) {
		char *url;
		zarray_get(urls, i, &url);
		//printf("  %3d: %s\n", i, url);
	}

	if (zarray_size(urls) == 0) {
		image_source_enumerate_free(urls);
		//printf("No cameras found.\n");
		return 0;
	}
	zarray_get(urls, 0, &state->url);
	//image_source_enumerate_free(urls);
	if (!state->getopt_options.autoCamera) {
		state->url =
			"dc1394://b09d01008e366c?fidx=0&white-balance-manual=1&white-balance-red=400&white-balance-blue=714";
	}

	state->isrc = image_source_open(state->url);
	if (state->isrc == NULL) {
		//printf("Unable to open device %s\n", state->url);
		return 0;
	}

	image_source_t *isrc = state->isrc;

	if (isrc->start(isrc)) {
		//printf("Can't start image source\n");
		return 0;
	}

	image_source_format_t isrc_format;
	state->isrc->get_format(state->isrc, 0, &isrc_format);
	state->lookupTable = getLookupTable(isrc_format.width, isrc_format.height);

	state->isrcReady = 1;
	image_source_data_t isdata;
	int res;
	state->imageValid = 0;

	while (state->running) {
		pthread_mutex_lock(&state->image_mutex);
		//Let PID know that I am here
		pthread_cond_signal(&state->image_cv);
		double bruce_theta = -state->pos_theta;
		double bruce_x = state->pos_x;
		double bruce_y = state->pos_y;
		res = isrc->get_frame(isrc, &isdata);
		if (!res) {
			if (state->imageValid == 1) {
				image_u32_destroy(state->im);
			}
			state->im = image_convert_u32(&isdata);
			state->imageValid = 1;
		}

		isrc->release_frame(isrc, &isdata);

		if (state->getopt_options.verbose) {
			//printf("Got frame %p\n", state->im);
		}
		if (state->imageValid == 1) {
			// HOMOGRAPHY BEFORE BARREL DISTORTION CORRECTION GOES HERE
			correctDistortion(state->im, state->lookupTable);
			//Blue
			state->num_pts_tape =
				line_detection(state->im, state->tape);
			//printf("Pts: %d\n",state->num_pts_tape);
			//might wanna make diff d.s.
			//Also, gonna need to copy image
			//Green
			if(!state->doing_pid) {
				state->num_balls = blob_detection(state->im, state->balls, state->hue, 0xff039dfd, state->thresh);
			}
			pthread_mutex_lock(&state->haz_map_mutex);
			int obstacle = 1;
			haz_map_translate(&state->hazMap, bruce_x, bruce_y);
			find_point_pos(state, bruce_theta, bruce_x, bruce_y, obstacle);

			pthread_mutex_unlock(&state->haz_map_mutex);

			//find_point_pos( state, obstacle);
		} else {
			//printf("shouldn't get heree!!!\n");
		}
		pthread_mutex_unlock(&state->image_mutex);
		usleep(10000);
	}

	if (state->imageValid == 1) {
		//printf("Final image destroy\n");
		pthread_mutex_lock(&state->image_mutex);
		image_u32_destroy(state->im);
		state->imageValid = 0;
		pthread_mutex_unlock(&state->image_mutex);
	}

	if (!state->getopt_options.no_video) {
		state->isrc->close(state->isrc);
	}
	return NULL;
}

/*
   static void * driver_monitor(void *data) {
//int systemTry = system("./maebot_driver");
//if (systemTry) {} //ignore status

return NULL;
}
*/

void sensor_handler (const lcm_recv_buf_t *rbuf, const char * channel,
					 const maebot_sensor_data_t * msg, void * data){

	state_t* state = data;

	for(int i = 0; i < 3; i++){
		state->acc[i] = msg->accel[i];
		state->gyro[i] = msg->gyro[i];
		state->gyro_int[i] += state->gyro[i];// state->gyro_bias[i]; //msg->gyro_int[i] -state->gyro_int_offset[i] - state->gyro_bias[i];
	}
	//printf("Gryo2: %g, %lld, ", state->gyro[2], state->gyro_int[2]);
	state->gyro_int[2] -= GYRO_OFF_2;
	//printf("%lld\n,", state->gyro_int[2]);

	//state->gyro[2] -=
	/*printf ("  gyro values = (%d, %d, %d)\n",
	  msg->gyro[0], msg->gyro[1], msg->gyro[2]);
	  printf ("  gyro_int values= (%lld, %lld, %lld)\n",
	  msg->gyro_int[0], msg->gyro_int[1], msg->gyro_int[2]);*/

	//printf("%d, %lld\n",msg->gyro[2], msg->gyro_int[2]);
	//printf("%d, %lld, %lld, %g\n", state->gyro[2], state->gyro_int[2], state->gyro_int_offset[2], state->gyro_bias[2]);
}

void* lcm_handle_loop(void *data) {
	state_t *state = data;

	maebot_sensor_data_t_subscription_t * sensor_sub =
		maebot_sensor_data_t_subscribe(
									   state->lcm, "MAEBOT_SENSOR_DATA",
									   &sensor_handler, state
									  ); //subscribe to gyro/accelerometer data

	maebot_motor_feedback_t_subscription_t * odometry_sub =
		maebot_motor_feedback_t_subscribe(state->lcm,
										  "MAEBOT_MOTOR_FEEDBACK",
										  &odometry_handler, state); //subscribe to odometry data

	//int hz = 15;
	while (state->running) {
		// Set up the LCM file descriptor for waiting. This lets us monitor it
		// until somethign is "ready" to happen. In this case, we are ready to
		// receive a message.
		lcm_handle(state->lcm);
		/*
		   int lcm_fd = lcm_get_fileno(state->lcm);
		   fd_set fds;
		   FD_ZERO(&fds);
		   FD_SET(lcm_fd, &fds);

		// Handle message if appropriate
		struct timeval timeout = {
		0,              // Seconds
		1000000/hz      // Microseconds
		};

		int status = select(lcm_fd + 1, &fds, 0, 0, &timeout);

		if (0 == status) {
		continue;
		} else {
		// LCM has events ready to be processed
		lcm_handle(state->lcm);
		}
		*/
	}

	//clean up
	maebot_sensor_data_t_unsubscribe(state->lcm, sensor_sub);
	maebot_motor_feedback_t_unsubscribe(state->lcm, odometry_sub);

	return NULL;
}

void* FSM(void* data){
	state_t* state = data;
	explorer_t explorer;
	explorer_state_t curState, nextState;
	curState = EX_START;
	nextState = curState;
	clock_t start_time = clock();
	while(state->running){
		switch(curState){
			case EX_MOVE_FORWARD:{
				path_t* path = explorer_get_move(&explorer);
				while(path->position != path->length){
					position_t waypoint = path->waypoints[path->position];
					driveToPosition(state, waypoint);
					path->position++;
				}
				path_destroy(path);
				nextState = EX_ANALYZE;
				break;}
			case EX_TURN_LEFT:{
				rotateTheta(state, M_PI/2.0);
				nextState = EX_ANALYZE;
				break;}
			case EX_TURN_RIGHT:{
				rotateTheta(state, -M_PI/2.0);
				nextState = EX_ANALYZE;
				break;}
			case EX_ZAP_DIAMOND:{
				//Still need to get diamond coords
				double diamond_x = 0, diamond_y = 0;
				double dx = diamond_x - state->pos_x;
				double dy = diamond_y - state->pos_y;
				double dtheta = atan2(dy, dx);
				double originalTheta = state->pos_theta;
				//rotate toward diamond
				driveToTheta(state, dtheta);

				//shoot diamond
				fireLaser(state);
				//update diamond to zapped

				driveToTheta(state, originalTheta);
				nextState = EX_ANALYZE;
				break;}
			case EX_GOHOME:{
				break;}
			case EX_EXIT:{
				rotateTheta(state, -2*M_PI + 0.001);
				rotateTheta(state, 2*M_PI - 0.001);
				return NULL;
				break;}
			case EX_START:rotateTheta(state, 2*M_PI - 0.001);
			case EX_ANALYZE:{
				clock_t curTime = clock();
				state->fsm_time_elapsed =
					(double)(curTime - start_time) / CLOCKS_PER_SEC;
				if(state->fsm_time_elapsed >= 180){
					nextState = EX_EXIT;
					break;
				}
			}
			default: nextState = explorer_run(&explorer, &state->hazMap, state->pos_x, state->pos_y, state->pos_theta);
		}
		curState = nextState;
	}
	return NULL;
}

void* position_tracker(void *data) {
    state_t *state = data;
    int i;

    state->pathTaken = malloc(sizeof(path_t));
    path_t *path = state->pathTaken;
    path->length = MAX_POS_SAMPLES;
    path->waypoints = malloc(sizeof(position_t) * path->length);
    for (i = 0; i < MAX_POS_SAMPLES; i++) {
        path->waypoints[i].x = 0;
        path->waypoints[i].y = 0;
    }

    state->pathTakenValid = 1;


    while (state->running) {
        for (i = 1; i < path->length; i++) {
            path->waypoints[i - 1].x = path->waypoints[i].x;
            path->waypoints[i - 1].y = path->waypoints[i].y;
        }
        path->waypoints[path->length - 1].x = state->pos_x;
        path->waypoints[path->length - 1].y = state->pos_y;

//	printf("call world map set x: %f y: %f \n", state->pos_x, state->pos_y);

		world_map_set(&state->world_map, state->pos_x, state->pos_y, WORLD_MAP_SEEN);
	//		state->pos_x += 1;
	//		state->pos_y += 2;



        usleep(POS_SAMPLES_INTERVAL);
    }

    return NULL;
}

void * calibrator(void* data){
	state_t *state = data;

	while(state->running){
		if(state->calibrate){
			state->calibrate = 0;
			LEDStatus(state, CALIBRATE_GYRO);
			calibrate_gyros(&state->gyro_int[2], &state->calibrating, &state->gyro_ticks_per_theta);
			LEDStatus(state, NONE);
		}else{
			sleep(.05);
		}
	}
	return NULL;
}

int main(int argc, char ** argv)
{
	vx_global_init();

	state_t * state = calloc(1, sizeof(state_t));
	global_state = state;
	state->gopt = getopt_create();
	state->app.display_finished = display_finished;
	state->app.display_started = display_started;
	state->app.impl = state;
	state->veh.dispatch_order = -10;
	state->veh.touch_event = touch_event;
	state->veh.mouse_event = mouse_event;
	state->veh.key_event = key_event;
	state->veh.destroy = nodestroy;
	state->veh.impl  = state;
	state->pos.x     = 0;
	state->pos.y     = 0;
	state->pos.theta = 0;
	state->pos_x     = 0;
	state->pos_y     = 0;
	state->pos_z     = 0;
	state->pos_theta = 0;
    state->pathTakenValid = 0;
    state->targetPathValid = 0;
	state->odometry_seen = 0;
	state->goToMouseCoords = 0;
	state->gyro_ticks_per_theta = -145000.0;	//obtained through testing

	//Initialize to identity so, can multiply
	state->var_matrix    = matd_identity(2);
	state->stored_matrices = malloc(sizeof(matd_t*)*MAX_NUM_ELLIPSES);
	state->stored_pos    = malloc(sizeof(odometry_t)*MAX_NUM_ELLIPSES);
	state->stored_mat_num = 0;

	state->cur_var       = matd_create(3,3);
	state->last_var      = matd_create(3,3);

	state->alpha = 0.05;
	state->beta  = 0.02;

	state->init_last_mouse = 0;
	state->red = 0x3a;
	state->green = 0x76;
	state->blue = 0x41;
	state->thresh = 20.0;
	state->hue    = 145.0;
	state->green_pid = malloc(sizeof(pid_ctrl_t));
	state->green_pid_out = 0;
	state->theta_pid = malloc(sizeof(pid_ctrl_t));
	state->isrcReady = 0;
	state->im = NULL;
	state->cmd_val = 0;
	state->motor_count = 0;
	state->diff_x        = 0;
	state->dist          = 0;
	state->diamond_seen  = 0;
	state->num_pid_zeros = 0;
	state->doing_pid     = 0;
	state->doing_pid_theta     = 0;
	pid_init(state->green_pid, 1.0, 0, 0, 0, 16, 100);
	//pid_init(state->theta_pid, 2.0, 0.3, 3.5, 0, .1, 2*M_PI);
	pid_init(state->theta_pid, 0.5, 0.2, 0.4, 0, .1, M_PI);

	haz_map_init(&state->hazMap, HAZ_MAP_MAX_WIDTH, HAZ_MAP_MAX_HEIGHT);
	//haz_map_set(&state->hazMap, HAZ_MAP_MAX_WIDTH/2 + 10, HAZ_MAP_MAX_HEIGHT/2 + 10, HAZ_MAP_OBSTACLE);
	//haz_map_compute_config(&state->hazMap);
	/*for (i = 0; i < 10; i++) {
		haz_map_set(&state->hazMap, HAZ_MAP_MAX_WIDTH/2 + 2, HAZ_MAP_MAX_HEIGHT/2 + i, HAZ_MAP_OBSTACLE);
		haz_map_set(&state->hazMap, HAZ_MAP_MAX_WIDTH/2 + 2, HAZ_MAP_MAX_HEIGHT/2 + 10 + i, HAZ_MAP_OBSTACLE);
		haz_map_set(&state->hazMap, HAZ_MAP_MAX_WIDTH/2 + 2 + i, HAZ_MAP_MAX_HEIGHT/2 + i, HAZ_MAP_OBSTACLE);
	}*/


	world_map_init(&state->world_map, WORLD_MAP_MAX_WIDTH, WORLD_MAP_MAX_HEIGHT);


	//state->targetPath = haz_map_get_path(&state->hazMap, 40, 40);
	//state->targetPathValid = 1;


	//Should be width
	state->tape = calloc(1000, sizeof(pixel_t));
	state->num_pts_tape = 0;

	state->running = 1;

	lcm_t * lcm = lcm_create (NULL);
	state->lcm = lcm;
	state->sensor_channel = "MAEBOT_SENSOR";
	state->odometry_channel = "MAEBOT_ODOMETRY";
	state->displayStarted = state->displayFinished = 0;

	state->left_offset = 50;

	pthread_mutex_init(&state->layer_mutex, NULL);
	pthread_mutex_init(&state->cmd_mutex, NULL);
	pthread_mutex_init(&state->lsr_mutex, NULL);
	pthread_mutex_init(&state->lcm_mutex, NULL);
	pthread_mutex_init(&state->running_mutex, NULL);
	pthread_mutex_init(&state->image_mutex, NULL);
	pthread_mutex_init(&state->haz_map_mutex, NULL);
	pthread_mutex_init(&state->world_map_mutex, NULL);

	state->layer_map = zhash_create(sizeof(vx_display_t*), sizeof(vx_layer_t*), zhash_uint64_hash, zhash_uint64_equals);

	signal(SIGINT, handler);

	getopt_add_bool(state->gopt, 'h', "help", 0, "Show this help");
	getopt_add_bool(state->gopt, 'v', "verbose", 0, "Show extra debugging output");
	getopt_add_bool(state->gopt, 'c', "auto-camera", 0, "Automatically detect which camera to use");
	getopt_add_bool(state->gopt, '\0', "no-video", 0, "Disable video");
	getopt_add_int (state->gopt, 'l', "limitKBs", "-1", "Remote display bandwidth limit. < 0: unlimited.");
	getopt_add_double (state->gopt, 'd', "decimate", "0", "Decimate image by this amount before showing in vx");

	if (!getopt_parse(state->gopt, argc, argv, 0) ||
		getopt_get_bool(state->gopt,"help")) {
		getopt_do_usage(state->gopt);
		exit(-1);
	}

	state->getopt_options.verbose = getopt_get_bool(state->gopt, "verbose");
	state->getopt_options.autoCamera = getopt_get_bool(state->gopt, "auto-camera");
	state->getopt_options.no_video = getopt_get_bool(state->gopt, "no-video");
	state->getopt_options.limitKBs = getopt_get_int(state->gopt, "limitKBs");
	state->getopt_options.decimate = pow(2, getopt_get_double(state->gopt, "decimate"));

	//pthread_create(&state->dmon_thread, NULL, driver_monitor, state);
	pthread_create(&state->camera_thread, NULL, camera_analyze, state);
	pthread_create(&state->cmd_thread,  NULL, send_cmds, state);
	pthread_create(&state->lsr_thread,  NULL, send_lsr, state);
	//pthread_create(&state->led_thread,  NULL, send_led, state);
	pthread_create(&state->gui_thread,  NULL, gui_create, state);
	pthread_create(&state->lcm_handle_thread, NULL, lcm_handle_loop, state);
	//pthread_create(&state->fsm_thread, NULL, FSM, state);
	pthread_create(&state->position_tracker_thread, NULL, position_tracker, state);
	pthread_create(&state->motion_thread,  NULL, motionFxn, state);


	pthread_create(&state->calibrator_thread, NULL, calibrator, state);



	/*	find_H_matrix(state);
		int obstacle = 0, x_px = 156, y_px = 352;
		for(x_px; x_px < 525; x_px++){
		find_point_pos( state, x_px, y_px, &state->hazMap, obstacle);
		} */

	//pthread_join(state->camera_thread, NULL);

	if (pthread_join(state->gui_thread, NULL) != 0) {
		//printf("Problem here!\n");
	} else {
		//printf("All good!\n");
	}

	// clean up
	destroyLookupTable(state->lookupTable);
    haz_map_destroy(&state->hazMap);
    if (state->pathTaken == 1) {
        path_destroy(state->pathTaken);
    }
    if (state->targetPathValid == 1) {
        path_destroy(state->targetPath);
    }
    printf("Exited Cleanly!\n");
    //maebot_sensor_data_t_unsubscribe(lcm, sensor_sub);
    //maebot_sensor_data_t_unsubscribe(lcm, odometry_sub);
    //system("kill `pgrep -f './maebot_driver'`");

    return 0;
}
