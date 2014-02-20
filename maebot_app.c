#include "maebot_app.h"
#include "led.h"
#include "calibration.h"
#include "odometry.h"
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
#define FORWARD 0
#define BACKWARD 1
#define LEFT 2
#define RIGHT 3
#define STOP 4

#define LONG_SPEED 0.5
#define ROT_SPEED 0.2

#define FBDIV 100000.0
#define LRDIV 100000.0

void setLaser(state_t* state, int lsr_val){
//lsr_val == 1 ? laser is on : laser is off
	pthread_mutex_lock(&state->lsr_mutex);
	state->lsr.laser_power = lsr_val;
	pthread_mutex_unlock(&state->lsr_mutex);
}

void fireLaser(state_t* state){
	printf("pew pew\n");
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

void moveBot(state_t* state, int cmd_val){
	pthread_mutex_lock(&state->cmd_mutex);
	printf("max_rev: %d, max_fwd: %d\n",MAX_REVERSE_SPEED,MAX_FORWARD_SPEED);
	switch(cmd_val){
		case FORWARD:
			state->cmd.motor_left_speed = LONG_SPEED;
			state->cmd.motor_right_speed = LONG_SPEED;
			break;
		case BACKWARD:
			state->cmd.motor_left_speed = -LONG_SPEED;
			state->cmd.motor_right_speed = -LONG_SPEED;
			break;
		case LEFT:
			state->cmd.motor_left_speed = -ROT_SPEED;
			state->cmd.motor_right_speed = ROT_SPEED;
			break;
		case RIGHT:
			state->cmd.motor_left_speed = ROT_SPEED;
			state->cmd.motor_right_speed = -ROT_SPEED;
			break;
		default:
			state->cmd.motor_left_speed = 0;
			state->cmd.motor_right_speed = 0;
	}
	pthread_mutex_unlock(&state->cmd_mutex);
}

static int touch_event (vx_event_handler_t * vh, vx_layer_t * vl, vx_camera_pos_t * pos, vx_touch_event_t * mouse)
{
    return 0;
}
static int mouse_event (vx_event_handler_t * vh, vx_layer_t * vl, vx_camera_pos_t * pos, vx_mouse_event_t * mouse)
{
    return 0;
}

static int key_event (vx_event_handler_t * vh, vx_layer_t * vl, vx_key_event_t * key)
{
    state_t *state = vh->impl;

    if (!key->released) {
        if (key->key_code == 'w' || key->key_code == 'W') {
            // forward
			moveBot(state, FORWARD);
			LEDStatus(state, MOVE_FORWARD);
        } else if (key->key_code == 'a' || key->key_code == 'A' ) {
            // turn left
			moveBot(state, LEFT);
			LEDStatus(state, TURN_LEFT);
        } else if (key->key_code == 's' || key->key_code == 'S') {
            // reverse
			moveBot(state, BACKWARD);
			LEDStatus(state, MOVE_BACKWARD);
        } else if (key->key_code == 'd' || key->key_code == 'D') {
            // turn right
			moveBot(state, RIGHT);
			LEDStatus(state, TURN_RIGHT);
        } else if(key->key_code == 'l' || key->key_code == 'L') {
			// fire laser
			fireLaser(state);
		}
    } else {
        // when key released, speeds default to 0
		moveBot(state, STOP);
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
            global_state->running = 0;
            break;
        default:
            break;
    }
}

static void * send_cmds(void * data)
{
    state_t * state = data;

    while (state->running) {

        pthread_mutex_lock(&state->cmd_mutex);
        {
            //state->cmd.timestamp = utime_now();
            maebot_diff_drive_t_publish(state->lcm,  "MAEBOT_DIFF_DRIVE", &state->cmd);
        }
        pthread_mutex_unlock(&state->cmd_mutex);

        usleep(50000); // send at 20 hz
    }
    return NULL;
}

static void * send_lsr(void * data){
	state_t * state = data;

	while(state->running) {
		pthread_mutex_lock(&state->lsr_mutex);
		{
			maebot_laser_t_publish(state->lcm, "MAEBOT_LASER", &state->lsr);
		}
		pthread_mutex_unlock(&state->lsr_mutex);

		usleep(50000);
	}

	return NULL;
}

static void * send_led(void * data){
	state_t * state = data;

	while(state->running){
		pthread_mutex_lock(&state->led_mutex);
		{
			maebot_leds_t_publish(state->lcm, "MAEBOT_LEDS", &state->led);
		}
		pthread_mutex_unlock(&state->led_mutex);

		usleep(50000);
	}

	return NULL;
}

static void * driver_monitor(void *data) {
    int systemTry = system("bash driver_monitor.sh");
    if (systemTry) {} //ignore status

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
    state->veh.impl = state;
    state->pos_x    = 0;
    state->pos_y    = 0;
    state->pos_theta= 0;

    state->running = 1;

    lcm_t * lcm = lcm_create (NULL);
    state->lcm = lcm;
    state->sensor_channel = "MAEBOT_SENSOR"; 
    state->odometry_channel = "MAEBOT_ODOMETRY"; 

    state->vw = vx_world_create();
    state->displayStarted = state->displayFinished = 0;


    pthread_mutex_init(&state->layer_mutex, NULL);
    pthread_mutex_init(&state->cmd_mutex, NULL);
    pthread_mutex_init(&state->lsr_mutex, NULL);

    maebot_sensor_data_t_subscription_t * sensor_sub = 
	maebot_sensor_data_t_subscribe(
		state->lcm, "MAEBOT_SENSOR_DATA", 
		&sensor_handler, state
	); //subscribe to gyro/accelerometer data
    maebot_sensor_data_t_subscription_t * odometry_sub =
	maebot_motor_feedback_t_subscribe(state->lcm,
		"MAEBOT_MOTOR_FEEDBACK", 
		&odometry_handler, state); //subscribe to odometry data

    state->layer_map = zhash_create(sizeof(vx_display_t*), sizeof(vx_layer_t*), zhash_ptr_hash, zhash_ptr_equals);

    signal(SIGINT, handler);

    getopt_add_bool(state->gopt, 'h', "help", 0, "Show this help");
    getopt_add_bool(state->gopt, 'v', "verbose", 0, "Show extra debugging output");
    getopt_add_bool(state->gopt, '\0', "no-video", 0, "Disable video");
    getopt_add_int (state->gopt, 'l', "limitKBs", "-1", "Remote display bandwidth limit. < 0: unlimited.");
    getopt_add_double (state->gopt, 'd', "decimate", "1", "Decimate image by this amount before showing in vx");

    if (!getopt_parse(state->gopt, argc, argv, 0) ||
        getopt_get_bool(state->gopt,"help")) {
        getopt_do_usage(state->gopt);
        exit(-1);
    }

    state->getopt_options.verbose = getopt_get_bool(state->gopt, "verbose");
    state->getopt_options.no_video = getopt_get_bool(state->gopt, "no-video");
    state->getopt_options.limitKBs = getopt_get_int(state->gopt, "limitKBs");
    state->getopt_options.decimate = getopt_get_double(state->gopt, "decimate");

    pthread_create(&state->dmon_thread, NULL, driver_monitor, state);
    pthread_create(&state->cmd_thread,  NULL, send_cmds, state);
    pthread_create(&state->lsr_thread,  NULL, send_lsr, state);
    pthread_create(&state->led_thread,  NULL, send_led, state);
    pthread_create(&state->gui_thread,  NULL, gui_create, state);

    pthread_join(state->gui_thread, NULL);

    //clean up
    //maebot_sensor_data_t_unsubscribe(lcm, sensor_sub); 
    //maebot_sensor_data_t_unsubscribe(lcm, odometry_sub);
    return 0;
}
