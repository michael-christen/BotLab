#include "maebot_app.h"
#include "led.h"
#include "calibration.h"
#include "odometry.h"
#include "pid_ctrl.h"
#include "drive_ctrl.h"
#include "barrel_distortion.h"
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

#define LONG_SPEED 0.5
#define ROT_SPEED 0.2

#define FBDIV 100000.0
#define LRDIV 100000.0

#define MOUSE_MOVE_THRESHOLD 10

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
    double arc_val = 5;
    if (cmd_val & FORWARD) {
	if(cmd_val & RIGHT) {
	    driveRad(state, -arc_val, LONG_SPEED);
	} else if(cmd_val & LEFT) {
	    driveRad(state, arc_val, LONG_SPEED);
	} else {
	    driveRad(state, -1000, LONG_SPEED);
	}
    } else if(cmd_val & BACKWARD) {
	if(cmd_val & RIGHT) {
	    driveRad(state, -arc_val, -LONG_SPEED);
	} else if(cmd_val & LEFT) {
	    driveRad(state, arc_val, -LONG_SPEED);
	} else {
	    driveStraight(state, -LONG_SPEED);
	}
    } else if(cmd_val & RIGHT) {
	driveRot(state, -ROT_SPEED);
    } else if(cmd_val & LEFT) {
	driveRot(state, ROT_SPEED);
    } else {
	driveStop(state);
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
            state->goalMouseX = man_point[0];
            state->goalMouseY = man_point[1];
        }
    }


    return 0;
}

static int key_event (vx_event_handler_t * vh, vx_layer_t * vl, vx_key_event_t * key)
{
    state_t *state = vh->impl;

    int cmd_val = 0;
    if (!key->released) {
	// forward
        if (key->key_code == 'w' || key->key_code == 'W') {
	    cmd_val |= FORWARD;
        } else if (key->key_code == 'a' || key->key_code == 'A' ) {
	    cmd_val |= LEFT;
        } else if (key->key_code == 's' || key->key_code == 'S') {
	    cmd_val |= BACKWARD;
        } else if (key->key_code == 'd' || key->key_code == 'D') {
	    cmd_val |= RIGHT;
	} else if(key->key_code == 'l' || key->key_code == 'L') {
	    // fire laser
	    fireLaser(state);
	}
    }
    moveBot(state, cmd_val);
    if (cmd_val & FORWARD) {
	LEDStatus(state, MOVE_FORWARD);
    } else if(cmd_val & BACKWARD) {
	LEDStatus(state, MOVE_BACKWARD);
    } else if(cmd_val & RIGHT) {
	LEDStatus(state, TURN_RIGHT);
    } else if(cmd_val & LEFT) {
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

        pthread_mutex_lock(&state->lcm_mutex);
        pthread_mutex_lock(&state->cmd_mutex);
        {
            //state->cmd.timestamp = utime_now();
            maebot_diff_drive_t_publish(state->lcm,  "MAEBOT_DIFF_DRIVE", &(state->cmd));
        }
        pthread_mutex_unlock(&state->cmd_mutex);
        pthread_mutex_unlock(&state->lcm_mutex);

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

/*
static void * driver_monitor(void *data) {
    //int systemTry = system("./maebot_driver");
    //if (systemTry) {} //ignore status

    return NULL;
}
*/

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

    int hz = 15;
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

	stateType_t curState, nextState;
	curState = stop;
	nextState = curState;
	int threshold = 0.3;
	int rthreshold = 0.1;
	int timesRotated45 = 0;
	while(state->running){

	switch(curState){
		case stop:
			moveBot(state, STOP);
			nextState = analyze;
			break;
		case move_forward:
			moveBot(state, FORWARD);
			nextState = analyze;
			break;
		case analyze:{
			if(1){//new_diamond_in_view){
				nextState = zap_diamond;
				timesRotated45--; //in case diamond is opposite the turns
			}else if(1){//new_branch_in_view){
				nextState = take_branch;
				timesRotated45 = 0;
			}else if(!state->moving){
				if(timesRotated45 == 3){
					return NULL;	//nowhere to go
				}
				driveToTheta(state, state->pos_theta - M_PI/4.0);
				timesRotated45++;
			}
			break;}
		case zap_diamond:{
			int wasMoving = 0;
			if(!state->moving){
				wasMoving = 1;
				moveBot(state, STOP);
			}
			//Still need to get diamond coords
			double diamond_x, diamond_y;
			double dx = diamond_x - state->pos_x;
			double dy = diamond_y - state->pos_y;
			double 	dtheta = atan2(dy, dx);
			double originalTheta = state->pos_theta;
			//rotate toward diamond
			driveToTheta(state, dtheta);
			while(abs(state->pos_theta - dtheta) > rthreshold){
				usleep(1000);
			}

			//shoot diamond
			fireLaser(state);
			//update diamond to zapped

			if(wasMoving){
				//return to original theta
				driveToTheta(state, originalTheta);
				while(abs(state->pos_theta - originalTheta) > rthreshold){
					usleep(1000);
				}
				moveBot(state, FORWARD);
			}
			nextState = analyze;
			break;}
		case take_branch:{
			//Assuming branch is defined as in free space - no collisions
			double branch_x, branch_y;
			double dx = branch_x - state->pos_x;
			double dy = branch_y - state->pos_y;
			double dtheta = atan2(dy, dx);

			//rotate bot toward branch opening
			driveToTheta(state, dtheta);
			while(abs(state->pos_theta - dtheta) > rthreshold){
				usleep(1000);
			}
			
			//drive forward to branch
			moveBot(state, FORWARD);
			while(abs(state->pos_x - branch_x) > threshold && abs(state->pos_y - branch_y) > threshold){
				usleep(1000);
			}
			moveBot(state, STOP);

			//rotate bot to face branch
			double branch_theta;	//Theta between current hall and branch
			driveToTheta(state, branch_theta);
			while(abs(state->pos_theta - branch_theta) > rthreshold){
				usleep(1000);
			}
			nextState = move_forward;
			break;}
		default: nextState = curState;
		}
		curState = nextState;
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
    state->veh.impl = state;
    state->pos_x    = 0;
    state->pos_y    = 0;
    state->pos_z    = 0;
    state->pos_theta= 0;
    state->odometry_seen = 0;
    state->init_last_mouse = 0;

    grid_map_init(&state->gridMap, GRID_MAP_MAX_WIDTH, GRID_MAP_MAX_HEIGHT);

	state->lookupTable = getLookupTable(752,480);

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
    pthread_mutex_init(&state->lcm_mutex, NULL);


    state->layer_map = zhash_create(sizeof(vx_display_t*), sizeof(vx_layer_t*), zhash_ptr_hash, zhash_ptr_equals);

    signal(SIGINT, handler);

    getopt_add_bool(state->gopt, 'h', "help", 0, "Show this help");
    getopt_add_bool(state->gopt, 'v', "verbose", 0, "Show extra debugging output");
    getopt_add_bool(state->gopt, 'c', "auto-camera", 0, "Automatically detect which camera to use");
    getopt_add_bool(state->gopt, '\0', "no-video", 0, "Disable video");
    getopt_add_int (state->gopt, 'l', "limitKBs", "-1", "Remote display bandwidth limit. < 0: unlimited.");
    getopt_add_double (state->gopt, 'd', "decimate", "1", "Decimate image by this amount before showing in vx");

    if (!getopt_parse(state->gopt, argc, argv, 0) ||
        getopt_get_bool(state->gopt,"help")) {
        getopt_do_usage(state->gopt);
        exit(-1);
    }

    state->getopt_options.verbose = getopt_get_bool(state->gopt, "verbose");
    state->getopt_options.autoCamera = getopt_get_bool(state->gopt, "auto-camera");
    state->getopt_options.no_video = getopt_get_bool(state->gopt, "no-video");
    state->getopt_options.limitKBs = getopt_get_int(state->gopt, "limitKBs");
    state->getopt_options.decimate = getopt_get_double(state->gopt, "decimate");

    //pthread_create(&state->dmon_thread, NULL, driver_monitor, state);
    pthread_create(&state->cmd_thread,  NULL, send_cmds, state);
    pthread_create(&state->lsr_thread,  NULL, send_lsr, state);
    //pthread_create(&state->led_thread,  NULL, send_led, state);
    pthread_create(&state->gui_thread,  NULL, gui_create, state);
    pthread_create(&state->lcm_handle_thread, NULL, lcm_handle_loop, state);
	pthread_create(&state->fsm_thread, NULL, FSM, state);

	pthread_join(state->fsm_thread, NULL);
    pthread_join(state->gui_thread, NULL);

    // clean up
    vx_world_destroy(state->vw);
	destroyLookupTable(state->lookupTable);
    grid_map_destroy(&state->gridMap);
    //maebot_sensor_data_t_unsubscribe(lcm, sensor_sub);
    //maebot_sensor_data_t_unsubscribe(lcm, odometry_sub);
    //system("kill `pgrep -f './maebot_driver'`");

    return 0;
}
