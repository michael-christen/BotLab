#include "color_app.h"

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
static int key_event (vx_event_handler_t * vh, vx_layer_t * vl, vx_key_event_t * key)
{
    state_t *state = vh->impl;

    state->cmd_val = 0;
    if (!key->released) {
	// forward
		if(key->key_code == 'r') {
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
		}
		state->red &= 0xff;
		state->green &= 0xff;
		state->blue &= 0xff;
		printf("r: %d, g: %d, b: %d, t: %f",
				state->red, state->green, state->blue,
				state->thresh);
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

    state->isrcReady = 1;
    image_source_data_t isdata;
    int res;
    state->imageValid = 0;

    while (state->running) {
        pthread_mutex_lock(&state->image_mutex);
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
			fill_color(state->red, state->green, state->blue,
					state->thresh, state->im);
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
    state->veh.key_event = key_event;
    state->red = 0x3a;
    state->green = 0x76;
    state->blue = 0x41;
    state->thresh = 52.0;
    state->isrcReady = 0;
    state->im = NULL;
	state->cmd_val = 0;

    state->running = 1;


    state->vw = vx_world_create();
    state->displayStarted = state->displayFinished = 0;


    pthread_mutex_init(&state->layer_mutex, NULL);
    pthread_mutex_init(&state->cmd_mutex, NULL);
    pthread_mutex_init(&state->lsr_mutex, NULL);
    pthread_mutex_init(&state->lcm_mutex, NULL);
    pthread_mutex_init(&state->running_mutex, NULL);
    pthread_mutex_init(&state->image_mutex, NULL);


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

    pthread_create(&state->camera_thread, NULL, camera_analyze, state);
    pthread_create(&state->gui_thread,  NULL, gui_create, state);


    if (pthread_join(state->gui_thread, NULL) != 0) {
        //printf("Problem here!\n");
    } else {
        //printf("All good!\n");
    }

    // clean up
    vx_world_destroy(state->vw);
    //printf("Exited Cleanly!\n");
    //maebot_sensor_data_t_unsubscribe(lcm, sensor_sub);
    //maebot_sensor_data_t_unsubscribe(lcm, odometry_sub);
    //system("kill `pgrep -f './maebot_driver'`");


    return 0;
}
