#include "maebot_app.h"
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


    pthread_mutex_lock(&state->cmd_mutex);
    if (!key->released) {
        if (key->key_code == 'w' || key->key_code == 'W') {
            // forward
            state->cmd.motor_left_speed = MAX_FORWARD_SPEED;
            state->cmd.motor_right_speed = MAX_FORWARD_SPEED;
        } else if (key->key_code == 'a' || key->key_code == 'A' ) {
            // turn left
            state->cmd.motor_left_speed = MAX_REVERSE_SPEED;
            state->cmd.motor_right_speed = MAX_FORWARD_SPEED;

        } else if (key->key_code == 's' || key->key_code == 'S') {
            // reverse
            state->cmd.motor_left_speed = MAX_REVERSE_SPEED;
            state->cmd.motor_right_speed = MAX_REVERSE_SPEED;
        } else if (key->key_code == 'd' || key->key_code == 'D') {
            // turn right
            state->cmd.motor_left_speed = MAX_REVERSE_SPEED;
            state->cmd.motor_right_speed = MAX_FORWARD_SPEED;
        }
    } else {
        // when key released, speeds default to 0
        state->cmd.motor_left_speed = 0;
        state->cmd.motor_right_speed = 0;
    }
    pthread_mutex_unlock(&state->cmd_mutex);

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

    state->running = 1;
    state->displayStarted = state->displayFinished = 0;
    state->lcm = lcm_create(NULL);
    state->vw = vx_world_create();
    pthread_mutex_init(&state->layer_mutex, NULL);
    pthread_mutex_init(&state->cmd_mutex, NULL);

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
    pthread_create(&state->gui_thread,  NULL, gui_create, state);

    pthread_join(state->gui_thread, NULL);

    return 0;
}
