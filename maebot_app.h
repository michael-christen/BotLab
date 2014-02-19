#ifndef MAEBOT_APP_H
#define MAEBOT_APP_H

//////////////
// INCLUDES
//////////////

// C Libraries
#include <pthread.h>

// VX
#include "vx/vx.h"

// EECS 467 Libraries
#include "common/getopt.h"
#include "common/image_util.h"
#include "common/timestamp.h"
#include "imagesource/image_source.h"
#include "imagesource/image_convert.h"

// LCM
#include "lcmtypes/maebot_diff_drive_t.h"


//////////////
// CONSTANTS
//////////////

// XXX these need to be fixed based on actual spec
#define MAX_REVERSE_SPEED -32768
#define MAX_FORWARD_SPEED 32767


//////////////
// STRUCTS
//////////////

typedef struct 
{
    int verbose, no_video, limitKBs;
    double decimate;
} getopt_options_t;

typedef struct
{
    getopt_options_t  getopt_options;
    vx_application_t app;
    vx_event_handler_t veh;

    maebot_diff_drive_t cmd;
    pthread_mutex_t cmd_mutex;
    pthread_t cmd_thread;

    int running, displayStarted, displayFinished;

    getopt_t * gopt;
    char * url;
    image_source_t *isrc;
    int fidx;

    lcm_t * lcm;

    pthread_mutex_t layer_mutex;
    pthread_t gui_thread;

    vx_world_t * vw;
    zhash_t *layer_map; // <display, layer>

    pthread_t dmon_thread;
} state_t;


//////////////
// FUNCTIONS
//////////////

void* gui_create(void * data);
void display_finished(vx_application_t * app, vx_display_t * disp);
void display_started(vx_application_t * app, vx_display_t * disp);

#endif
