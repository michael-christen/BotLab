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
#include "eecs467_util.h"
#include "common/getopt.h"
#include "common/image_util.h"
#include "common/timestamp.h"
#include "imagesource/image_source.h"
#include "imagesource/image_convert.h"

// LCM
#include "lcmtypes/maebot_diff_drive_t.h"
#include "lcmtypes/maebot_laser_t.h"
#include "lcmtypes/maebot_leds_t.h"
#include "lcmtypes/maebot_sensor_data_t.h"

//////////////
// CONSTANTS
//////////////

#define NUM_LAYERS 3

// XXX these need to be fixed based on actual spec
#define MAX_REVERSE_SPEED -32768
#define MAX_FORWARD_SPEED 32767


//////////////
// STRUCTS
//////////////

typedef struct layer_data_t layer_data_t;
typedef struct state_t state_t;
typedef struct getopt_options_t getopt_options_t;


struct getopt_options_t {
    int verbose, no_video, limitKBs;
    double decimate;
};

struct layer_data_t {
    int enable;
    const char* name;
    vx_world_t *world;
    vx_layer_t *layer;
    float position[4];
    float lowLeft[2];
    float upRight[2];

    int (*init)(state_t *state, layer_data_t *layerData);
    int (*displayInit)(state_t *state, layer_data_t *layerData);
    int (*render)(state_t *state, layer_data_t *layerData);
    int (*destroy)(state_t *state, layer_data_t *layerData);
};

struct state_t {
    getopt_options_t  getopt_options;
    vx_application_t app;
    vx_event_handler_t veh;
    parameter_gui_t  *pg;

    maebot_diff_drive_t cmd;
    pthread_mutex_t cmd_mutex;
    pthread_t cmd_thread;

    int running, displayStarted, displayFinished;

    maebot_laser_t lsr;
    pthread_mutex_t lsr_mutex;
    pthread_t lsr_thread;

    maebot_leds_t led;
    pthread_mutex_t led_mutex;
    pthread_t led_thread;

//		not sure if we need this?
/*	 maebot_sensor_data_t sensor;
    pthread_mutex_t sensor_mutex;
    pthread_t sensor_thread;	
*/ 
    int acc[3];
    int gyro[3];
    int gyro_int[3];
    double gyro_0_bias, gyro_1_bias, gyro_2_bias;

    //Position info from odometry
    double pos_x, pos_y;
    double pos_theta;

    getopt_t * gopt;
    char * url;
    image_source_t *isrc;
    int fidx;

    lcm_t * lcm;

    pthread_mutex_t layer_mutex;
    pthread_t gui_thread;

    int layerCount;
    layer_data_t layers[NUM_LAYERS];

    vx_world_t * vw;
    zhash_t *layer_map; // <display, layer>

    pthread_t dmon_thread;
};

//////////////
// FUNCTIONS
//////////////

void* gui_create(void * data);
void display_finished(vx_application_t * app, vx_display_t * disp);
void display_started(vx_application_t * app, vx_display_t * disp);

#endif
