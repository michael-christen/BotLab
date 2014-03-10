#ifndef MAEBOT_APP_H
#define MAEBOT_APP_H

//////////////
// INCLUDES
//////////////

// C Libraries
#include <pthread.h>
#include"time.h"
#include "vx/vx.h"

// MAEBOT
#include "barrel_distortion.h"
#include "pixel.h"
#include "haz_map.h"
#include "pid_ctrl.h"
#include "path.h"
#include "world_map.h"

// EECS 467 Libraries
#include "common/getopt.h"
#include "common/image_util.h"
#include "common/timestamp.h"
#include "imagesource/image_source.h"
#include "imagesource/image_convert.h"
#include "common/matd.h"

// LCM
#include "lcmtypes/maebot_diff_drive_t.h"
#include "lcmtypes/maebot_laser_t.h"
#include "lcmtypes/maebot_leds_t.h"
#include "lcmtypes/maebot_sensor_data_t.h"

//////////////
// CONSTANTS
//////////////
#define MAX_POS_SAMPLES 20
#define POS_SAMPLES_INTERVAL 500000
#define NUM_LAYERS 4
#define BRUCE_DIAMETER 10.5
#define BRUCE_HEIGHT 10

// XXX these need to be fixed based on actual spec
#define MAX_REVERSE_SPEED -32768
#define MAX_FORWARD_SPEED 32767
#define CM_TO_VX 0.1


//////////////
// STRUCTS
//////////////

typedef struct layer_data_t layer_data_t;
typedef struct state_t state_t;
typedef struct getopt_options_t getopt_options_t;


struct getopt_options_t {
    int verbose, no_video, limitKBs, autoCamera;
    double decimate;
};

struct layer_data_t {
    int enable;
    const char* name;
    vx_world_t *world;
    vx_layer_t *layer;
    float position[4];

    int (*init)(state_t *state, layer_data_t *layerData);
    int (*displayInit)(state_t *state, layer_data_t *layerData);
    int (*render)(state_t *state, layer_data_t *layerData);
    int (*destroy)(state_t *state, layer_data_t *layerData);
};

typedef enum {UNKNOWN, OCCUPIED, UNOCCUPIED} grid_status;

typedef struct grid_cell{
	grid_status status;
	clock_t created;
} grid_cell;

struct state_t {
    getopt_options_t  getopt_options;
    vx_application_t app;
    vx_event_handler_t veh;

	pthread_t fsm_thread;

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

	pthread_mutex_t drive_mutex;
	pthread_cond_t drive_cond;
	double goal_x, goal_y, goal_theta;
	int waiting_on_pos, waiting_on_theta;


    int acc[3];
    int gyro[3];
    int gyro_int[3];
    double gyro_0_bias, gyro_1_bias, gyro_2_bias;
    const char *sensor_channel;

    //Position info from odometry
    int positionQueueP, positionQueueCount;
    position_t positionQueue[MAX_POS_SAMPLES];
    double pos_x, pos_y, pos_z;
	double last_x, last_y;
    double pos_theta;
	double last_theta;
    int32_t prev_left_ticks, prev_right_ticks;
    int    odometry_seen;
    const char *odometry_channel;

	//map
	grid_cell obstacle_map[200][200]; //10cm x 10 cm

	//bot is moving forward or back
	int translating;
	int rotating;
	int moving;


    getopt_t * gopt;
    char * url;
    image_source_t *isrc;
    int fidx;

    lcm_t * lcm;
    pthread_mutex_t lcm_mutex;

    pthread_t lcm_handle_thread;
    pthread_t position_tracker_thread;

    pthread_mutex_t layer_mutex;
    pthread_mutex_t running_mutex;
    pthread_t gui_thread;

    int layerCount;
    layer_data_t layers[NUM_LAYERS];

    vx_world_t * vw;
    zhash_t *layer_map; // <display, layer>

    // Mouse event stuff
    int init_last_mouse;
    vx_mouse_event_t last_mouse;
    double mouseDownX, mouseDownY;
    double goalMouseX, goalMouseY;

    pthread_t dmon_thread;

    pixel_t* lookupTable;

    // Haz map
    haz_map_t hazMap;

    //Tape data
    pixel_t* tape;
    unsigned int num_pts_tape;

    uint32_t red, green, blue;
    double thresh;

    pid_ctrl_t *green_pid;
    double      green_pid_out;
};


//////////////
// FUNCTIONS
//////////////

void* gui_create(void * data);
void display_finished(vx_application_t * app, vx_display_t * disp);
void display_started(vx_application_t * app, vx_display_t * disp);

#endif
