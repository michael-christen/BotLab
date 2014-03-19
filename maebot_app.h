#ifndef MAEBOT_APP_H
#define MAEBOT_APP_H

//////////////
// INCLUDES
//////////////

// C Libraries
#include <pthread.h>
#include "time.h"
#include "vx/vx.h"
#include <stdlib.h>


// MAEBOT
#include "blob_detection.h"
#include "barrel_distortion.h"
#include "pixel.h"
#include "haz_map.h"
#include "pid_ctrl.h"
#include "odometry.h"
#include "path.h"
#include "world_map.h"
#include "explorer.h"
#include "line_detection.h"

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
#define MAX_POS_SAMPLES 50
#define POS_SAMPLES_INTERVAL 500000
#define NUM_LAYERS 4
#define BRUCE_DIAMETER 10.5
#define BRUCE_HEIGHT 10
#define MAX_NUM_ELLIPSES 0

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
typedef struct fired_from fired_from_t;
typedef enum command_val cmd_val_t;


struct getopt_options_t {
    int verbose, no_video, limitKBs, autoCamera, mouseGuidance;
    double decimate;
};

struct fired_from {
	double x, y, theta;
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

enum command_val {
	STOP,
	FORWARD,
	BACKWARD,
	LEFT,
	RIGHT,
	PID
};

struct state_t {
    getopt_options_t  getopt_options;
    vx_application_t app;
    vx_event_handler_t veh;

	pthread_t fsm_thread;
	pthread_t calibrator_thread;

    maebot_diff_drive_t cmd;
    pthread_mutex_t cmd_mutex;
    pthread_t cmd_thread;
	cmd_val_t cmd_val;
	int   motor_count;

    pthread_t motion_thread;

    volatile int running;
    int displayStarted, displayFinished;

    maebot_laser_t lsr;
    pthread_mutex_t lsr_mutex;
    pthread_t lsr_thread;

    maebot_leds_t led;
    pthread_mutex_t led_mutex;
    pthread_t led_thread;

	pthread_mutex_t drive_mutex;
	pthread_cond_t drive_cond;
	double goal_x, goal_y, goal_theta;

	int FSM;
	double fsm_time_elapsed; //time_t
	double fsmTimeElapsed; //clock_t

    int acc[3];
    int gyro[3];
    int64_t gyro_int[3];
	double gyro_ticks_per_theta;
	int64_t save_gyro;
	double save_theta;
    const char *sensor_channel;

	int calibrate;	//signal calibration
	int calibrating;	//actively calibrating

    //Position info from odometry
    int pathTakenValid, targetPathValid;
    path_t *pathTaken, *targetPath;

	odometry_t pos;
	odometry_t last_pos;
	matd_t *cur_var;
	matd_t *last_var;
	double pos_x, pos_y, pos_z;
	double last_x, last_y;
	double pos_theta;
	double last_theta;

    int32_t prev_left_ticks, prev_right_ticks;
    int    odometry_seen;
    const char *odometry_channel;
	matd_t *var_matrix;

	int    stored_mat_num;
	odometry_t *stored_pos;
	matd_t **stored_matrices;

	double alpha, beta;

	//map
	world_map_t world_map;

	//bot is moving forward or back
	int translating;
	int rotating;
	int moving;
    //explorer_t explorer;

    getopt_t * gopt;
    char * url;
    image_source_t *isrc;
    int isrcReady;
    int fidx;
    int imageValid;
    image_u32_t *im;
    pthread_mutex_t image_mutex;
    pthread_mutex_t haz_map_mutex;
	pthread_mutex_t world_map_mutex;
	pthread_cond_t  image_cv;
    pthread_t camera_thread;

    int num_balls;
    ball_t balls[MAX_NUM_BALLS];
	fired_from_t zapped_diamonds[50];
	int num_zapped_diamonds;


    lcm_t * lcm;
    pthread_mutex_t lcm_mutex;

    pthread_t lcm_handle_thread;
    pthread_t position_tracker_thread;

    pthread_mutex_t layer_mutex;
    pthread_mutex_t running_mutex;
    pthread_t gui_thread;

    int layerCount;
    layer_data_t layers[NUM_LAYERS];

    zhash_t *layer_map; // <display, layer>

    // Mouse event stuff
    int init_last_mouse;
    vx_mouse_event_t last_mouse;
    double mouseDownX, mouseDownY;
    double goalMouseX, goalMouseY;
	int  click_following;
    uint8_t goToMouseCoords;

    pthread_t dmon_thread;

    pixel_t* lookupTable;

    // Haz map
    haz_map_t hazMap;

    //Tape data
    pixel_t* tape;
    unsigned int num_pts_tape;
	matd_t* H;

    uint32_t red, green, blue;
    double thresh;
	double hue;

    pid_ctrl_t *green_pid;
    double      green_pid_out;
    double      diff_x;
    int         diamond_seen;
	int         doing_pid;
	int         num_pid_zeros;

	double     dist;

	pid_ctrl_t *theta_pid;
	int         doing_pid_theta;

	double left_offset;
	int min_pxs;
};

//////////////
// FUNCTIONS
//////////////

void* gui_create(void * data);
void display_finished(vx_application_t * app, vx_display_t * disp);
void display_started(vx_application_t * app, vx_display_t * disp);

#endif
