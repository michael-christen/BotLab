#ifndef EXPLORER_H
#define EXPLORER_H
#include "maebot_app.h"
#include "path.h"
#include "haz_map.h"
#include "world_map.h"

#define EXPLORER_MOVE			0
#define EXPLORER_DIAMOND		1
#define EXPLORER_GOHOME			2
#define EXPLORER_TURN_LEFT  	3
#define EXPLORER_TURN_RIGHT 	4
#define EXPLORER_MAX_WALLDIST	HAZ_MAP_UNIT_TO_CM * 40
#define EXPLORER_MIN_WALLDIST	HAZ_MAP_UNIT_TO_CM * 7
#define EXPLORER_TRACE_DIST		1
#define EXPLORER_DRIVE_DIST		5

#define EXPLORER_REGION_LEFT	0
#define EXPLORER_REGION_FORWARD	1

typedef struct explorer_t explorer_t;
typedef enum explorer_state explorer_state_t;

struct explorer_t {
	int goHome;
	path_t *path;
	double theta;
};

enum explorer_state{
	EX_ANALYZE,
	EX_MOVE,
	EX_TURN,
	EX_ZAP_DIAMOND,
	EX_GOHOME,
	EX_START,
	EX_EXIT,
	EX_WAIT,
	EX_DEFAULT
};

path_t * choose_path(void * data, double pre_analyze_theta);

#endif
