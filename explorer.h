#ifndef EXPLORER_H
#define EXPLORER_H

#include "path.h"
#include "haz_map.h"

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
};

enum explorer_state{
	stop,
	move_forward,
	analyze,
	zap_diamond,
	take_branch
};

void explorer_init(explorer_t *ex);
explorer_state_t explorer_run(explorer_t *ex, haz_map_t *hm, double x, double y, double theta);
int explorer_check_region(explorer_t *ex, haz_map_t *hm, int region, double theta);
path_t* explorer_get_move(explorer_t *ex);

#endif