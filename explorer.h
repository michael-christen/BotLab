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

typedef struct explorer_t explorer_t;
typedef enum explorer_state explorer_state_t;

struct explorer_t {
	int goHome;
	path_t *path;
};

enum explorer_state{
	EX_ANALYZE,
	EX_MOVE_FORWARD,
	EX_TURN_LEFT,
	EX_TURN_RIGHT,
	EX_ZAP_DIAMOND,
	EX_GOHOME,
	EX_EXIT
};

void explorer_init(explorer_t *ex);
explorer_state_t explorer_run(explorer_t *ex, haz_map_t *hm, double x, double y, double theta);
path_t* explorer_get_move(explorer_t *ex);

#endif
