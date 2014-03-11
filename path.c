#include "path.h"
#include "stdlib.h"

void path_destroy(path_t *path) {
	free(path->waypoints);
	path->waypoints = NULL;
}
