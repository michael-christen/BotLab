#include "path.h"
#include "stdlib.h"

void path_destroy(path_t *path) {
	if (path->length > 0) {
		free(path->waypoints);
		path->waypoints = NULL;
	}
	free(path);
}
