#include "explorer.h"
#include "common/matd.h"
#include <math.h>
#include <stdio.h>

void explorer_init(explorer_t *ex) {
	ex->goHome = 0;
}

explorer_state_t explorer_run(explorer_t *ex, haz_map_t *hm, double x, double y, double theta) {
	int forwardDist = explorer_check_region(ex, hm, EXPLORER_REGION_FORWARD, theta);
	int leftDist = explorer_check_region(ex, hm, EXPLORER_REGION_LEFT, theta);
	printf("forwardDist: %d\n", forwardDist);
	printf("leftDist: %d\n", leftDist);	
	return EX_EXIT;
}

int explorer_check_region(explorer_t *ex, haz_map_t *hm, int region, double theta) {
	matd_t *forwardRot, *checkPosTrans, *checkPosRot, *checkPos, *origin, *pos, *trans;
	double originD[3] = {0, 0, 1};
	double fRotD[9] = {cos(theta), -sin(theta), 0, sin(theta), cos(theta), 0, 0, 0, 1};

	origin = matd_create_data(3, 1, originD);
	pos = matd_identity(3);
	matd_put(pos, 0, 2, hm->width/2);
	matd_put(pos, 1, 2, hm->height/2);
	forwardRot = matd_create_data(3, 3, fRotD);
	trans = matd_identity(3);
	int count = 1;
	int u, v;
	int hmMaxY = hm->height;
	int hmMinY = 0;
	int hmMaxX = hm->width;
	int hmMinX = 0;
	int cont = 1;
	int dist = -1;
	haz_map_tile_t tile;

	while (cont == 1) {
		if (region == EXPLORER_REGION_FORWARD) {
			matd_put(trans, 1, 2, count * EXPLORER_TRACE_DIST);
		} else {
			matd_put(trans, 0, 2, -count * EXPLORER_TRACE_DIST);
		}
		checkPosTrans = matd_multiply(pos, forwardRot);
		checkPosRot = matd_multiply(checkPosTrans, trans);
		checkPos = matd_multiply(checkPosRot, origin);
		u = matd_get(checkPos, 0, 0);
		v = matd_get(checkPos, 1, 0);
		matd_destroy(checkPosTrans);
		matd_destroy(checkPosRot);
		matd_destroy(checkPos);
		//printf("u: %d, v: %d\n", u, v);

		if (u > hmMinX && u < hmMaxX && v > hmMinY && v < hmMaxY) {
			haz_map_get(hm, &tile, u, v);

			switch (tile.type) {
				case HAZ_MAP_OBSTACLE:
					dist = count;
					cont = 0;
				break;
				default:
					count++;
				break;
			}
		} else {
			// wall not found
			cont = 0;
			break;
		}
	}

	matd_destroy(forwardRot);
	matd_destroy(pos);
	matd_destroy(trans);
	return dist;
}

path_t* explorer_get_move(explorer_t *ex) {
	return ex->path;
}

double explorer_get_theta(explorer_t *ex) {
	return ex->theta;
}
