#include "explorer.h"
#include "common/matd.h"


void explorer_init(explorer_t *ex) {
	ex->goHome = 0;
}

explorer_state_t explorer_run(explorer_t *ex, haz_map_t *hm, double x, double y, double theta) {
	/*matd_t *forwardRot, *leftRot, *checkPosRot, *checkPos, *pos, *trans;
	double positionD[3] = {x, y, 1};
	double transD[2];
	double fRotD[9] = {cos(theta), -sin(theta), 0, sin(theta), cos(theta), 0, 0, 0, 1};
	double lRotD[9] = {cos(theta + PI/2), -sin(theta + PI/2), 0, sin(theta + PI/2), cos(theta + M_PI/2), 0, 0, 0, 1};

	pos = matd_create_data(3, 1, positionD);
	forwardRot = matd_create_data(3, 3, fRotD);
	leftRot = matd_create_data(3, 3, lRotD);
	trans = matd_identity(3);

	int forwardDist = -1;
	int leftDist = -1;
	int count = 1;
	int x, y, point;
	int hmMaxY = y + hm->height/2;
	int hmMinY = y - hm->height/2;
	int hmMaxX = x + hm->width/2;
	int hmMinX = x - hm->width/2;

	while (1) {
		matd_put(trans, 2, 3, count * EXPLORER_TRACE_DIST);
		checkPosRot = matd_multiply(forwardRot, pos);
		checkPos = matd_multiply(trans, checkPosRot);
		x = matd_get(checkPos, 1, 1);
		y = matd_get(checkPos, 2, 1);
		matd_destroy(checkPosRot);
		matd_destroy(checkPos);

		if (x > hmMinY && x < hmMaxX && y > hmMinY && y < hmMaxY) {
			point = haz_map_get(hm, x, y);

			switch (point) {
				case HAZ_MAP_OBSTACLE:
					forwardDist = matd_get(trans, 2, 3);
				break;
			}

			count++;
		} else {
			// Forward wall not found
			break;
		}
	}

	matd_destroy(forwardRot);
	matd_destroy(leftRot);
	matd_destroy(pos);
	matd_destroy(trans);*/
	return EX_EXIT;
}

path_t* explorer_get_move(explorer_t *ex) {
	return ex->path;
}
