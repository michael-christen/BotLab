#include "haz_map.h"
#include <math.h>
#include <stdio.h>
#include "common/zarray.h"

// Haz Map origin at bottom left corner

double map(double x, double in_min, double in_max, double out_min, double out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void haz_map_init(haz_map_t *hm, int w, int h) {
	hm->image = image_u32_create(w, h);
	hm->width = w;
	hm->height = h;
	position_t o[8] = {{-1,-1},{0,-1},{1,-1},{1,0},{1,1},{0,1},{-1,1},{-1,0}};

	int i, j, k, count, newX, newY;
	haz_map_tile_t tile;
	haz_map_tile_t *tileP;
	tile.type = HAZ_MAP_FREE;
	tile.val = 1;
	for (i = 0; i < h; i++) {
		for (j = 0; j < w; j++) {
			tileP = &hm->hazMap[i*hm->width + j];
			tileP->x = j;
			tileP->y = i;
			count = 0;
			for (k = 0; k < 8; k++) {
				newX = j + o[k].x;
				newY = i + o[k].y;
				if (haz_map_in_bounds(hm, newX, newY)) {
					tileP->neighbors[count++] = &hm->hazMap[newY*hm->width + newX];
				}
			}
			tileP->numNeighbors = count;
			tile.type = HAZ_MAP_FREE;
			tile.val = 1;
			haz_map_set_data(hm, j, i, &tile);
		}
	}
}

void haz_map_set(haz_map_t *hm, int x, int y, uint8_t type) {
	int maxU, minU, u, v;
	double mapAngle, dist, val;
	haz_map_tile_t tile;
	int maxV, minV;
	//Bounds checking
	if(y*hm->image->stride + x >= hm->image->height*hm->image->stride) {
		return;
	}

	switch (type) {
		case HAZ_MAP_OBSTACLE:
			tile.type = HAZ_MAP_OBSTACLE;
			haz_map_set_data(hm, x, y, &tile);
			maxV = fmin(y + HAZ_MAP_CONFIG_RAIDUS, hm->height - 1);
			minV = fmax(y - HAZ_MAP_CONFIG_RAIDUS, 0);
			for (v = minV; v <= maxV; v++) {
				mapAngle = map(v, minV, maxV, 0, M_PI);
				maxU = fmin(x + HAZ_MAP_CONFIG_RAIDUS * sin(mapAngle), hm->width - 1);
				minU = fmax(x - HAZ_MAP_CONFIG_RAIDUS * sin(mapAngle), 0);
				for (u = minU; u <= maxU; u++) {
					dist = sqrt(pow(u - x, 2) + pow(y - v, 2));
					if (dist <= HAZ_MAP_OBSTACLE_RADIUS) {
						tile.type = HAZ_MAP_OBSTACLE;
						tile.val = HAZ_MAP_HUGE_DIST;
						haz_map_set_data(hm, u, v, &tile);
					} else {
						haz_map_get(hm, &tile, u, v);
						val = pow(HAZ_MAP_CONFIG_RAIDUS - dist, 2);
						if (tile.type == HAZ_MAP_UNKNOWN || (tile.type == HAZ_MAP_FREE && tile.val < val)) {
							tile.type = HAZ_MAP_FREE;
							tile.val = val;
							haz_map_set_data(hm, u, v, &tile);
						}
					}
					
				}
			}
		break;

		default:
			tile.type = type;
			tile.val = 0;
			haz_map_set_data(hm, u, v, &tile);
		break;
	}
}

void haz_map_set_data(haz_map_t *hm, int x, int y, haz_map_tile_t *data) {
	int color, offset;
	hm->hazMap[y*hm->width + x].type = data->type;
	hm->hazMap[y*hm->width + x].val = data->val;

	switch (data->type) {
		case HAZ_MAP_UNKNOWN:
			color = 0xFF777777;
		break;
		case HAZ_MAP_FREE:
			offset = map(data->val, 0, HAZ_MAP_VAL_MAX, 0, 255);
			//color = 0xFF5EFFFF;
			color = 0xFFFF00FF | (offset << 8);
			//printf("val: 0x%10x\n", color);
		break;
		case HAZ_MAP_OBSTACLE:
			color = 0xFFFF0044;
		break;
		default:
			color = 0xFF000000;
		break;
	}
	hm->image->buf[y*hm->image->width + x] = color;
}

void haz_map_translate(haz_map_t *hm, int newX, int newY, int oldX, int oldY) {
	int diffX = newX - oldX;
	int diffY = newY - oldY;
	int lowX = 0;
	int highX = hm->width;
	int lowY = 0;
	int highY = hm->height;
	haz_map_tile_t tile;

	if (diffY >= 0) {
		lowY = diffY;
	} else {
		highY = hm->height - diffY;
	}

	if (diffX >= 0) {
		lowX = diffX;
	} else {
		highX = hm->width - diffX;
	}

	for (int y = 0; y < hm->height; y++) {
		for (int x = 0; x < hm->width; x++) {
			if (x < lowX || x > highX || y < lowY || y > highY) {
				tile.type = HAZ_MAP_UNKNOWN;
				haz_map_set_data(hm, x, y, &tile);
			} else {
				haz_map_get(hm, &tile, x, y);
				haz_map_set_data(hm, x - diffX, y - diffY, &tile);
			}
		}
	}

}

void haz_map_get(haz_map_t *hm, haz_map_tile_t *tile, int x, int y) {
	tile->type = hm->hazMap[y*hm->width + x].type;
	tile->val = hm->hazMap[y*hm->width + x].val;
}

void haz_map_destroy(haz_map_t *hm) {
	image_u32_destroy(hm->image);
}

int haz_map_in_bounds(haz_map_t *hm, int x, int y) {
	if (x >= 0 && x < hm->width && y >= 0 && y < hm->height) {
		return 1;
	} else {
		return 0;
	}
}

path_t* haz_map_get_path(haz_map_t *hm, int startX, int startY, int endX, int endY) {
	uint32_t i, checkIndex;
	int64_t startIndex;
	uint8_t cont = 1;
	dijkstra_dists_t curDist;
	dijkstra_data_t curData, checkData;
	zarray_t *dData, *pq;
	haz_map_tile_t *curTile, *checkTile;
	uint32_t checkX, checkY;

	dData = zarray_create(sizeof(dijkstra_data_t));
	pq = zarray_create(sizeof(dijkstra_dists_t));

	startIndex = (startY * hm->width) + startX;
	for (i = 0; i < hm->width * hm->height; i++) {
		curData.shortestPathKnown = 0;
		curData.parentIndex = 0;
		curData.pathCount = 0;
		if (i == startIndex) {
			curData.dist = 0;
		} else {
			curData.dist = HAZ_MAP_HUGE_DIST;
		}

		zarray_add(dData, &curData);
	}

	curDist.tileIndex = startIndex;
	curDist.dist = 0;

	zarray_add(pq, &curDist);

	while (cont && zarray_size(pq) > 0) {
		uint8_t valid = 0;
		dijkstra_dists_t minDist;
		uint32_t newD, minIndex;
		// Find current smallest distance
		// NOTE: should be replaced by a priority queue
		for (i = 0; i < zarray_size(pq); i++) {
			if (valid == 0){
				zarray_get(pq, i, &minDist);
				minIndex = i;
				valid = 1;
			} else {
				zarray_get(pq, i, &curDist);
				if (curDist.dist < minDist.dist) {
					minDist = curDist;
					minIndex = i;
				}
			}
		}
		zarray_remove_index(pq, minIndex, 1);
		//printf("here!\n");

		zarray_get(dData, minDist.tileIndex, &curData);
		//printf("curData.dist %d\n", curData.dist);
		curTile = &hm->hazMap[minDist.tileIndex];
		if (curData.shortestPathKnown == 0) {
			curData.shortestPathKnown = 1;

			if (curTile->x == endX && curTile->y == endY) {
				printf("Found path at %d, %d of length %d\n", curTile->x, curTile->y, curData.pathCount);
				cont = 0;
				break;
			}

			// for each neighbor
			//printf("cur tile: %u, %u\n", curTile->x, curTile->y);
			for (i = 0; i < curTile->numNeighbors; i++) {
				checkTile = curTile->neighbors[i];
				//printf("check tile: %u, %u\n", checkTile->x, checkTile->y);
				checkIndex = checkTile->y*hm->width + checkTile->x;
				newD = curData.dist + checkTile->val;
				zarray_get(dData, checkIndex, &checkData);
				//printf("newD %d, data %d\n", newD, checkData.dist);
				if (newD < checkData.dist) {
					checkData.dist = newD;
					checkData.parentIndex = minDist.tileIndex;
					checkData.pathCount = curData.pathCount + 1;
					zarray_set(dData, checkIndex, &checkData, NULL);
					curDist.tileIndex = checkIndex;
					curDist.dist = newD;
					zarray_add(pq, &curDist);

				}
			}
			// end for each
		}
	}

	printf("Path Start of count %d\n", curData.pathCount);
	printf("startIndex %u\n", startIndex);
	printf("endTile %u, %u\n", curTile->x, curTile->y);
	int curIndex;
	path_t *retPath;
	retPath = malloc(sizeof(path_t));
	retPath->waypoints = malloc(sizeof(position_t) * curData.pathCount);
	retPath->length = curData.pathCount;
	retPath->position = 0;
	for (i = retPath->length - 1; i >= 0; i--) {
		curIndex = curTile->y*hm->width + curTile->x;
		if (curIndex != startIndex) {
			zarray_get(dData, curIndex, &curData);
			curTile = &hm->hazMap[curData.parentIndex];
			retPath->waypoints[i].x = curTile->x;// - hm->width/2;
			retPath->waypoints[i].y = curTile->y;// - hm->height/2;
		} else {
			break;
		}
	}
	zarray_destroy(dData);
	return retPath;
}
