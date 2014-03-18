#include "haz_map.h"
#include <math.h>
#include <stdio.h>
#include "common/zarray.h"
#include <stdlib.h>

// Haz Map origin at bottom left corner

double map(double x, double in_min, double in_max, double out_min, double out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void haz_map_init(haz_map_t *hm, int w, int h) {
	int i, j, k, count, newX, newY;
	hm->image = image_u32_create(w, h);
	hm->width = w;
	hm->height = h;
	hm->x = 0;
	hm->y = 0;
	hm->max_free_val = pow(HAZ_MAP_CONFIG_RAIDUS * 1.0 / GRID_RES, 2) + HAZ_MAP_REPULSE_FACTOR;
	position_t o[8] = {{-1,-1},{0,-1},{1,-1},{1,0},{1,1},{0,1},{-1,1},{-1,0}};
	double distFactors[8];

	for (i = 0; i < 8; i++) {
		distFactors[i] = sqrt(fabs(o[i].x) + fabs(o[i].y));
	}

	haz_map_tile_t tile;
	haz_map_tile_t *tileP;
	tile.type = HAZ_MAP_UNKNOWN;
	tile.val = HAZ_MAP_HUGE_DIST;
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
					tileP->neighbors[count].distFactor = distFactors[k];
					tileP->neighbors[count++].tile = &hm->hazMap[newY*hm->width + newX];
				}
			}
			tileP->numNeighbors = count;
			haz_map_set_data(hm, j, i, &tile);
		}
	}
}

void haz_map_set(haz_map_t *hm, int x, int y, uint8_t type) {
	haz_map_tile_t tile;
	time(&(tile.timestamp));
	//Bounds checking
	if(y*hm->width + x >= hm->height*hm->width) {
		return;
	}

	switch (type) {
		case HAZ_MAP_OBSTACLE:
			tile.type = HAZ_MAP_OBSTACLE;
			tile.val = HAZ_MAP_HUGE_DIST;
		break;

		case HAZ_MAP_UNKNOWN:
			tile.type = HAZ_MAP_UNKNOWN;
			tile.val = HAZ_MAP_HUGE_DIST;
		default:
			tile.type = type;
			tile.val = 1;
		break;
	}
	haz_map_set_data(hm, x, y, &tile);
}

void haz_map_set_data(haz_map_t *hm, int x, int y, haz_map_tile_t *data) {
	hm->hazMap[y*hm->width + x].type = data->type;
	hm->hazMap[y*hm->width + x].val = data->val;
	hm->hazMap[y*hm->width + x].timestamp = data->timestamp;
}

void haz_map_set_image_data(haz_map_t *hm, int x, int y, haz_map_tile_t *data) {
	int color, offset;
	switch (data->type) {
		case HAZ_MAP_UNKNOWN:
			color = 0xFF777777;
		break;
		case HAZ_MAP_FREE:
			if (data->val == 1) {
				color = 0xFFCCFFCC;
			} else {
				offset = map(data->val, HAZ_MAP_REPULSE_FACTOR, hm->max_free_val, 0, 255);
				color = 0xFFCC00CC | ((255 - offset) << 8);
			}
		break;
		case HAZ_MAP_OBSTACLE:
			color = 0xFFFF0044;
		break;
		default:
			color = 0xFF000000;
		break;
	}
	hm->image->buf[y*hm->image->stride + x] = color;
}

void haz_map_translate(haz_map_t *hm, double newX, double newY) {
	double diffX = newX - hm->gridX;
	double diffY = newY - hm->gridY;
	int transX = 0;
	int transY = 0;
	hm->x = newX;
	hm->y = newY;

	int lowX = 0;
	int highX = hm->width;
	int lowY = 0;
	int highY = hm->height;
	haz_map_tile_t tile;

	if (fabs(diffX) >= GRID_RES) {
		transX = (int) diffX / GRID_RES;
		if (transX >= 0) {
			lowX = transX;
		} else {
			highX = hm->width + transX;
		}
	}

	if (fabs(diffY) >= GRID_RES) {
		transY = (int) diffY / GRID_RES;
		if (transY >= 0) {
			lowY = transY;
		} else {
			highY = hm->height + transY;
		}
	}

	//printf("nX: %f, nY: %f, oX: %f, oY: %f\n", newX, newY, oldX, oldY);
	//printf("diffX: %f, diffY: %f\n", hm->diffX, hm->diffY);
	int x, y;
	if (transY != 0 || transX != 0) {
		hm->gridX += (transX * GRID_RES);
		hm->gridY += (transY * GRID_RES);
		if (transY >= 0) {
			y = 0;
		} else {
			y = hm->width - 1;
		}
		while (1) {
			if (transY >= 0) {
				if (y >= hm->height) break;
			} else {
				if (y < 0) break;
			}

			if (transX >= 0) {
				x = 0;
			} else {
				x = hm->width - 1;
			}

			while(1) {
				if (transX >= 0) {
					if (x >= hm->width) break;
				} else {
					if (x < 0) break;
				}

				if (x >= lowX && x < highX && y >= lowY && y < highY) {
					haz_map_get(hm, &tile, x, y);
					haz_map_set_data(hm, x - transX, y - transY, &tile);
					tile.type = HAZ_MAP_UNKNOWN;
					tile.val = HAZ_MAP_HUGE_DIST;
					haz_map_set_data(hm, x, y, &tile);
				} else {
					tile.type = HAZ_MAP_UNKNOWN;
					tile.val = HAZ_MAP_HUGE_DIST;
					haz_map_set_data(hm, x, y, &tile);
				}

				if (transX >= 0) {
					x++;
				} else {
					x--;
				}
			}

			if (transY >= 0) {
				y++;
			} else {
				y--;
			}
		}
	}
}

void haz_map_get(haz_map_t *hm, haz_map_tile_t *tile, int x, int y) {
	int i;
	haz_map_tile_t *tTile = &hm->hazMap[y*hm->width + x];
	tile->type = tTile->type;
	tile->val = tTile->val;
	tile->timestamp = tTile->timestamp;
	tile->x = tTile->x;
	tile->y = tTile->y;
	tile->numNeighbors = tTile->numNeighbors;
	tile->type = tTile->type;
	tile->val = tTile->val;
	for (i = 0; i < tile->numNeighbors; i++) {
		tile->neighbors[i] = tTile->neighbors[i];
	}

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

path_t* haz_map_get_path(haz_map_t *hm, double endX, double endY) {
	uint32_t i, checkIndex;
	int64_t startIndex;
	path_t *retPath;
	uint8_t cont = 1;
	dijkstra_dists_t curDist;
	dijkstra_data_t curData, checkData;
	zarray_t *dData, *pq;
	haz_map_tile_t *curTile, *checkTile;
	int adjEndX = ((endX - hm->x) / GRID_RES) + hm->width/2;
	int adjEndY = ((endY - hm->y) / GRID_RES) + hm->height/2;
	//int adjEndX = 0;
	//int adjEndY = 4;
	int startX = hm->width/2;
	int startY = hm->height/2;
	int testindex = adjEndY*hm->width + adjEndX;
	if(testindex >= HAZ_MAP_MAX_WIDTH * HAZ_MAP_MAX_HEIGHT) { printf("testindex %d out of range in djikstra\n", testindex); return;}
	curTile = &hm->hazMap[testindex];
	/*if (curTile->type == HAZ_MAP_OBSTACLE) {
		retPath = malloc(sizeof(path_t));
		retPath->length = retPath->distance = 0;
		return retPath;
	}*/
	//int startX = 0;
	//int startY = 0;

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
			curData.dist = HAZ_MAP_MEGA_DIST;
		}

		zarray_add(dData, &curData);
	}

	curDist.tileIndex = startIndex;
	curDist.dist = 0;

	zarray_add(pq, &curDist);
	double newD;

	while (cont && zarray_size(pq) > 0) {
		uint8_t valid = 0;
		dijkstra_dists_t minDist;
		uint32_t minIndex = 0;
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
			//printf("cx: %d, cy: %d\n", curTile->x, curTile->y);

			// for each neighbor
			//printf("cur tile: %u, %u\n", curTile->x, curTile->y);
			for (i = 0; i < curTile->numNeighbors; i++) {
				checkTile = curTile->neighbors[i].tile;
				//printf("check tile: %u, %u\n", checkTile->x, checkTile->y);
				checkIndex = checkTile->y*hm->width + checkTile->x;
				newD = curData.dist + (checkTile->val * curTile->neighbors[i].distFactor);
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
	int curIndex = adjEndY*hm->width + adjEndX;
	curTile = &hm->hazMap[curIndex];
	zarray_get(dData, curIndex, &curData);
	double distance = 0;
	haz_map_tile_t *prevTile = curTile;
	retPath = malloc(sizeof(path_t));
	retPath->waypoints = malloc(sizeof(position_t) * curData.pathCount);
	retPath->length = curData.pathCount;
	retPath->position = 0;
	for (i = retPath->length - 1; i >= 0; i--) {
		curIndex = curTile->y*hm->width + curTile->x;
		if (curTile != prevTile) {
			if (curTile->x != prevTile->x && curTile->y != prevTile->y) {
				distance += GRID_RES * 1.41421; // GRID_RES * sqrt(2)
			} else {
				distance += GRID_RES;
			}
		}
		if (curIndex != startIndex) {
			zarray_get(dData, curIndex, &curData);
			retPath->waypoints[i].x = (((double)curTile->x - hm->width/2) * GRID_RES) + hm->x;
			retPath->waypoints[i].y = (((double)curTile->y - hm->height/2) * GRID_RES) + hm->y;
			prevTile = curTile;
			curTile = &hm->hazMap[curData.parentIndex];
		} else {
			break;
		}
	}
	retPath->distance = distance;
	zarray_destroy(dData);
	return retPath;
}

void haz_map_cleanup(haz_map_t *hm) {
	uint32_t x, y, n, count;
	time_t now;
	time(&now);
	haz_map_tile_t tile;
	for (y = 0; y < hm->height; y++) {
		for (x = 0; x < hm->width; x++) {
			haz_map_get(hm, &tile, x, y);

			if (difftime(now, tile.timestamp) > HAZ_MAP_TTL) {
				haz_map_set(hm, x, y, HAZ_MAP_UNKNOWN);
			} else if (tile.type == HAZ_MAP_OBSTACLE) {
				count = 0;
				for (n = 0; n < tile.numNeighbors; n++) {
					if (tile.neighbors[n].tile->type == HAZ_MAP_OBSTACLE) {
						count++;
					}
				}

				if (count < 1) {
					haz_map_set(hm, x, y, HAZ_MAP_FREE);
				}
			} else if (tile.type == HAZ_MAP_FREE) {
				tile.val = 1;
				haz_map_set_data(hm, x, y, &tile);
			}
		}
	}
	// If the grid cells immediately touching bruce are unknown, set
	// them as free
	for (y = hm->height/2-2; y < hm->height/2 + 2; y++) {
		for (x = hm->width/2-2; x < hm->width/2 + 2; x++) {
			haz_map_get(hm, &tile, x, y);
			if (tile.type == HAZ_MAP_UNKNOWN) {
				haz_map_set(hm, x, y, HAZ_MAP_FREE);
			}
		}
	}
}

void haz_map_compute_config(haz_map_t *hm) {
	//return;
	int maxU, minU, x, y, u, v, confDist;
	double mapAngle, dist, obstDist, val, offset, obstOffset;
	haz_map_tile_t tile;
	uint8_t tileType;
	int maxV, minV; //count;
	double originX = hm->width / 2;
	double originY = hm->height / 2;
	offset = HAZ_MAP_CONFIG_RAIDUS * 1.0 / GRID_RES;
	obstOffset = HAZ_MAP_OBSTACLE_RADIUS * 1.0 / GRID_RES;
	obstDist = 0;
	confDist = 0;
	for (y = 0; y < hm->height; y++) {
		for (x = 0; x < hm->width; x++) {
			haz_map_get(hm, &tile, x, y);
			if (tile.type == HAZ_MAP_OBSTACLE) {
				maxV = fmin(y + offset, hm->height - 1);
				minV = fmax(y - offset, 0);
				obstDist = sqrt(pow(x - originX, 2) + pow(y - originY, 2));
				for (v = minV; v <= maxV; v++) {
					mapAngle = map(v, minV, maxV, 0, M_PI);
					maxU = fmin(x + offset * sin(mapAngle), hm->width - 1);
					minU = fmax(x - offset * sin(mapAngle), 0);
					//printf("maxU: %d, minU: %d\n", maxU, minU);
					for (u = minU; u <= maxU; u++) {
						dist = sqrt(pow(u - x, 2) + pow(y - v, 2));
						confDist  = sqrt(pow(u - originX, 2) + pow(v - originY, 2));
						if (dist >= obstOffset) {
							if (confDist < obstDist) {
								haz_map_get(hm, &tile, u, v);
								val = pow(offset - dist, 2) + HAZ_MAP_REPULSE_FACTOR;
								if (tile.type == HAZ_MAP_UNKNOWN || (tile.type == HAZ_MAP_FREE && tile.val < val)) {
									tile.type = HAZ_MAP_FREE;
									tile.val = val;
									haz_map_set_data(hm, u, v, &tile);
									haz_map_set_image_data(hm, u, v, &tile);
								}
							}
						} else {
							tile.type = HAZ_MAP_OBSTACLE;
							tile.val = HAZ_MAP_HUGE_DIST;
							haz_map_set_image_data(hm, u, v, &tile);
						}

					}
				}
			} else {
				haz_map_set_image_data(hm, x, y, &tile);
			}
		}
	}
}
