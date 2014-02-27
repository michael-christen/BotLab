#include "blob_detection.h"


unsigned int is_ball(double color_threshold,
                     uint32_t template_px,
                     uint32_t px) {
    return color_dist(template_px, px) < color_threshold;
}

unsigned int getNeighbors(image_u32_t *im, int x, int y,
	int neighbors[MAX_NUM_NEIGHBORS],
    uint32_t template_px, double color_threshold) {
    int id;
    unsigned int len = 0;
    uint32_t px;
    neighbors[0] = 0;
    //Searching *'s
    //***
    //*.~
    //~~~
    for(int j = -1; j <= 0; ++j) {
	//edge
	if(j + y < 0 || j + y > im->height) {
	    continue;
	}
	for(int i = -1; i <= 1; ++i) {
	    //edge
	    if(i + x < 0 || i + x > im->width) {
		continue;
	    }
	    //only search *'s
	    if(j >= 0 && i >= 0) {
		break;
	    }
	    id = im->stride*(y+j) + (x+i);
	    px = im->buf[id];
	    if(is_ball(color_threshold,template_px,px)) {
		neighbors[len++] = id;
	    }
	}
    }
    return len;
}

void getNLabels(int n_labels[], int labels[], int neighbors[], int len_neighbors) {
    for(int i = 0; i < len_neighbors; ++i) {
	n_labels[i] = labels[neighbors[i]];
    }
}

int minLabel(int n_labels[], int len_labels) {
    int minLabel = n_labels[0];
    for(int i = 1; i < len_labels; ++i) {
	if(n_labels[i] < minLabel) minLabel = n_labels[i];
    }
    return minLabel;
}

void unionLabels(Set *links[MAX_NUM_BALLS], int n_labels[MAX_NUM_NEIGHBORS],
	int len_neighbors) {
    //Pass over twice
    //1st time gets all set to first neighbor
    //2nd updates rest
    if(links[n_labels[0]]) {
	for(int i = 0; i < 2; ++i) {
	    for(int j = 1; j < len_neighbors; ++j) {
		if(links[n_labels[j]]) {
		    set_union(links[n_labels[0]], links[n_labels[j]]);
		}
	    }
	}
    }
}

int blob_detection(image_u32_t *im, ball_t *final_balls,
                   uint32_t template_px, uint32_t show_px,
                   double color_threshold) {
    //aka max #labels
    //list of links b/t labels
    //Array of Set *
    Set * links [MAX_NUM_BALLS] = {0};
    ball_t balls [MAX_NUM_BALLS];
    int final_num_balls = 0;
    int num_links = 0;
    int label_num;
    //each px has a label, 0 is default
    int labels [im->stride*im->height];
    //Immediate neighbor labels
    int n_labels [MAX_NUM_NEIGHBORS];
    //Immediate neighbor id's
    int neighbors[MAX_NUM_NEIGHBORS];
    int len_neighbors;
    int y, x, id, i;
    uint32_t px;
    label_num = 1;
    //1st pass
    for(y = 0; y < im->height; ++y) {
            for(x = 0; x < im->width; ++x) {
                id = im->stride*y + x;
            px = im->buf[id];
            if(is_ball(color_threshold,template_px,px)){
                len_neighbors = getNeighbors(im, x, y, neighbors,
                                    template_px, color_threshold);
                if(len_neighbors) {
                    getNLabels(n_labels, labels, neighbors,
                        len_neighbors);
                    labels[id] = minLabel(n_labels, len_neighbors);
                    unionLabels(links, n_labels, len_neighbors);
                }
                else {
                    if(label_num < MAX_NUM_BALLS) {
                    labels[id] = label_num;
                    links[label_num] = set_init(label_num);
                    num_links ++;
                    label_num ++;
                    }
                }
            }
            else {
                labels[id] = 0;
            }
        }
    }
    //Init balls
    for(i = 0; i <= label_num; ++i) {
	balls[i].x = 0;
	balls[i].y = 0;
	balls[i].num_px = 0;
    }
    //2nd pass
    for(y = 0; y < im->height; ++y) {
	for(x = 0; x < im->width; ++x) {
	    id = im->stride*y + x;
	    px = im->buf[id];
	    if(is_ball(color_threshold,template_px,px)
           && links[labels[id]]){
		/*
		if(!links[labels[id]]) {
		    printf("id: %d, labels[id]: %d, links[labels[id]]: %d\n",
			    id, labels[id], links[labels[id]]);
		}
		*/
		labels[id] = set_find(links[labels[id]])->val;
		balls[labels[id]].x += x;
		balls[labels[id]].y += y;
		balls[labels[id]].num_px ++;
		im->buf[id] = show_px;
	    }
	}
    }
    //Filter out noise / not enough pixels
    for(i = 1; i < label_num; ++i) {
	if(balls[i].num_px >= MIN_PXS &&
		balls[i].num_px <= MAX_PXS) {
	    final_balls[final_num_balls++] = balls[i];
	}
    }
    //Get coordinates, not sum
    for(i = 0; i < final_num_balls; ++i) {
	final_balls[i].x = (final_balls[i].x+0.0)/
	    final_balls[i].num_px;
	final_balls[i].y = (final_balls[i].y+0.0)/
	    final_balls[i].num_px;
    }
    return final_num_balls;
}
