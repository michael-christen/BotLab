#include "blob_detection.h"

uint32_t MIN(uint32_t a, uint32_t b, uint32_t c) {
    uint32_t min = a;
    if(b < min) min = b;
    if(c < min) min = c;
    return min;
}

uint32_t MAX(uint32_t a, uint32_t b, uint32_t c) {
    uint32_t max = a;
    if(b > max) max = b;
    if(c > max) max = c;
    return max;
}

void RGBtoHSV( uint32_t r, uint32_t g, uint32_t b, double *h, double *s, double *v)
{
    double min, max, delta;
    min = MIN( r, g, b ) + 0.0;
    max = MAX( r, g, b ) + 0.0;
    *v = max;
    // v
    delta = max - min;
    if( max != 0 ) {
	*s = delta / max;
    }
    // s
    else {
	// r = g = b = 0,
	// s = 0,
	// v is undefined
	*s = 0;
	*h = -1;
	return;
    }
    if( r == max) {
	*h = ( g - b + 0.0) / delta;
    }
    // between yellow & magenta
    else if( g == max) {
	*h = 2 + ( b - r + 0.0) / delta;
    }
    // between cyan & yellow
    else {
	    *h = 4 + ( r - g + 0.0) / delta;
    }
    // between magenta & cyan
    *h *= 60;
    // degrees
    if( *h < 0)
    {
	*h += 360;
    }
}

//Returns distance from test_px to match_px 
unsigned int color_dist(uint32_t p1, uint32_t p2) {
    //Only first 8 bits are used until computation
    uint32_t r1, g1, b1, r2, g2, b2;
    double   h1, s1, v1, h2, s2, v2;
    //uint32_t a1, a2;
    r1 = p1 & 0xFF;
    r2 = p2 & 0XFF;
    g1 = (p1 >> 8) & 0xFF;
    g2 = (p2 >> 8) & 0xFF;
    b1 = (p1 >> 16) & 0XFF;
    b2 = (p2 >> 16) & 0XFF;
    //Can ignore alpha values
//    a1 = (p1 >> 24) & 0xFF;
 //   a2 = (p2 >> 24) & 0xFF;
    //return abs(r1 - r2) + abs(g1 - g2) + abs(b1 - b2);
    RGBtoHSV(r1,g1,b1,&h1,&s1,&v1);
    RGBtoHSV(r2,g2,b2,&h2,&s2,&v2);
    return sqrt(pow((r1 - r2),2) + pow((g1 - g2),2) + pow((b1 - b2),2));
    //return sqrt(pow((h1 - h2),2) + pow((s1 - s2),2) + pow((v1 - v2),2));
//    return abs(h1 - h2) + abs(s1 - s2) + abs(v1 - v2);
}

unsigned int is_ball(uint32_t px) {
    return color_dist(TEMPLATE_PX, px) < COLOR_THRESHOLD;
}

unsigned int getNeighbors(image_u32_t *im, int x, int y, 
	int neighbors[MAX_NUM_NEIGHBORS]) {
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
	    if(is_ball(px)) {
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

int blob_detection(image_u32_t *im, ball_t *final_balls) {
    //aka max #labels
    //list of links b/t labels
    //Array of Set *
    Set * links [MAX_NUM_BALLS];
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
	    if(is_ball(px)){
		len_neighbors = getNeighbors(im, x, y, neighbors);
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
	    if(is_ball(px) && links[labels[id]]){
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
		im->buf[id] = SHOW_PX;
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
