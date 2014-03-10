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
		/*
		im->buf[id] = dist_to_grey(color_dist(template_px, px));
	    }
    }
	    */
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

	balls[i].t.x = 0;
	balls[i].t.y = 0;

	balls[i].b.x = 0;
	balls[i].b.y = im->height;

	balls[i].l.x = im->width;
	balls[i].l.y = 0;

	balls[i].r.x = 0;
	balls[i].r.y = 0;

	balls[i].valid  = 1;
	balls[i].num_px = 0;
    }
    //2nd pass
    for(y = 0; y < im->height; ++y) {
	for(x = 0; x < im->width; ++x) {
	    id = im->stride*y + x;
	    px = im->buf[id];
	    if(is_ball(color_threshold,template_px,px)
           && links[labels[id]]){
		labels[id] = set_find(links[labels[id]])->val;
		//Add ball data
		balls[labels[id]].x += x;
		balls[labels[id]].y += y;
		balls[labels[id]].num_px ++;
		//Update t,b,l,r
		if(x > balls[labels[id]].r.x) {
		    balls[labels[id]].r.x = x;
		    balls[labels[id]].r.y = y;
		    balls[labels[id]].valid = 1;
		    //If == then not our diamond
		} else if(x == balls[labels[id]].r.x){
		    balls[labels[id]].valid = 0;
		}
		if(x < balls[labels[id]].l.x) {
		    balls[labels[id]].l.x = x;
		    balls[labels[id]].l.y = y;
		    balls[labels[id]].valid = 1;
		} else if(x == balls[labels[id]].l.x){
		    balls[labels[id]].valid = 0;
		}
		if(y > balls[labels[id]].t.y) {
		    balls[labels[id]].t.x = x;
		    balls[labels[id]].t.y = y;
		    balls[labels[id]].valid = 1;
		} else if(y == balls[labels[id]].t.y){
		    balls[labels[id]].valid = 0;
		}
		if(y < balls[labels[id]].b.y) {
		    balls[labels[id]].b.x = x;
		    balls[labels[id]].b.y = y;
		    balls[labels[id]].valid = 1;
		} else if(y == balls[labels[id]].b.y){
		    balls[labels[id]].valid = 0;
		}

		im->buf[id] = show_px;
	    }
	}
    }

    //Filter out non-diamonds
    int err = 2;
    //printf("%d possible diamonds\n",label_num);
    for(i = 1; i < label_num; ++i) {
	//Assert right num_pxs
	int right_num_pxs = (balls[i].num_px >= MIN_PXS &&
		balls[i].num_px <= MAX_PXS);

	if(right_num_pxs) {
	    //Assert top and bottom y within left and right x
	    int top_and_bot_in_lr =
		(balls[i].t.x > balls[i].l.x - err &&
		 balls[i].t.x < err + balls[i].r.x &&
		 balls[i].b.x > balls[i].l.x - err &&
		 balls[i].b.x < err + balls[i].r.x);

	    //Assert top is always inline with bottom
	    int top_inline_bot =
		pixel_width(balls[i].t,balls[i].b) < 3*err;

	    //assert width is <= height
	    int width = pixel_width(balls[i].l,balls[i].r);
	    int height = pixel_height(balls[i].t,balls[i].b);

	    double w_over_h = (width+0.0) / (height+0.0);
	    int width_lte_height = w_over_h <= 1.2 && w_over_h > 0.5;
	    printf("i: %d, px_size: %d, t&b_in_lr: %d, t_inline_b: %d, w<=h: %d, w/h: %f,, height: %d, width: %d\n",
		   i, balls[i].num_px, top_and_bot_in_lr,
		   top_inline_bot, width_lte_height, w_over_h,
		   height, width);

	    if(top_and_bot_in_lr &&
	       //top_inline_bot    &&
	       width_lte_height  &&
	       right_num_pxs) {
		final_balls[final_num_balls] = balls[i];
		//Get coordinates, not sum
		final_balls[final_num_balls].x
		    = (final_balls[final_num_balls].x+0.0)/
		    final_balls[final_num_balls].num_px;
		final_balls[final_num_balls].y =
		    (final_balls[final_num_balls].y+0.0)/
		    final_balls[final_num_balls].num_px;
		printf("%d passed, x: %f, y: %f\n",i,
		       final_balls[final_num_balls].x,
		       final_balls[final_num_balls].y);
		final_num_balls ++;
	    }
	}
    }

    return final_num_balls;
}
