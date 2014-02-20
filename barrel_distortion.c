#include "barrel_distortion.h"

static double getCurRadius(int x,int y){
	return sqrt(pow(x,2) + pow(y,2));
}

static double getTheta(int x,int y){
	return atan(y/x);
}

static double getFixedRadius(int x, int y, int C){
	double r = getCurRadius(x, y);

	return 1/(C * pow(r,2) + r);
}

void correctDistortion(image_u32_t* im, int C){
	image_u32_t copy = *im;

	int x, y, newx, newy;
	double theta, r;
	int originx = im->width / 2;
	int originy = im->height /2;
	for(y = 0; y < im->height; y++){
		for(x = 0; x < im->width; x++){
			//Make transparent
			copy.buf[y * copy.stride + x] = 0x0;
		}
	}
	for(y = 0; y < im->height; y++){
		for(x = 0; x < im->width; x++){
			theta = getTheta(x-originx,y-originy);
			r = getFixedRadius(x-originx,y-originy,C);

			newx = r*cos(theta)+originx;
			newy = r*sin(theta)+originy;

			copy.buf[newy * copy.stride + newx] = im->buf[y * im->stride + x];
		}
	}

	*im = copy;
}
