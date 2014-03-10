#include "barrel_distortion.h"

pixel_t getPixel(double orig_x, double orig_y, double x, double y) {
	pixel_t result;
	double delty = y - orig_y;
	double deltx = x - orig_x;
	double theta = atan2(delty, deltx);
	double curR  = sqrt(pow(deltx,2)+pow(delty,2));
	double newR  = C * pow(curR, 2) + curR;
	result.x     = cos(theta) * newR + orig_x; 
	result.y     = sin(theta) * newR + orig_y;
	if(deltx > 0 && delty < 0){
		
	}
	return result;
}

pixel_t* getLookupTable(int width, int height){

	pixel_t* table = calloc(width * height, sizeof(pixel_t));
	double originx = width / 2.0;
	double originy = height / 2.0;
	int x, y;
	for(y = 0; y < height; y++){
		for(x = 0; x < width; x++){
			table[y * width + x] = getPixel(originx, originy, x, y);
		}
	} 

	return table;
}

void destroyLookupTable(pixel_t* table){
	free(table);
}

void correctDistortion(image_u32_t* im, pixel_t* lookupTable){
	uint32_t* cbuf = calloc(im->stride * im->height, sizeof(uint32_t));

	int x, y;

	pixel_t distp;

	for(y = 0; y < im->height; y++){
		for(x = 0; x < im->width; x++){
			distp = lookupTable[y * im->width + x];
			if(distp.x < 0 || distp.y < 0 || distp.x >= im->width || distp.y >= im->height){
				cbuf[y * im->stride + x] = 0x0;
			}else{
				cbuf[y * im->stride + x] = im->buf[distp.y * im->stride + distp.x];
			}
		}
	}
	free(im->buf);
	im->buf = cbuf;
}
