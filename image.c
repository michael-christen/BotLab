#include "image.h"

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
double color_dist(uint32_t p1, uint32_t p2) {
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
