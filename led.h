#ifndef LED_H
#define LED_H

#include "maebot_app.h"

#define RED 0x100000
#define GREEN 0x001000
#define BLUE 0x000010
#define PURPLE 0x100010
#define YELLOW 0x101000
#define ORANGE 0x100100
#define CYAN 0x001010
#define WHITE 0x101010
#define OFF 0x0

typedef enum{
NONE,
MOVE_FORWARD,
MOVE_BACKWARD,
TURN_LEFT,
TURN_RIGHT,
LASER_ON
} Status;

void setLeds(state_t* state, uint32_t left, uint32_t right){
	state->led.top_rgb_led_left = left;
	state->led.top_rgb_led_right = right;
}

void LEDStatus(state_t* state, Status stat){
	pthread_mutex_lock(&state->led_mutex);
	switch(stat){
		case MOVE_FORWARD:
			setLeds(state, BLUE, BLUE);	
			break;
		case MOVE_BACKWARD:
			setLeds(state, ORANGE, ORANGE);
			break;
		case TURN_LEFT:
			setLeds(state, ORANGE, BLUE);
			break;
		case TURN_RIGHT:
			setLeds(state, ORANGE, BLUE);
			break;
		case LASER_ON:
			setLeds(state, RED, RED);
			break;
		default:
			setLeds(state, OFF, OFF);
	}
	pthread_mutex_unlock(&state->led_mutex);
}

#endif
