#include "calibration.h"
#include <unistd.h>
#include <math.h>

void calibrate_gyros(int64_t *gyro2_int, int *calibrating, double *gyro_ticks_per_theta){
//Inputs should be : &state->gyro_int, &state->gyro_bias
//void * data){
	//this is gonna be a call at the start, we let the robot sit and let the gyroscopes accumulate error.
	//find slope of integral...then i think we'll sutbtract that every time we use a gyroscope reading,
	//because it's the expected amount of error

	double sum = 0.0;
	printf("Calibrating rotation gyro...\n");
	printf("There will be 4 tests\n");

	for(int i = 0; i < 4; i++){
		int64_t orig_2 = *gyro2_int;
		printf("Test %d:\n", i);
		printf("Rotate Bruce %sclockwise 720 degrees and press 'c' when done\n", (i < 2 ? "counter-" : ""));
		*calibrating = 1;

		while(*calibrating){
			usleep(1000);
		}

		int64_t total = *gyro2_int - orig_2;
		double test_ticks_per_theta = fabs(total / 4*M_PI);
		sum += test_ticks_per_theta;

		printf("Test %i result: %f gyro ticks per theta\n", i, test_ticks_per_theta);
	}

	*gyro_ticks_per_theta = sum / -40.0;

	printf("Rotation gyro calibrated to %f gyro ticks per theta\n", *gyro_ticks_per_theta);

	return;
}
