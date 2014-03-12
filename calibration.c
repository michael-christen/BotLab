#include "calibration.h"
//#include "maebot_app.h" //uhh had to do this for state_t but i don't like it
#include <unistd.h>

/*void sensor_handler (const lcm_recv_buf_t *rbuf, const char * channel,
        const maebot_sensor_data_t * msg, void * data)
{
    //do i need to be using mutex?

   state_t * state = data;*/
/*
    printf ("Received message on channel \"%s\":\n", channel);
    printf ("  timestamp   = %lld\n", msg->utime);
    printf ("  acceleration    = (%d, %d, %d)\n",
            msg->accel[0], msg->accel[1], msg->accel[2]);
    printf ("  gyro values = (%d, %d, %d)\n",
            msg->gyro[0], msg->gyro[1], msg->gyro[2]);
    printf ("  gyro_int values= (%lld, %lld, %lld)\n",
            msg->gyro_int[0], msg->gyro_int[1], msg->gyro_int[2]);
    printf ("  line sensor values= (%d, %d, %d)\n",
            msg->line_sensors[0], msg->line_sensors[1], msg->line_sensors[2]);
    printf(" range value = (%d)\n", msg->range);
*/
/*
    for( int i = 0; i < 3; i++){
	state->acc[i]  = msg->accel[i];
	state->gyro[i]  = msg->gyro[i];
	state->gyro_int[i]  = msg->gyro_int[i];
    }

    return;

}*/

void calibrate_gyros(int64_t gyro_int[], double gyro_bias[],
	int64_t gyro_int_offset[]){
//Inputs should be : &state->gyro_int, &state->gyro_bias
//void * data){
	//this is gonna be a call at the start, we let the robot sit and let the gyroscopes accumulate error.
	//find slope of integral...then i think we'll sutbtract that every time we use a gyroscope reading,
	//because it's the expected amount of error


	//again, unsure if i should be using mutex

	//state_t * state = data;

	int64_t orig_0 = gyro_int[0];
	int64_t orig_1 = gyro_int[1];
	int64_t orig_2 = gyro_int[2];
	printf("Calibrating gyros...\n");
	
	double time = 2.5;
	sleep(time);
	gyro_int_offset = gyro_int;

	int64_t biased_0 = gyro_int[0];
	int64_t biased_1 = gyro_int[1];
	int64_t biased_2 = gyro_int[2];
	//printf("After calibration:\nG1: %ld\nG2: %ld\nG3: %ld\n",
	//	   (long)biased_0, (long)biased_1, (long)biased_2);

	//some rise/run stuff
	double slope_0 = (biased_0 - orig_0)/time;
	double slope_1 = (biased_1 - orig_1)/time;
	double slope_2 = (biased_2 - orig_2)/time;

	gyro_bias[0] = slope_0;
	gyro_bias[1] = slope_1;
	gyro_bias[2] = slope_2;

	printf("Gyros calibrated:\n");

	for(int i = 0; i < 3; i++){
	    /*	switch(i){
		case 0: printf("Calc %i: %g\n", i, slope_0); break;
		case 1: printf("Calc %i: %g\n", i, slope_1); break;
		case 2: printf("Calc %i: %g\n", i, slope_2); break;
		}*/
		printf("Gyro %i: %g\n", i, gyro_bias[i]);
	}

	return;
}
