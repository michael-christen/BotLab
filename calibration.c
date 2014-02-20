#include "calibration.h"
#include "maebot_app.h" //uhh had to do this for state_t but i don't like it
#include <unistd.h>

void sensor_handler (const lcm_recv_buf_t *rbuf, const char * channel, 
        const maebot_sensor_data_t * msg, void * data)
{
	//do i need to be using mutex?

 	state_t * state = data;

    printf ("Received message on channel \"%s\":\n", channel);
    printf ("  timestamp   = %d\n", msg->utime);
    printf ("  acceleration    = (%d, %d, %d)\n",
            msg->accel[0], msg->accel[1], msg->accel[2]);
    printf ("  gyro values = (%d, %d, %d)\n",
            msg->gyro[0], msg->gyro[1], msg->gyro[2]);
    printf ("  gyro_int values= (%d, %d, %d)\n",
            msg->gyro_int[0], msg->gyro_int[1], msg->gyro_int[2]);
    printf ("  line sensor values= (%d, %d, %d)\n",
            msg->line_sensors[0], msg->line_sensors[1], msg->line_sensors[2]);
	 printf	(" range value = (%d)\n", msg->range);

	 for( int i = 0; i < 3; i++){
		state->acc[i]  = msg->accel[i];
		state->gyro[i]  = msg->gyro[i];
		state->gyro_int[i]  = msg->gyro_int[i];
	 }

	return;

}

void calibrate_gyros(void * data){
	//this is gonna be a call at the start, we let the robot sit and let the gyroscopes accumulate error. 
	//find slope of integral...then i think we'll sutbtract that every time we use a gyroscope reading, 
	//because it's the expected amount of error


	//again, unsure if i should be using mutex	
	
	state_t * state = data;

	int orig_0 = state->gyro_int[0];
	int orig_1 = state->gyro_int[1];
	int orig_2 = state->gyro_int[2];

	//???Do you really think it's necessary to wait 3 minutes
	sleep(180);

	int biased_0 = state->gyro_int[0];
	int biased_1 = state->gyro_int[1];
	int biased_2 = state->gyro_int[2];

	//some rise/run stuff
	double slope_0 = (biased_0 - orig_0)/180.0;
	double slope_1 = (biased_1 - orig_1)/180.0;
	double slope_2 = (biased_2 - orig_2)/180.0;

	state->gyro_0_bias = slope_0;	
	state->gyro_1_bias = slope_1;	
	state->gyro_2_bias = slope_2;	

	return;
}
