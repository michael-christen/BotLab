#ifndef __ODOMETRY__H__
#define __ODOMETRY__H__
#include "lcmtypes/maebot_motor_feedback_t.h"
#include <math.h>

#define DIST_BETWEEN_WHEELS 8.1
#define TICKS_PER_CM        45.3 

void odometry_handler (
	const lcm_recv_buf_t *rbuf,
	const char * channel,
	const maebot_motor_feedback_t * msg,
	void * data
);

double getDistFromTicks(int32_t ticks);


#endif