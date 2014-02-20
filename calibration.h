#ifndef __CALIBRATION_H__
#define __CALIBRATION_H__
#include "lcmtypes/maebot_sensor_data_t.h"


static void sensor_handler (const lcm_recv_buf_t *rbuf, const char * channel, 
        const maebot_sensor_data_t * msg, void * data);



#endif
