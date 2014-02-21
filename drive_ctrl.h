#ifndef __DRIVE_CTRL__H__
#define __DRIVE_CTRL__H__
//For state_t, don't like it
#include "maebot_app.h"
#include "odometry.h"
#include "lcmtypes/maebot_diff_drive_t.h"
#include <pthread.h>
#include <math.h>
#include <float.h>

// dl / dr = (rc - b/2)/(rc + b/2)
// w = speed / rc
//+radius veers to left, - radius veers to right
void driveRad(state_t * state, double radius, double speed);

void driveStraight(state_t * state, double speed);

//w  = speed / Rc
//Rc = 0, so speed would go to 0, but in this case
//we need to view it differently
void driveRot(state_t * state, double omega);

void driveStop(state_t * state);

#endif