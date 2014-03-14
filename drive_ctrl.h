#ifndef __DRIVE_CTRL__H__
#define __DRIVE_CTRL__H__
//For state_t, don't like it
#include "maebot_app.h"
#include "odometry.h"
#include "path.h"
#include "lcmtypes/maebot_diff_drive_t.h"
#include "pid_ctrl.h"
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

//Drive to a specific theta relative to global coords
void driveToTheta(state_t * state, double theta);

//Rotate the bot a specific theta
void rotateTheta(state_t * state, double theta);

void driveToPosition(state_t * state, position_t position);

double getThetaDist(double from, double to);

double getTheta(double cur_x, double cur_y,
						double new_x, double new_y);

double getDist(double cur_x, double cur_y,
						double new_x, double new_y);

#endif
