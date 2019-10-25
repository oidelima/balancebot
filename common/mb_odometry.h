/*******************************************************************************
* mb_odometry.h
*
* 
*******************************************************************************/
#ifndef MB_ODOMETRY_H
#define MB_ODOMETRY_H

#include "mb_defs.h"
#include "mb_structs.h"

#define THRESHOLD_DELTA_PSI 0.5

void mb_odometry_init(mb_odometry_t* mb_odometry, float x, float y, float psi);
void mb_odometry_update(mb_odometry_t* mb_odometry, mb_state_t* mb_state);
float mb_clamp_radians(float angle);
int mb_in_range(float num, float min, float max);

#endif