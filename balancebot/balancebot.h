#ifndef BB_H
#define BB_H

#include <stdlib.h>
#include <stdio.h>
#include <unistd.h> // for isatty()
#include <string.h>
#include <math.h> // for M_PI
#include <signal.h>
#include <pthread.h>
#include <rc/mpu.h>
#include <rc/math/quaternion.h>
#include "../common/mb_defs.h"
#include "../common/mb_structs.h"
#include "../common/mb_motor.h"
#include "../common/mb_controller.h"
#include "../common/mb_odometry.h"
#include "../xbee_serial/xbee_receive.h"

#define BALANCE_OFFSET      1*M_PI/180;
#define SOFT_START_TIME     0.1;
#define DT                  0.01;
#define V_NOMINAL           12;
#define BATTERY_CHECK_HZ    0.1;

// inner loop controller 100hz
#define D1_GAIN			1.05
#define D1_ORDER		2
#define D1_NUM			{-4.945, 8.862, -3.967}
#define D1_DEN			{ 1.000, -1.481, 0.4812}
#define D1_NUM_LEN		3
#define D1_DEN_LEN		3
#define D1_SATURATION_TIMEOUT	0.4



// global variables
rc_mpu_data_t mpu_data;
pthread_mutex_t state_mutex;
pthread_mutex_t setpoint_mutex;
mb_state_t mb_state;
mb_setpoints_t mb_setpoints;
mb_odometry_t mb_odometry;

xbee_packet_t xbeeMsg;
int XBEE_portID;

// functions
void balancebot_controller();

//threads
void* setpoint_control_loop(void* ptr);
void* printf_loop(void* ptr);

#endif