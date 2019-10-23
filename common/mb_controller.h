#ifndef MB_CONTROLLER_H
#define MB_CONTROLLER_H


#include "mb_structs.h"
#define CFG_PATH "pid.cfg"

typedef struct controller_params ctrl_params;
struct controller_params{
    float kp;
    float ki;
    float kd;
    float tf;
};

extern ctrl_params body_angle;
extern ctrl_params position;
extern ctrl_params steering;

int mb_controller_init();
int mb_controller_load_config();
int mb_controller_update(mb_state_t* mb_state);
int mb_controller_cleanup();

#endif

