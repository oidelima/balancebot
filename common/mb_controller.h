#ifndef MB_CONTROLLER_H
#define MB_CONTROLLER_H


#include "mb_structs.h"
#define CFG_PATH "pid.cfg"

extern ctrl_params_t body_angle;
extern ctrl_params_t position;
extern ctrl_params_t steering;

int mb_controller_init();
int mb_controller_load_config();
int mb_controller_update(mb_state_t* mb_state);
int mb_controller_cleanup();

#endif

