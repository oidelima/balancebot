#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/types.h>
#include "mb_controller.h"
#include "mb_defs.h"

ctrl_params_t body_angle;
ctrl_params_t position;
ctrl_params_t steering;

/*******************************************************************************
* int mb_controller_init()
*
* this initializes the controllers from the configuration file
* you can use this as is or modify it if you want a different format
*
* return 0 on success
*
*******************************************************************************/

int mb_controller_init(){
    mb_controller_load_config();
    /* TODO initialize your controllers here*/

    return 0;
}

/*******************************************************************************
* int mb_controller_load_config()
*
* this provides a basic configuration load routine
* you can use this as is or modify it if you want a different format
*
* return 0 on success
*
*******************************************************************************/


int mb_controller_load_config(){
    FILE* file = fopen(CFG_PATH, "r");
    if (file == NULL){
        printf("Error opening %s\n", CFG_PATH );
        return -1;
    }

    char line[300];
    int linenum=0;
    float kp, ki, kd, tf;
    char ctrl_id;

    printf("Parsed Config File as below - ");

    while(fgets(line, 300, file) != NULL)
    {
            kp = 0.0;
            ki = 0.0;
            kd = 0.0;
            tf = 0.0;
            ctrl_id = ' ';

            linenum++;
            if(line[0] == '#') continue;

            if(sscanf(line, "%c %f %f %f %f", &ctrl_id, &kp, &ki, &kd, &tf) != 5)
            {
                    printf("Syntax error in line %d\n", linenum);
                    continue;
            }

            printf("Controller %c:  Kp = %f Ki = %f Kd = %f Tf = %f\n", ctrl_id, kp, ki, kd, tf);

            switch(ctrl_id) {

                case 'B':
                    body_angle.kp = kp;
                    body_angle.ki = ki;
                    body_angle.kd = kd;
                    body_angle.tf = tf;
                    break;
                case 'P':
                    position.kp = kp;
                    position.ki = ki;
                    position.kd = kd;
                    position.tf = tf;
                    break;
                case 'S':
                    steering.kp = kp;
                    steering.ki = ki;
                    steering.kd = kd;
                    steering.tf = tf;
                    break;
                case 'T':
                    RATE_SENST_FWD = kp;
                    RATE_SENST_TURN = kd;
                    TASK = tf;
                default:
                    continue;
                    
            }
            
    }

    fclose(file);
    return 0;

}


/*******************************************************************************
* int mb_controller_update()
* 
* 
* take inputs from the global mb_state
* write outputs to the global mb_state
*
* this should only be called in the imu call back function, no mutex needed
*
* return 0 on success
*
*******************************************************************************/

int mb_controller_update(mb_state_t* mb_state){
    /*TODO: Write your controller here*/
    return 0;
}


/*******************************************************************************
* int mb_controller_cleanup()
* 
* TODO: Free all resources associated with your controller
*
* return 0 on success
*
*******************************************************************************/

int mb_controller_cleanup(){
    return 0;
}