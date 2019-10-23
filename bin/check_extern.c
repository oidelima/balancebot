#include <stdio.h>
#include "read_cfg.h"

extern data body_angle;
extern data position;
extern data steering;

int main(){

    if(read_cfg() == -1){
        printf("Error while reading cfg file");
    }

    printf("\nBody Angle Params - %f %f %f %f\n", body_angle.kp,
                                              body_angle.ki,
                                              body_angle.kd,
                                              body_angle.tf);

    printf("Position Params - %f %f %f %f\n", position.kp,
                                            position.ki,
                                            position.kd,
                                            position.tf);    

    printf("Steering Params - %f %f %f %f\n", steering.kp,
                                            steering.ki,
                                            steering.kd,
                                            steering.tf);

    return 0;                                           
}
