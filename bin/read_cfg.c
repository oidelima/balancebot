#include <stdio.h>

#include "read_cfg.h"

data body_angle;
data position;
data steering;

int read_cfg(){

    FILE* file = fopen("pid.cfg", "r");
    if (file == NULL){
        printf("Error opening pid.cfg\n");
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
            printf("Parsing Line %d", linenum);
            if(line[0] == '#') continue;

            if(sscanf(line, "%c %f %f %f %f", &ctrl_id, &kp, &ki, &kd, &tf) != 5)
            {
                    printf("Syntax error in line %d\n", linenum);
                    continue;
            }

            printf("Line %c:  Kp = %f Ki = %f Kd = %f Tf = %f\n", ctrl_id, kp, ki, kd, tf);

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
                default:
                    continue;
                    
            }
            
    }

    fclose(file);
    return 0;
}
