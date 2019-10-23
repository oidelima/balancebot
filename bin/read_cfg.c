#include <stdio.h>

int main(){

    FILE *cfg;

    cfg = fopen("pid.cfg", "r");

    if (cfg == NULL){
        perror("Error while opening file");
        return -1;
    }

    char line[256];
    int linenum=0;
    while(fgets(line, 256, cfg) != NULL)
    {
            float kp, ki, kd, tf;
            linenum++;
            if(line[0] == '#') continue;

            if(sscanf(line, "%f %f %f %f", &kp, &ki, &kd, &tf) != 4)
            {
                    printf("Syntax error in line %d\n", linenum);
                    continue;
            }

            printf("Line %d:  Kp = %f Ki = %f Kd = %f Tf = %f\n", linenum, kp, ki, kd, tf);
    }

}
