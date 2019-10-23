#ifndef READ_CFG
#define READ_CFG

typedef struct read_cfg 
{
    /* data */
    float kp;
    float ki;
    float kd;
    float tf;

} data;

extern data body_angle;
extern data position;
extern data steering;

int read_cfg();

#endif