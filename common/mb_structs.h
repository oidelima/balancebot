#ifndef MB_STRUCTS_H
#define MB_STRUCTS_H

typedef struct mb_state mb_state_t;

struct mb_state{
    // raw sensor inputs
    float   theta;             // body angle (rad)
    float   phi;               // average wheel angle (rad)
    int     left_encoder;      // left encoder counts since last reading
    int     right_encoder;     // right encoder counts since last reading

    float   left_torque;
    float   right_torque;

    float   gyro_z;

    //outputs
    float  dutyL;  //left wheel command [-1..1]
    float  dutyR; //right wheel command [-1..1]

    float opti_x;
    float opti_y;
    float opti_roll;
    float opti_pitch;
    float opti_yaw;

    //TODO: Add more variables to this state as needed
    float vBattery;

    float left_w_angle;
    float right_w_angle;

    float SLC_d1_u;
    float SLC_d2_u;
    float SLC_d2_i_u;
    float SLC_d3_u;
    
};

typedef struct mb_setpoints mb_setpoints_t;
struct mb_setpoints{
    float theta;
    float phi;
    float psi;

    float fwd_velocity; // fwd velocity in m/s
    float turn_velocity; // turn velocity in rad/s
    int manual_ctl;
};

typedef struct mb_odometry mb_odometry_t;
struct mb_odometry{

    float x;        //x position from initialization in m
    float y;        //y position from initialization in m
    float psi;      //orientation from initialization in rad
    float phi;

    int last_left_encoder;      // last left encoder reading
    int last_right_encoder;     // last right encoder reading
};

typedef struct ctrl_params ctrl_params_t;
struct ctrl_params{
    float kp;
    float ki;
    float kd;
    float tf;
};

#endif