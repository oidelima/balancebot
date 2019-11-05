/*******************************************************************************
* mb_odometry.c
*
* TODO: Implement these functions to add odometry functionality 
*
*******************************************************************************/

#include "../balancebot/balancebot.h"

void mb_odometry_init(mb_odometry_t* mb_odometry, float x, float y, float psi){
/* TODO */
    mb_odometry->x = x;
    mb_odometry->y = y;
    mb_odometry->psi = psi;
}

void mb_odometry_update(mb_odometry_t* mb_odometry, mb_state_t* mb_state){
/* TODO */
    float encoder2distant = 2*M_PI/ENCODER_RES/GEAR_RATIO;

    float delta_SR = (mb_state->right_encoder - mb_odometry->last_right_encoder)*encoder2distant*WHEEL_R_DIA/2;
    float delta_SL = (mb_state->left_encoder - mb_odometry->last_left_encoder)*encoder2distant*WHEEL_L_DIA/2;
    mb_odometry->last_right_encoder = mb_state->right_encoder;
    mb_odometry->last_left_encoder = mb_state->left_encoder;

    float alpha = (delta_SR - delta_SL)/WHEEL_BASE;
    float delta_S = (delta_SR + delta_SL)/2;

    float delta_psi_gyro = mb_state->gyro_z/SAMPLE_RATE_HZ;
    // printf("\nalpha:  %7.3f |", alpha);
    // printf("gyro :    %7.3f  |", delta_psi_gyro);
    // printf("delta:    %7.3f ", fabsf(delta_psi_gyro - alpha));
    if(fabsf(delta_psi_gyro - alpha) > THRESHOLD_DELTA_PSI){
        alpha = delta_psi_gyro;
        // printf("\ndelta_psi explode!!\n");
    }


    float delta_x = delta_S*cos(mb_odometry->psi + alpha/2);
    float delta_y = delta_S*sin(mb_odometry->psi + alpha/2);

    mb_odometry->x += delta_x;
    mb_odometry->y += delta_y;
    mb_odometry->psi += alpha;
    mb_odometry->phi += delta_S;

    // mb_odometry->psi = mb_clamp_radians(mb_odometry->psi);
}


float mb_clamp_radians(float angle){
    float clamped = angle;

    clamped -= 2*M_PI * floor((clamped+M_PI)/(2*M_PI));

    return clamped;
}