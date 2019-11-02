#include "../balancebot/balancebot.h";

void mb_tp_init(mb_tp_t* mb_tp, double K_p, double K_a, double K_b){


    mb_tp->k_p = K_p;
    mb_tp->k_a = K_a;
    mb_tp->k_b = K_b;
    mb_tp->v = 0.0;
    mb_tp->w= 0.0;
    mb_tp->lap = 1;
    mb_tp->dest = 1;

}

void mb_tp_update(mb_tp_t* mb_tp, mb_state_t* mb_state, mb_setpoints_t* mb_setpoints,  x, y, epsilon){

        double x_dest_arr[] = {1, 1, 0, 0};
        double y_dest_arr[] = {0, 1, 1, 0};
        double beta_off[] = {0.0, M_PI/2, M_PI, 3*M_PI/2};



        double x_dest, y_dest, p, goal_ang, alpha;
        x_dest = x_dest_arr[mb_tp.dest-1];
        y_dest = y_dest_arr[mp_tp.dest-1];
        p = sqrt(pow((x - x_dest),2)+pow((y - y_dest),2));
        if (p > epsilon){
            goal_ang = bound(atan2(y_dest - y, x_dest - x), 0 , 2*M_PI);
            alpha = bound(-theta + goal_ang, -M_PI, M_PI);
            if(-M_PI*1.5/2 < alpha && alpha <= M_PI*1.5/2){
                mb_setpoints.fwd_velocity = K_p*p;
            }else{
                goal_ang = bound(atan2(-y_dest + y, -x_dest + x), 0 , 2*M_PI);
                alpha = -theta + goal_ang;
                mb_setpoints.fwd_velocity = -K_p*p;
            }
            beta = -theta - alpha + beta_off[mb_tp.dest-1];
            mb_setpoints.turn_velocity = K_a*alpha + K_b*beta;
            theta = bound(mb_odometry.psi;, 0, 2*M_PI);
            x = mb_odometry.x;
            y = mb_odometry.y;
        }




}