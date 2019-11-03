#include "../balancebot/balancebot.h";
#include <math.h>

void mb_tp_init(mb_tp_t* mb_tp, double K_p, double K_a, double K_b){


    mb_tp->k_p = K_p;
    mb_tp->k_a = K_a;
    mb_tp->k_b = K_b;
    mb_tp->lap = 1;
    mb_tp->dest = 1;
    mb_tp->x_dest_arr[] = {1, 1, 0, 0};
    mb_tp->y_dest_arr[] = {0, 1, 1, 0};
    mb_tp->beta_off[] = {0.0, M_PI/2, M_PI, 3*M_PI/2};
    mb_tp->epsilon = 0.01;

}

double bound(double ang, double min, double max){
	if(ang>max){
		ang -= 2*M_PI;
	}else if (ang < min){
		ang += 2*M_PI;
	}
	return ang;
}

void mb_tp_update(mb_tp_t* mb_tp, mb_state_t* mb_state, mb_setpoints_t* mb_setpoints, mb_odometry_t* mb_odometry ){


        double x, y, x_dest, y_dest, p, goal_ang, alpha;
        x_dest = mb_tp.x_dest_arr[mb_tp.dest-1];
        y_dest = mb_tp.y_dest_arr[mp_tp.dest-1];
        x = mb_odometry.x;
        y = mb_odometry.y;
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
            beta = -theta - alpha + mb_tp.beta_off[mb_tp.dest-1];
            mb_setpoints.turn_velocity = K_a*alpha + K_b*beta;
        }else if(mb_tp.lap <= 4){
            mb_tp.dest = (mb_tp.dest + 1) % 4;
            if(mb_tp.dest == 1){
                mb_tp.lap += 1;
            }
        }

        if(mb_tp.lap == 5){
            mb_setpoints.fwd_velocity = 0.0;
            mb_setpoints.turn_velocity = 0.0;
        }


}