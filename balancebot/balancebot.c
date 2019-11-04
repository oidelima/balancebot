/*******************************************************************************
* balancebot.c
*
* Main template code for the BalanceBot Project
* based on rc_balance
* 
*******************************************************************************/

#include <math.h>
#include <rc/start_stop.h>
#include <rc/adc.h>
#include <rc/servo.h>
#include <rc/mpu.h>
#include <rc/dsm.h>
#include <rc/cpu.h>
#include <rc/bmp.h>
#include <rc/button.h>
#include <rc/led.h>
#include <rc/pthread.h>
#include <rc/encoder_eqep.h>
#include <rc/time.h>
#include <stdlib.h>
#include <time.h>

#include "balancebot.h"

#include <rc/math.h>
rc_filter_t SLC_D1 = RC_FILTER_INITIALIZER;  //body angle controller
rc_filter_t SLC_D2 = RC_FILTER_INITIALIZER;  //position controller
rc_filter_t SLC_D2_i = RC_FILTER_INITIALIZER; //position integral control
rc_filter_t SLC_D3 = RC_FILTER_INITIALIZER;	 //steering controller

rc_filter_t le_lpf = RC_FILTER_INITIALIZER;
rc_filter_t re_lpf = RC_FILTER_INITIALIZER;

double le, re; //output from LPFs


/*******************************************************************************
* int main() 
*
*******************************************************************************/
int main(){
	// make sure another instance isn't running
    // if return value is -3 then a background process is running with
    // higher privaledges and we couldn't kill it, in which case we should
    // not continue or there may be hardware conflicts. If it returned -4
    // then there was an invalid argument that needs to be fixed.
    if(rc_kill_existing_process(2.0)<-2) return -1;

	// start signal handler so we can exit cleanly
    if(rc_enable_signal_handler()==-1){
        fprintf(stderr,"ERROR: failed to start signal handler\n");
        return -1;
    }

	if(rc_cpu_set_governor(RC_GOV_PERFORMANCE)<0){
        fprintf(stderr,"Failed to set governor to PERFORMANCE\n");
        return -1;
    }

	// initialize enocders
    if(rc_encoder_eqep_init()==-1){
        fprintf(stderr,"ERROR: failed to initialize eqep encoders\n");
        return -1;
    }

    // initialize adc
    if(rc_adc_init()==-1){
        fprintf(stderr, "ERROR: failed to initialize adc\n");
        return -1;
    }

	//start dsm connection 
    if(rc_dsm_init()==-1){
		fprintf(stderr,"failed to start initialize DSM\n");
		return -1;
	}

	// printf("initializing xbee... \n");
	// //initalize XBee Radio
	// int baudrate = BAUDRATE;
	// if(XBEE_init(baudrate)==-1){
	// 	fprintf(stderr,"Error initializing XBee\n");
	// 	return -1;
	// };

    // make PID file to indicate your project is running
	// due to the check made on the call to rc_kill_existing_process() above
	// we can be fairly confident there is no PID file already and we can
	// make our own safely.
	rc_make_pid_file();

	// start printf_thread if running from a terminal
	// if it was started as a background process then don't bother

	

	
	printf("starting print thread... \n");
	pthread_t  printf_thread;
	rc_pthread_create(&printf_thread, printf_loop, (void*) NULL, SCHED_OTHER, 0);

	// start control thread
	printf("starting setpoint thread... \n");
	pthread_t  setpoint_control_thread;
	rc_pthread_create(&setpoint_control_thread, setpoint_control_loop, (void*) NULL, SCHED_FIFO, 50);

	// start battery_checker thread
	printf("starting battery_checker thread... \n");
	pthread_t battery_thread;
	if(rc_pthread_create(&battery_thread, __battery_checker, (void*) NULL, SCHED_OTHER, 0)){
                fprintf(stderr, "failed to start battery thread\n");
                return -1;
	}

	// TODO: start motion capture message recieve thread

	// set up IMU configuration
	printf("initializing imu... \n");
	// set up mpu configuration
	rc_mpu_config_t mpu_config = rc_mpu_default_config();
	mpu_config.dmp_sample_rate = SAMPLE_RATE_HZ;
	mpu_config.orient = ORIENTATION_Z_DOWN;
	mpu_config.dmp_fetch_accel_gyro=1;

	// now set up the imu for dmp interrupt operation
	if(rc_mpu_initialize_dmp(&mpu_data, mpu_config)){
		printf("rc_mpu_initialize_failed\n");
		return -1;
	}

	rc_nanosleep(3E9); // wait for imu to stabilize

	//initialize state mutex
    pthread_mutex_init(&state_mutex, NULL);
    pthread_mutex_init(&setpoint_mutex, NULL);

	//attach controller function to IMU interrupt
	printf("initializing controllers...\n");
	mb_controller_init();
									
	if(rc_filter_pid(&SLC_D1, body_angle.kp, body_angle.ki, body_angle.kd, body_angle.tf, DT)){
			fprintf(stderr,"ERROR in rc_balance, failed to make filter D1\n");
			return -1;
	}
	SLC_D1.gain = D1_GAIN;
	rc_filter_enable_saturation(&SLC_D1, -1.0, 1.0);
	rc_filter_enable_soft_start(&SLC_D1, SOFT_START_TIME/DT);
									
	if(rc_filter_pid(&SLC_D2, position.kp, 0.0, position.kd, position.tf, DT)){
			fprintf(stderr,"ERROR in rc_balance, failed to make filter D1\n");
			return -1;
	}
	SLC_D2.gain = D2_GAIN;
	rc_filter_enable_saturation(&SLC_D2, -0.15, 0.15); 			//need to find the limits for theta - now +/- 30 deg
	rc_filter_enable_soft_start(&SLC_D2, SOFT_START_TIME/DT);

	if(rc_filter_pid(&SLC_D2_i, 0.0, position.ki, 0.0, position.tf, DT)){
			fprintf(stderr,"ERROR in rc_balance, failed to make filter D1\n");
			return -1;
	}
	SLC_D2_i.gain = D2_GAIN;
	rc_filter_enable_saturation(&SLC_D2_i, -0.05, 0.05); 			//need to find the limits for theta - now +/- 30 deg
	rc_filter_enable_soft_start(&SLC_D2_i, SOFT_START_TIME/DT);

	if(rc_filter_pid(&SLC_D3, steering.kp, steering.ki, steering.kd, steering.tf, DT)){
			fprintf(stderr,"ERROR in rc_balance, failed to make filter D1\n");
			return -1;
	}
	SLC_D3.gain = D3_GAIN;
	rc_filter_enable_saturation(&SLC_D3, -1.0, 1.0); 			
	rc_filter_enable_soft_start(&SLC_D3, SOFT_START_TIME/DT);

	// Initialize LPFs for encoders
	rc_filter_first_order_lowpass(&le_lpf, DT, TIME_CONSTANT);
	rc_filter_first_order_lowpass(&re_lpf, DT, TIME_CONSTANT);

	// Prefill filters
	rc_filter_prefill_inputs(&SLC_D1, 0.0);
	rc_filter_prefill_inputs(&SLC_D2, 0.0);
	rc_filter_prefill_inputs(&SLC_D2_i, 0.0);
	rc_filter_prefill_inputs(&SLC_D3, 0.0);
	rc_filter_prefill_inputs(&le_lpf, 0.0);
	rc_filter_prefill_inputs(&re_lpf, 0.0);
	rc_filter_prefill_outputs(&SLC_D1, 0.0);
	rc_filter_prefill_outputs(&SLC_D2, 0.0);
	rc_filter_prefill_outputs(&SLC_D2_i, 0.0);
	rc_filter_prefill_outputs(&SLC_D3, 0.0);
	rc_filter_prefill_outputs(&le_lpf, 0.0);
	rc_filter_prefill_outputs(&re_lpf, 0.0);


	printf("Inner Loop controller SLC_D1:\n");
	rc_filter_print(SLC_D1);

	printf("Outer Loop position controller SLC_D2:\n");
	rc_filter_print(SLC_D2);
	
	printf("Steering controller SLC_D3:\n");
	rc_filter_print(SLC_D3);

	printf("initializing motors...\n");
	mb_motor_init();

	printf("Setting brakes ON!\n");
	mb_motor_brake(1);

	printf("resetting encoders...\n");
	rc_encoder_eqep_write(1, 0);
	rc_encoder_eqep_write(2, 0);


	printf("initializing odometry...\n");
	mb_odometry_init(&mb_odometry, 0.1,2.0,-0.1);



	printf("attaching imu interupt...\n");
	rc_mpu_set_dmp_callback(&balancebot_controller);

	printf("\nwe are running!!!...\n");
	// done initializing so set state to RUNNING
	rc_set_state(RUNNING); 

	// Keep looping until state changes to EXITING
	while(rc_get_state()!=EXITING){

		// all the balancing is handled in the imu interupt function
		// other functions are handled in other threads
		// there is no need to do anything here but sleep
		// always sleep at some point
		rc_nanosleep(1E9);
	}
	
	// exit cleanly
	rc_filter_free(&SLC_D1);
	rc_filter_free(&SLC_D2);
	rc_filter_free(&SLC_D3);
	rc_mpu_power_off();
	mb_motor_cleanup();
	rc_led_cleanup();
	rc_dsm_cleanup();
	rc_encoder_eqep_cleanup();
	rc_remove_pid_file(); // remove pid file LAST 
	return 0;
}


/*******************************************************************************
* void balancebot_controller()
*
* discrete-time balance controller operated off IMU interrupt
* Called at SAMPLE_RATE_HZ
*
* TODO: You must implement this function to keep the balancebot balanced
* 
*******************************************************************************/
void balancebot_controller(){
	//lock state and setpoint mutex
	pthread_mutex_lock(&state_mutex);
	pthread_mutex_lock(&setpoint_mutex);

	// Read IMU
	mb_state.theta = mpu_data.dmp_TaitBryan[TB_PITCH_X] - BALANCE_OFFSET;
	
	mb_state.gyro_z = mpu_data.gyro[2]*M_PI/180.0;

	// Read encoders
	mb_state.left_encoder = rc_filter_march(&le_lpf, ENC_2_POL * rc_encoder_eqep_read(LEFT_MOTOR));
	mb_state.right_encoder = rc_filter_march(&re_lpf, ENC_1_POL * rc_encoder_eqep_read(RIGHT_MOTOR));
	// mb_state.left_encoder = ENC_2_POL * rc_encoder_eqep_read(LEFT_MOTOR);
	// mb_state.right_encoder = ENC_1_POL * rc_encoder_eqep_read(RIGHT_MOTOR);


	// Update odometry 
	mb_odometry_update(&mb_odometry, &mb_state);

	// Read motor current
	mb_state.left_torque = mb_motor_read_current(LEFT_MOTOR) /0.5;
	mb_state.right_torque = mb_motor_read_current(RIGHT_MOTOR) /0.5;


	// collect encoder positions, right wheel is reversed
	mb_state.right_w_angle = (mb_state.right_encoder * 2.0 * M_PI) \
							/(GEAR_RATIO * ENCODER_RES);
	mb_state.left_w_angle = (mb_state.left_encoder * 2.0 * M_PI) \
							/(GEAR_RATIO * ENCODER_RES);

	mb_state.phi = ((mb_state.left_w_angle+mb_state.right_w_angle)/2) + mb_state.theta;
	// mb_state.phi = mb_odometry.phi/(WHEEL_DIAMETER/2) + mb_state.theta;
	// mb_state.phi = mb_odometry.phi/(WHEEL_DIAMETER/2);

	/************************************************************
	* OUTER LOOP PHI controller D2
	*************************************************************/
	
	if(POSITION_HOLD){
		// if(fabs(mb_setpoints.fwd_velocity) > 0.001) mb_setpoints.phi += mb_setpoints.fwd_velocity*DT/(WHEEL_DIAMETER/2);  
		
		// reset integral control each time setpoint is reached
		if (fabs(mb_state.phi-mb_setpoints.phi) < 0.01){
			rc_filter_reset(&SLC_D2_i);
		}

		mb_state.SLC_d2_i_u = rc_filter_march(&SLC_D2_i,-(mb_state.phi-mb_setpoints.phi));
		mb_state.SLC_d2_u = rc_filter_march(&SLC_D2,-(mb_state.phi-mb_setpoints.phi));
		mb_setpoints.theta = mb_state.SLC_d2_u + mb_state.SLC_d2_i_u;
	}   
	else mb_setpoints.theta = 0.0;


	/************************************************************
	* INNER LOOP Body Angle Theta controller
	*************************************************************/

	mb_state.SLC_d1_u = rc_filter_march(&SLC_D1,(mb_state.theta-mb_setpoints.theta));

	/**********************************************************
	* Steering controller D3
	***********************************************************/
	// if(fabs(mb_setpoints.turn_velocity)>0.001) mb_setpoints.psi += mb_setpoints.turn_velocity * DT/WHEEL_BASE;
	mb_state.SLC_d3_u = rc_filter_march(&SLC_D3,mb_setpoints.psi - mb_odometry.psi);

	mb_state.dutyL = mb_state.SLC_d1_u - mb_state.SLC_d3_u;
	mb_state.dutyR = mb_state.SLC_d1_u + mb_state.SLC_d3_u;

	mb_motor_set(LEFT_MOTOR, mb_state.dutyL);
	mb_motor_set(RIGHT_MOTOR, mb_state.dutyR);

	// XBEE_getData();
	// double q_array[4] = {xbeeMsg.qw, xbeeMsg.qx, xbeeMsg.qy, xbeeMsg.qz};
	// double tb_array[3] = {0, 0, 0};
	// rc_quaternion_to_tb_array(q_array, tb_array);
	// mb_state.opti_x = xbeeMsg.x;
	// mb_state.opti_y = -xbeeMsg.y;	    //xBee quaternion is in Z-down, need Z-up
	// mb_state.opti_roll = tb_array[0];
	// mb_state.opti_pitch = -tb_array[1]; //xBee quaternion is in Z-down, need Z-up
	// mb_state.opti_yaw = -tb_array[2];   //xBee quaternion is in Z-down, need Z-up
	
	// unlock setpoint mutex
	pthread_mutex_unlock(&setpoint_mutex);
	//unlock state mutex
	pthread_mutex_unlock(&state_mutex);
}


/*******************************************************************************
*  setpoint_control_loop()
*
*  sets current setpoints based on dsm radio data, odometry, and Optitrak
*
*
*******************************************************************************/
void* setpoint_control_loop(void* ptr){
	// lock setpoint mutex
	pthread_mutex_lock(&setpoint_mutex);

	double fwd_input, turn_input;
	static int sp_loop_id;
	static int done = 0;
	static int swtch = 0;
	static double start_psi, start_phi, start_x, start_y;

	static int T2_turn, T2_round;
	static int T2_state = 0; // 0 for turning, 1 for trun done, 2 for going straight, 3 for straight done
	static double pre_phi;

	// unlock setpoint mutex
	pthread_mutex_unlock(&setpoint_mutex);

	while(1){
		if(rc_dsm_is_new_data()){
				// TODO: Handle the DSM data from the Spektrum radio reciever
				// You may should implement switching between manual and autonomous mode
				// using channel 5 of the DSM data.

				if(rc_dsm_ch_normalized(ARM_CH)<0.1){
					mb_setpoints.manual_ctl = 1;
					swtch = 0;
				}
				else {
					mb_setpoints.manual_ctl = 0;
					if(!swtch){
						start_phi = mb_state.phi;
						start_psi = mb_odometry.psi;
						start_x = mb_odometry.x;
						start_y = mb_odometry.y;
						pre_phi = mb_odometry.phi;

						T2_state = 2;
						swtch = 1;
					}
				}

				if(!mb_setpoints.manual_ctl){
				// Auto Mode
				//send motor commands
					switch(TASK){
						case 2:
							if(done == 0){
								switch(T2_state){
									case 0:
										// set psi setpoint +90
										mb_setpoints.psi -= M_PI/2;
										T2_state = 1;
										break;
									case 1:
										// check turn done?
										if(fabs(mb_odometry.psi-mb_setpoints.psi) < 0.02){
											T2_turn += 1;
											// check task done?
											if(T2_turn == 1){
												T2_round += 1;
												T2_turn = 0;
												if(T2_round == 4){
													done = 1;
												}
											}
											// into state2
											// printf("\n~~~Case 1 done~~~~\n");
											T2_state = 2;
										}
										break;
									case 2:
										// set phi adding 1m
										if(fabs(mb_odometry.phi-pre_phi) < 0.9  && fabs(mb_setpoints.phi - (pre_phi+1)/(WHEEL_DIAMETER/2)<0.1)){
											// mb_setpoints.fwd_velocity = 0.5*RATE_SENST_FWD;
											mb_setpoints.fwd_velocity = 0.5*RATE_SENST_FWD;
										}else{
											mb_setpoints.fwd_velocity = 0;
											mb_setpoints.phi = (pre_phi + 1)/(WHEEL_DIAMETER/2); //
											pre_phi = mb_setpoints.phi*(WHEEL_DIAMETER/2);
											T2_state = 3;
										}
										break;
									case 3:
										// check finish straight?
										// printf("TASK 2: Case 3\n");

										if(fabs(mb_odometry.phi/(WHEEL_DIAMETER/2)-mb_setpoints.phi) < 0.05){
											// printf("\n~~~Case 3 done~~~~\n");
											T2_state = 0;
										}
										break;
								}
							}
							break;

						case 3:
							if(mb_state.phi - start_phi >= 11.2/(WHEEL_DIAMETER/2) || done == 1){
								mb_setpoints.fwd_velocity = 0.0;
								// mb_setpoints.theta = 0.0;    		
								done = 1;
							}
							else {
								mb_setpoints.fwd_velocity = 0.5 * RATE_SENST_FWD;   //adjust normalized fwd velocity based on speed/timing/theta .... 
								// mb_setpoints.theta = 0.15;			find value for theta to set if u want to use this
							}
							break;
						default:
							continue;
					}

					if(fabs(mb_setpoints.fwd_velocity) > 0.001) mb_setpoints.phi += mb_setpoints.fwd_velocity*DT/(WHEEL_DIAMETER/2);
					if(fabs(mb_setpoints.turn_velocity) > 0.001) mb_setpoints.psi += mb_setpoints.turn_velocity*DT/WHEEL_BASE;

				}

				if(mb_setpoints.manual_ctl){
				// Manual Mode
					fwd_input = rc_dsm_ch_normalized(FWD_CH) * FWD_POL;
					turn_input = rc_dsm_ch_normalized(TURN_CH) * TURN_POL;

					rc_saturate_double(&fwd_input, -1.0, 1.0);
					rc_saturate_double(&turn_input, -1.0, 1.0);

					if(fabs(fwd_input)<DEAD_ZONE) fwd_input = 0.0;
					if(fabs(turn_input)<DEAD_ZONE) turn_input = 0.0;

					mb_setpoints.fwd_velocity = fwd_input * RATE_SENST_FWD;
					mb_setpoints.turn_velocity = turn_input * RATE_SENST_TURN;

					if(fabs(mb_setpoints.fwd_velocity) > 0.001) mb_setpoints.phi += mb_setpoints.fwd_velocity*DT/(WHEEL_DIAMETER/2);
					if(fabs(mb_setpoints.turn_velocity)>0.001) mb_setpoints.psi += mb_setpoints.turn_velocity*DT/WHEEL_BASE;
				}

		}
		else if(rc_dsm_is_connection_active()==0){
			mb_setpoints.theta = 0;
			mb_setpoints.fwd_velocity = 0;
			mb_setpoints.turn_velocity = 0;
			mb_setpoints.manual_ctl = 1;
			continue;
		}

	 	rc_nanosleep(1E9 / RC_CTL_HZ);
	}
	sp_loop_id++;
	return NULL;
}




/*******************************************************************************
* printf_loop() 
*
* prints diagnostics to console
* this only gets started if executing from terminal
*
* TODO: Add other data to help you tune/debug your code
*******************************************************************************/
void* printf_loop(void* ptr){
	rc_state_t last_state, new_state; // keep track of last state

	char fileName[45];
	sprintf(fileName, "%d", (int)time(NULL));
	strcat(fileName, "_log.csv");
	FILE* fp = fopen(fileName, "a");
	int num_var = 25;
	/*char * headers[] = {"MODE", "Phi set", "Psi set", "Theta", "Phi","Psi",  "Left encoder", "Right encoder", "Left w angle", "Right w angle",
        "X", "Y","Yaw",  "Error theta", "D2_u","Error phi","Theta set","D3_u" , "Error phi", "Duty L", "Duty R"};

  	//Printing headers to csv
	  for (int j = 0; j < num_var; j++){
		if (j > 0) {
	    	fputc(',', fp);
	  	}
		fprintf (fp, "%s", headers[j]);
		}
	fputs("\r\n", fp);*/
    


	while(rc_get_state()!=EXITING){
		new_state = rc_get_state();
		// check if this is the first time since being paused
		if(new_state==RUNNING && last_state!=RUNNING){
			printf("\nRUNNING: Hold upright to balance.\n");
			printf(" DSM | SETPOINTS |                            STATES								|            MOCAP            |");
			printf("\n");
			printf(" MODE|");
			printf("  θ_set  |");
			printf("  φ_set  |");
			printf("  ψ_set  ||  ||");

			printf("    θ    |");
			printf("    φ    |");			
			// printf("  L Enc  |");
			// printf("  R Enc  |");
			// printf("  L phi  |");
			// printf("  R phi  |");
			// printf("    X    |");
			// printf("    Y    |");
			// printf("  ODO φ  |");
			printf("    ψ    ||  ||");

			printf("  err_θ  |");
			printf("   D2_u  |");
			printf("  err_φ  |");
			printf("  θ_set  |");
			printf("   D3_u  |");
			printf("  err_ψ  |");
			printf(" duty_L  |");
			printf(" duty_R  |");

			// printf(" tau_L   | ");
			// printf(" tau_R   |");

			printf("\n");
		}
		else if(new_state==PAUSED && last_state!=PAUSED){
			printf("\nPAUSED\n");
		}
		last_state = new_state;



		if(new_state == RUNNING){

			printf("\r");
			//Add Print stattements here, do not follow with /n
			pthread_mutex_lock(&state_mutex);
			if(mb_setpoints.manual_ctl) printf("  M  |");
			else printf("  A  |");
			printf("%7.3f  |", mb_setpoints.theta/M_PI*180);
			printf("%7.3f  |", mb_setpoints.phi);
			printf("%7.3f  ||  ||", mb_setpoints.psi);

			printf("%7.3f  |", mb_state.theta/M_PI*180);
			printf("%7.3f  |", mb_state.phi);
			// printf("%7.3f  |", mb_odometry.psi);
			// printf("%7d  |", mb_state.left_encoder);
			// printf("%7d  |", mb_state.right_encoder);
			// printf("%7.3f  |", mb_state.left_w_angle);
			// printf("%7.3f  |", mb_state.right_w_angle);
			// printf("%7.3f  |", mb_odometry.x);
			// printf("%7.3f  |", mb_odometry.y);
			// printf("%7.3f  |", mb_odometry.phi/(WHEEL_DIAMETER/2));
			printf("%7.3f  ||  ||", mb_odometry.psi);

			// printf("%7.3f  |", mb_state.SLC_d1_u);
			printf("%7.3f  |", (mb_state.theta-mb_setpoints.theta)/M_PI*180);
			printf("%7.3f  |", mb_state.SLC_d2_u);
			printf("%7.3f  |", -(mb_state.phi-mb_setpoints.phi));
			printf("%7.3f  |", mb_setpoints.theta);
			printf("%7.3f  |", mb_state.SLC_d3_u);
			printf("%7.3f  |", -(mb_odometry.psi-mb_setpoints.psi));

			printf("%7.3f  |", mb_state.dutyL);
			printf("%7.3f  |", mb_state.dutyR);
			// printf("%7.3f  |", mb_state.left_torque);
			// printf("%7.3f  |", mb_state.right_torque);


			 //Logger


            double readings[] = {mb_setpoints.manual_ctl, mb_setpoints.phi, mb_setpoints.psi, mb_state.theta, mb_state.phi, mb_odometry.psi, mb_state.left_encoder, mb_state.right_encoder, mb_state.left_w_angle,mb_state.right_w_angle,
                 mb_state.opti_x, mb_state.opti_y, mb_state.opti_yaw, mb_state.theta-mb_setpoints.theta, mb_state.SLC_d2_u, -(mb_state.phi-mb_setpoints.phi),
             	mb_setpoints.theta, mb_state.SLC_d3_u, -(mb_odometry.psi-mb_setpoints.psi), mb_state.dutyL, mb_state.dutyR, mb_odometry.x, mb_odometry.y, mb_odometry.psi, mb_state.gyro_z};

             writeMatrixToFile(fp, readings,  num_var);


			pthread_mutex_unlock(&state_mutex);
			fflush(stdout);
		}
		rc_nanosleep(1E9/PRINTF_HZ);


	}
	fclose(fp);
	return NULL;
} 


int writeMatrixToFile(FILE* fp, double matrix[], int num_var) {
  
  //Printing values to csv
  for (int i = 0; i < num_var; i++) {

  	if (i > 0) {
    	fputc(',', fp);
  	}
  	fprintf(fp, "%lf", matrix[i]);
	}
  fputs("\r\n", fp);

  
  return 0;
}

void* __battery_checker(void* ptr)
{
        double new_v;
        while(rc_get_state()!=EXITING){
                new_v = rc_adc_batt();
                // if the value doesn't make sense, use nominal voltage
                // if (new_v>14.0 || new_v<10.0) new_v = V_NOMINAL;
                mb_state.vBattery = new_v;
                rc_usleep(1000000 / BATTERY_CHECK_HZ);
        }
        return NULL;
}
