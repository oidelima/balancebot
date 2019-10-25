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

#include "balancebot.h"

#include <rc/math.h>
rc_filter_t SLC_D1 = RC_FILTER_INITIALIZER;  //body angle controller
rc_filter_t SLC_D2 = RC_FILTER_INITIALIZER;  //position controller 
rc_filter_t SLC_D3 = RC_FILTER_INITIALIZER;	 //steering controller

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

	rc_nanosleep(2E9); // wait for imu to stabilize

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
									
	if(rc_filter_pid(&SLC_D2, position.kp, position.ki, position.kd, position.tf, DT)){
			fprintf(stderr,"ERROR in rc_balance, failed to make filter D1\n");
			return -1;
	}
	SLC_D2.gain = D2_GAIN;
	rc_filter_enable_saturation(&SLC_D2, -0.52, 0.52); 			//need to find the limits for theta - now +/- 30 deg
	rc_filter_enable_soft_start(&SLC_D2, SOFT_START_TIME/DT);

	if(rc_filter_pid(&SLC_D3, steering.kp, steering.ki, steering.kd, steering.tf, DT)){
			fprintf(stderr,"ERROR in rc_balance, failed to make filter D1\n");
			return -1;
	}
	SLC_D3.gain = D3_GAIN;
	rc_filter_enable_saturation(&SLC_D3, -1.0, 1.0); 			
	rc_filter_enable_soft_start(&SLC_D3, SOFT_START_TIME/DT);

	printf("Setting brakes ON!");
	mb_motor_brake(1);

	printf("Inner Loop controller SLC_D1:\n");
	rc_filter_print(SLC_D1);

	printf("Outer Loop position controller SLC_D2:\n");
	rc_filter_print(SLC_D2);
	
	printf("Steering controller SLC_D3:\n");
	rc_filter_print(SLC_D3);

	printf("initializing motors...\n");
	mb_motor_init();

	printf("resetting encoders...\n");
	rc_encoder_eqep_write(1, 0);
	rc_encoder_eqep_write(2, 0);

	printf("initializing odometry...\n");
	mb_odometry_init(&mb_odometry, 0.0,0.0,0.0);

	printf("attaching imu interupt...\n");
	rc_mpu_set_dmp_callback(&balancebot_controller);

	printf("we are running!!!...\n");
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
	mb_state.left_encoder = ENC_2_POL * rc_encoder_eqep_read(2);
	mb_state.right_encoder = ENC_1_POL * rc_encoder_eqep_read(1);

	// Update odometry 
	mb_odometry_update(&mb_odometry, &mb_state);
	
	// collect encoder positions, right wheel is reversed
	mb_state.right_w_angle = (mb_state.right_encoder * 2.0 * M_PI) \
							/(GEAR_RATIO * ENCODER_RES);
	mb_state.left_w_angle = (mb_state.left_encoder * 2.0 * M_PI) \
							/(GEAR_RATIO * ENCODER_RES);

	mb_state.phi = ((mb_state.left_w_angle+mb_state.right_w_angle)/2) + mb_state.theta;
	
	
	// Calculate controller outputs
	if(!mb_setpoints.manual_ctl){
		// Auto Mode
		//send motor commands

	}

	if(mb_setpoints.manual_ctl){
		// Manual Mode
		//send motor commands

		/************************************************************
        * OUTER LOOP PHI controller D2
        * Move the position setpoint based on phi_dot.
        * Input to the controller is phi error (setpoint-state).
        *************************************************************/
        
		if(POSITION_HOLD){
			if(fabs(mb_setpoints.fwd_velocity) > 0.001) mb_setpoints.phi += mb_setpoints.fwd_velocity*DT/(WHEEL_DIAMETER/2);  
		
			mb_state.SLC_d2_u = rc_filter_march(&SLC_D2,-(mb_state.phi-mb_setpoints.phi));
			mb_setpoints.theta = mb_state.SLC_d2_u;
		}   
        else mb_setpoints.theta = 0.0;

		/************************************************************
		* INNER LOOP ANGLE Theta controller D1
		* Input to D1 is theta error (setpoint-state). Then scale the
		* output u to compensate for changing battery voltage.
		*************************************************************/

		// SLC_D1.gain = D1_GAIN * V_NOMINAL/mb_state.vBattery;
		mb_state.SLC_d1_u = rc_filter_march(&SLC_D1,(mb_state.theta-mb_setpoints.theta));

		/**********************************************************
        * Steering controller D3
        * move the setpoint gamma based on user input like phi
        ***********************************************************/
        if(fabs(mb_setpoints.turn_velocity)>0.001) mb_setpoints.psi += mb_setpoints.turn_velocity * DT;
        mb_state.SLC_d3_u = rc_filter_march(&SLC_D3,mb_setpoints.psi - mb_odometry.psi);

		mb_state.dutyL = mb_state.SLC_d1_u - mb_state.SLC_d3_u;
		mb_state.dutyR = mb_state.SLC_d1_u + mb_state.SLC_d3_u;

		mb_motor_set(LEFT_MOTOR, mb_state.dutyL);
		mb_motor_set(RIGHT_MOTOR, mb_state.dutyR);
	}


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

	// setpoints [0 0 0] (theta, phi, psi) for test
	// mb_setpoints.theta = 0;
	mb_setpoints.phi = 0;
	mb_setpoints.psi = 0;

	mb_setpoints.fwd_velocity = 0;
	mb_setpoints.turn_velocity = 0;
	mb_setpoints.manual_ctl = 1;

	// unlock setpoint mutex
	pthread_mutex_unlock(&setpoint_mutex);

	while(1){

		if(rc_dsm_is_new_data()){
				// TODO: Handle the DSM data from the Spektrum radio reciever
				// You may should implement switching between manual and autonomous mode
				// using channel 5 of the DSM data.
		}
	 	rc_nanosleep(1E9 / RC_CTL_HZ);
	}
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
	
	int row = 0;
	int num_var = 19;
	double* M = (double*) malloc(num_var * sizeof(double));

	while(rc_get_state()!=EXITING){
		new_state = rc_get_state();
		// check if this is the first time since being paused
		if(new_state==RUNNING && last_state!=RUNNING){
			printf("\nRUNNING: Hold upright to balance.\n");
			printf("                 SENSORS               										//|            MOCAP            |//");
			printf("\n");
			printf("    θ    |");
			printf("    φ    |");
			printf("    ψ    |");
			printf("  L Enc  |");
			printf("  R Enc  |");
			printf("  L phi  |");
			printf("  R phi  |");
			printf("    X    |");
			printf("    Y    |");
			printf("    ψ    |");
			printf("   D1_u  |");
			printf("  err_θ  |");
			printf("   D2_u  |");
			printf("  err_φ  |");
			printf("  θ_set  |");
			printf("   D3_u  |");
			printf("  err_ψ  |");
			printf(" duty_L  |");
			printf(" duty_R  |");

			printf("\n");
		}
		else if(new_state==PAUSED && last_state!=PAUSED){
			printf("\nPAUSED\n");
		}
		last_state = new_state;

        //Logger init
		char fileName[] = "log.csv";

		if(new_state == RUNNING){

			printf("\r");
			//Add Print stattements here, do not follow with /n
			pthread_mutex_lock(&state_mutex);
			printf("%7.3f  |", mb_state.theta);
			printf("%7.3f  |", mb_state.phi);
			printf("%7.3f  |", mb_odometry.psi);
			printf("%7d  |", mb_state.left_encoder);
			printf("%7d  |", mb_state.right_encoder);
			printf("%7.3f  |", mb_state.left_w_angle);
			printf("%7.3f  |", mb_state.right_w_angle);
			printf("%7.3f  |", mb_state.opti_x);
			printf("%7.3f  |", mb_state.opti_y);
			printf("%7.3f  |", mb_state.opti_yaw);
			printf("%7.3f  |", mb_state.SLC_d1_u);
			printf("%7.3f  |", mb_state.theta-mb_setpoints.theta);
			printf("%7.3f  |", mb_state.SLC_d2_u);
			printf("%7.3f  |", -(mb_state.phi-mb_setpoints.phi));
			printf("%7.3f  |", mb_setpoints.theta);
			printf("%7.3f  |", mb_state.SLC_d3_u);
			printf("%7.3f  |", -(mb_odometry.psi-mb_setpoints.psi));
			printf("%7.3f  |", mb_state.dutyL);
			printf("%7.3f  |", mb_state.dutyR);


			
			//Logger
			double readings[] = {mb_state.theta, mb_state.phi, mb_odometry.psi, mb_state.left_encoder, mb_state.right_encoder,mb_state.left_w_angle,mb_state.right_w_angle,
			    mb_state.opti_x, mb_state.opti_y, mb_state.opti_yaw, mb_state.SLC_d1_u, mb_state.theta-mb_setpoints.theta, mb_state.SLC_d2_u, -(mb_state.phi-mb_setpoints.phi), 
				mb_setpoints.theta, mb_state.SLC_d3_u, -(mb_odometry.psi-mb_setpoints.psi), mb_state.dutyL, mb_state.dutyR};
			M = realloc(M, num_var*(row+1)*sizeof(double));
		    for (int col = 0; col < num_var; col++)
        		*(M + row*num_var + col) = readings[col];
		    row++;
            writeMatrixToFile(fileName, M, row, num_var);

			// printf("%7.3f  |", mb_state.opti_x);
			// printf("%7.3f  |", mb_state.opti_y);
			// printf("%7.3f  |", mb_state.opti_yaw);

			// printf("%7.3f  |", mb_odometry.x);
			// printf("%7.3f  |", mb_odometry.y);
			// printf("%7.3f  |", mb_odometry.psi*180/M_PI);


			// printf("%7.3f  |", mb_state.SLC_d1_u);
			// printf("%7.3f  |", mb_state.theta-mb_setpoints.theta);
			// printf("%7.3f  |", mb_state.vBattery);
			pthread_mutex_unlock(&state_mutex);
			fflush(stdout);
		}
		rc_nanosleep(1E9/PRINTF_HZ);


	}
	return NULL;
} 


int writeMatrixToFile(char* fileName, double* matrix, int height, int width) {
  FILE* fp = fopen(fileName, "w");
  if (fp == NULL) {
	return 1;
  }

  char * headers[] = {"Theta", "Phi", "Psi" "Left encoder", "Right encoder","L phi ", "R phi ", "X", "Y", "Psi", "D1_U", "Error_u","D2_u", "err_φ", "θ_set", "D3_u", "err_ψ", "duty_L", "duty_R" };


  //Printing headers to csv
  for (int j = 0; j < width; j++){
        if (j > 0) {
    	fputc(',', fp);
  	}
	fprintf (fp, "%s", headers[j]);
	}
  fputs("\r\n", fp);


  //Printing values to csv
  for (int i = 0; i < height; i++) {
	for (int j = 0; j < width; j++) {
  	if (j > 0) {
    	fputc(',', fp);
  	}
  	fprintf(fp, "%lf", matrix[i*width +j]);
	}
	fputs("\r\n", fp);
  }
  fclose(fp);
  return 0;
}

static int __arm_controller(void)
{
        __zero_out_controller();
        rc_encoder_eqep_write(1,0);
        rc_encoder_eqep_write(2,0);
        // prefill_filter_inputs(&D1,cstate.theta);
        mb_setpoints.manual_ctl = 1;
        return 0;
}

static int __zero_out_controller(void)
{
        rc_filter_reset(&SLC_D1);
        rc_filter_reset(&SLC_D2);
        //rc_filter_reset(&D3);
        // setpoint.theta = 0.0;
        // setpoint.phi   = 0.0;
        // setpoint.gamma = 0.0;
        // rc_motor_set(0,0.0);
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