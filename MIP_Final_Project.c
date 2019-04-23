/*******************************************************************************
*Alex Tran
*A11747930 
*Final Project!!!!!!
*
* This code is meant to balance the MIP...
*******************************************************************************/

// usefulincludes is a collection of common system includes for the lazy
// This is not necessary for roboticscape projects but here for convenience
#include <rc_usefulincludes.h> 
// main roboticscape API header
#include <roboticscape.h>
#include "balance_config.h"

//Global Variables
rc_imu_data_t imu_data;

//Complementary filter struct
typedef struct filt_out_t {
	float theta_fx;
	float theta_g_rawx;
	float theta_a_rawx;
	float last_theta_ax, last_theta_a_rawx, theta_ax;
	float last_theta_gx, last_theta_g_rawx, theta_gx;
} filt_out_t;

filt_out_t filt;

// Keep track of important system state variables
typedef struct state_t{
	float theta;  		 //body angle
	float phi;   		//wheel angular position
	float x;     	 	//ground referenced position
	float u1;			//motor duty cycle
	float u2;     		//theta_ref from outer loop
	float time_count;
} sys_state_t;

// Struct to hold important setpoint parameters for outer and inner loop
typedef struct setpoint_t{
	float theta_ref;  	 // MIP body angle
	float theta_err;
	float phi_ref;  	//wheel angular position
	float phi_err;
} setpoint_t;

//more global structs
state_t state;
rc_imu_data_t imu_data;
setpoint_t setpoint;

// function declarations
void on_pause_pressed();
void on_pause_released();
void inner_loop();
void* print_data();

//int looptime = 1000; //us main loop sleep period
/*******************************************************************************
* int main() 
*
* This template main function contains these critical components
* - call to rc_initialize() at the beginning
* - main while loop that checks for EXITING condition
* - rc_cleanup() at the end
*******************************************************************************/
int main(){
	// always initialize cape library first
	if(rc_initialize()){
		fprintf(stderr,"ERROR: failed to initialize rc_initialize(), are you root?\n");
		return -1;
	}

	// do your own initialization here
	printf("\nMAE 144 Robotics Final Project\n");

	rc_set_pause_pressed_func(&on_pause_pressed);
	rc_set_pause_released_func(&on_pause_released);

	rc_imu_config_t conf = rc_default_imu_config(); //initialize rc config struct
	conf.dmp_sample_rate = INNER_LOOP_HZ; //set DMP interrupt sample rate


	if(rc_initialize_imu_dmp(&imu_data, conf)){
		fprintf(stderr,"rc_initialize_imu_failed\n");
		return -1;
	}

	//imu dmp runs this interrupt function every .01s.  
	//calculates the theta estimate and inner_loop controller difference equation
	rc_set_imu_interrupt_func(&inner_loop);

	//print data thread
	pthread_t print_thread;
	pthread_create(&print_thread, NULL, print_data, (void*) NULL);

	//outer loop thread
	pthread_t outer_loop_thread;
	pthread_create(&outer_loop_thread, NULL, outer_loop, (void*) NULL);

	// done initializing so set state to RUNNING
	rc_set_state(RUNNING); 

	// Keep looping until state changes to EXITING
	while(rc_get_state()!=EXITING){
		// handle other states
		if(rc_get_state()==RUNNING){
			// do things

			//printf("\r");
			
			//printf(" %10f %10f %10f\n", theta_a_raw, theta_g_raw, theta_f);
		}

		else if(rc_get_state()==PAUSED){
			// do other things
			rc_set_led(GREEN, OFF);
			rc_set_led(RED, ON);
		}
		// always sleep at some point
		rc_usleep(1000);
	}
	
	pthread_join(print_thread, NULL);
	pthread_join(outer_loop_thread, NULL);

	// exit cleanly
	rc_cleanup(); 
	return 0;
}


/*******************************************************************************
* void on_pause_released() 
*	
* Make the Pause button toggle between paused and running states.
*******************************************************************************/
void on_pause_released(){
	printf("pause released???\n");

	// toggle betewen paused and running modes
	if(rc_get_state()==RUNNING)		rc_set_state(PAUSED);
	else if(rc_get_state()==PAUSED)	rc_set_state(RUNNING);
	return;
}

/*******************************************************************************
* void on_pause_pressed() 
*
* If the user holds the pause button for 2 seconds, set state to exiting which 
* triggers the rest of the program to exit cleanly.
*******************************************************************************/
void on_pause_pressed(){
	printf("pause pressed???\n");
	int i=0;
	const int samples = 100;	// check for release 100 times in this period
	const int us_wait = 2000000; // 2 seconds

	// now keep checking to see if the button is still held down
	for(i=0;i<samples;i++){
		rc_usleep(us_wait/samples);
		if(rc_get_pause_button() == RELEASED){
			return;
		}
	}
	printf("long press detected, shutting down\n");
	rc_set_state(EXITING);
	return;
}

void* print_data(void* ptr){
	while(rc_get_state()!=EXITING){
		// handle other states
		if(rc_get_state()==RUNNING){
			//printf("\r");
			printf(" %7.5f |", state.theta);
			printf(" %7.5f |", setpoint.theta_ref);
			//printf(" %7.5f |", state.phi);
			//printf(" %7.5f |", setpoint.phi_ref);
			//printf(" %7.5f ", state.x);
			printf(" %7.5f ", state.u1);
			//printf(" %7.5f ", state.u2);
			printf("\n");
			//fflush(stdout);
		}
		else if(rc_get_state()==PAUSED){
			// do other things
			rc_set_led(GREEN, OFF);
			rc_set_led(RED, ON);
		}
		rc_usleep(50000);
	}
	return NULL;
}


void inner_loop(){
	//read raw IMU data, create struct with filtered data??

    const float h = .01;
	const float tc = .1;
    const float wc = 1/tc;
	const float b0 = h*wc;
	const float a0 = (wc*h-1)*-1;
	static int y_accel, z_accel, y_gyro, z_gyro, x_gyro;
	static float theta_f;
	static float theta_g_raw = 0;
	static float theta_a_raw = 0;
	static float last_theta_a, last_theta_a_raw, theta_a;
	static float last_theta_g, last_theta_g_raw, theta_g;

	if(rc_read_accel_data(&imu_data)<0){
		printf("read accel data failed!");
	}
	y_accel = imu_data.raw_accel[1]*ACCEL_RAW_TO_MS2;
	z_accel = imu_data.raw_accel[2]*ACCEL_RAW_TO_MS2;

	if(rc_read_gyro_data(&imu_data)<0){
        printf("read gyro data failed!");
    }

	x_gyro = imu_data.raw_gyro[0]*GYRO_RAW_TO_RADS;
	//printf("accel-y: %6d accel-z: %6d", y_accel, z_accel);
	theta_a_raw = -atan2(z_accel, y_accel);
	theta_g_raw += x_gyro*h;
	//printf("accel angle raw: %5f gyro angle raw: %5f\n", theta_a_raw, theta_g_raw);

	/******LOW PASS FILTER for accelerometer*****/
	theta_a = a0*last_theta_a + b0*last_theta_a_raw;
	last_theta_a = theta_a;
	last_theta_a_raw = theta_a_raw;

	/******HIGH PASS FILTER for gyroscope*****/
	theta_g  = a0*last_theta_g + theta_g_raw - last_theta_g_raw;
	last_theta_g = theta_g;
	last_theta_g_raw = theta_g_raw;
	//printf("%f %f\n", a0, b0);
	//printf(" theta_g_filtered: %10f\n", theta_g);

	state.theta = theta_g+theta_a;
	filt.theta_g_rawx = theta_g_raw;
	filt.theta_a_rawx = theta_a_raw;
	//printf(" %10f %10f %10f\n", theta_a_raw, theta_g_raw, theta_f);

	//turn on motors from standby
	rc_enable_motors(MOTOR_CH_L);
	rc_enable_motors(MOTOR_CH_R);

	/********************** D1 DIFFERENCE EQUATION *************************/

	
	// Declare local Variables
	static float u1[3], theta_err[3]; //difference equation variables
	static float D1_num[] = D1_NUM;
	static float D1_den[] = D1_DEN;

	setpoint.theta_ref = -CAPE_OFFSET_ANGLE; //state.u2;

	theta_err[0] = setpoint.theta_ref - state.theta;

	//D1 Control Difference Equation
	u1[0] = (D1_den[1] * u1[1]) - (D1_den[2] * u1[2]) + D1_K*(-1 * D1_num[0] * theta_err[0] + D1_num[1] * theta_err[1] - D1_num[2] * theta_err[2])/D1_den[0];

	// Update past values
	theta_err[2] = theta_err[1];
	theta_err[1] = theta_err[0];
	u1[2] = u1[1];
	u1[1] = u1[0];

	// Update global variables
	setpoint.theta_err = theta_err[0];
	state.u1 = u1[0];

	//limit duty cycle
	if (state.u1 > MAX_DUTY_CYCLE) {
		state.u1 = MAX_DUTY_CYCLE;
	}
	else if (state.u1 < MIN_DUTY_CYCLE) {
		state.u1 = MIN_DUTY_CYCLE;
	}
	
	//send motor drive signal w/ output duty cycle!!!  INPUT INTO REAL WORLD PLANT!!!!!!!! FUKKKKK 
	rc_set_motor(MOTOR_CH_R, MOTOR_POLARITY_R * state.u1);
	rc_set_motor(MOTOR_CH_L, MOTOR_POLARITY_L * state.u1);

	return; //of the jedi
}

void* outer_loop(void* ptr){
	while(rc_get_state()!=EXITING){
		if(rc_get_state() == RUNNING){

			//Local Variables
			static float u2[3], phi_err[3]; //difference equation variables
			static float L_wheel, R_wheel, wheel_avg;
			static float D2_num[] = D2_NUM;
			static float D2_den[] = D2_DEN;

			//Estimate Wheel position
			L_wheel = (rc_get_encoder_pos(ENC_CH_R)* 2*PI)/(ENC_POLARITY_R*ENC_STEPS*GEAR_RATIO);
			R_wheel = (rc_get_encoder_pos(ENC_CH_L)* 2*PI)/ (ENC_POLARITY_L*ENC_STEPS*GEAR_RATIO);

			//calculate avg wheel pos
			wheel_avg = 0.5*(wheel_L + wheel_R);

			//Update Global vars
			state.wheel_L_RAD = L_wheel;
			state.wheel_R_RAD = L_wheel;
			state.phi = wheel_avg;
			state.x = wheel_avg * WHEEL_RADIUS;

			//calculate current error from setpoint
			phi_err[0] = setpoint.phi_ref - state.phi;

			//D2 Control difference equation
			u2[0] = (-D2_den[1]*u2[1] - D2_den[2]*u2[2] + D2_num[0]*phi_err[0] - D2_num[1]*phi_err[1] + D2_num[2]*phi_err[2] + D2_num[3]*phi_err[3])/D2_den[0];

			//Update past values
			u2[2] = u2[1];
			u2[1] = u2[0];
			phi_err[2] = phi_err[1];
			phi_err[1] = phi_err[0];

			//update global vars
			state.u2 = u2[0];
			setpoint.phi_err = phi_err[0];
		}
	
		rc_usleep(1e6/OUTER_LOOP_HZ);
	}
	return NULL; //NULL pointer
}



