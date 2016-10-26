#include <webots/robot.h>
#include <webots/differential_wheels.h>
#include <webots/distance_sensor.h>
#include <stdio.h>

/*
 * Macros
 */

#define TIME_STEP 8

#define BIAS_SPEED 400

#define THRESHOLD 150

#define LEFT_FOLLOW	0
#define RIGHT_FOLLOW	1

#if 1
  #define PRINT_(a,b) printf (a,b)
#else
  #define PRINT_(a,b) (void)0
#endif

/* Britenberg coeficients */
double follow_weightleft[8] = {-4.5,-3.75,0,0,0,-0,3.75,4.5};
double follow_weightright[8] = {4.5,3.75,-0,0,0,0,-3.75,-4.5};

/*
 * Helper functions
 */

int selectFollowDirection(int sensors_value[8]) {

	int i;
	int leftValues = 0;
	int rightValues = 0;

	for(i = 0; i < 8; i++) {
		if(i <=3) {
			rightValues += sensors_value[i];
		} else {
			leftValues += sensors_value[i];
		}
	}

	if(leftValues > rightValues) {
		return LEFT_FOLLOW;
	}

	return RIGHT_FOLLOW;
} 

/*
 * Main program.
 */

int main(int argc, char **argv) {

	/* Variables */

	WbDeviceTag distance_sensor[8];
	int i;
	double speed[2];
	int sensors_value[8];
	int followDirection;
	
	/* Necessary to initialize webots stuff */

	wb_robot_init();

	/* Get and enable the distance sensors. */

	for (i = 0; i < 8; i++) {

		char device_name[4];

		/* Get distance sensors */

		sprintf(device_name, "ps%d", i);
		distance_sensor[i] = wb_robot_get_device(device_name);
		wb_distance_sensor_enable(distance_sensor[i],TIME_STEP*2);
	}

	/* Main loop */

	do {

		speed[0]=BIAS_SPEED;
		speed[1]=BIAS_SPEED;

		/* Get sensors values */

		for (i = 0; i < 8; i++) {
			sensors_value[i] = wb_distance_sensor_get_value(distance_sensor[i]);
		}
    
    /* Necessary when the robot enters an acute angle */
    double weightValue = 4.5;
    followDirection = selectFollowDirection(sensors_value);
    if(followDirection == RIGHT_FOLLOW) {
    	follow_weightleft[0] = -weightValue;
    	follow_weightleft[7] = -weightValue;
    	follow_weightright[0] = weightValue;
    	follow_weightright[7] = weightValue;
    } else {
    	follow_weightleft[0] = weightValue;
    	follow_weightleft[7] = weightValue;
    	follow_weightright[0] = -weightValue;
    	follow_weightright[7] = -weightValue;
    }
    
    /* Calculate wheels speed */
		for (i=0; i<8; i++) {
			speed[0] += follow_weightleft[i] * (sensors_value[i] - THRESHOLD);
			speed[1] += follow_weightright[i] * (sensors_value[i] - THRESHOLD);
		}
	
		wb_differential_wheels_set_speed(speed[0],speed[1]);

	} while (wb_robot_step(TIME_STEP) != -1);
	
	/* Enter here exit cleanup code */

	/* Necessary to cleanup webots stuff */

	wb_robot_cleanup();

	return 0;
}
