#include <webots/robot.h>
#include <webots/differential_wheels.h>
#include <webots/distance_sensor.h>
#include <webots/camera.h>
#include <webots/led.h>

#define TIME_STEP 16
#define POLICY_SIZE 23

#define RED 0
#define GREEN 1
#define BLUE 2

#define LEFT 0
#define RIGHT 1

#define NUM_LEDS 4

typedef struct {
	int state;
	int action;
} policy;

policy policyArr[POLICY_SIZE];

int currentState;

double epsilon = 1;

WbDeviceTag sensor0;
WbDeviceTag sensor1;
WbDeviceTag sensor6;
WbDeviceTag sensor7;

WbDeviceTag camera;

WbDeviceTag leds[NUM_LEDS];

int getState();
int chooseAnAction(int state);
int getStateAction(int s);
int getDistance(int state, int similarState);
int getSimilarState(int state);
void initialize();
void performAtion(int actionNumber);
int getDiscreteSensorValue(double sensorValue);
int generateStateValue(double v1, double v2, double v3, double v4, int v5);


int main(int argc, char **argv) {
	/* define variables */

	/* necessary to initialize webots stuff */

	wb_robot_init();
	
	/* Get and enable camera */
	
	camera = wb_robot_get_device("camera");
	wb_camera_enable(camera, TIME_STEP);

	/* Get and enable the distance sensors. */
	
	sensor0 = wb_robot_get_device("ps0");
	wb_distance_sensor_enable(sensor0,TIME_STEP*4);
	sensor1 = wb_robot_get_device("ps1");
	wb_distance_sensor_enable(sensor1,TIME_STEP*4);
	sensor6 = wb_robot_get_device("ps6");
	wb_distance_sensor_enable(sensor6,TIME_STEP*4);
	sensor7 = wb_robot_get_device("ps7");
	wb_distance_sensor_enable(sensor7,TIME_STEP*4);
	
	leds[0] = wb_robot_get_device("led0");
	leds[1] = wb_robot_get_device("led2");
	leds[2] = wb_robot_get_device("led4");
	leds[3] = wb_robot_get_device("led6");
	


	/*printf("s0: %d, s1: %d, s6: %d, s7: %d\n", sensor0, sensor1, sensor6, sensor7);
	printf("Camera: %d\n", camera);*/

	initialize();

	do {
		currentState = getState();
		int action = chooseAnAction(currentState);
		
		//printf("State: %d \t Action: %d\n", currentState, action);
		
		performAtion(action);
		
		wb_robot_step(TIME_STEP);
		
	} while (1);
	
	/* Necessary to cleanup webots stuff */

	wb_robot_cleanup();

	return 0;
}

int getDiscreteSensorValue(double sensorValue) {

	if(sensorValue < 300) {
		return 0;
	}
	if(sensorValue < 1700) {
		return 1;
	}
	if(sensorValue < 3000) {
		return 2;
	}

	return 3;
}

int generateStateValue(double v1, double v2, double v3, double v4, int v5) {

	/*printf("v1: %d, v2: %d, v3: %d, v4: %d, v5: %d\n", getDiscreteSensorValue(v1),
		getDiscreteSensorValue(v2), getDiscreteSensorValue(v3), 
		getDiscreteSensorValue(v4), v5);*/

	return getDiscreteSensorValue(v1) + getDiscreteSensorValue(v2) * 4 
	+ getDiscreteSensorValue(v3) * 16 + getDiscreteSensorValue(v4) * 64 + v5 * 256;
}

int getMaxColor() {
	int i, j;
	const unsigned char *image;
	int red=0, blue=0, green=0;
	int width, height;

	/* This is used to refresh the camera. */
	image = wb_camera_get_image(camera);
	wb_robot_step(TIME_STEP);
	width = wb_camera_get_width(camera);
	height = wb_camera_get_height(camera);
	
	for (i = 0; i < width; i++) {
		/* We read only the lower part of the image to avoid the sky. */
		for (j = 0; j < height; j++) {
			red += wb_camera_image_get_red(image, width, i, j);
			blue += wb_camera_image_get_blue(image, width, i, j);
			green += wb_camera_image_get_green(image, width, i, j);
		}
	}
	
	//printf("Red: %d \t Green: %d \t Blue: %d \n", red, green, blue);
	
	if(red > green) {
		if(red > blue) {
			return RED;
		}
		return BLUE;
	}
	if(green > blue) {
		return GREEN;
	}
	return BLUE;
}

int getState() {
	double s0 = wb_distance_sensor_get_value(sensor0);
	double s1 = wb_distance_sensor_get_value(sensor1);
	double s6 = wb_distance_sensor_get_value(sensor6);
	double s7 = wb_distance_sensor_get_value(sensor7);
	
	wb_robot_step(TIME_STEP);

	return generateStateValue(s0, s1, s6, s7, getMaxColor());
}

int getStateAction(int s)	{
	int i;
	for(i = 0; i < POLICY_SIZE; i++) {
		if(policyArr[i].state == s) {
			return policyArr[i].action;
		}
	}
	return -1;
}

int chooseAnAction(int state){
	int action = getStateAction(state);
	if(action != -1) {
		return action;
	}
	
	int similarState = getSimilarState(state);
	return getStateAction(similarState);
}

int getDistance(int state, int similarState) {
	int auxState = state;
	int firstState = auxState % 4;
	auxState = auxState / 4;
	int secondState = auxState % 4;
	auxState = auxState / 4;
	int thirdState = auxState % 4;
	auxState = auxState / 4;
	int fourthState = auxState % 4;
	auxState = auxState / 4;
	int fifthState = auxState % 4;
	
	int auxSimilar = similarState;
	int firstSimilar = auxSimilar % 4;
	auxSimilar = auxSimilar / 4;
	int secondSimilar = auxSimilar % 4;
	auxSimilar = auxSimilar / 4;
	int thirdSimilar = auxSimilar % 4;
	auxSimilar = auxSimilar / 4;
	int fourthSimilar = auxSimilar % 4;
	auxSimilar = auxSimilar / 4;
	int fifthSimilar = auxSimilar % 4;
	
	int colorDistance = 1000;
	
	if(fifthState == fifthSimilar) {
		colorDistance = 0;
	}
	
	int v1, v2, v3, v4;
	
	if(firstState - firstSimilar >= 0) {
		v1 = firstState - firstSimilar;
	} else {
		v1 = firstSimilar - firstState;
	}
	
	if(secondState - secondSimilar >= 0) {
		v2 = secondState - secondSimilar;
	} else {
		v2 = secondSimilar - secondState;
	}
	
	if(thirdState - thirdSimilar >= 0) {
		v3 = thirdState - thirdSimilar;
	} else {
		v3 = thirdSimilar - thirdState;
	}
	
	if(fourthState - fourthSimilar >= 0) {
		v4 = fourthState - fourthSimilar;
	} else {
		v4 = fourthSimilar - fourthState;
	}
	
	return v1 + v2 + v3 + v4 + colorDistance;
}

int getSimilarState(int state) {
	int i, distance, minDistance;
	int similar = policyArr[0].state;
	minDistance = getDistance(state, policyArr[0].state);;
	for(i = 0; i < POLICY_SIZE; i ++) {
		distance = getDistance(state, policyArr[i].state);
		if(distance < minDistance) {
			similar = policyArr[i].state;
		}
	}
	
	//printf("Similar state: %d\n", similar);
	return similar;
}

void turnForwardTurn(int direction, int forwardTimes) {
	int i;
	int forwardSpeed = 400;
	double speed[2];
	
	if(direction == LEFT) {
		speed[0] = -300;
		speed[1] = 300;
	} else {
		speed[0] = 300;
		speed[1] = -300;
	}
	
	for(i=0; i<70; i++) {
		wb_differential_wheels_set_speed(speed[0],speed[1]);
		wb_robot_step(TIME_STEP);
	}
  
  wb_differential_wheels_set_speed(0,0);
  wb_robot_step(TIME_STEP);
  wb_differential_wheels_set_speed(0,0);
  wb_robot_step(TIME_STEP);
	
	for(i=0; i<forwardTimes; i++) {
		wb_differential_wheels_set_speed(forwardSpeed,forwardSpeed);
		wb_robot_step(TIME_STEP);
	}
  
  wb_differential_wheels_set_speed(0,0);
  wb_robot_step(TIME_STEP);
  wb_differential_wheels_set_speed(0,0);
  wb_robot_step(TIME_STEP);
	
	for(i=0; i<70; i++) {
		wb_differential_wheels_set_speed(-speed[0],-speed[1]);
		wb_robot_step(TIME_STEP);
	}
  
  wb_differential_wheels_set_speed(0,0);
  wb_robot_step(TIME_STEP);
  wb_differential_wheels_set_speed(0,0);
  wb_robot_step(TIME_STEP);
}

void goNearTheBar() {
	double s0 = wb_distance_sensor_get_value(sensor0);
	double s7 = wb_distance_sensor_get_value(sensor7);
	
	while(s0 < 2000 && s7 < 2000) {
		wb_differential_wheels_set_speed(400,400);
		wb_robot_step(TIME_STEP);
		
		s0 = wb_distance_sensor_get_value(sensor0);
		s7 = wb_distance_sensor_get_value(sensor7);
	}
}

void pushTheBar() {
	int i;
	for(i=0; i<10; i++) {
		wb_differential_wheels_set_speed(400,400);
		wb_robot_step(TIME_STEP);
	}
}

void performAtion(int actionNumber) {
	int i;
	int forwardSpeed = 400;
	
	//printf("Action number: %d\n", actionNumber);
	
	// 0 = ir para adelante
	// 1 = ir al rojo
	// 2 = ir al verde
	// 3 = ir al azul
	
	int actualColor = getMaxColor();
	
	for(i = 0; i < NUM_LEDS; i++) {
		wb_led_set(leds[i], 0);
	}
	
	wb_led_set(leds[actualColor], 1);
	wb_robot_step(TIME_STEP);
	
	
	//wb_led_set(leds[actionNumber], 1);
	
	
	if (actionNumber == 0) {
		// seguir derecho
		for(i=0; i<20; i++) {
			wb_differential_wheels_set_speed(forwardSpeed,forwardSpeed);
			wb_robot_step(TIME_STEP);
		}
	} else if(actionNumber == 1) {
		// ir al rojo
		if(actualColor == RED) {
			goNearTheBar();
			pushTheBar();
			return;
		}
		while(getMaxColor() != RED) {
			turnForwardTurn(LEFT, 75);
			goNearTheBar();
		}
		turnForwardTurn(LEFT, 50);
		goNearTheBar();
		pushTheBar();
	} else if(actionNumber == 2) {
		// ir al verde
		if(actualColor == GREEN) {
			goNearTheBar();
			pushTheBar();
			return;
		}
		if(actualColor == RED){
			while(getMaxColor() != GREEN) {
				turnForwardTurn(RIGHT, 30);
				goNearTheBar();
			}
			turnForwardTurn(RIGHT, 30);
			goNearTheBar();
			pushTheBar();
		}
		if(actualColor == BLUE){
			while(getMaxColor() != GREEN) {
				turnForwardTurn(LEFT, 30);
				goNearTheBar();
			}
			turnForwardTurn(LEFT, 30);
			goNearTheBar();
			pushTheBar();
		}
	} else if(actionNumber == 3) {
		// ir al azul
		if(actualColor == BLUE) {
			goNearTheBar();
			pushTheBar();
			return;
		}
		while(getMaxColor() != BLUE) {
			turnForwardTurn(RIGHT, 75);
			goNearTheBar();
		}
		turnForwardTurn(RIGHT, 50);
		goNearTheBar();
		pushTheBar();
	}
}

void initialize() {
policyArr[0].state = 151;
policyArr[0].action = 2;
policyArr[1].state = 155;
policyArr[1].action = 3;
policyArr[2].state = 214;
policyArr[2].action = 0;
policyArr[3].state = 215;
policyArr[3].action = 2;
policyArr[4].state = 226;
policyArr[4].action = 0;
policyArr[5].state = 230;
policyArr[5].action = 3;
policyArr[6].state = 231;
policyArr[6].action = 0;
policyArr[7].state = 395;
policyArr[7].action = 3;
policyArr[8].state = 407;
policyArr[8].action = 3;
policyArr[9].state = 411;
policyArr[9].action = 3;
policyArr[10].state = 470;
policyArr[10].action = 2;
policyArr[11].state = 471;
policyArr[11].action = 2;
policyArr[12].state = 482;
policyArr[12].action = 1;
policyArr[13].state = 486;
policyArr[13].action = 1;
policyArr[14].state = 487;
policyArr[14].action = 1;
policyArr[15].state = 651;
policyArr[15].action = 3;
policyArr[16].state = 663;
policyArr[16].action = 0;
policyArr[17].state = 667;
policyArr[17].action = 0;
policyArr[18].state = 726;
policyArr[18].action = 2;
policyArr[19].state = 727;
policyArr[19].action = 2;
policyArr[20].state = 731;
policyArr[20].action = 2;
policyArr[21].state = 738;
policyArr[21].action = 1;
policyArr[22].state = 742;
policyArr[22].action = 1;
}
