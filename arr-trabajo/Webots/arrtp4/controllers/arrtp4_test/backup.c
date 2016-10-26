#include <webots/robot.h>
#include <webots/differential_wheels.h>
#include <webots/distance_sensor.h>
#include <stdio.h>
#include <stdlib.h>

#define TIME_STEP 16
#define POLICY_SIZE 9

typedef struct p {
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

int getState();
int chooseAnAction(int state);
int getStateAction(int s);
int getDistance(int state, int similarState);
int getSimilarState(int state);
void initialize();
void performAtion(int actionNumber);
int getDiscreteSensorValue(double sensorValue);
int generateStateValue(double v1, double v2, double v3, double v4);


int main(int argc, char **argv) {


	/* define variables */

	/* necessary to initialize webots stuff */

	wb_robot_init();

	/* Get and enable the distance sensors. */
	
	sensor0 = wb_robot_get_device("ps0");
  wb_distance_sensor_enable(sensor0,TIME_STEP*4);
	sensor1 = wb_robot_get_device("ps1");
	wb_distance_sensor_enable(sensor1,TIME_STEP*4);
	sensor6 = wb_robot_get_device("ps6");
	wb_distance_sensor_enable(sensor6,TIME_STEP*4);
	sensor7 = wb_robot_get_device("ps7");
	wb_distance_sensor_enable(sensor7,TIME_STEP*4);

	initialize();

	do {
		currentState = getState();
		int action = chooseAnAction(currentState);
		printf("ASDFASDFASDFASDFSADF\n");
		performAtion(action);
		printf("WWWWWWWWWWWWWw\n");
	} while (1);
	
	/* Necessary to cleanup webots stuff */

	wb_robot_cleanup();

	return 0;
}

int getDiscreteSensorValue(double sensorValue) {

	if(sensorValue < 500) {
		return 0;
	}
	if(sensorValue < 1500) {
		return 1;
	}
	if(sensorValue < 2500) {
		return 2;
	}

	return 3;
}

int generateStateValue(double v1, double v2, double v3, double v4) {
	return getDiscreteSensorValue(v1) + getDiscreteSensorValue(v2) * 4 + getDiscreteSensorValue(v3) * 16 + getDiscreteSensorValue(v4) * 64;
}

int getState() {
	double v0 = wb_distance_sensor_get_value(sensor0);
	double v1 = wb_distance_sensor_get_value(sensor1);
	double v6 = wb_distance_sensor_get_value(sensor6);
	double v7 = wb_distance_sensor_get_value(sensor7);
	return generateStateValue(v0, v1, v6, v7);
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
	
	int auxSimilar = similarState;
	int firstSimilar = auxSimilar % 4;
	auxSimilar = auxSimilar / 4;
	int secondSimilar = auxSimilar % 4;
	auxSimilar = auxSimilar / 4;
	int thirdSimilar = auxSimilar % 4;
	auxSimilar = auxSimilar / 4;
	int fourthSimilar = auxSimilar % 4;
	
	return (firstState - firstSimilar) + (secondState - secondSimilar) 
		+ (thirdState - thirdSimilar) + (fourthState - fourthSimilar);
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
	return similar;
}

void performAtion(int actionNumber) {

	double speed[2];
	if(actionNumber == 0) {
	// seguir derecho
		speed[0] = 400;
		speed[1] = 400;
	} else if(actionNumber == 1) {
	// girar 45 a la izquierda
		speed[0] = -800;
		speed[1] = 800;
	} else if(actionNumber == 2) {
	// girar 45 a la derecha
		speed[0] = 800;
		speed[1] = -800;
	} else if(actionNumber == 3) {
	// girar 10 a la izquierda
		speed[0] = -300;
		speed[1] = 300;
	} else if(actionNumber == 4) {
	// girar 10 a la derecha
		speed[0] = 300;
		speed[1] = -300;
	}
	
	int i;

	for(i=0; i<5; i++) {
		wb_differential_wheels_set_speed(speed[0],speed[1]);
		wb_robot_step(TIME_STEP);
	}

	int forwardSpeed = 400;

	for(i=0; i<3; i++) {
		wb_differential_wheels_set_speed(forwardSpeed,forwardSpeed);
		wb_robot_step(TIME_STEP);
	}
	
	
}

void initialize() {
policyArr[0].state = 139;
policyArr[0].action = 2;
policyArr[1].state = 143;
policyArr[1].action = 2;
policyArr[2].state = 199;
policyArr[2].action = 3;
policyArr[3].state = 203;
policyArr[3].action = 4;
policyArr[4].state = 215;
policyArr[4].action = 0;
policyArr[5].state = 219;
policyArr[5].action = 0;
policyArr[6].state = 226;
policyArr[6].action = 1;
policyArr[7].state = 227;
policyArr[7].action = 3;
policyArr[8].state = 231;
policyArr[8].action = 1;
policyArr[9].state = 242;
policyArr[9].action = 1;
}
