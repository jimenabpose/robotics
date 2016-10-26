#include <webots/robot.h>
#include <webots/differential_wheels.h>
#include <webots/distance_sensor.h>
#include <stdio.h>
#include <stdlib.h>

#define TIME_STEP 16

// Q_SIZE = 6*3^0 + 6*3^1 + 6*3^2  + 6*3^3
#define Q_SIZE 256
#define ACTIONS_SIZE 5

const double gama = 0.9;
const double alpha = 0.1;
int iterations = 10;

double Q[Q_SIZE][ACTIONS_SIZE];
int currentState;

WbDeviceTag distance_sensor[8];
double sensors_value[8];
double previous_sensors_value[8];

int rewardDifference = 1000;

int currentState;

double epsilon = 1;
  
int getState();
int chooseAnAction();
double maximum(int newState);
void updateQ(int action, int reward, int newState);
void initialize();
int getReward(int newState);
void updateEpsilon();
void performAtion(int actionNumber);
void printQ();
int getDiscreteSensorValue(int sensorValue);
int generateStateValue(int v1, int v2, int v3, int v4);

 
int main(int argc, char **argv)
{
	/* define variables */

	int i;

	/* necessary to initialize webots stuff */

	wb_robot_init();

	/* Get and enable the distance sensors. */

	for (i = 0; i < 8; i++) {

		char device_name[4];

		/* get distance sensors */

		sprintf(device_name, "ps%d", i);
		distance_sensor[i] = wb_robot_get_device(device_name);
		wb_distance_sensor_enable(distance_sensor[i],TIME_STEP*4);

		previous_sensors_value[i] = wb_distance_sensor_get_value(distance_sensor[i]);
	}

	initialize();

	do {

		currentState = getState();

		int action = chooseAnAction();
		performAtion(action);
		int reward = getReward(getState());

		printf("state: %d, action: %d, reward: %d, epsilon: %g\n", currentState, action, reward, epsilon);
		
		updateQ(action, reward, getState());
		updateEpsilon();
		//printQ();

		for (i = 0; i < 8; i++) {
			previous_sensors_value[i] = sensors_value[i];
		}

	} while (true);
	
	/* Enter here exit cleanup code */
  
	/* Necessary to cleanup webots stuff */

	wb_robot_cleanup();

	return 0;
}

int getDiscreteSensorValue(int sensorValue) {

	if(sensorValue < 200) {
		return 0;
	}
	if(sensorValue < 500) {
		return 1;
	}
	if(sensorValue < 800) {
		return 2;
	}

	return 3;
}

int generateStateValue(int v1, int v2, int v3, int v4) {
	return getDiscreteSensorValue(v1) + getDiscreteSensorValue(v2) * 4 + getDiscreteSensorValue(v3) * 16 + getDiscreteSensorValue(v4) * 64;
}

int getState() {

	int i;

	/* get sensors values */

	for (i = 0; i < 8; i++) {
		sensors_value[i] = wb_distance_sensor_get_value(distance_sensor[i]);
	}

	return generateStateValue(sensors_value[0], sensors_value[5], sensors_value[6], sensors_value[7]);
}

int chooseAnAction(){
//En este caso, el agente elige para un estado dado,
//la acción que maximiza el estimado de Q(s,a) con probabilidad (1-ε)
//cualquier acción al azar con probabilidad ε.

	double random = (double)rand() / RAND_MAX;
  
	if(random > (1 - epsilon)) {
		// devuelvo uno random
		int action_random = (int)(((double)rand() / RAND_MAX) * 5);
		if(action_random == 5) {
			return 4;
		}
		return action_random;
	} else {
		// devuelvo el maximo
		int i;
		int maxAction = 0;
		for(i = 0; i < ACTIONS_SIZE; i ++) {
			if(Q[currentState][i] > Q[currentState][maxAction]) {
				maxAction = i;
			}
		}

		return maxAction;
	}
}

double maximum(int newState){
	int i;
  int maxAction = 0;
	for(i = 0; i < ACTIONS_SIZE; i ++) {
		if(Q[newState][i] > Q[newState][maxAction]) {
			maxAction = i;
		}
	}
	return Q[newState][maxAction];
}

void updateQ(int action, int reward, int newState) {
	//Q(s, a) ← Q(s, a) + α [r + γ max a’ Q(s’, a’) - Q(s, a)]
//Q[newState][action] = Q[newState][action] + action * (reward + gama * maximum(newState));
  Q[currentState][action] = Q[currentState][action] + alpha * (reward + gama * maximum(newState));
}

void initialize() {
	int i, j;
	for(i = 0; i < Q_SIZE; i++) {
		for(j = 0; j < ACTIONS_SIZE; j++) {
			Q[i][j] = 0;
		}
	}
}

int getReward(int newState){
	/*int rewardMin = 8;
	int rewardMax = 20;
	if(currentState > rewardMin && currentState < rewardMax) {
		return 1;
	}
  return 0;*/
  
  /*int reward = 2;
  
  if(sensors_value[7] > 200 || sensors_value[0] > 200) {
  	reward -= 1;
  }
  
  if(sensors_value[5] < 200 || sensors_value[5] > 800) {
  	reward -= 1;
  }
  
  if(sensors_value[6] > sensors_value[5]) {
  	reward -= 1;
  }
  
  return reward;*/
  
  
  int leftMaxDistance = 2500;
  int leftMinDistance = 500;
  
  int frontMaxDistance = 500;
  
  int leftDifference = 400;
  
  int diagonalMinDistance = 2000;
  
  if(sensors_value[5] > leftMinDistance && sensors_value[5] < leftMaxDistance) {
  	
  	if(sensors_value[0] > frontMaxDistance || sensors_value[7] > frontMaxDistance) {
  		return -2;
  	}
		
		if((previous_sensors_value[0] > frontMaxDistance && sensors_value[0] < frontMaxDistance)
			|| (previous_sensors_value[7] > frontMaxDistance && sensors_value[7] < frontMaxDistance)) {
			return 1;
		}
  	
  	if((previous_sensors_value[5] - sensors_value[5]) > leftDifference
  		|| (previous_sensors_value[5] - sensors_value[5]) < -leftDifference) {
  		return 0;
		}
		
		return 3;
  }
  
  if(sensors_value[6] > diagonalMinDistance || sensors_value[1] > frontMaxDistance) {
  	return -3;
  }
  
  if(previous_sensors_value[6] > diagonalMinDistance && sensors_value[6] < diagonalMinDistance) {
		return 1;
	}
	
	if(previous_sensors_value[1] > frontMaxDistance && sensors_value[1] < frontMaxDistance) {
		return 1;
	}
  
  return -2;
}

void updateEpsilon() {
  if(epsilon >= 0.01) {
    epsilon = epsilon - 0.001;
  } else {
    epsilon = 0;
  }
}

void performAtion(int actionNumber) {
  double speed[2];
	if(actionNumber == 0) {
	// seguir derecho
		speed[0] = 0;
		speed[1] = 0;
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

	for(i=0; i<2; i++) {
		wb_differential_wheels_set_speed(speed[0],speed[1]);
		wb_robot_step(TIME_STEP);
	}

	speed[0] = 400;
	speed[1] = 400;

	for(i=0; i<2; i++) {
		wb_differential_wheels_set_speed(speed[0],speed[1]);
		wb_robot_step(TIME_STEP);
	}
}

void printQ() {
	int i, j;
	printf("Q: \n");
	for(i = 0; i < 20; i++) {
		for(j = 0; j < ACTIONS_SIZE; j++) {
			printf("%g ", Q[i][j]);
		}
		printf("\n");
	}
}
