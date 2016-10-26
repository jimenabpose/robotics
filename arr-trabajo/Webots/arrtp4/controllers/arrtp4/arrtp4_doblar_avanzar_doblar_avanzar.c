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

double Q[Q_SIZE][ACTIONS_SIZE];
int P[Q_SIZE];
int validP[Q_SIZE];
int currentState;

WbDeviceTag distance_sensor[8];
double sensors_value[8];
double previous_sensors_value[8];

double epsilon = 1;
  
void saveQ();

void saveP();
void buildP();
int getPValue(int state);
int getSimilarState(int state);
int getDistance(int state, int similar);
int checkStateValidity(int state);
int getMaxAction(int state);

int getState();
int chooseAnAction();
double maximum(int newState);
void updateQ(int action, int reward, int newState);
void initialize();
int getReward(int newState);
void updateEpsilon();
void performAtion(int actionNumber);
void printQ();
int getDiscreteSensorValue(double sensorValue);
int generateStateValue(double v1, double v2, double v3, double v4);

 
int main(int argc, char **argv)
{
	/* define variables */

	int i, iterations = 0;
	int newState, reward;
	
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
		
		newState = getState();
		
		reward = getReward(newState);

		printf("state: %d, action: %d, reward: %d, epsilon: %g\n", currentState, action, reward, epsilon);
		
		updateQ(action, reward, newState);
		
		updateEpsilon();
		
		for (i = 0; i < 8; i++) {
			previous_sensors_value[i] = sensors_value[i];
		}
		
		iterations ++;
		
		if(iterations > 100) {
  	  saveP();
			iterations = 0;
		}

	} while (true);
	
	/* Enter here exit cleanup code */
  
	/* Necessary to cleanup webots stuff */

	wb_robot_cleanup();

	return 0;
}

void saveQ() {
	int i, j;
	FILE * matrix = fopen("QValues.txt", "w");
	for(i = 0; i < Q_SIZE; i++) {
		int maxAction = 0;
		for(j = 0; j < ACTIONS_SIZE; j++) {
			if(j != ACTIONS_SIZE-1) {
				fprintf(matrix, "%g,", Q[i][j]);
			} else {
				fprintf(matrix, "%g", Q[i][j]);
			}
			if(Q[i][j] > Q[i][maxAction]) {
				maxAction = j;
			}
		}
		fprintf(matrix, "\n");
	}
	fclose(matrix);
}

void saveP() {
	buildP();
	int i;
	FILE * p = fopen("policy.txt", "w");
	for(i = 0; i < Q_SIZE; i++) {
		fprintf(p, "P[%d]=%d;\n",i, P[i]);
	}
	fclose(p);
}

void buildP() {
	int i;
	
	for(i = 0; i < Q_SIZE; i++) {
		validP[i] = checkStateValidity(i);
	}
	
	for(i = 0; i < Q_SIZE; i++) {
		P[i] = getPValue(i);
	}
}

int getPValue(int state) {
	if(validP[state]) {
		return getMaxAction(state);
	}

	int similarState = getSimilarState(state);
	return getMaxAction(similarState);
}

int getMaxAction(int state) {
	int i;
	int maxAction = 0;
	for(i = 0; i < ACTIONS_SIZE; i++) {
		if(Q[state][i] > Q[state][maxAction]) {
			maxAction = i;
		}
	}
	return maxAction;
}

int getSimilarState(int state) {
	int i;
	int similarState = 0;
	int similarDistance = 50;
	for(i = 0; i < Q_SIZE; i++) {
		if(validP[i]) {
			int distance = getDistance(state, similarState);
			if(distance < 0) {
				distance = -distance;
			}
			
			if(distance < similarDistance) {
				similarDistance = distance;
				similarState = i;
			}
		}
	}
	
	return similarState;
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

int checkStateValidity(int state) {
	int i;
	int maxValue = Q[state][0];
	
	for(i = 0; i < ACTIONS_SIZE; i++) {
		if(Q[state][i] > maxValue) {
			maxValue = Q[state][i];
		}
	}
	
	if(maxValue > 0) {
		return 1;
	} else {
		return 0;
	}
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

	int i;

	/* get sensors values */

	for (i = 0; i < 8; i++) {
		sensors_value[i] = wb_distance_sensor_get_value(distance_sensor[i]);
	}

	return generateStateValue(sensors_value[0], sensors_value[1], sensors_value[6], sensors_value[7]);
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
	
  Q[currentState][action] = Q[currentState][action] + alpha * (reward + (gama * maximum(newState))
  	- Q[currentState][action]);
}

int getReward(int newState){

	int auxState = newState;
	int s0 = auxState % 4;
	auxState = auxState / 4;
	int s1 = auxState % 4;
	auxState = auxState / 4;
	int s6 = auxState % 4;
	auxState = auxState / 4;
	int s7 = auxState % 4;
	auxState = auxState / 4;
	int color = auxState % 4;
	
	if (color == BLUE) {
		if (s6 > s1) {
			return -2;

		}
		else if (s6 < s1) {
			return 1;

		}
		else {
			return 2;
		}
	}

	if (color == RED) {
		if (s6 < s1) {
			return -2;
		}
		else if (s6 > s1) {
			return 1;
		}
		else {
			return 2;
		}
	}

	if (color == GREEN) {
		if (s6 == s1) {
			return 2;
		}
		else {
			return 0;
		}
	}

	return -2;
}

void updateEpsilon() {
	double decrementRate = 0.001;
  if(epsilon >= decrementRate) {
    epsilon = epsilon - decrementRate;
  } else {
    epsilon = 0;
  }
}

void performAtion(int actionNumber) {

	double speed[2];

	if (actionNumber < 3) {

		if(actionNumber == 0) {
			// seguir derecho
			speed[0] = 400;
			speed[1] = 400;
		} else if(actionNumber == 1) {
			// girar 10 a la izquierda
			speed[0] = -300;
			speed[1] = 300;
		} else if(actionNumber == 2) {
			// girar 10 a la derecha
			speed[0] = 300;
			speed[1] = -300;
		}

		int i;
		
		for(i=0; i<20; i++) {
			wb_differential_wheels_set_speed(speed[0],speed[1]);
			wb_robot_step(TIME_STEP);
		}
		
		int forwardSpeed = 400;
		
		for(i=0; i<8; i++) {
			wb_differential_wheels_set_speed(forwardSpeed,forwardSpeed);
			wb_robot_step(TIME_STEP);
		}
	}
	else {

		int i;
		int forwardSpeed = 400;
		if(actionNumber == 3) {
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

		for(i=0; i<50; i++) {
			wb_differential_wheels_set_speed(forwardSpeed,forwardSpeed);
			wb_robot_step(TIME_STEP);
		}
		
		for(i=0; i<70; i++) {
			wb_differential_wheels_set_speed(-speed[0],-speed[1]);
			wb_robot_step(TIME_STEP);
		}
		
		for(i=0; i<25; i++) {
			wb_differential_wheels_set_speed(forwardSpeed,forwardSpeed);
			wb_robot_step(TIME_STEP);
		}
	
}
	
#define ROTATE_SPEED 300
	
void rotate90(int direction) {

	int i;

	for(i=0; i<70; i++) {
		wb_differential_wheels_set_speed(speed[0],speed[1]);
		wb_robot_step(TIME_STEP);
	}
	
	for(i=0; i<50; i++) {
		wb_differential_wheels_set_speed(forwardSpeed,forwardSpeed);
		wb_robot_step(TIME_STEP);
	}
	
	for(i=0; i<70; i++) {
		wb_differential_wheels_set_speed(-speed[0],-speed[1]);
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

void initialize() {
	int i, j;
	for(i = 0; i < Q_SIZE; i++) {
		for(j = 0; j < ACTIONS_SIZE; j++) {
			Q[i][j] = 0;
		}
	}
}
