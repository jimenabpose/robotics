#include <webots/robot.h>
#include <webots/differential_wheels.h>
#include <webots/distance_sensor.h>
#include <stdio.h>
#include <stdlib.h>

#define TIME_STEP 16

// x * y ^ n
// x = maximo valor que puede tomar el sensor discretizado
// y = cantidad de valores discretos (x - 1 = y)
// n = numero de sensor (de 0 a cantidad de sensores)
// La cantidad de sensores es la cantidad de terminos

// Q_SIZE = 5*6^0 + 5*6^1 + 5*6^2 + 5*6^3

#define Q_SIZE 10000
#define ACTIONS_SIZE 3

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
  int accumReward = 0;
  FILE * accumRewards = fopen("accumRewards.csv", "w");
  fprintf(accumRewards, "0,0\n");
  fflush(accumRewards);
  
	
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
    
    accumReward += reward;

		printf("state: %d, action: %d, reward: %d, epsilon: %g\n", currentState, action, reward, epsilon);
		
		updateQ(action, reward, newState);
		
		updateEpsilon();
		
		for (i = 0; i < 8; i++) {
			previous_sensors_value[i] = sensors_value[i];
		}
		
		iterations ++;
		
		if(iterations % 50 == 0) {
  	  saveP();
      fprintf(accumRewards, "%d,%d\n", iterations, accumReward);
      fflush(accumRewards);
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
	int firstState = auxState % 10;
	auxState = auxState / 10;
	int secondState = auxState % 10;
	auxState = auxState / 10;
	int thirdState = auxState % 10;
	auxState = auxState / 10;
	int fourthState = auxState % 10;
	
	int auxSimilar = similarState;
	int firstSimilar = auxSimilar % 10;
	auxSimilar = auxSimilar / 10;
	int secondSimilar = auxSimilar % 10;
	auxSimilar = auxSimilar / 10;
	int thirdSimilar = auxSimilar % 10;
	auxSimilar = auxSimilar / 10;
	int fourthSimilar = auxSimilar % 10;
	
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

	if(sensorValue < 60) {
		return 0;
	}
  if(sensorValue < 100) {
		return 1;
	}
	if(sensorValue < 300) {
		return 2;
	}
	if(sensorValue < 500) {
		return 3;
	}
	if(sensorValue < 1500) {
		return 4;
	}
	if(sensorValue < 2100) {
		return 5;
	}
	if(sensorValue < 2500) {
		return 6;
	}
	if(sensorValue < 2900) {
		return 7;
	}
	if(sensorValue < 3300) {
		return 8;
	}
	return 9;
}

int generateStateValue(double v1, double v2, double v3, double v4) {
	return getDiscreteSensorValue(v1) + getDiscreteSensorValue(v2) * 10 + 
		getDiscreteSensorValue(v3) * 100 + getDiscreteSensorValue(v4) * 1000;
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
		int action_random = (int)(((double)rand() / RAND_MAX) * ACTIONS_SIZE);
		if(action_random == ACTIONS_SIZE) {
			return ACTIONS_SIZE - 1;
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
	
  Q[currentState][action] = Q[currentState][action] + alpha * (reward + (gama * maximum(newState)) - Q[currentState][action]);
}

int getReward(int newState){

	int auxState = currentState;
	int prevS0 = auxState % 10;
	auxState = auxState / 10;
	int prevS1 = auxState % 10;
	auxState = auxState / 10;
	int prevS6 = auxState % 10;
	auxState = auxState / 10;
	int prevS7 = auxState % 10;

	auxState = newState;
	int s0 = auxState % 10;
	auxState = auxState / 10;
	int s1 = auxState % 10;
	auxState = auxState / 10;
	int s6 = auxState % 10;
	auxState = auxState / 10;
	int s7 = auxState % 10;
	
	int emptyState = 0;
	
  printf("s6: %d, s1: %d\n", s6, s1);
	if(s6 == s1) {
		return 2;
	}
	
	if(s0 == emptyState || s7 == emptyState) {
		return -2;
	}
	
	int actualDiff = s6 - s1;
	int prevDiff = prevS6 - prevS1;
	
	if(actualDiff > 0) {
		if(prevDiff > 0) {
			if(actualDiff < prevDiff) {
				return 1;
			} else if(actualDiff == prevDiff) {
        return 0;
      } else {
				return -1;
			}
		} else {
			if(actualDiff < -prevDiff) {
				return 1;
			} else if(actualDiff == -prevDiff) {
        return 0;
      } else {
				return -1;
			}
		}
	} else {
		if(prevS6 - prevS1 > 0) {
			if(-actualDiff < prevDiff) {
				return 1;
			} else if(-actualDiff == prevDiff) {
        return 0;
      } else {
				return -1;
			}
		} else {
			if(-actualDiff < -prevDiff) {
				return 1;
			} else if(-actualDiff == -prevDiff) {
        return 0;
      } else {
				return -1;
			}
		}
	}
}

void updateEpsilon() {
	double decrementRate = 0.005;
  if(epsilon >= decrementRate) {
    epsilon = epsilon - decrementRate;
  } else {
    epsilon = 0;
  }
}

void performAtion(int actionNumber) {

	double speed[2];
	int i;
	int forwardSpeed = 400;

	if(actionNumber == 0) {
		for(i=0; i<8; i++) {
			wb_differential_wheels_set_speed(forwardSpeed,forwardSpeed);
			wb_robot_step(TIME_STEP);
		}
	} else {

		if(actionNumber == 1) {
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

		for(i=0; i<40; i++) {
			wb_differential_wheels_set_speed(forwardSpeed,forwardSpeed);
			wb_robot_step(TIME_STEP);
		}
	
		for(i=0; i<70; i++) {
			wb_differential_wheels_set_speed(-speed[0],-speed[1]);
			wb_robot_step(TIME_STEP);
		}
	
		/*for(i=0; i<15; i++) {
			wb_differential_wheels_set_speed(forwardSpeed,forwardSpeed);
			wb_robot_step(TIME_STEP);
		}*/
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
