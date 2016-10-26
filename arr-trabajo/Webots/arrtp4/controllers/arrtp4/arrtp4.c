#include <webots/robot.h>
#include <webots/differential_wheels.h>
#include <webots/distance_sensor.h>
#include <webots/camera.h>
#include <stdio.h>
#include <stdlib.h>

#define TIME_STEP 16

#define RED 0
#define GREEN 1
#define BLUE 2

#define LEFT 0
#define RIGHT 1

#define Q_SIZE 768
#define ACTIONS_SIZE 4

const double gama = 0.9;
const double alpha = 0.1;

double Q[Q_SIZE][ACTIONS_SIZE];
double prevQ[Q_SIZE][ACTIONS_SIZE];
int P[Q_SIZE];
int validP[Q_SIZE];
int currentState;

WbDeviceTag camera;
WbDeviceTag distance_sensor[8];
double sensors_value[8];
double previous_sensors_value[8];

double epsilon = 1;
  
double getQSum();
void copyQ();
void saveQ();
void saveP();
void saveCompleteP();
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
int generateStateValue(double v1, double v2, double v3, double v4, int v5);
int getMaxColor();

 
int main(int argc, char **argv)
{
	/* define variables */

	int i, iterations = 0;
	int newState, reward;
	int accumReward = 0;
	
	// Accum Rewards
  FILE * accumRewards = fopen("accumRewards.csv", "w");
  fprintf(accumRewards, "0,0\n");
  fflush(accumRewards);
  
  // Q convergence
  FILE * qConvergence = fopen("qConvergence.csv", "w");
  fprintf(qConvergence, "0,0\n");
  fflush(qConvergence);
	
	/* necessary to initialize webots stuff */

	wb_robot_init();
	
	/* Get and enable camera */
	camera = wb_robot_get_device("camera");
	wb_camera_enable(camera, TIME_STEP);

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

		printf("color: %d, state: %d, action: %d, reward: %d, epsilon: %g\n", getMaxColor(), currentState, action, reward, epsilon);
		
		updateQ(action, reward, newState);
		
		updateEpsilon();
		
		for (i = 0; i < 8; i++) {
			previous_sensors_value[i] = sensors_value[i];
		}
		
		iterations ++;
		
		if(iterations % 50 == 0) {
  	  saveP();
  	  saveCompleteP();
  	  saveQ();
  	  
  	  // accumRewards
      fprintf(accumRewards, "%d,%d\n", iterations, accumReward);
      fflush(accumRewards);
     
      // Q convergence
      fprintf(qConvergence, "%d,%g\n", iterations, getQSum());
      fflush(qConvergence);
      copyQ();
		}

	} while (true);
	
	/* Enter here exit cleanup code */
  
	/* Necessary to cleanup webots stuff */

	wb_robot_cleanup();

	return 0;
}

double getQSum() {
	int i, j;
	double sum;
	for(i = 0; i < Q_SIZE; i++) {
		for(j = 0; j < ACTIONS_SIZE; j++) {
			sum += Q[i][j];
		}
	}
	return sum;
}

void copyQ() {
	int i, j;
	for(i = 0; i < Q_SIZE; i++) {
		for(j = 0; j < ACTIONS_SIZE; j++) {
			prevQ[i][j] = Q[i][j];
		}
	}
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
	//buildP();
	int i;
	int k = 0;
	FILE * p = fopen("policy.txt", "w");
	for(i = 0; i < Q_SIZE; i++) {
		if(checkStateValidity(i)) {
			fprintf(p, "policyArr[%d].state = %d;\n", k, i);
			fprintf(p, "policyArr[%d].action = %d;\n", k, getMaxAction(i));
			k++;
		}
	}
	fclose(p);
}

void saveCompleteP() {
	int i;
	int k = 0;
	FILE * p = fopen("policyComplete.txt", "w");
	for(i = 0; i < Q_SIZE; i++) {
			fprintf(p, "policyArr[%d].state = %d;\n", k, i);
			fprintf(p, "policyArr[%d].action = %d;\n", k, getMaxAction(i));
			k++;
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
	
	return (firstState - firstSimilar) + (secondState - secondSimilar) 
		+ (thirdState - thirdSimilar) + (fourthState - fourthSimilar) + (fifthState - fifthSimilar);
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

	int i;
	
	/* get sensors values */
	for (i = 0; i < 8; i++) {
		sensors_value[i] = wb_distance_sensor_get_value(distance_sensor[i]);
	}

	return generateStateValue(sensors_value[0], sensors_value[1], 
		sensors_value[6], sensors_value[7], getMaxColor());
}

int chooseAnAction(){
//En este caso, el agente elige para un estado dado,
//la acci?n que maximiza el estimado de Q(s,a) con probabilidad (1-?)
//cualquier acci?n al azar con probabilidad ?.

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
	//Q(s, a) ? Q(s, a) + ? [r + ? max a? Q(s?, a?) - Q(s, a)]
	
  Q[currentState][action] = Q[currentState][action] + alpha * (reward + (gama * maximum(newState))
  	- Q[currentState][action]);
}

int getReward(int newState){

	int auxState = newState;
	auxState = auxState / 4;
	int s1 = auxState % 4;
	auxState = auxState / 4;
	int s6 = auxState % 4;
	auxState = auxState / 4;
	auxState = auxState / 4;
	int color = auxState % 4;

	if (color == BLUE) {
		if (s6 > s1) {
			return -2;
		} else if (s6 < s1) {
			return 1;
		} else {
			return 2;
		}
	}

	if (color == RED) {
		if (s6 < s1) {
			return -2;
		} else if (s6 > s1) {
			return 1;
		} else {
			return 2;
		}
	}

	if (color == GREEN) {
		if (s6 == s1) {
			return 3;
		} else {
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
	
	for(i=0; i<forwardTimes; i++) {
		wb_differential_wheels_set_speed(forwardSpeed,forwardSpeed);
		wb_robot_step(TIME_STEP);
	}
	
	for(i=0; i<70; i++) {
		wb_differential_wheels_set_speed(-speed[0],-speed[1]);
		wb_robot_step(TIME_STEP);
	}
}

void goNearTheBar() {
	double s0 = wb_distance_sensor_get_value(distance_sensor[0]);
	double s7 = wb_distance_sensor_get_value(distance_sensor[7]);
	
	while(s0 < 2500 && s7 < 2500) {
		wb_differential_wheels_set_speed(400,400);
		wb_robot_step(TIME_STEP);
		
		s0 = wb_distance_sensor_get_value(distance_sensor[0]);
		s7 = wb_distance_sensor_get_value(distance_sensor[7]);
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
	
	// 0 = ir para adelante
	// 1 = ir al rojo
	// 2 = ir al verde
	// 3 = ir al azul
	
	int actualColor = getMaxColor();
	
	if (actionNumber == 0) {
		printf("SIGO DERECHO\n");
		// seguir derecho
		for(i=0; i<20; i++) {
			wb_differential_wheels_set_speed(forwardSpeed,forwardSpeed);
			wb_robot_step(TIME_STEP);
		}
	} else if(actionNumber == 1) {
		// ir al rojo
		printf("VOY AL ROJO\n");
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
		printf("VOY AL VERDE\n");
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
		printf("VOY AL AZUL\n");
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
