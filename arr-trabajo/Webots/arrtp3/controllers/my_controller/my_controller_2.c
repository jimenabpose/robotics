#include <webots/robot.h>
#include <webots/differential_wheels.h>
#include <webots/distance_sensor.h>
#include <stdio.h>
#include <stdlib.h>

#define TIME_STEP 16

// Q_SIZE = 6*3^0 + 6*3^1 + 6*3^2  + 6*3^3
#define Q_SIZE 26
#define ACTIONS_SIZE 5

const double gama = 0.9;
const double alpha = 0.1;

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
int generateStateValue(int v1, int v2, int v3);

 
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
  
  /*
   * You should declare here DeviceTag variables for storing
   * robot devices like this:
   *  WbDeviceTag my_sensor = wb_robot_get_device("my_sensor");
   *  WbDeviceTag my_actuator = wb_robot_get_device("my_actuator");
   */
  
  /* main loop */
  
  /*
  Definir la representación de los estados (conjunto S) y las acciones (conjunto A)
	del agente y la función de refuerzo. El objetivo es hallar una función Q: S × A →
	R que maximice la recompensa total del agente con las acciones que puede to-
	mar en cada estado.
  
  1. Inicializar Q(s, a) arbitrariamente (por ejemplo, como matriz nula)
	2. Repetir para cada episodio:
		2.1. Leer el estado s
		2.2. Repetir para cada paso del episodio, hasta que s sea terminal:
			2.2.1. Elegir una acción mediante política ε-greedy teniendo en cuenta el es-
				tado actual de la función Q
			2.2.2. Tomar la acción a
			2.2.3. Leer el nuevo estado s’ y obtener el refuerzo r a partir de evaluar la
				función de refuerzo
			2.2.4. Q(s, a) ← Q(s, a) + α [r + γ max a’ Q(s’, a’) - Q(s, a)]
			2.2.5. Actualizar ε para que siga un decrecimiento lineal a través del tiempo
			2.2.6. s ← s’
  
  */
  
  
	initialize();
	
	int newState = getState();
	
  do {
		currentState = newState;
		
		int action = chooseAnAction();
		
		performAtion(action);
		
		newState = getState();
		
		int reward = getReward(newState);
		printf("state: %d, action: %d, reward: %d, epsilon: %g\n", currentState, action, reward, epsilon);
		
		updateQ(action, reward, newState);
		
		updateEpsilon();
		
		/*if(epsilon < 0.5) {
			printQ();
		}*/
		
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
	if(sensorValue < 800) {
		return 1;
	}
	
	return 2;
}

int generateStateValue(int v1, int v2, int v3) {
	return getDiscreteSensorValue(v1) + getDiscreteSensorValue(v2) * 3 + getDiscreteSensorValue(v3) * 9;
}

int getState() {
	int i;
	/* get sensors values */
  for (i = 0; i < 8; i++) {
    sensors_value[i] = wb_distance_sensor_get_value(distance_sensor[i]);
  }
  
//	return (int) (sensors_value[5] / 5);
//int sensors_avg = (sensors_value[5] + sensors_value[4] + sensors_value[6]) / 3;
	return generateStateValue(sensors_value[0], sensors_value[5], sensors_value[6]);
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
	printf("Q[curr][a] = %g | reward = %d | maximum = %g | alpha = %g | gama = %g\n", Q[currentState][action], reward, maximum(newState), alpha, gama);
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
  
  
  int leftMaxDistance = 2000;
  int leftMinDistance = 200;
  
  int frontMaxDistance = 1000;
  
  int leftDifference = 500;
  
  if(sensors_value[5] > leftMinDistance && sensors_value[5] < leftMaxDistance) {
  
  	if(sensors_value[0] > frontMaxDistance || sensors_value[7] > frontMaxDistance) {
  		return -2;
  	}
  	/*
  	if((previous_sensors_value[5] - sensors_value[5]) > leftDifference
  		|| (previous_sensors_value[5] - sensors_value[5]) < -leftDifference) {
  		return 0;
		}
		*/
		return 2;
  }
  
  return -2;
}

void updateEpsilon() {
  if(epsilon >= 0.01) {
    epsilon = epsilon - 0.0005;
  } else {
    epsilon = 0;
  }
}

void performAtion(int actionNumber) {
  double speed[2];
	if(actionNumber == 0) {
	// girar 45 a la izquierda
		speed[0] = -800;
		speed[1] = 800;
	} else if(actionNumber == 1) {
	// girar 45 a la derecha
		speed[0] = 800;
		speed[1] = -800;
	} else if(actionNumber == 2) {
	// girar 10 a la izquierda
		speed[0] = 100;
		speed[1] = 400;
	} else if(actionNumber == 3) {
	// girar 10 a la derecha
		speed[0] = 400;
		speed[1] = 100;
	} else if(actionNumber == 4) {
	// seguir derecho
		speed[0] = 400;
		speed[1] = 400;
	} 

  wb_differential_wheels_set_speed(speed[0],speed[1]);
  wb_robot_step(TIME_STEP);
  
  speed[0] = 400;
  speed[1] = 400;
  wb_differential_wheels_set_speed(speed[0],speed[1]);
  wb_robot_step(TIME_STEP);
  
}

void printQ() {
	int i, j;
	printf("Q: \n");
	for(i = 0; i < Q_SIZE; i++) {
		for(j = 0; j < ACTIONS_SIZE; j++) {
			printf("%g ", Q[i][j]);
		}
		printf("\n");
	}
}
