#include <webots/robot.h>
#include <webots/differential_wheels.h>
#include <webots/distance_sensor.h>
#include <stdio.h>
#include <stdlib.h>

#define TIME_STEP 64

#define Q_SIZE 1000
#define ACTIONS_SIZE 5

const double gama = 0.9;
int iterations = 10;

int Q[Q_SIZE][ACTIONS_SIZE];
int currentState;

double sensors_value[8];

int rewardDifference = 1000;

int currentState;

double epsilon = 1;
  
int getState();
int chooseAnAction();
int maximum(int newState);
void updateQ(int action, int reward, int newState);
void initialize();
int getReward();
void updateEpsilon();
void performAtion(int actionNumber);
void printQ();

 
int main(int argc, char **argv)
{
  /* define variables */
  WbDeviceTag distance_sensor[8];
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
	
  do {
  	/* get sensors values */
    for (i = 0; i < 8; i++) {
      sensors_value[i] = wb_distance_sensor_get_value(distance_sensor[i]);
    }
  
		currentState = getState();
		
		int action = chooseAnAction();
		
		performAtion(action);
		
		int reward = getReward();
		printf("state: %d, action: %d, reward: %d, epsilon: %g\n", currentState, action, reward, epsilon);
		
		updateQ(action, reward, getState());
		
		updateEpsilon();
		
		//printQ();
		
  } while (true);
  
  /* Enter here exit cleanup code */
  
  /* Necessary to cleanup webots stuff */
  wb_robot_cleanup();
  
  return 0;
}

int getState() {
//	return (int) (sensors_value[5] / 5);
	int sensors_avg = (sensors_value[5] + sensors_value[4] + sensors_value[6]) / 3;
	return sensors_avg / 50;
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

int maximum(int newState){
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
  Q[currentState][action] = Q[currentState][action] + 0.1 * (reward + gama * maximum(newState));
}

void initialize() {
	int i, j;
	for(i = 0; i < Q_SIZE; i++) {
		for(j = 0; j < ACTIONS_SIZE; j++) {
			Q[i][j] = 0;
		}
	}
}

int getReward(){
	int rewardMin = 8;
	int rewardMax = 20;
	if(currentState > rewardMin && currentState < rewardMax) {
		return 1;
	}
  return 0;
}

void updateEpsilon() {
  if(epsilon >= 0.01) {
    epsilon = epsilon - 0.0001;
  } else {
    epsilon = 0;
  }
}

void performAtion(int actionNumber) {
  double speed[2];
	if(actionNumber == 0) {
	// seguir derecho
		speed[0] = 400;
		speed[1] = 400;
	} else if(actionNumber == 1) {
	// girar 45 a la izquierda
		speed[0] = -400;
		speed[1] = 400;
	} else if(actionNumber == 2) {
	// girar 45 a la derecha
		speed[0] = 400;
		speed[1] = -400;
	} else if(actionNumber == 3) {
	// girar 10 a la izquierda
		speed[0] = -100;
		speed[1] = 100;
	} else if(actionNumber == 4) {
	// girar 10 a la derecha
		speed[0] = 100;
		speed[1] = -100;
	}
	
	wb_differential_wheels_set_speed(speed[0],speed[1]);
	wb_robot_step(TIME_STEP);
}

void printQ() {
	int i, j;
	printf("Q: \n");
	for(i = 0; i < 20; i++) {
		for(j = 0; j < ACTIONS_SIZE; j++) {
			printf("%d ", Q[i][j]);
		}
		printf("\n");
	}
}
