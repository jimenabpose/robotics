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

	if(sensorValue < 60) {
		return 0;
	}
	if(sensorValue < 500) {
		return 1;
	}
	if(sensorValue < 1500) {
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
	
  Q[currentState][action] = Q[currentState][action] + alpha * (reward + (gama * maximum(newState))
  	- Q[currentState][action]);
}

int getReward(int newState){
  
  int leftMaxDistance = 1000;
  int leftMinDistance = 100;
  
  int frontMaxDistance = 100;
  
  int leftDifference = 50;
  
  if(sensors_value[5] > leftMinDistance && sensors_value[5] < leftMaxDistance) {
  	
  	if(sensors_value[0] > frontMaxDistance || sensors_value[7] > frontMaxDistance) {
  		return -1;
  	}
  	
  	if((previous_sensors_value[5] - sensors_value[5]) > leftDifference
  		|| (previous_sensors_value[5] - sensors_value[5]) < -leftDifference) {
  		return 0;
		}
		
		return 2;
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

	for(i=0; i<3; i++) {
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

void initialize() {
	int i, j;
	for(i = 0; i < Q_SIZE; i++) {
		for(j = 0; j < ACTIONS_SIZE; j++) {
			Q[i][j] = 0;
		}
	}
	
P[0]=1;
P[1]=0;
P[2]=3;
P[3]=1;
P[4]=0;
P[5]=0;
P[6]=3;
P[7]=0;
P[8]=4;
P[9]=0;
P[10]=0;
P[11]=0;
P[12]=3;
P[13]=0;
P[14]=0;
P[15]=0;
P[16]=4;
P[17]=0;
P[18]=0;
P[19]=3;
P[20]=0;
P[21]=0;
P[22]=0;
P[23]=2;
P[24]=2;
P[25]=0;
P[26]=0;
P[27]=0;
P[28]=3;
P[29]=0;
P[30]=0;
P[31]=0;
P[32]=4;
P[33]=0;
P[34]=0;
P[35]=0;
P[36]=4;
P[37]=0;
P[38]=0;
P[39]=2;
P[40]=0;
P[41]=0;
P[42]=0;
P[43]=0;
P[44]=0;
P[45]=0;
P[46]=0;
P[47]=0;
P[48]=4;
P[49]=0;
P[50]=0;
P[51]=0;
P[52]=0;
P[53]=0;
P[54]=0;
P[55]=0;
P[56]=0;
P[57]=0;
P[58]=0;
P[59]=0;
P[60]=4;
P[61]=0;
P[62]=0;
P[63]=0;
P[64]=0;
P[65]=1;
P[66]=0;
P[67]=4;
P[68]=0;
P[69]=0;
P[70]=1;
P[71]=3;
P[72]=0;
P[73]=0;
P[74]=0;
P[75]=0;
P[76]=0;
P[77]=0;
P[78]=0;
P[79]=0;
P[80]=2;
P[81]=0;
P[82]=0;
P[83]=0;
P[84]=0;
P[85]=0;
P[86]=0;
P[87]=0;
P[88]=1;
P[89]=0;
P[90]=0;
P[91]=0;
P[92]=0;
P[93]=0;
P[94]=0;
P[95]=0;
P[96]=0;
P[97]=0;
P[98]=0;
P[99]=0;
P[100]=0;
P[101]=0;
P[102]=0;
P[103]=2;
P[104]=0;
P[105]=4;
P[106]=0;
P[107]=0;
P[108]=0;
P[109]=0;
P[110]=0;
P[111]=0;
P[112]=0;
P[113]=0;
P[114]=0;
P[115]=0;
P[116]=0;
P[117]=0;
P[118]=2;
P[119]=0;
P[120]=0;
P[121]=0;
P[122]=0;
P[123]=0;
P[124]=4;
P[125]=0;
P[126]=0;
P[127]=2;
P[128]=0;
P[129]=3;
P[130]=0;
P[131]=2;
P[132]=0;
P[133]=0;
P[134]=0;
P[135]=3;
P[136]=0;
P[137]=0;
P[138]=0;
P[139]=0;
P[140]=0;
P[141]=0;
P[142]=0;
P[143]=0;
P[144]=0;
P[145]=1;
P[146]=0;
P[147]=0;
P[148]=0;
P[149]=0;
P[150]=0;
P[151]=0;
P[152]=0;
P[153]=0;
P[154]=0;
P[155]=0;
P[156]=0;
P[157]=0;
P[158]=0;
P[159]=0;
P[160]=0;
P[161]=0;
P[162]=0;
P[163]=3;
P[164]=0;
P[165]=0;
P[166]=0;
P[167]=0;
P[168]=0;
P[169]=0;
P[170]=0;
P[171]=0;
P[172]=0;
P[173]=1;
P[174]=0;
P[175]=0;
P[176]=4;
P[177]=2;
P[178]=0;
P[179]=0;
P[180]=0;
P[181]=0;
P[182]=0;
P[183]=0;
P[184]=0;
P[185]=0;
P[186]=0;
P[187]=0;
P[188]=1;
P[189]=0;
P[190]=0;
P[191]=2;
P[192]=0;
P[193]=0;
P[194]=0;
P[195]=1;
P[196]=0;
P[197]=0;
P[198]=0;
P[199]=1;
P[200]=0;
P[201]=0;
P[202]=0;
P[203]=0;
P[204]=0;
P[205]=0;
P[206]=0;
P[207]=0;
P[208]=0;
P[209]=1;
P[210]=1;
P[211]=0;
P[212]=0;
P[213]=0;
P[214]=0;
P[215]=1;
P[216]=0;
P[217]=0;
P[218]=0;
P[219]=0;
P[220]=0;
P[221]=0;
P[222]=0;
P[223]=0;
P[224]=0;
P[225]=0;
P[226]=2;
P[227]=3;
P[228]=0;
P[229]=0;
P[230]=0;
P[231]=3;
P[232]=0;
P[233]=0;
P[234]=0;
P[235]=0;
P[236]=0;
P[237]=1;
P[238]=0;
P[239]=0;
P[240]=4;
P[241]=0;
P[242]=1;
P[243]=2;
P[244]=0;
P[245]=3;
P[246]=0;
P[247]=3;
P[248]=3;
P[249]=3;
P[250]=3;
P[251]=0;
P[252]=2;
P[253]=2;
P[254]=0;
P[255]=0;

}