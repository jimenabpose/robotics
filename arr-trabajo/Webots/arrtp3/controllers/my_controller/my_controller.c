#include <webots/robot.h>
#include <webots/differential_wheels.h>
#include <webots/distance_sensor.h>
#include <stdio.h>
#include <stdlib.h>

#define TIME_STEP 16

// Q_SIZE = 3*4^0 + 3*4^1 + 3*4^2  + 3*4^3
#define Q_SIZE 256
#define ACTIONS_SIZE 5

int P[Q_SIZE];
int validP[Q_SIZE];
int currentState;

WbDeviceTag distance_sensor[8];
double sensors_value[8];

double epsilon = 1;
  
int getState();
int chooseAnAction();
void initialize();
void performAtion(int actionNumber);
int getDiscreteSensorValue(double sensorValue);
int generateStateValue(double v1, double v2, double v3, double v4);

 
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
	}

	initialize();
	
	do {

		currentState = getState();

		int action = chooseAnAction();
		
		performAtion(action);
		
	} while (true);
	
	/* Necessary to cleanup webots stuff */

	wb_robot_cleanup();

	return 0;
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
	return P[currentState];
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


void initialize() {

P[0]=3;
P[1]=0;
P[2]=4;
P[3]=4;
P[4]=3;
P[5]=4;
P[6]=4;
P[7]=4;
P[8]=0;
P[9]=4;
P[10]=4;
P[11]=4;
P[12]=4;
P[13]=4;
P[14]=4;
P[15]=4;
P[16]=4;
P[17]=4;
P[18]=4;
P[19]=4;
P[20]=4;
P[21]=4;
P[22]=4;
P[23]=4;
P[24]=2;
P[25]=0;
P[26]=4;
P[27]=4;
P[28]=4;
P[29]=4;
P[30]=4;
P[31]=4;
P[32]=4;
P[33]=4;
P[34]=4;
P[35]=4;
P[36]=0;
P[37]=4;
P[38]=4;
P[39]=4;
P[40]=4;
P[41]=4;
P[42]=4;
P[43]=4;
P[44]=4;
P[45]=4;
P[46]=4;
P[47]=4;
P[48]=4;
P[49]=4;
P[50]=4;
P[51]=4;
P[52]=0;
P[53]=4;
P[54]=4;
P[55]=4;
P[56]=4;
P[57]=4;
P[58]=4;
P[59]=4;
P[60]=4;
P[61]=4;
P[62]=4;
P[63]=4;
P[64]=0;
P[65]=4;
P[66]=4;
P[67]=4;
P[68]=0;
P[69]=4;
P[70]=4;
P[71]=4;
P[72]=4;
P[73]=4;
P[74]=4;
P[75]=4;
P[76]=4;
P[77]=4;
P[78]=4;
P[79]=4;
P[80]=4;
P[81]=4;
P[82]=4;
P[83]=4;
P[84]=2;
P[85]=4;
P[86]=4;
P[87]=4;
P[88]=2;
P[89]=4;
P[90]=4;
P[91]=4;
P[92]=4;
P[93]=4;
P[94]=4;
P[95]=4;
P[96]=4;
P[97]=4;
P[98]=4;
P[99]=4;
P[100]=4;
P[101]=4;
P[102]=4;
P[103]=4;
P[104]=4;
P[105]=4;
P[106]=4;
P[107]=4;
P[108]=4;
P[109]=4;
P[110]=4;
P[111]=4;
P[112]=4;
P[113]=4;
P[114]=4;
P[115]=4;
P[116]=4;
P[117]=4;
P[118]=4;
P[119]=4;
P[120]=4;
P[121]=4;
P[122]=4;
P[123]=4;
P[124]=4;
P[125]=4;
P[126]=4;
P[127]=4;
P[128]=4;
P[129]=4;
P[130]=4;
P[131]=4;
P[132]=4;
P[133]=4;
P[134]=4;
P[135]=4;
P[136]=4;
P[137]=4;
P[138]=4;
P[139]=4;
P[140]=4;
P[141]=4;
P[142]=4;
P[143]=4;
P[144]=4;
P[145]=4;
P[146]=4;
P[147]=4;
P[148]=4;
P[149]=4;
P[150]=4;
P[151]=4;
P[152]=4;
P[153]=4;
P[154]=4;
P[155]=4;
P[156]=4;
P[157]=4;
P[158]=4;
P[159]=4;
P[160]=4;
P[161]=4;
P[162]=4;
P[163]=4;
P[164]=4;
P[165]=4;
P[166]=4;
P[167]=4;
P[168]=4;
P[169]=4;
P[170]=4;
P[171]=4;
P[172]=4;
P[173]=4;
P[174]=4;
P[175]=4;
P[176]=4;
P[177]=4;
P[178]=4;
P[179]=4;
P[180]=4;
P[181]=4;
P[182]=4;
P[183]=4;
P[184]=4;
P[185]=4;
P[186]=4;
P[187]=4;
P[188]=4;
P[189]=4;
P[190]=4;
P[191]=4;
P[192]=4;
P[193]=4;
P[194]=4;
P[195]=4;
P[196]=4;
P[197]=4;
P[198]=4;
P[199]=4;
P[200]=4;
P[201]=4;
P[202]=4;
P[203]=4;
P[204]=4;
P[205]=4;
P[206]=4;
P[207]=4;
P[208]=4;
P[209]=4;
P[210]=4;
P[211]=4;
P[212]=4;
P[213]=4;
P[214]=4;
P[215]=4;
P[216]=4;
P[217]=4;
P[218]=4;
P[219]=4;
P[220]=4;
P[221]=4;
P[222]=4;
P[223]=4;
P[224]=4;
P[225]=4;
P[226]=4;
P[227]=4;
P[228]=4;
P[229]=4;
P[230]=4;
P[231]=4;
P[232]=4;
P[233]=4;
P[234]=4;
P[235]=4;
P[236]=4;
P[237]=4;
P[238]=4;
P[239]=4;
P[240]=4;
P[241]=4;
P[242]=4;
P[243]=4;
P[244]=4;
P[245]=4;
P[246]=4;
P[247]=4;
P[248]=4;
P[249]=4;
P[250]=4;
P[251]=4;
P[252]=4;
P[253]=4;
P[254]=4;
P[255]=4;

}
